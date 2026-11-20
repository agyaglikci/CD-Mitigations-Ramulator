#include <vector>
#include <unordered_map>
#include <limits>
#include <random>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "translation/translation.h"
#include "addr_mapper/impl/rit.h"
#include "dram_controller/impl/plugin/throttleable.h"
#include "dram_controller/impl/pluginutil/device_config.h"

namespace Ramulator {

class AQUA : public IControllerPlugin, public Implementation, public IThrottleable {
  RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, AQUA, "AQUA", "AQUA.")

  private:
    IDRAM* m_dram = nullptr;
    LinearMapperBase_with_rit* m_addr_mapper = nullptr;
    ITranslation* m_translation = nullptr;
    DeviceConfig m_cfg;

    Clk_t m_clk = 0;

    int m_num_art_entries = -1;
    int m_num_fpt_entries = -1;
    int m_num_qrows_per_bank = -1;
    int m_art_threshold = -1;
    int m_reset_period_ns = -1;
    Clk_t m_reset_period_clk = -1;
    bool m_is_debug = false;

    int m_rqa_head = 0;
    int m_num_banks_total = -1;
    int m_num_qrows_total = -1;

    int m_RD_req_id = -1;
    int m_WR_req_id = -1;
    int m_aqua_copy_read_id = -1;
    int m_aqua_copy_write_id = -1;

    int m_rank_level = -1;
    int m_bank_level = -1;
    int m_row_level = -1;
    int m_col_level = -1;

    int m_num_ranks = -1;
    int m_num_banks_per_rank = -1;
    int m_num_rows_per_bank = -1;
    int m_num_cls = -1;

    // per bank hot-row tracker (same as Graphene)
    // indexed using flattened <rank id, bank id>
    // e.g., if rank 0, bank 4, index is 4
    // if rank 1, bank 5, index is 16 (assuming 16 banks/rank) + 5
    std::vector<std::unordered_map<int, int>> m_aggressor_row_tracker;
    // spillover counter per bank
    std::vector<int> m_spillover_counter;
    // per bank row indirection table is implemented in 'src/addr_mapper/impl/linear_mappers_with_rit.cpp'

    std::unordered_map<int, std::pair<int,int>> m_reverse_pointer_table;

    // rng
    std::mt19937 generator;
    std::uniform_int_distribution<int> distribution;

    // statistics
    int s_num_migrations = 0;
    int s_num_r_migrations = 0;
    int s_num_self_collisions = 0;

  public:
    void init() override { 
      m_num_art_entries = param<int>("num_art_entries").required();
      m_num_fpt_entries = param<int>("num_fpt_entries").required();
      m_num_qrows_per_bank = param<int>("num_qrows_per_bank").required();
      m_art_threshold = param<int>("art_threshold").required();
      m_reset_period_ns = param<int>("reset_period_ns").required();
      m_is_debug = param<bool>("debug").default_val(false);
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IDRAMController>();
      m_dram = m_ctrl->m_dram;
      m_addr_mapper = (LinearMapperBase_with_rit*) memory_system->get_ifce<IAddrMapper>();
      m_translation = frontend->get_ifce<ITranslation>();

      m_cfg.set_device(m_ctrl);
      throttleable_setup(m_cfg.m_num_banks, 2);

      m_reset_period_clk = m_reset_period_ns / ((float) m_dram->m_timing_vals("tCK_ps") / 1000.0f);

      m_RD_req_id = m_dram->m_requests("read");
      m_WR_req_id = m_dram->m_requests("write");
      m_aqua_copy_read_id = m_dram->m_requests("aqua-copy-read");
      m_aqua_copy_write_id = m_dram->m_requests("aqua-copy-write");

      m_rank_level = m_dram->m_levels("rank");
      m_bank_level = m_dram->m_levels("bank");
      m_row_level = m_dram->m_levels("row");
      m_col_level = m_dram->m_levels("column");

      m_num_ranks = m_dram->get_level_size("rank");
      m_num_banks_per_rank = m_dram->get_level_size("bankgroup") == -1 ? 
                             m_dram->get_level_size("bank") : 
                             m_dram->get_level_size("bankgroup") * m_dram->get_level_size("bank");
      m_num_rows_per_bank = m_dram->get_level_size("row");
      m_num_cls = m_dram->get_level_size("column") / 8;

      m_num_banks_total = m_num_banks_per_rank * m_num_ranks;
      m_num_qrows_total = m_num_qrows_per_bank * m_num_banks_total;

      // Initialize hot-row tracker
      for (int i = 0; i < m_num_banks_total; i++) {
        std::unordered_map<int, int> table;
        m_aggressor_row_tracker.push_back(table);
      }
      // Initialize spillover counter
      m_spillover_counter = std::vector<int>(m_num_banks_total, 0);
      // Initialize rank-global AQUA indirection table in the addr_mapper
      m_addr_mapper->init_aqua_rit(m_num_fpt_entries * 2);

      reserve_rows_for_aqua();
      
      // setup random number generator
      generator = std::mt19937(1337);
      distribution = std::uniform_int_distribution<int>(0, m_num_rows_per_bank-1);

      // Register statistics
      register_stat(s_num_migrations).name("aqua_migrations");
      register_stat(s_num_r_migrations).name("aqua_r_migrations");
      register_stat(s_num_self_collisions).name("aqua_self_collisions");

      if (m_is_debug) {
        std::cout << "AQUA is implemented." << std::endl
                  << "ART size: " << m_num_art_entries << std::endl
                  << "FPT size: " << m_num_fpt_entries << std::endl
                  << "ART threshold: " << m_art_threshold << std::endl
                  << "Number of rows: " << m_num_rows_per_bank << std::endl
                  << "Number of banks: " << m_num_ranks * m_num_banks_per_rank << std::endl
                  << "------------------------------------------------" << std::endl;
      }
    }

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
      // Tick myself
      m_clk++;

      if (m_clk % m_reset_period_clk == 0) {
        // Reset hrt and unlock rit
        for (int i = 0; i < m_num_banks_per_rank * m_num_ranks; i++) {
          m_aggressor_row_tracker[i].clear();
          m_spillover_counter[i] = 0;
          // m_addr_mapper->rit_unlock();
        }
      }

      if (request_found) {
        if (m_dram->m_command_meta(req_it->command).is_opening && m_dram->m_command_scopes(req_it->command) == m_row_level) {
          int flat_bank_id = req_it->addr_vec[m_bank_level];
          int accumulated_dimension = 1;
          for (int i = m_bank_level - 1; i >= m_rank_level; i--) {
            accumulated_dimension *= m_dram->m_organization.count[i + 1];
            flat_bank_id += req_it->addr_vec[i] * accumulated_dimension;
          }
          
          int row_id = req_it->addr_vec[m_row_level];

          if (m_is_debug) {
            std::cout << "----------------------------" << std::endl;
            std::cout << "AQUA: ACT on row " << row_id << "         " << m_clk << std::endl;
            std::cout << "  └  " << "bank: " << flat_bank_id << std::endl;
          }

          // Check HRT
          if (m_aggressor_row_tracker[flat_bank_id].find(row_id) == m_aggressor_row_tracker[flat_bank_id].end()) {
            if (m_is_debug) {
              std::cout << "  └  " << "row " << row_id << " not in HRT." << std::endl;
            }
            // if row is not in the table, check if the table is full 
            if (m_aggressor_row_tracker[flat_bank_id].size() < m_num_art_entries) {
              if (m_is_debug) {
                std::cout << "  └  " << "HRT is not full, inserting with count 1." << std::endl;
              }
              // if table is not full, insert the row
              m_aggressor_row_tracker[flat_bank_id][row_id] = 1;
            } else {
              if (m_is_debug) {
                std::cout << "  └  " << "HRT is full, searching for a row to evict." << std::endl;
              }
              // if table is full, find a row to evict
              bool found = false;
              int to_remove = -1;
              int spillover_value = -1;

              for (auto it = m_aggressor_row_tracker[flat_bank_id].begin(); it != m_aggressor_row_tracker[flat_bank_id].end(); it++) {
                // if we find an entry with spillover counter value, evict it
                if (it->second == m_spillover_counter[flat_bank_id]) {
                  if (m_is_debug) {
                    std::cout << "  └  " << "found a row to evict: " << it->first << std::endl;
                  }
                  // if we find an entry, record it
                  spillover_value = it->second;
                  to_remove = it->first;
                  found = true;
                  break;
                }
              }

              if (found) {
                if (m_is_debug) {
                  std::cout << "Removing row " << to_remove << " from HRT." << std::endl;
                  std::cout << "Adding row " << row_id << " to HRT." << std::endl;
                }
                // remove to_remove from the table
                m_aggressor_row_tracker[flat_bank_id].erase(to_remove);
                // add row_id to the table
                m_aggressor_row_tracker[flat_bank_id][row_id] = spillover_value + 1;
              }
              else {
                if (m_is_debug) {
                  std::cout << "  └  " << "no row to evict, incrementing spillover counter." << std::endl;
                }
                m_spillover_counter[flat_bank_id] += 1;
                return;
              }
            }
          } else {
            if (m_is_debug) { 
              std::cout << "  └  " << "row " << row_id << " in HRT. Incrementing its counter." << std::endl;
            }
            // if row in table, increment its activation count
            m_aggressor_row_tracker[flat_bank_id][row_id] += 1;
          }
          // dump HRT for debug
          // if (m_is_debug) {
          //   std::cout << "==========================" << std::endl;
          //   std::cout << "HRT[" << flat_bank_id << "].size(): " << m_hot_row_tracker[flat_bank_id].size() << std::endl;
          //   for (auto entry: m_hot_row_tracker[flat_bank_id]) {
          //     std::cout << entry.first << ":\t" << entry.second << std::endl; 
          //   }
          //   std::cout << "Spillover counter: " << m_spillover_counter[flat_bank_id] << std::endl;
          //   std::cout << "==========================" << std::endl;
          // }

          // row is now in the table, check if the count exceeds the threshold
          if (m_is_debug) {
            std::cout << "Row " << row_id << " in ART" << std::endl;
            std::cout << "  └  " << "threshold: " << m_art_threshold << std::endl;
            std::cout << "  └  " << "count: " << m_aggressor_row_tracker[flat_bank_id][row_id] << std::endl;
          }
          if (m_aggressor_row_tracker[flat_bank_id][row_id] % m_art_threshold == 0) {
            if (m_is_debug) {
              std::cout << "Row " << row_id << " needs quarantine!" << std::endl;
              std::cout << "  └  " << "RQA head: " << m_rqa_head << std::endl;
            }

            int art_bank = flat_bank_id;
            int art_key = row_id;

            int slot = -1;
            if (row_id < m_num_qrows_per_bank) {
              int candidate = flat_bank_id + row_id * m_num_banks_total;
              if (m_reverse_pointer_table.find(candidate) != m_reverse_pointer_table.end())
                slot = candidate;
            } else {
              int existing_bank = -1, existing_row = -1;
              if (m_addr_mapper->check_aqua_rit(flat_bank_id, row_id, existing_bank, existing_row)) {
                int candidate = existing_bank + existing_row * m_num_banks_total;
                if (m_reverse_pointer_table.find(candidate) != m_reverse_pointer_table.end())
                  slot = candidate;
              }
            }

            if (slot == m_rqa_head) {
              s_num_self_collisions++;
              if (m_is_debug) {
                std::cout << "SELF-COLLISION [" << m_clk << "] bank=" << art_bank
                          << " row_id==m_rqa_head==" << slot
                          << " (art_key=" << art_key << "), skipping migration."
                          << std::endl;
              }
              m_aggressor_row_tracker[art_bank].erase(art_key);
              m_rqa_head = (m_rqa_head + 1) % m_num_qrows_total;
              return;
            }

            int dst_bank = m_rqa_head % m_num_banks_total;
            int dst_row = m_rqa_head / m_num_banks_total;

            auto head_it = m_reverse_pointer_table.find(m_rqa_head);
            if (head_it != m_reverse_pointer_table.end()) {
              if (m_is_debug) {
                std::cout << "RQA head is valid, evicting row " << m_rqa_head << std::endl;
              }
              int prev_org_bank = head_it->second.first;
              int prev_org_row = head_it->second.second;
              m_reverse_pointer_table.erase(m_rqa_head);
              m_addr_mapper->aqua_rit_remove_entry(prev_org_bank, prev_org_row, dst_bank, dst_row);

              issue_migration(req_it, dst_bank, dst_row, prev_org_bank, prev_org_row);
              s_num_r_migrations++;
            } else {
              if (m_is_debug) {
                std::cout << "RQA head is empty, issuing migration." << std::endl;
              }
            }

            if (slot != -1) {
              auto slot_it = m_reverse_pointer_table.find(slot);
              int org_bank = slot_it->second.first;
              int org_row = slot_it->second.second;
              m_reverse_pointer_table.erase(slot);
              int old_bank = slot % m_num_banks_total;
              int old_row = slot / m_num_banks_total;
              m_aggressor_row_tracker[old_bank].erase(old_row);
              m_addr_mapper->aqua_rit_remove_entry(org_bank, org_row, old_bank, old_row);

              issue_migration(req_it, old_bank, old_row, dst_bank, dst_row);

              m_reverse_pointer_table[m_rqa_head] = {org_bank, org_row};
              m_addr_mapper->aqua_rit_insert_entry(org_bank, org_row, dst_bank, dst_row);
            } else {
              issue_migration(req_it, art_bank, art_key, dst_bank, dst_row);

              m_reverse_pointer_table[m_rqa_head] = {art_bank, art_key};
              m_addr_mapper->aqua_rit_insert_entry(art_bank, art_key, dst_bank, dst_row);
            }

            m_aggressor_row_tracker[art_bank].erase(art_key);
            m_aggressor_row_tracker[dst_bank].erase(dst_row);

            if (m_is_debug) {
              std::cout << "AQUA_RIT SIZE [" << m_clk << "] after_migration="
                        << m_addr_mapper->m_aqua_rit.size()
                        << " slot=" << slot
                        << " art_bank=" << art_bank << " art_key=" << art_key
                        << " dst_bank=" << dst_bank << " dst_row=" << dst_row
                        << std::endl;
            }

            s_num_migrations++;
            m_rqa_head = (m_rqa_head + 1) % m_num_qrows_total;
          }
        }
      }
    }

    void issue_migration(ReqBuffer::iterator& req_it, int src_bank, int src_row, int dst_bank, int dst_row) {
      if (m_is_debug) {
        std::cout << "MIGRATION START [" << m_clk << "] src_bank=" << src_bank
                   << " src=" << src_row << " dst_bank=" << dst_bank
                   << " dst=" << dst_row << " num_cls=" << m_num_cls << std::endl;
      }

      increment_operation(m_cfg.get_flat_bank_id(*req_it), req_it->source_id);

      std::vector<int> addr_vec;
      for (int i = 0; i < req_it->addr_vec.size(); i++){
        addr_vec.push_back(req_it->addr_vec[i]);
      }

      m_addr_mapper->set_flat_bank_id(addr_vec, src_bank);
      addr_vec[m_row_level] = src_row;
      for (int cl = 0; cl < m_num_cls; cl++){
        addr_vec[m_col_level] = cl << 3;
        Request swap_read_req(addr_vec, m_aqua_copy_read_id);
        if (!m_ctrl->priority_send(swap_read_req)){
          std::cerr << "Check priority queue max size." << std::endl;
          exit(1);
        }
      }

      m_addr_mapper->set_flat_bank_id(addr_vec, dst_bank);
      addr_vec[m_row_level] = dst_row;
      for (int cl = 0; cl < m_num_cls; cl++){
        addr_vec[m_col_level] = cl << 3;
        Request swap_write_req(addr_vec, m_aqua_copy_write_id);
        if (!m_ctrl->priority_send(swap_write_req)){
          std::cerr << "Check priority queue max size." << std::endl;
          exit(1);
        }
      }

      if (m_is_debug) {
        std::cout << "MIGRATION END   [" << m_clk << "] src_bank=" << src_bank
                   << " src=" << src_row << " dst_bank=" << dst_bank
                   << " dst=" << dst_row << " total_migrations=" << s_num_migrations << std::endl;
      }
    }

    void reserve_rows_for_aqua() {
      Addr_t max_addr = m_translation->get_max_addr();
      // traverse all cls and reserve them if they use rows that store RCT
      Request req(0, 0);
      for (Addr_t addr = 0; addr < max_addr; addr += 64) {
        // apply address mapping
        req.addr = addr;
        m_addr_mapper->apply(req);
        Addr_t row_id = req.addr_vec[m_row_level];
        if (row_id < m_num_qrows_per_bank){
          m_translation->reserve("AQUA", addr);
        }
      }
    }
};

}       // namespace Ramulator
