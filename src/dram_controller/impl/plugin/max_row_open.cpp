#include <vector>
#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/pluginutil/device_config.h"
#include <cstdio>


namespace Ramulator {

class MaxRowOpen : public IControllerPlugin, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, MaxRowOpen, "MaxRowOpen", "Force-close rows that exceed a maximum open time (RowPress tmro cap).")

    private:

    IDRAM* m_dram = nullptr;
    DeviceConfig m_cfg;
    Clk_t m_clk = 0;

    int m_max_row_open_ns= -1;
    uint64_t m_max_row_open_clk = 0;
    bool m_debug = false;

    int m_rank_level = -1;
    int m_bankgroup_level = -1;
    int m_bank_level = -1;
    int m_row_level = -1;
    int m_num_banks_total = -1;
    int m_num_ranks = -1;
    int m_num_bankgroups = -1;
    int m_num_banks = -1;


    int m_PRE_req_id = -1;
    std::vector<int64_t> m_row_opened_at;
    std::vector<std::vector<int>> m_open_addr_vec;



    public:

    void init() override {
        m_max_row_open_ns = param<int>("max_open_ns").desc("Max time (ns) a row can stay open before being force-closed (RowPress tmro cap)").required();
        m_debug = param<bool>("debug").default_val(false);
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        m_ctrl = cast_parent<IDRAMController>();
        m_dram = m_ctrl->m_dram;
        m_cfg.set_device(m_ctrl);

        m_max_row_open_clk = (uint64_t)(m_max_row_open_ns / ((float) m_dram->m_timing_vals("tCK_ps") / 1000.0f));

        m_rank_level = m_dram->m_levels("rank");
        m_bankgroup_level = m_dram->m_levels("bankgroup");
        m_bank_level = m_dram->m_levels("bank");
        m_row_level = m_dram->m_levels("row");

        m_num_ranks = m_dram->get_level_size("rank");
        m_num_bankgroups = m_dram->get_level_size("bankgroup");
        m_num_banks = m_dram->get_level_size("bank");
        m_num_banks_total = m_num_ranks * m_num_bankgroups * m_num_banks;

        m_PRE_req_id = m_dram->m_requests("close-row");

        m_row_opened_at.assign(m_num_banks_total, -1);
        m_open_addr_vec.assign(m_num_banks_total, std::vector<int>());
    }

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
        m_clk++;
        if (request_found) {
            if (m_dram->m_command_meta(req_it->command).is_closing ||
                m_dram->m_command_meta(req_it->command).is_refreshing) {

                if (req_it->addr_vec[m_bankgroup_level] == -1 && req_it->addr_vec[m_bank_level] == -1){
                    for (int b = 0; b < m_num_banks; b++){
                        for (int bg = 0; bg < m_num_bankgroups; bg++){
                            int rank_id = req_it->addr_vec[m_rank_level];
                            int flat_bank_id = b + bg * m_num_banks + rank_id * m_num_banks * m_num_bankgroups;
                            m_row_opened_at[flat_bank_id] = -1;
            
                        }
                    }
                }
                else if (req_it->addr_vec[m_bankgroup_level] == -1){
                    for (int bg = 0; bg < m_num_bankgroups; bg++) {
                        int bank_id = req_it->addr_vec[m_bank_level];
                        int rank_id = req_it->addr_vec[m_rank_level];
                        int flat_bank_id = bank_id + bg * m_num_banks + rank_id * m_num_banks * m_num_bankgroups;
                        m_row_opened_at[flat_bank_id] = -1;
                    }
                }
                else {
                    int flat_bank_id = m_cfg.get_flat_bank_id(*req_it);
                    m_row_opened_at[flat_bank_id] = -1;
                }

            }
            else if (m_dram->m_command_meta(req_it->command).is_opening &&
                m_dram->m_command_scopes(req_it->command) == m_row_level) {

                int flat_bank_id = m_cfg.get_flat_bank_id(*req_it);
                m_row_opened_at[flat_bank_id] = m_clk;
                m_open_addr_vec[flat_bank_id] = req_it->addr_vec;
            }

        }

        for (int flat_bank_id = 0; flat_bank_id < m_num_banks_total; flat_bank_id++) {
            if (m_row_opened_at[flat_bank_id] == -1) continue; // if the bank is already open, continue

            int64_t open_for = (int64_t)m_clk - m_row_opened_at[flat_bank_id];
            if (open_for <= (int64_t)m_max_row_open_clk) continue; // if the open time did not exceed the max row open time, continue

            if (request_found && m_cfg.get_flat_bank_id(*req_it) == flat_bank_id) continue;

            Request pre_req(m_open_addr_vec[flat_bank_id], m_PRE_req_id);
            if (m_ctrl->priority_send(pre_req)) {
                if (m_debug) {
                    std::printf("[MaxRowOpen] bank=%d forced PRE after %lld clk (cap=%llu)\n",
                                flat_bank_id, (long long)open_for, (unsigned long long)m_max_row_open_clk);
                }
                m_row_opened_at[flat_bank_id] = -1;
            }
        }

    }






};

}       // namespace Ramulator
