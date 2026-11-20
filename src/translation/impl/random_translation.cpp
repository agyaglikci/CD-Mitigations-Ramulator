#include <iostream>
#include <set>
#include <unordered_set>
#include <vector>
#include <random>

#include "base/base.h"
#include "translation/translation.h"
#include "frontend/frontend.h"
#include "memory_system/memory_system.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"


namespace Ramulator {

class RandomTranslation : public ITranslation, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(ITranslation, RandomTranslation, "RandomTranslation", "Randomly allocate physical pages to virtual pages.");

  IFrontEnd* m_frontend;

  protected:
    std::mt19937_64 m_allocator_rng;

    Addr_t m_max_paddr;         // Max physical address
    Addr_t m_pagesize;          // Page size in bytes
    int    m_offsetbits;        // The number of bits for the page offset
    size_t m_num_pages;         // The total number of physical pages 

    std::vector<bool> m_free_physical_pages;   // The set of remaining pages.
    size_t m_num_free_physical_pages;

    using Translation_t = std::vector<std::unordered_map<Addr_t, Addr_t>>;
    Translation_t m_translation;    // A vector of <vpn:ppn> maps, each core has its own map

    std::unordered_set<Addr_t> m_reserved_pages;   // A vector of reserved pages

    // Subarray-aware allocation: pages are drawn from a pool of rows below
    // subarray_rows * num_subarrays, and attacker cores get a deterministic pool
    // pinned to a subarray span. Enabled when subarray_rows > 0 and num_subarrays > 0.
    bool   m_subarray_alloc = false;
    int    m_subarray_rows = 0;
    int    m_num_subarrays = 0;
    Addr_t m_row_limit     = 0;             // = subarray_rows * num_subarrays
    IAddrMapper* m_addr_mapper = nullptr;
    int    m_row_idx = -1;
    std::vector<int> m_bank_level_idx;
    std::vector<int> m_bank_level_size;
    std::unordered_set<int> m_attacker_cores;
    int m_attacker_subarray = 0;
    int m_attacker_bank = 0;
    std::vector<Addr_t> m_attacker_ppn_pool;
    size_t m_attacker_pool_next = 0;
    bool m_attacker_pool_built = false;
    std::unordered_set<int> m_attacker_pool_rows;
    bool m_debug = false;

    // CD attack
    int m_attacker_span = 1;
    int m_attacker_victim_row_offset = -1;

    
    std::vector<Addr_t> m_benign_ppn_pool;
    bool m_benign_pool_built = false;


  public:
    void init() override {
      int seed = param<int>("seed").desc("The seed for the random number generator used to allocate pages.").default_val(123);
      m_allocator_rng.seed(seed);

      m_max_paddr   = param<Addr_t>("max_addr").desc("Max physical address of the memory system.").required();
      m_pagesize    = param<Addr_t>("pagesize_KB").desc("Pagesize in KB.").default_val(4) << 10;
      m_offsetbits  = calc_log2(m_pagesize);

      // Initially, all physical pages are free
      m_num_pages = m_max_paddr / m_pagesize;
      m_free_physical_pages.resize(m_num_pages, true);
      m_num_free_physical_pages = m_num_pages;

      m_frontend = cast_parent<IFrontEnd>();
      m_translation.resize(m_frontend->get_num_cores());

      // Subarray geometry (optional — if subarray_rows/num_subarrays are 0, plain random allocation).
      m_subarray_rows = param<int>("subarray_rows").desc("Number of rows per subarray.").default_val(0);
      m_num_subarrays = param<int>("num_subarrays").desc("Number of subarrays per bank.").default_val(0);
      m_subarray_alloc = (m_subarray_rows > 0 && m_num_subarrays > 0);
      m_row_limit = static_cast<Addr_t>(m_subarray_rows) * static_cast<Addr_t>(m_num_subarrays);
      std::vector<int> attacker_cores = param<std::vector<int>>("attacker_cores").desc("Core ids using deterministic (non-random) physical addressing.").default_val(std::vector<int>{});
      for (int cid:attacker_cores) m_attacker_cores.insert(cid);
      m_attacker_subarray = param<int>("attacker_subarray").desc("Subarray index attacker cores are pinned to.").default_val(0);
      m_attacker_bank = param<int>("attacker_bank").desc("Flat bank index (channel, rank, bankgroup, bank) attacker cores are pinned to.").default_val(0);
      m_debug = param<bool>("debug").desc("Print per-assignment PPN/subarray info.").default_val(false);

      // CD attack
      m_attacker_span = param<int>("attacker_span").desc("Number of consecutive subarrays (centered on attacker_subarray) the attacker pool spans.").default_val(1);
      m_attacker_victim_row_offset = param<int>("attacker_victim_row_offset").desc("Row offset within attacker_subarray excluded from the pool (never accessed). -1 = no victim.").default_val(-1);


      m_logger = Logging::create_logger("RandomTranslation");
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      if (!m_subarray_alloc) return;
      m_addr_mapper = memory_system->get_ifce<IAddrMapper>();
      IDRAM* dram   = memory_system->get_ifce<IDRAM>();
      m_row_idx     = dram->m_levels("row");
      for (const char* level : {"channel", "rank", "bankgroup", "bank"}) {
        m_bank_level_idx.push_back(dram->m_levels(level));
        m_bank_level_size.push_back(dram->get_level_size(level));
      }
      m_logger->info(
        "Subarray-aware allocation enabled: subarray_rows={}, num_subarrays={}, row_limit={}.",
        m_subarray_rows, m_num_subarrays, m_row_limit);
    }

    // Runs a candidate PPN through the real AddrMapper and returns its decoded
    // row index. Shared by in_row_limit, build_attacker_pool, and the debug prints
    // so they all agree with what the DRAM controller will actually see.
    AddrVec_t decode(Addr_t ppn) {
      Request tmp(static_cast<Addr_t>(ppn) << m_offsetbits, Request::Type::Read);
      m_addr_mapper->apply(tmp);
      return tmp.addr_vec;
    }

    int decode_row(Addr_t ppn) {
      return decode(ppn)[m_row_idx];
    }

    int flat_bank(const AddrVec_t& addr_vec) {
      int bank = 0;
      for (size_t i = 0; i < m_bank_level_idx.size(); i++) {
        bank = bank * m_bank_level_size[i] + addr_vec[m_bank_level_idx[i]];
      }
      return bank;
    }

    // Returns true if this PPN decodes to a row below the configured row limit.
    bool in_row_limit(Addr_t ppn) {
      return static_cast<Addr_t>(decode_row(ppn)) < m_row_limit;
    }

    void build_attacker_pool() {
      if (!m_subarray_alloc) {
        m_logger->error("attacker_cores set but subarray-aware allocation disabled (subarray_rows/num_subarrays missing).");
        return;
      }
      // CD attack
      int span_lo = m_attacker_subarray - (m_attacker_span - 1) / 2;
      int span_hi = m_attacker_subarray + m_attacker_span / 2;
      Addr_t victim_row = (m_attacker_victim_row_offset >= 0)
          ? static_cast<Addr_t>(m_attacker_subarray) * m_subarray_rows + m_attacker_victim_row_offset
          : static_cast<Addr_t>(-1);

      std::set<int> banks_in_span;
      int  dbg_run_row = -1;
      Addr_t dbg_run_start = 0;
      int dbg_transitions_printed = 0;
      for (Addr_t ppn = 0; ppn < (Addr_t)m_num_pages; ppn++) {
        AddrVec_t addr_vec = decode(ppn);
        int row = addr_vec[m_row_idx];
        if (m_debug) {
          if (row != dbg_run_row) {
            if (dbg_run_row != -1 && dbg_transitions_printed < 40) {
              std::cout << "[ROW-SCAN] ppn[" << dbg_run_start << ".." << (ppn - 1)
                         << "] (" << (ppn - dbg_run_start) << " pages) -> row="
                         << dbg_run_row << std::endl;
              dbg_transitions_printed++;
            }
            dbg_run_row = row;
            dbg_run_start = ppn;
          }
        }
        if (static_cast<Addr_t>(row) >= m_row_limit) continue;
        int sa = row / m_subarray_rows;
        int bank = flat_bank(addr_vec);
        bool in_span = (static_cast<Addr_t>(row) == victim_row)
                       || (sa >= span_lo && sa <= span_hi);
        if (in_span) banks_in_span.insert(bank);
        if (bank != m_attacker_bank) continue;
        if (static_cast<Addr_t>(row) == victim_row) {
          m_reserved_pages.insert(ppn);
        } else if (sa >= span_lo && sa <= span_hi) {
          m_reserved_pages.insert(ppn);
          if (m_attacker_pool_rows.insert(row).second) {
             m_attacker_ppn_pool.push_back(ppn);
          }
        }
      }
      if (m_attacker_ppn_pool.empty()) {
        std::string valid;
        for (int b : banks_in_span) valid += " " + std::to_string(b);
        m_logger->error(
          "Attacker pool EMPTY: attacker_bank={} has no rows in subarray span [{}..{}]. "
          "The MOP4CLXOR_RowSwap mapper correlates subarray with bank, so only some banks appear. "
          "Valid banks for this span:{}.",
          m_attacker_bank, span_lo, span_hi, valid);
        throw std::runtime_error(
          "RandomTranslation: attacker_bank=" + std::to_string(m_attacker_bank) +
          " has no rows in subarray span [" + std::to_string(span_lo) + ".." +
          std::to_string(span_hi) + "]; valid banks:" + valid);
      }
      m_logger->info("Attacker pool: {} row PPNs in bank {}, subarrays [{}..{}] (victim_row_offset={}, reserved pages={}).",
                     m_attacker_ppn_pool.size(), m_attacker_bank, span_lo, span_hi, m_attacker_victim_row_offset,
                     m_reserved_pages.size());
      if (m_debug) {
        std::cout << "[ATTACKER-POOL] bank=" << m_attacker_bank
                   << " subarrays=[" << span_lo << ".." << span_hi << "]"
                   << " subarray_rows=" << m_subarray_rows
                   << " victim_row=" << (victim_row == static_cast<Addr_t>(-1) ? -1 : (int)victim_row)
                   << " pool_size=" << m_attacker_ppn_pool.size() << std::endl;
      }
    }


    void build_benign_pool() {
      m_benign_ppn_pool.clear();
      for (Addr_t ppn = 0; ppn < (Addr_t)m_num_pages; ppn++) {
        if (m_reserved_pages.find(ppn) != m_reserved_pages.end()) continue;
        if (!in_row_limit(ppn)) continue;
        m_benign_ppn_pool.push_back(ppn);
      }
      if (m_debug) {
        std::cout << "[BENIGN-POOL] pool_size=" << m_benign_ppn_pool.size() << std::endl;
      }
    }

    bool translate(Request& req) override {
      if (!m_attacker_cores.empty() && !m_attacker_pool_built){
        build_attacker_pool();
        m_attacker_pool_built = true;
      }
      if (m_subarray_alloc && !m_benign_pool_built){
        build_benign_pool();
        m_benign_pool_built = true;
      }

      Addr_t vpn = req.addr >> m_offsetbits;

      auto& core_translation = m_translation[req.source_id];
      auto target = core_translation.find(vpn);
      if (target == core_translation.end()) {
        if (m_attacker_cores.count(req.source_id)){
          if (m_attacker_pool_next >= m_attacker_ppn_pool.size()){
            m_logger->error("Attacker pool exhausted for Addr {}, VPN {}.", req.addr, vpn);
            return false;
          }
          else {
            core_translation[vpn] = m_attacker_ppn_pool[m_attacker_pool_next++];
            if (m_debug) {
              AddrVec_t addr_vec = decode(core_translation[vpn]);
              int row = addr_vec[m_row_idx];
              std::cout << "[ATTACKER] core=" << req.source_id
                         << " vpn=" << vpn << " ppn=" << core_translation[vpn]
                         << " bank=" << flat_bank(addr_vec)
                         << " row=" << row << " subarray=" << (row / m_subarray_rows)
                         << std::endl;
            }
          }
        }
        // No previous translation record. Assign a new page
        else if (m_num_free_physical_pages == 0) {
          // We run out of physical pages. Randomly replace a previously assigned page (swap latency not modeled!)
          Addr_t ppn_to_replace;
          if (m_subarray_alloc) {
            ppn_to_replace = m_benign_ppn_pool[m_allocator_rng() % m_benign_ppn_pool.size()];
          } else {
            ppn_to_replace = m_allocator_rng() % m_num_pages;
            while (m_reserved_pages.find(ppn_to_replace) != m_reserved_pages.end()) {
              ppn_to_replace = m_allocator_rng() % m_num_pages;
            }
          }
          core_translation[vpn] = ppn_to_replace;
          m_logger->warn("Swapping out PPN {} for Addr {}, VPN {}.", ppn_to_replace, req.addr, vpn);
          if (m_debug && m_subarray_alloc) {
            AddrVec_t addr_vec = decode(ppn_to_replace);
            int row = addr_vec[m_row_idx];
            std::cout << "[BENIGN]   core=" << req.source_id
                       << " vpn=" << vpn << " ppn=" << ppn_to_replace
                       << " bank=" << flat_bank(addr_vec)
                       << " row=" << row << " subarray=" << (row / m_subarray_rows)
                       << std::endl;
          }
        } else {
          // We have available physical pages. Randomly assign one.
          Addr_t ppn_to_assign;
          if (m_subarray_alloc) {
            ppn_to_assign = m_benign_ppn_pool[m_allocator_rng() % m_benign_ppn_pool.size()];
            while (!m_free_physical_pages[ppn_to_assign]) {
              ppn_to_assign = m_benign_ppn_pool[m_allocator_rng() % m_benign_ppn_pool.size()];
            }
          } else {
            ppn_to_assign = m_allocator_rng() % m_num_pages;
            while (
              (m_reserved_pages.find(ppn_to_assign) != m_reserved_pages.end())
              || (!m_free_physical_pages[ppn_to_assign])
            ) {
              ppn_to_assign = m_allocator_rng() % m_num_pages;
            }
          }
          core_translation[vpn] = ppn_to_assign;
          m_free_physical_pages[ppn_to_assign] = false;
          m_num_free_physical_pages--;
          if (m_debug && m_subarray_alloc) {
            AddrVec_t addr_vec = decode(ppn_to_assign);
            int row = addr_vec[m_row_idx];
            std::cout << "[BENIGN]   core=" << req.source_id
                       << " vpn=" << vpn << " ppn=" << ppn_to_assign
                       << " bank=" << flat_bank(addr_vec)
                       << " row=" << row << " subarray=" << (row / m_subarray_rows)
                       << std::endl;
          }
        }
      } 

      // We either found an existing translation or have assigned a new page
      Addr_t p_addr = (core_translation[vpn] << m_offsetbits) | (req.addr & ((1 << m_offsetbits) - 1));

      DEBUG_LOG(DTRANSLATE, m_logger, "Translated Addr {}, VPN {} to Addr {}, PPN {}.", req.addr, vpn, p_addr, core_translation[vpn]);

      req.addr = p_addr;
      return true;
    };    

    bool reserve(const std::string& type, Addr_t addr) override {
      Addr_t ppn = addr >> m_offsetbits;
      // Add page to reserved pages if it is not already reserved
      m_reserved_pages.insert(ppn);
      // std::cout << "Reserved PPN " << ppn << "." << std::endl;
      return true;
    };

    Addr_t get_max_addr() override {
      return m_max_paddr;
    };
};

}   // namespace Ramulator