#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <list>
#include <random>
#include <algorithm>
#include <cassert>

#include "base/base.h"
#include "translation/translation.h"
#include "frontend/frontend.h"
#include "memory_system/memory_system.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"


namespace Ramulator {

// Ported from ColumnKeeper (ramulator2/src/translation/impl/buddy_allocator.cpp).
class BuddyAllocator : public ITranslation, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(ITranslation, BuddyAllocator, "BuddyAllocator", "Buddy allocator for physical page allocation.");

  IFrontEnd* m_frontend;

  protected:
    std::mt19937_64 m_allocator_rng;

    Addr_t m_max_paddr;         // Max physical address
    Addr_t m_pagesize;          // Page size in bytes
    int    m_offsetbits;        // The number of bits for the page offset
    size_t m_num_pages;         // The total number of physical pages (rounded to power of 2)
    size_t m_num_pages_raw;     // The total number of physical pages before rounding
    int    m_frag_percentage;   // The percentage of physical pages to reserve for fragmentation

    // ----- Buddy system data structures -----
    int m_max_order;                              // log2(m_num_pages), the highest order
    std::vector<std::list<Addr_t>> m_free_lists;  // Free lists indexed by order [0..m_max_order]
    std::vector<int>  m_block_order;              // Order of the block each page belongs to (-1 if allocated)
    std::vector<bool> m_page_allocated;           // Whether a page is allocated

    size_t m_num_free_physical_pages;

    using Translation_t = std::vector<std::unordered_map<Addr_t, Addr_t>>;
    Translation_t m_translation;    // A vector of <vpn:ppn> maps, each core has its own map


    // ----- 2MB fragmentation (split into 1MB blocks that cannot merge back) -----
    int m_order_2mb = 0;    // Buddy order for a 2MB block
    int m_order_1mb = -1;   // Buddy order for a 1MB block (= m_order_2mb - 1)
    std::unordered_set<Addr_t> m_fragmented_2mb_starts; // 2MB-aligned PPN starts permanently split to 1MB



  public:
    void init() override {
      int seed = param<int>("seed").desc("The seed for the random number generator.").default_val(123);
      m_allocator_rng.seed(seed);

      m_max_paddr       = param<Addr_t>("max_addr").desc("Max physical address of the memory system.").required();
      m_pagesize        = param<Addr_t>("pagesize_KB").desc("Pagesize in KB.").default_val(4) << 10;
      m_frag_percentage = param<int>("frag_percentage").desc("Percentage of 2MB blocks to permanently split into 1MB blocks.").default_val(0);

      m_offsetbits  = calc_log2(m_pagesize);

      // Compute total pages and round down to nearest power of 2 (buddy requirement)
      m_num_pages_raw = m_max_paddr / m_pagesize;
      m_max_order = 0;
      {
        size_t n = m_num_pages_raw;
        while (n > 1) { n >>= 1; m_max_order++; }
      }
      m_num_pages = size_t(1) << m_max_order;
      m_num_free_physical_pages = m_num_pages;

      // Initialize buddy free lists
      m_free_lists.resize(m_max_order + 1);
      m_block_order.resize(m_num_pages, -1);   // -1 = allocated / not in a free block
      m_page_allocated.resize(m_num_pages, false);

      // Place a single block of order m_max_order at PPN 0
      m_free_lists[m_max_order].push_back(0);
      m_block_order[0] = m_max_order;

      m_frontend = cast_parent<IFrontEnd>();
      m_translation.resize(m_frontend->get_num_cores());

      m_logger = Logging::create_logger("BuddyAllocator");

      // Compute buddy orders for 2MB / 1MB fragmentation
      {
        Addr_t pages_per_2mb = (Addr_t(1) << 21) / m_pagesize;  // 2MB / pagesize
        m_order_2mb = calc_log2(pages_per_2mb);
        m_order_1mb = m_order_2mb - 1;
      }

      // ----- Fragmentation simulation: split X% of 2MB blocks into 1MB -----
      if (m_frag_percentage > 0 && m_order_2mb > 0 && m_order_2mb <= m_max_order) {
        size_t num_2mb_blocks = m_num_pages >> m_order_2mb;
        size_t num_frag = (num_2mb_blocks * m_frag_percentage) / 100;

        // Shuffle all 2MB-aligned block starts
        std::vector<Addr_t> all_2mb_starts(num_2mb_blocks);
        for (size_t i = 0; i < num_2mb_blocks; i++)
          all_2mb_starts[i] = static_cast<Addr_t>(i) << m_order_2mb;
        std::shuffle(all_2mb_starts.begin(), all_2mb_starts.end(), m_allocator_rng);

        size_t count = 0;
        for (size_t i = 0; i < all_2mb_starts.size() && count < num_frag; i++) {
          if (split_2mb_to_1mb(all_2mb_starts[i])) {
            m_fragmented_2mb_starts.insert(all_2mb_starts[i]);
            count++;
          }
        }

        m_logger->info("Fragmentation: split {} / {} 2MB blocks into 1MB blocks (requested {}%).",
                       count, num_2mb_blocks, m_frag_percentage);
      }

    };

    bool translate(Request& req) override {
      Addr_t vpn = req.addr >> m_offsetbits;

      auto& core_translation = m_translation[req.source_id];
      auto target = core_translation.find(vpn);
      if (target == core_translation.end()) {
        // No previous translation record. Assign a new page.
        Addr_t ppn;
        if (m_num_free_physical_pages == 0) {
          // Out of free pages — randomly evict a non-reserved page
          ppn = evict_and_allocate(req.source_id);
          if (ppn < 0) {
            m_logger->error("Failed to evict a page for Addr {}, VPN {}.", req.addr, vpn);
            return false;
          }
        } else {
          ppn = buddy_allocate();
          if (ppn < 0) {
            m_logger->error("Buddy allocate failed for Addr {}, VPN {}.", req.addr, vpn);
            return false;
          }
        }
        core_translation[vpn] = ppn;
      }

      // We either found an existing translation or have assigned a new page
      Addr_t p_addr = (core_translation[vpn] << m_offsetbits) | (req.addr & ((1 << m_offsetbits) - 1));

      req.addr = p_addr;
      return true;
    };

    Addr_t get_max_addr() override {
      return m_max_paddr;
    };

  // ============================
  //  Buddy system helpers
  // ============================
  private:

    /**
     * @brief Allocate a single page (order 0) using the buddy system.
     * @return The PPN of the allocated page, or -1 on failure.
     */
    Addr_t buddy_allocate() {
      // Find the smallest order that has a free block
      int order = 0;
      while (order <= m_max_order && m_free_lists[order].empty()) {
        order++;
      }
      if (order > m_max_order) return -1;  // no free memory

      // Remove the block from the free list
      Addr_t block = m_free_lists[order].front();
      m_free_lists[order].pop_front();
      m_block_order[block] = -1;

      // Split down to order 0
      while (order > 0) {
        order--;
        Addr_t buddy = block + (Addr_t(1) << order);
        // Put the upper buddy on the free list
        m_free_lists[order].push_back(buddy);
        m_block_order[buddy] = order;
      }

      // Mark the page as allocated
      m_page_allocated[block] = true;
      m_num_free_physical_pages--;
      return block;
    }

    /**
     * @brief Allocate a specific PPN through the buddy system.
     *        Splits larger blocks as needed to carve out exactly the requested page.
     * @return The requested PPN if successful, or -1 on failure.
     */
    Addr_t buddy_allocate_specific(Addr_t target_ppn) {
      if (target_ppn < 0 || target_ppn >= (Addr_t)m_num_pages) return -1;
      if (m_page_allocated[target_ppn]) return -1;

      // Find which free block contains target_ppn
      // Walk up orders to find the block that covers this PPN
      for (int order = 0; order <= m_max_order; order++) {
        // The block start for this PPN at this order
        Addr_t block_start = (target_ppn >> order) << order;

        // Check if this block_start is on the free list at this order
        auto& fl = m_free_lists[order];
        auto it = std::find(fl.begin(), fl.end(), block_start);
        if (it == fl.end()) continue;

        // Found the encompassing block — remove it
        fl.erase(it);
        m_block_order[block_start] = -1;

        // Split down until we isolate target_ppn at order 0
        Addr_t current = block_start;
        int cur_order = order;
        while (cur_order > 0) {
          cur_order--;
          Addr_t mid = current + (Addr_t(1) << cur_order);
          if (target_ppn >= mid) {
            // target is in the upper half — free the lower half
            m_free_lists[cur_order].push_back(current);
            m_block_order[current] = cur_order;
            current = mid;
          } else {
            // target is in the lower half — free the upper half
            m_free_lists[cur_order].push_back(mid);
            m_block_order[mid] = cur_order;
          }
        }

        // current == target_ppn at this point
        m_page_allocated[current] = true;
        m_num_free_physical_pages--;
        return current;
      }
      return -1;  // should not happen if bookkeeping is correct
    }

    /**
     * @brief Split a 2MB-aligned block into two free 1MB blocks.
     *        Finds and splits larger free blocks as needed to isolate the 2MB region.
     * @param block_2mb_start The 2MB-aligned starting PPN.
     * @return true if successful.
     */
    bool split_2mb_to_1mb(Addr_t block_2mb_start) {
      for (int order = m_order_2mb; order <= m_max_order; order++) {
        Addr_t block_start = (block_2mb_start >> order) << order;
        auto& fl = m_free_lists[order];
        auto it = std::find(fl.begin(), fl.end(), block_start);
        if (it == fl.end()) continue;

        // Remove the encompassing block from the free list
        fl.erase(it);
        m_block_order[block_start] = -1;

        // Split down to order m_order_2mb
        Addr_t current = block_start;
        int cur_order = order;
        while (cur_order > m_order_2mb) {
          cur_order--;
          Addr_t mid = current + (Addr_t(1) << cur_order);
          if (block_2mb_start >= mid) {
            // target in upper half — free the lower half
            m_free_lists[cur_order].push_back(current);
            m_block_order[current] = cur_order;
            current = mid;
          } else {
            // target in lower half — free the upper half
            m_free_lists[cur_order].push_back(mid);
            m_block_order[mid] = cur_order;
          }
        }

        // Now current == block_2mb_start at order m_order_2mb
        // Split into two 1MB blocks
        Addr_t lower = current;
        Addr_t upper = current + (Addr_t(1) << m_order_1mb);
        m_free_lists[m_order_1mb].push_back(lower);
        m_block_order[lower] = m_order_1mb;
        m_free_lists[m_order_1mb].push_back(upper);
        m_block_order[upper] = m_order_1mb;

        return true;
      }
      return false;
    }

    /**
     * @brief Free a single page and coalesce with its buddy if possible.
     */
    void buddy_free(Addr_t ppn) {
      if (!m_page_allocated[ppn]) return;

      m_page_allocated[ppn] = false;
      m_num_free_physical_pages++;

      int order = 0;
      Addr_t block = ppn;

      while (order < m_max_order) {
        // Prevent coalescing 1MB buddies back into a fragmented 2MB block
        if (order == m_order_1mb) {
          Addr_t block_2mb_start = (block >> m_order_2mb) << m_order_2mb;
          if (m_fragmented_2mb_starts.count(block_2mb_start)) break;
        }

        Addr_t buddy = block ^ (Addr_t(1) << order);

        // The buddy must be free and must be a whole free block at this order
        if (buddy >= (Addr_t)m_num_pages)       break;
        if (m_page_allocated[buddy])             break;
        if (m_block_order[buddy] != order)       break;

        // Remove the buddy from the free list
        auto& fl = m_free_lists[order];
        auto it = std::find(fl.begin(), fl.end(), buddy);
        if (it == fl.end()) break;  // safety check
        fl.erase(it);
        m_block_order[buddy] = -1;

        // Merge: the merged block starts at the lower address
        block = std::min(block, buddy);
        order++;
      }

      // Insert the (possibly merged) block into the free list
      m_free_lists[order].push_back(block);
      m_block_order[block] = order;
    }

    /**
     * @brief  Evict a random non-reserved mapped page, free it, then buddy-allocate a new page.
     * @return The PPN of the newly allocated page, or -1 on failure.
     */
    Addr_t evict_and_allocate(int requesting_core_id) {
      // Collect all evictable <core_id, vpn, ppn> triples
      struct MappingEntry { int core_id; Addr_t vpn; Addr_t ppn; };
      std::vector<MappingEntry> candidates;

      for (int c = 0; c < (int)m_translation.size(); c++) {
        for (auto& [vpn, ppn] : m_translation[c]) {
          candidates.push_back({c, vpn, ppn});
        }
      }

      if (candidates.empty()) return -1;

      // Pick a random victim
      size_t idx = m_allocator_rng() % candidates.size();
      auto& victim = candidates[idx];

      // Remove the mapping
      m_translation[victim.core_id].erase(victim.vpn);

      // Free the page through the buddy system
      buddy_free(victim.ppn);

      // Now allocate a fresh page
      return buddy_allocate();
    }
};

}   // namespace Ramulator
