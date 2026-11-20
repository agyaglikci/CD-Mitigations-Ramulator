#include <vector>
#include <unordered_map>
#include <map>
#include <utility>

#include "base/base.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

class LinearMapperBase_with_rit : public IAddrMapper {
  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1;

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    int m_rank_level = -1;
    int m_bank_level = -1;
    int m_row_level = -1;
    int m_num_rit_entries = -1;

    struct RIT_entry {
      // src_row is the key of the unordered_map
      int dst_row;
      bool lock;
    };
    std::vector<std::unordered_map<int, RIT_entry>> m_row_indirection_table;

    struct AquaRitEntry {
      int dst_bank;
      int dst_row;
    };
    std::map<std::pair<int,int>, AquaRitEntry> m_aqua_rit;
    int m_num_aqua_rit_entries = -1;

  public:
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system);
    void init_rit(int num_banks, int num_rit_entries);
    int check_rit(int flat_bank_id, int src_row);
    bool is_rit_full(int flat_bank_id);
    bool is_rit_locked(int flat_bank_id, int src_row);
    void apply_indirection(Request& req);
    void rit_unlock();
    void rit_insert_entry(int flat_bank_id, int src_row, int dst_row);
    void rit_remove_entry(int flat_bank_id, int src_row, int dst_row);
    std::pair<int, int> get_unswap_pair(int flat_bank_id, const std::unordered_map<int, int>& exclusion_list);
    void dump_rit(int flat_bank_id);

    void init_aqua_rit(int num_entries);
    bool check_aqua_rit(int flat_bank_id, int src_row, int& dst_bank, int& dst_row);
    void aqua_rit_insert_entry(int src_bank, int src_row, int dst_bank, int dst_row);
    void aqua_rit_remove_entry(int src_bank, int src_row, int dst_bank, int dst_row);
    void set_flat_bank_id(std::vector<int>& addr_vec, int flat_bank_id);
};

}   // namespace Ramulator