#include "rit.h"
#include <bitset>

namespace Ramulator {

void LinearMapperBase_with_rit::setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
  m_dram = memory_system->get_ifce<IDRAM>();

  // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
  const auto& count = m_dram->m_organization.count;
  m_num_levels = count.size();
  m_addr_bits.resize(m_num_levels);
  for (size_t level = 0; level < m_addr_bits.size(); level++) {
    m_addr_bits[level] = calc_log2(count[level]);
  }

  // Last (Column) address have the granularity of the prefetch size
  m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

  int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
  m_tx_offset = calc_log2(tx_bytes);

  // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
  try {
    m_row_bits_idx = m_dram->m_levels("row");
  } catch (const std::out_of_range& r) {
    throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
  }

  // Assume column is always the last level
  m_col_bits_idx = m_num_levels - 1;
}

// initialize RIT
void LinearMapperBase_with_rit::init_rit(int num_banks, int num_rit_entries){
  m_num_rit_entries = num_rit_entries;
  m_rank_level = m_dram->m_levels("rank");
  m_bank_level = m_dram->m_levels("bank");
  m_row_level = m_dram->m_levels("row");

  // setup RIT
  for (int i = 0; i < num_banks; i++) {
    std::unordered_map<int, RIT_entry> rit_bank;
    m_row_indirection_table.push_back(rit_bank);
  }
}

// check if the entry is in the RIT
int LinearMapperBase_with_rit::check_rit(int flat_bank_id, int src_row){
  if (m_row_indirection_table[flat_bank_id].find(src_row) != m_row_indirection_table[flat_bank_id].end()) {
    return m_row_indirection_table[flat_bank_id][src_row].dst_row;
  }
  return -1;
}

// check if the RIT is full
bool LinearMapperBase_with_rit::is_rit_full(int flat_bank_id){
  return m_row_indirection_table[flat_bank_id].size() >= m_num_rit_entries - 8;
}

// check if the entry is locked
bool LinearMapperBase_with_rit::is_rit_locked(int flat_bank_id, int src_row){
  return m_row_indirection_table[flat_bank_id][src_row].lock;
}

// performs the indirection if the row is in the RIT
void LinearMapperBase_with_rit::apply_indirection(Request& req){
  if (m_num_rit_entries == -1 && m_num_aqua_rit_entries == -1){
    // Neither RIT is initiliazed, indirection won't be performed.
    return;
  }

  int flat_bank_id = req.addr_vec[m_bank_level];
  int accumulated_dimension = 1;
  for (int i = m_bank_level - 1; i >= m_rank_level; i--) {
    accumulated_dimension *= m_dram->m_organization.count[i + 1];
    flat_bank_id += req.addr_vec[i] * accumulated_dimension;
  }
  int src_row = req.addr_vec[m_row_level];

  if (m_num_rit_entries != -1) {
    int dst_row = check_rit(flat_bank_id, src_row);
    if (dst_row != -1) {
      req.addr_vec[m_row_level] = dst_row;
      return;
    }
  }

  if (m_num_aqua_rit_entries != -1) {
    int dst_bank = -1, dst_row = -1;
    if (check_aqua_rit(flat_bank_id, src_row, dst_bank, dst_row)) {
      req.addr_vec[m_row_level] = dst_row;
      if (dst_bank != flat_bank_id) {
        set_flat_bank_id(req.addr_vec, dst_bank);
      }
    }
  }
}

// unlocks all the entries in the RIT at the end of each Epoch
void LinearMapperBase_with_rit::rit_unlock() {
  for (auto& bank : m_row_indirection_table) {
    for (auto& entry : bank) {
      entry.second.lock = false;
    }
  }
}

// inserts the entry and its pair into the RIT
void LinearMapperBase_with_rit::rit_insert_entry(int flat_bank_id, int src_row, int dst_row) {
  // insert the entry into the RIT
  RIT_entry entry0;
  entry0.dst_row = dst_row;
  entry0.lock = true;
  m_row_indirection_table[flat_bank_id][src_row] = entry0;
  // insert the pair of entry into the RIT
  RIT_entry entry1;
  entry1.dst_row = src_row;
  entry1.lock = true;
  m_row_indirection_table[flat_bank_id][dst_row] = entry1;

  if(m_row_indirection_table[flat_bank_id].size() > m_num_rit_entries){
    std::cerr << "RIT is full!!!!!!!!!! Check before insertion." << std::endl;
    exit(1);
  }
}

// removes the entry and its pair from the RIT
void LinearMapperBase_with_rit::rit_remove_entry(int flat_bank_id, int src_row, int dst_row) {
  // remove the entry from the RIT
  m_row_indirection_table[flat_bank_id].erase(src_row);
  // remove the pair of entry from the RIT
  m_row_indirection_table[flat_bank_id].erase(dst_row);
}

// gets a pair of entries from the RIT to unswap, the pair cannot be in the exclusion_list
std::pair<int, int> LinearMapperBase_with_rit::get_unswap_pair(int flat_bank_id, const std::unordered_map<int, int>& exclusion_list){
  std::pair<int, int> unswap_pair;
  for (auto& entry : m_row_indirection_table[flat_bank_id]) {
    if (!entry.second.lock && exclusion_list.find(entry.first) == exclusion_list.end() && exclusion_list.find(entry.second.dst_row) == exclusion_list.end()) {
      unswap_pair.first = entry.first;
      unswap_pair.second = entry.second.dst_row;
      return unswap_pair;
    }
  }
  std::cerr << "No unlocked entry found in the RIT! Should not happen!" << std::endl;
  exit(1);
}

void LinearMapperBase_with_rit::init_aqua_rit(int num_entries) {
  m_num_aqua_rit_entries = num_entries;
  m_rank_level = m_dram->m_levels("rank");
  m_bank_level = m_dram->m_levels("bank");
  m_row_level = m_dram->m_levels("row");
}

bool LinearMapperBase_with_rit::check_aqua_rit(int flat_bank_id, int src_row, int& dst_bank, int& dst_row) {
  auto it = m_aqua_rit.find({flat_bank_id, src_row});
  if (it == m_aqua_rit.end()) return false;
  dst_bank = it->second.dst_bank;
  dst_row = it->second.dst_row;
  return true;
}

void LinearMapperBase_with_rit::aqua_rit_insert_entry(int src_bank, int src_row, int dst_bank, int dst_row) {
  m_aqua_rit[{src_bank, src_row}] = {dst_bank, dst_row};
  m_aqua_rit[{dst_bank, dst_row}] = {src_bank, src_row};

  if ((int)m_aqua_rit.size() > m_num_aqua_rit_entries) {
    std::cerr << "AQUA rank-global RIT is full!!!!!!!!!! Check before insertion." << std::endl;
    exit(1);
  }
}

void LinearMapperBase_with_rit::aqua_rit_remove_entry(int src_bank, int src_row, int dst_bank, int dst_row) {
  m_aqua_rit.erase({src_bank, src_row});
  m_aqua_rit.erase({dst_bank, dst_row});
}

void LinearMapperBase_with_rit::set_flat_bank_id(std::vector<int>& addr_vec, int flat_bank_id) {
  addr_vec[m_bank_level] = flat_bank_id % m_dram->m_organization.count[m_bank_level];
  flat_bank_id /= m_dram->m_organization.count[m_bank_level];
  for (int i = m_bank_level - 1; i >= m_rank_level; i--) {
    addr_vec[i] = flat_bank_id % m_dram->m_organization.count[i];
    flat_bank_id /= m_dram->m_organization.count[i];
  }
}

// dumps RIT for debug
void LinearMapperBase_with_rit::dump_rit(int flat_bank_id) {
  std::cout << "======================" << std::endl
            << "RIT[" << flat_bank_id << "].size(): " << m_row_indirection_table[flat_bank_id].size() << std::endl;

  for (auto entry: m_row_indirection_table[flat_bank_id]){
    std::cout << entry.first << " -> " << entry.second.dst_row << "\t" << (entry.second.lock ? "locked": "unlocked") << std::endl;
  }
  std::cout << "======================" << std::endl;
}

class ChRaBaRoCo_with_rit final : public LinearMapperBase_with_rit, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, ChRaBaRoCo_with_rit, "ChRaBaRoCo_with_rit", "Applies a trival mapping to the address.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase_with_rit::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      for (int i = m_addr_bits.size() - 1; i >= 0; i--) {
        req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
      }
      // perform indirection
      LinearMapperBase_with_rit::apply_indirection(req);
    }
};

class RoBaRaCoCh_with_rit final : public LinearMapperBase_with_rit, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoBaRaCoCh_with_rit, "RoBaRaCoCh_with_rit", "Applies a RoBaRaCoCh mapping to the address.");
  private:
    bool m_debug = false;

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase_with_rit::setup(frontend, memory_system);
      m_debug = param<bool>("debug").default_val(false);
      if (!m_debug) {
        return;
      }
      std::printf("RoBaRaCoCh bit widths:\n");
      std::printf("Tx: %ld\n", m_tx_offset);
      std::printf("Ch: %d\n", m_addr_bits[m_dram->m_levels("channel")]);
      std::printf("Co: %d\n", m_addr_bits[m_dram->m_levels("column")]);
      std::printf("Ra: %d\n", m_addr_bits[m_dram->m_levels("rank")]);
      std::printf("Ba: %d\n", m_addr_bits[m_dram->m_levels("bank")]);
      std::printf("Bg: %d\n", m_addr_bits[m_dram->m_levels("bankgroup")]);
      std::printf("Ro: %d\n", m_addr_bits[m_dram->m_levels("row")]);
      for (int i = 0; i <= m_row_bits_idx; i++) {
        std::printf("%d ", m_addr_bits[i]);
      }
      std::printf("\n");
      int row_offset = m_tx_offset + m_addr_bits[m_addr_bits.size() - 1];
      for (int i = 0; i < m_row_bits_idx; i++) {
        row_offset += m_addr_bits[i];
      }
      std::printf("RowOffset: %d\n", row_offset);
      int flat_bank_offset = m_tx_offset + m_addr_bits[m_addr_bits.size() - 1];
      for (int i = 0; i <= m_dram->m_levels("rank"); i++) {
        flat_bank_offset += m_addr_bits[i];
      }
      std::printf("FlatBankOffset: %d\n", flat_bank_offset);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
      req.addr_vec[m_addr_bits.size() - 1] = slice_lower_bits(addr, m_addr_bits[m_addr_bits.size() - 1]);
      for (int i = 1; i <= m_row_bits_idx; i++) {
        req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
      }
      // perform indirection
      LinearMapperBase_with_rit::apply_indirection(req);
    }
};


class MOP4CLXOR_with_rit final : public LinearMapperBase_with_rit, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, MOP4CLXOR_with_rit, "MOP4CLXOR_with_rit", "Applies a MOP4CLXOR mapping to the address.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase_with_rit::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[m_col_bits_idx] = slice_lower_bits(addr, 2);
      for (int lvl = 0 ; lvl < m_row_bits_idx ; lvl++)
          req.addr_vec[lvl] = slice_lower_bits(addr, m_addr_bits[lvl]);
      req.addr_vec[m_col_bits_idx] += slice_lower_bits(addr, m_addr_bits[m_col_bits_idx]-2) << 2;
      req.addr_vec[m_row_bits_idx] = (int) addr;

      int row_xor_index = 0; 
      for (int lvl = 0 ; lvl < m_col_bits_idx ; lvl++){
        if (m_addr_bits[lvl] > 0){
          int mask = (req.addr_vec[m_col_bits_idx] >> row_xor_index) & ((1<<m_addr_bits[lvl])-1);
          req.addr_vec[lvl] = req.addr_vec[lvl] xor mask;
          row_xor_index += m_addr_bits[lvl];
        }
      }
      // perform indirection
      LinearMapperBase_with_rit::apply_indirection(req);
    }
};


class MOP4CLXOR_RowSwap_with_rit final : public LinearMapperBase_with_rit, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, MOP4CLXOR_RowSwap_with_rit, "MOP4CLXOR_RowSwap_with_rit",
      "MOP4CLXOR_RowSwap (MOP4CLXOR + row half-swap) with a Row Indirection Table for AQUA.");

  int m_row_bits  = -1;
  int m_low_bits  = -1;
  int m_high_bits = -1;
  int m_low_mask  = 0;

  public:
    void init() override { }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase_with_rit::setup(frontend, memory_system);
      m_row_bits  = m_addr_bits[m_row_bits_idx];
      m_low_bits  = m_row_bits / 2;
      m_high_bits = m_row_bits - m_low_bits;
      m_low_mask  = (1 << m_low_bits) - 1;
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[m_col_bits_idx] = slice_lower_bits(addr, 2);
      for (int lvl = 0 ; lvl < m_row_bits_idx ; lvl++)
          req.addr_vec[lvl] = slice_lower_bits(addr, m_addr_bits[lvl]);
      req.addr_vec[m_col_bits_idx] += slice_lower_bits(addr, m_addr_bits[m_col_bits_idx]-2) << 2;
      req.addr_vec[m_row_bits_idx] = (int) addr;

      int row_xor_index = 0;
      for (int lvl = 0 ; lvl < m_col_bits_idx ; lvl++){
        if (m_addr_bits[lvl] > 0){
          int mask = (req.addr_vec[m_col_bits_idx] >> row_xor_index) & ((1<<m_addr_bits[lvl])-1);
          req.addr_vec[lvl] = req.addr_vec[lvl] xor mask;
          row_xor_index += m_addr_bits[lvl];
        }
      }

      int r = req.addr_vec[m_row_bits_idx];
      int low_part  = r & m_low_mask;
      int high_part = r >> m_low_bits;
      req.addr_vec[m_row_bits_idx] = (low_part << m_high_bits) | high_part;

      LinearMapperBase_with_rit::apply_indirection(req);
    }
};

}   // namespace Ramulator