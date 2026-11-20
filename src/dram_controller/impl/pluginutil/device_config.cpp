#include "dram_controller/impl/pluginutil/device_config.h"

namespace Ramulator {

DeviceConfig::DeviceConfig() { }

DeviceConfig::DeviceConfig(IDRAMController* ctrl) {
    set_device(ctrl);
}

DeviceConfig::~DeviceConfig() { /* Currently no-op */ }

void DeviceConfig::set_device(IDRAMController* ctrl) {
    m_ctrl = ctrl;
    m_dram = ctrl->m_dram;
    m_rank_level = m_dram->m_levels("rank");
    m_bank_level = m_dram->m_levels("bank");
    m_bankgroup_level = m_dram->m_levels("bankgroup");
    m_row_level = m_dram->m_levels("row");
    m_col_level = m_dram->m_levels("column");

    m_num_ranks = m_dram->get_level_size("rank");
    m_num_bankgroups = m_dram->get_level_size("bankgroup");
    m_num_banks_per_bankgroup = m_dram->get_level_size("bankgroup") < 0 ? 0 : m_dram->get_level_size("bank");
    m_num_banks_per_rank = m_dram->get_level_size("bankgroup") < 0 ? 
                            m_dram->get_level_size("bank") : 
                            m_dram->get_level_size("bankgroup") * m_dram->get_level_size("bank");
    m_num_rows_per_bank = m_dram->get_level_size("row");
    m_num_cls = m_dram->get_level_size("column") / 8;

    m_num_banks = m_num_ranks * m_num_banks_per_rank;
}

void DeviceConfig::set_device(IDRAMController* ctrl, int subarray_size){
    set_device(ctrl);
    is_salp = m_dram->m_levels.contains("subarray");
    if (is_salp){
        m_subarray_level = m_dram->m_levels("subarray");
        m_num_subarrays_per_bank = m_dram->get_level_size("subarray");
        m_num_rows_per_subarray = m_dram->get_level_size("row");
    }
    else {
        m_num_subarrays_per_bank = m_num_rows_per_bank / subarray_size;
        m_num_rows_per_subarray = subarray_size;
    }
    // std::printf("[DEVCFG] is_salp=%d rows_per_bank=%d subarray_size=%d -> num_sub=%d rows_per_sub=%d\n",
    //         (int)is_salp, m_num_rows_per_bank, subarray_size,
    //         m_num_subarrays_per_bank, m_num_rows_per_subarray);

}

int DeviceConfig::get_subarray_id(const Request& req){
    return is_salp ? req.addr_vec[m_subarray_level] : req.addr_vec[m_row_level] / m_num_rows_per_subarray;
}

int DeviceConfig::get_flat_bank_id(const Request& req) {
    auto rank_id = req.addr_vec[m_rank_level];
    auto flat_bank_id = req.addr_vec[m_bank_level];
    auto accumulated_dimension = 1;
    for (int i = m_bank_level - 1; i >= m_rank_level; i--) {
        accumulated_dimension *= m_dram->m_organization.count[i + 1];
        flat_bank_id += req.addr_vec[i] * accumulated_dimension;
    }
    return flat_bank_id;
}

}

