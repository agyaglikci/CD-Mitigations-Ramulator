#include <vector>
#include <unordered_map>
#include <limits>
#include <random>
#include <cstdio>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/plugin/throttleable.h"
#include "dram_controller/impl/pluginutil/device_config.h"

namespace Ramulator {

class PARA : public IControllerPlugin, public Implementation, public IThrottleable {
  RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, PARA, "PARA", "PARA.")

  private:
    IDRAM* m_dram = nullptr;
    DeviceConfig m_cfg;

    float m_pr_threshold;

    int   m_seed;
    std::mt19937 m_generator;
    std::uniform_real_distribution<float> m_distribution;
    bool m_is_debug = false;

    int m_VRR_req_id = -1;
    int m_bank_level = -1;
    int m_row_level = -1;

    long s_preventive_refreshes = 0;

    bool m_count_write_through = false;
    std::vector<bool> m_wt_armed;
    std::vector<Clk_t> m_wt_wl_off;
    Clk_t m_clk = 0;
    int m_nRAS = -1;
    long s_write_through_draws = 0;
    long s_write_through_skipped = 0;

  public:
    void init() override { 
      m_pr_threshold = param<float>("threshold").desc("Probability threshold for issuing neighbor row refresh").required();
      if (m_pr_threshold <= 0.0f || m_pr_threshold >= 1.0f)
        throw ConfigurationError("Invalid probability threshold ({}) for PARA!", m_pr_threshold);

      m_seed = param<int>("seed").desc("Seed for the RNG").default_val(123);
      m_generator = std::mt19937(m_seed);
      m_distribution = std::uniform_real_distribution<float>(0.0, 1.0);

      m_is_debug = param<bool>("debug").default_val(false);
      m_count_write_through = param<bool>("count_write_through").default_val(false);
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IDRAMController>();
      m_dram = m_ctrl->m_dram;
  
      m_cfg.set_device(m_ctrl);
      throttleable_setup(m_cfg.m_num_banks, 2);

      if (!m_dram->m_commands.contains("VRR")) {
        throw ConfigurationError("PARA is not compatible with the DRAM implementation that does not have Victim-Row-Refresh (VRR) command!");
      }

      m_VRR_req_id = m_dram->m_requests("victim-row-refresh");
      m_bank_level = m_dram->m_levels("bank");
      m_row_level = m_dram->m_levels("row");

      register_stat(s_preventive_refreshes).name("para_preventive_refreshes");

      m_wt_armed.assign(m_cfg.m_num_banks, false);
      m_wt_wl_off.assign(m_cfg.m_num_banks, 0);
      m_nRAS = m_dram->m_timing_vals("nRAS");
      register_stat(s_write_through_draws).name("para_write_through_draws");
      register_stat(s_write_through_skipped).name("para_write_through_skipped");
    };

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
      m_clk++;
      if (request_found) {
        if (
          m_dram->m_command_meta(req_it->command).is_opening && 
          m_dram->m_command_scopes(req_it->command) == m_row_level
        ) {
          if (m_distribution(m_generator) < m_pr_threshold) {
            // increment operation counter
            increment_operation(m_cfg.get_flat_bank_id(*req_it), req_it->source_id);
            Request vrr_req(req_it->addr_vec, m_VRR_req_id);
            if (m_ctrl->priority_send(vrr_req)) {
              s_preventive_refreshes++;
              if (m_is_debug) {
                std::printf("[PARA] flat_bank=%d preventive refresh #%ld (row=%d)\n",
                            m_cfg.get_flat_bank_id(*req_it), s_preventive_refreshes,
                            req_it->addr_vec[m_row_level]);
              }
            }
          }
          if (m_count_write_through) {
            int flat_bank = m_cfg.get_flat_bank_id(*req_it);
            m_wt_armed[flat_bank] =
                req_it->type_id == Request::Type::Read || req_it->type_id == Request::Type::Write;
            m_wt_wl_off[flat_bank] = m_clk + m_nRAS;
          }
        }
        else if (m_count_write_through &&
                 m_dram->m_command_meta(req_it->command).is_accessing &&
                 req_it->type_id == Request::Type::Write) {
          int flat_bank = m_cfg.get_flat_bank_id(*req_it);
          if (m_wt_armed[flat_bank] && m_clk < m_wt_wl_off[flat_bank]) {
            s_write_through_skipped++;
          }
          else if (m_wt_armed[flat_bank]) {
            m_wt_armed[flat_bank] = false;
            s_write_through_draws++;
            bool hit = m_distribution(m_generator) < m_pr_threshold;
            if (m_is_debug) {
              std::printf("[PARA] WT draw flat_bank=%d row=%d hit=%d\n",
                          flat_bank, req_it->addr_vec[m_row_level], (int)hit);
            }
            if (hit) {
              increment_operation(flat_bank, req_it->source_id);
              Request vrr_req(req_it->addr_vec, m_VRR_req_id);
              if (m_ctrl->priority_send(vrr_req)) {
                s_preventive_refreshes++;
                if (m_is_debug) {
                  std::printf("[PARA] flat_bank=%d preventive refresh #%ld (row=%d)\n",
                              flat_bank, s_preventive_refreshes, req_it->addr_vec[m_row_level]);
                }
              }
            }
          }
        }
      }
    };

};

}       // namespace Ramulator
