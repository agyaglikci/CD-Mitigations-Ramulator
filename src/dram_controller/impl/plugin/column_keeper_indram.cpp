#include <vector>
#include <algorithm>
#include <cstdio>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/pluginutil/device_config.h"
#include "dram_controller/impl/plugin/prac.h"

#include "frontend/frontend.h"

namespace Ramulator {

class ColumnKeeperInDRAM : public IControllerPlugin, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, ColumnKeeperInDRAM, "ColumnKeeperInDRAM", "In-DRAM ColumnKeeper-D (dual counters, ALERT + RFMab).")

    private:
        IDRAM* m_dram = nullptr;
        DeviceConfig m_cfg;
        IPRAC* m_prac = nullptr;
        IFrontEnd* m_frontend = nullptr;

        int m_row_level = -1;
        int m_RFMab_id = -1;
        int m_subarray_size = -1;
        int m_num_subarrays = -1;
        int m_alert_thresh = -1;
        int m_bundle = 7;
        bool m_is_debug = false;
        bool m_dual = true;

        enum class Shadow { IDLE, WAIT, DELAY };
        Shadow m_shadow = Shadow::IDLE;
        Clk_t m_clk = 0;
        Clk_t m_shadow_start = 0;
        int m_shadow_delay_rem = 0;
        int m_abo_act_ns = 180;
        int m_abo_cycles = -1;
        int m_recovery_refs = -1;
        int m_delay_acts = 4;

        struct Tracker {
            bool valid = false;
            int sa = -1;
            uint32_t value = 0;
        };
        std::vector<std::vector<uint32_t>> m_odd;
        std::vector<std::vector<uint32_t>> m_even;
        std::vector<Tracker> m_tracker;

        long s_alerts = 0;
        long s_rfm_mitigations = 0;
        long s_row_refreshes = 0;

    public:
        void init() override {
            m_alert_thresh = param<int>("alert_threshold").required();
            m_subarray_size = param<int>("subarray_size").required();
            m_bundle = param<int>("bundle").default_val(7);
            m_is_debug = param<bool>("debug").default_val(false);
            m_abo_act_ns = param<int>("abo_act_ns").default_val(180);
            m_delay_acts = param<int>("abo_delay_acts").default_val(4);
            m_dual = param<bool>("dual_counter").default_val(true);
        }

        void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
            m_ctrl = cast_parent<IDRAMController>();
            m_dram = m_ctrl->m_dram;
            m_cfg.set_device(m_ctrl);
            m_frontend = frontend;

            m_row_level = m_dram->m_levels("row");
            m_RFMab_id = m_dram->m_commands("RFMab");
            m_num_subarrays = m_dram->get_level_size("row") / m_subarray_size;

            for (auto* plugin : m_ctrl->m_plugins) {
                if (auto* p = dynamic_cast<IPRAC*>(plugin)) {
                    m_prac = p;
                    break;
                }
            }
            if (!m_prac)
                throw ConfigurationError("[ColumnKeeperInDRAM] requires a PRAC plugin for ALERT/RFM.");
            m_recovery_refs = m_prac->get_num_abo_recovery_refs();
            m_abo_cycles = m_abo_act_ns / ((float)m_dram->m_timing_vals("tCK_ps") / 1000.0f);
            m_prac->set_pending_work_fn([this]() -> bool {
                for (int b = 0; b < m_cfg.m_num_banks; b++)
                    if (is_critical(b)) return true;
                return false;
            });

            m_odd.assign(m_cfg.m_num_banks, std::vector<uint32_t>(m_num_subarrays, 0));
            m_even.assign(m_cfg.m_num_banks, std::vector<uint32_t>(m_num_subarrays, 0));
            m_tracker.assign(m_cfg.m_num_banks, Tracker());

            register_stat(s_alerts).name("ckd_indram_alerts");
            register_stat(s_rfm_mitigations).name("ckd_indram_rfm_mitigations");
            register_stat(s_row_refreshes).name("ckd_indram_row_refreshes");
        }

        void update(bool request_found, ReqBuffer::iterator& req_it) override {
            m_clk++;
            bool warmup = m_frontend && !m_frontend->is_warmup_done();
            bool delay_ended = false;
            if (!warmup) m_shadow = Shadow::IDLE;
            else if (m_shadow == Shadow::WAIT && m_clk >= m_shadow_start) shadow_recovery();

            if (request_found) {
                auto& req = *req_it;
                if (m_dram->m_command_meta(req.command).is_opening &&
                    m_dram->m_command_scopes(req.command) == m_row_level) {
                    int b = m_cfg.get_flat_bank_id(req);
                    int sa = req.addr_vec[m_row_level] / m_subarray_size;
                    on_act(b, sa, warmup);
                    if (warmup && m_shadow == Shadow::DELAY && --m_shadow_delay_rem == 0) {
                        m_shadow = Shadow::IDLE;
                        delay_ended = true;
                    }
                }
                else if (req.command == m_RFMab_id) {
                    int bpr = m_cfg.m_num_banks_per_rank;
                    int rank = req.addr_vec[m_cfg.m_rank_level];
                    for (int i = 0; i < bpr; i++) mitigate(rank * bpr + i);
                }
            }

            if (warmup && !delay_ended && m_shadow == Shadow::IDLE && any_critical()) {
                m_shadow = Shadow::WAIT;
                m_shadow_start = m_clk + m_abo_cycles;
            }
        }

    private:
        bool any_critical() const {
            for (int b = 0; b < m_cfg.m_num_banks; b++)
                if (is_critical(b)) return true;
            return false;
        }

        void shadow_recovery() {
            if (m_is_debug) std::printf("[CKD-IDR] SHADOW-RECOVERY clk=%lu\n", (unsigned long)m_clk);
            int bpr = m_cfg.m_num_banks_per_rank;
            for (int i = 0; i < m_recovery_refs; i++)
                for (int r = 0; r < m_cfg.m_num_ranks; r++)
                    for (int k = 0; k < bpr; k++) mitigate(r * bpr + k);
            m_shadow = Shadow::DELAY;
            m_shadow_delay_rem = m_delay_acts;
        }

        bool is_critical(int b) const {
            return m_tracker[b].valid && m_tracker[b].value >= (uint32_t)m_alert_thresh;
        }

        void alert(int b) {
            if (!m_prac->is_abo_active()) s_alerts++;
            m_prac->set_abo_needed(true, IPRAC::ABOTrigger::SALTC, b);
        }

        void on_act(int b, int sa, bool warmup) {
            m_odd[b][sa]++;
            m_even[b][sa]++;
            if (sa > 0) {
                m_odd[b][sa - 1]++;
                if (!m_dual) m_even[b][sa - 1]++;
            }
            if (sa < m_num_subarrays - 1) {
                m_even[b][sa + 1]++;
                if (!m_dual) m_odd[b][sa + 1]++;
            }
            if (m_is_debug)
                std::printf("[CKD-IDR] ACT bank=%d sa=%d odd=%u even=%u\n", b, sa, m_odd[b][sa], m_even[b][sa]);

            check(b, sa);
            if (sa > 0) check(b, sa - 1);
            if (sa < m_num_subarrays - 1) check(b, sa + 1);
            if (!warmup && is_critical(b)) alert(b);
        }

        void check(int b, int sa) {
            uint32_t v = std::max(m_odd[b][sa], m_even[b][sa]);
            auto& t = m_tracker[b];
            if (!t.valid || v >= t.value) {
                t.valid = true;
                t.sa = sa;
                t.value = v;
            }
        }

        void mitigate(int b) {
            auto& t = m_tracker[b];
            if (!t.valid) return;
            int sa = t.sa;
            uint32_t thr = (uint32_t)m_alert_thresh;
            uint32_t odd0 = m_odd[b][sa], even0 = m_even[b][sa];

            m_odd[b][sa] = m_odd[b][sa] > thr ? m_odd[b][sa] - thr : 0;
            m_even[b][sa] = m_even[b][sa] > thr ? m_even[b][sa] - thr : 0;
            m_odd[b][sa] += m_bundle;
            m_even[b][sa] += m_bundle;
            if (sa > 0) {
                m_odd[b][sa - 1] += m_bundle;
                if (!m_dual) m_even[b][sa - 1] += m_bundle;
            }
            if (sa < m_num_subarrays - 1) {
                m_even[b][sa + 1] += m_bundle;
                if (!m_dual) m_odd[b][sa + 1] += m_bundle;
            }

            s_rfm_mitigations++;
            s_row_refreshes += m_bundle;
            if (m_is_debug)
                std::printf("[CKD-IDR] RFM bank=%d sa=%d odd %u->%u even %u->%u\n",
                            b, sa, odd0, m_odd[b][sa], even0, m_even[b][sa]);

            t = Tracker();
            auto retrack = [&](int s) {
                uint32_t v = std::max(m_odd[b][s], m_even[b][s]);
                if (!t.valid || v > t.value) {
                    t.valid = true;
                    t.sa = s;
                    t.value = v;
                }
            };
            if (sa > 0) retrack(sa - 1);
            retrack(sa);
            if (sa < m_num_subarrays - 1) retrack(sa + 1);
        }
};

}
