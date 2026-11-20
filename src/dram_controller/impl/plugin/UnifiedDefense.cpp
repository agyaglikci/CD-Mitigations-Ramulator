#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/plugin/PerBankCounters.h"
#include "frontend/frontend.h"

#include <vector>
#include <algorithm>
#include <cstdio>

namespace Ramulator {

    class UnifiedDefense: public IControllerPlugin, public Implementation {
        RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, UnifiedDefense, "UnifiedDefense", "Unified ColumnDisturb Mechanism")

    private:
        trigger_thresh m_trigger_threshold;
        DeviceConfig m_cfg;
        CounterType m_counter_type_global;
        std::vector<std::unique_ptr<PerBankCounters>> m_bank_counters;
        unsigned long s_num_mitigations_requested;
        int vrr_req_id = -1;
        int m_subarray_size = -1;
        int m_act_id = -1;
        int m_RFMab_id = -1;
        int m_RFMsb_id = -1;
        bool m_is_debug = false;
        IFrontEnd* m_frontend = nullptr;
        std::vector<std::pair<int, AddrVec_t>> m_shadow_acts;

    public:
        
        void init() override{
            std::string counter_type_string = param<std::string>("mitigation").required();
            std::transform(counter_type_string.begin(), counter_type_string.end(), counter_type_string.begin(), ::toupper);
            if(counter_type_string == "SINGLE") m_counter_type_global = CounterType::SINGLE;
            else if (counter_type_string == "DUAL") m_counter_type_global = CounterType::DUAL;
            else if (counter_type_string.rfind("PROBABILISTIC", 0) == 0) m_counter_type_global = CounterType::PROBABILISTIC;
            else if (counter_type_string == "NONE") m_counter_type_global = CounterType::NONE;
            else throw ConfigurationError("Unknown counter design type \"{}\" in {}!", counter_type_string, get_name());

            if (m_counter_type_global == CounterType::PROBABILISTIC) m_trigger_threshold.float_mtg = param<float>("alert_threshold").required();
            else m_trigger_threshold.int_mtg = param<int>("alert_threshold").required();

            m_subarray_size = param<int>("subarray_size").required();
            m_is_debug = param<bool>("debug").default_val(false);
        }

        bool mitigation_enable(const Request &req) const{
            bool is_opening = m_cfg.m_dram->m_command_meta(req.command).is_opening;
            bool is_row_level = m_cfg.m_dram->m_command_scopes(req.command) == m_cfg.m_row_level;
            return (is_opening && is_row_level) || (req.type_id == vrr_req_id);
        }

        void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
            m_frontend = frontend;
            m_cfg.set_device(cast_parent<IDRAMController>(), m_subarray_size);
            m_ctrl = cast_parent<IDRAMController>();

            for (int i = 0; i < m_cfg.m_num_banks; i++){
                switch (m_counter_type_global)
                {
                case CounterType::SINGLE:
                    m_bank_counters.push_back(std::make_unique<SingleCounters>(m_cfg, m_trigger_threshold.int_mtg));
                    break;
                case CounterType::DUAL:
                    m_bank_counters.push_back(std::make_unique<DualCounters>(m_cfg, m_trigger_threshold.int_mtg));
                    break;
                case CounterType::PROBABILISTIC:
                    m_bank_counters.push_back(std::make_unique<ProbabilisticCounters>(m_cfg, m_trigger_threshold.float_mtg));
                    break;
                case CounterType::NONE:
                    m_bank_counters.push_back(std::make_unique<NoneCounters>(m_cfg));
                    break;
                }
            }

            m_act_id = m_ctrl->m_dram->m_commands("ACT");
            m_RFMab_id = m_ctrl->m_dram->m_commands("RFMab");
            m_RFMsb_id = m_ctrl->m_dram->m_commands("RFMsb");
            s_num_mitigations_requested = 0;
            register_stat(s_num_mitigations_requested).name("num_mitigations_requested");
            if (m_ctrl->m_dram->m_requests.contains("victim-row-refresh"))
                vrr_req_id = m_ctrl->m_dram->m_requests("victim-row-refresh");
        }

        void update(bool request_found, ReqBuffer::iterator& req_it) override {
            if (!request_found) return;
            auto& req = *req_it;

            bool has_bank_wildcard = req.addr_vec[m_cfg.m_bank_level] == -1;
            bool has_bankgroup_wildcard = req.addr_vec[m_cfg.m_bankgroup_level] == -1;
            int subarray_id = m_cfg.get_subarray_id(req);

            if (has_bankgroup_wildcard && has_bank_wildcard) { // All BG, All Bank
                int offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
                for (int i = 0; i < m_cfg.m_num_banks_per_rank; i++) {
                    feed(req, offset + i, subarray_id);
                    int complete_bank_id = offset + i;
                    check_and_mitigate(req, complete_bank_id, subarray_id);
                }
            }
            else if (has_bankgroup_wildcard) { // All BG, Single Bank
                int rank_offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
                int bank_offset = req.addr_vec[m_cfg.m_bank_level];
                for (int i = 0; i < m_cfg.m_num_bankgroups; i++) {
                    int bg_offset = i * m_cfg.m_num_banks_per_bankgroup;
                    feed(req, rank_offset + bg_offset + bank_offset, subarray_id);
                    int complete_bank_id = rank_offset + bg_offset + bank_offset;
                    check_and_mitigate(req, complete_bank_id, subarray_id);
                }
            }
            else if (has_bank_wildcard) { // Single BG, All Bank
                int rank_offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
                int bg_offset = req.addr_vec[m_cfg.m_bankgroup_level] * m_cfg.m_num_banks_per_bankgroup; 
                for (int i = 0; i < m_cfg.m_num_banks_per_bankgroup; i++) {
                    feed(req, rank_offset + bg_offset + i, subarray_id);
                    int complete_bank_id = rank_offset + bg_offset + i;
                    check_and_mitigate(req, complete_bank_id, subarray_id);
                }
            }
            else { // Single BG, Single Bank
                auto flat_bank_id = m_cfg.get_flat_bank_id(req);
                feed(req, flat_bank_id, subarray_id);
                check_and_mitigate(req, flat_bank_id, subarray_id);
            }

            for (size_t i = 0; i < m_shadow_acts.size(); i++) {
                auto [bank, addr_vec] = m_shadow_acts[i];
                Request act(addr_vec, -1);
                act.command = m_act_id;
                m_bank_counters[bank]->on_request(act);
                if (m_is_debug) print_counts("SHADOW-ACT", bank, m_cfg.get_subarray_id(act));
                check_and_mitigate(act, bank, m_cfg.get_subarray_id(act));
            }
            m_shadow_acts.clear();
        }

        void feed(const Request &req, int flat_bank_id, int subarray_id) {
            m_bank_counters[flat_bank_id]->on_request(req);
            if (!m_is_debug) return;
            if (req.command == m_RFMab_id || req.command == m_RFMsb_id)
                std::printf("[CK] RFM bank=%d\n", flat_bank_id);
            else if (req.command == m_act_id)
                print_counts("ACT", flat_bank_id, subarray_id);
        }

        void print_counts(const char* tag, int flat_bank_id, int subarray_id) {
            auto c = [&](int s) -> long {
                if (s < 0 || s >= m_cfg.m_num_subarrays_per_bank) return -1;
                return m_bank_counters[flat_bank_id]->get_count(s);
            };
            std::printf("[CK] %s bank=%d sa=%d c=%ld,%ld,%ld\n", tag, flat_bank_id, subarray_id,
                        c(subarray_id - 1), c(subarray_id), c(subarray_id + 1));
        }

        void check_and_mitigate(const Request &req, int flat_bank_id, int subarray_id) {
            if (!mitigation_enable(req)) return; // if not a row opening command or vrr, skip

            if (subarray_id > 0 && m_bank_counters[flat_bank_id]->need_progress(subarray_id - 1))
                mitigate_at(req, flat_bank_id, subarray_id - 1);
            if (subarray_id < (m_cfg.m_num_subarrays_per_bank - 1) && m_bank_counters[flat_bank_id]->need_progress(subarray_id + 1))
                mitigate_at(req, flat_bank_id, subarray_id + 1);
            if (m_bank_counters[flat_bank_id]->need_progress(subarray_id))
                mitigate_at(req, flat_bank_id, subarray_id);
        }

        void mitigate_at(const Request &req, int flat_bank_id, int subarray_id) {
            Request cd_mitigate(req.addr_vec,m_cfg.m_dram->m_requests("open-row")); // Create a copy of the request to mitigate
            cd_mitigate.addr_vec[m_cfg.m_row_level] = m_bank_counters[flat_bank_id]->get_and_inc_rolling_pointer(subarray_id);
            zero_at(flat_bank_id, subarray_id);
            s_num_mitigations_requested++;
            bool sent = !m_frontend || m_frontend->is_warmup_done();
            if (sent)
                m_ctrl->priority_send(cd_mitigate);
            else
                m_shadow_acts.push_back({flat_bank_id, cd_mitigate.addr_vec});
            if (m_is_debug)
                std::printf("[CK] REFRESH bank=%d sa=%d row=%d c=%u sent=%d\n", flat_bank_id, subarray_id,
                            cd_mitigate.addr_vec[m_cfg.m_row_level], m_bank_counters[flat_bank_id]->get_count(subarray_id), (int)sent);
        }

        inline void zero_at(int flat_bank_id, int subarray_id) {m_bank_counters[flat_bank_id]->zero_counters(subarray_id);}

    };
}