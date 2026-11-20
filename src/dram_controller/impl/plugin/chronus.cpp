#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/plugin/prac.h"
#include "dram_controller/impl/pluginutil/device_config.h"
#include "frontend/frontend.h"

#include <algorithm>
#include <limits>
#include <vector>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace Ramulator {

class Chronus : public IControllerPlugin, public Implementation, public IPRAC {
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, Chronus, "Chronus", "Chronus.")

private:
    class PerBankCounters;

private:
    DeviceConfig m_cfg;
    std::vector<Chronus::PerBankCounters> m_bank_counters;
    std::vector<int> m_same_bank_offsets;

    Clk_t m_clk = 0;

    ABOState m_state = ABOState::NORMAL;
    Clk_t m_abo_recovery_start = std::numeric_limits<Clk_t>::max();

    int m_abo_act_ns = -1;
    int m_abo_recovery_refs = -1;
    int m_abo_thresh = -1;
    int m_trefi_vrr_rate = -1;
    bool m_impress = false;
    int m_nRC = -1;
    int m_nRP = -1;

    int m_abo_act_cycles = -1;

    bool m_is_abo_needed = false;
    bool m_abo_from_threshold = false;  // set by process_pre when threshold crossed

    bool m_debug    = false;
    std::function<bool()> m_pending_work_fn;
    bool m_shadow_wait = false;
    Clk_t m_shadow_start = 0;
    IFrontEnd* m_frontend = nullptr;
    ABOEventCallback m_abo_event_callback;
    uint32_t m_pending_abo_triggers = 0;
    int m_pending_abo_bank = -1;

    uint64_t s_num_recovery = 0;
    uint64_t s_abo_by_chronus_threshold = 0;
    uint64_t s_borrowed_refreshes = 0;
    uint64_t s_rfm_refreshes = 0;
#ifdef WARMUP_TEST
    bool m_warmup_dump_done = false;
#endif

public:
    void init() override { 
        m_debug = param<bool>("debug").default_val(false);
        m_abo_recovery_refs = param<int>("abo_recovery_refs").default_val(4);
        m_abo_act_ns = param<int>("abo_act_ns").default_val(180);
        m_abo_thresh = param<int>("abo_threshold").default_val(512);
        m_trefi_vrr_rate = param<int>("trefi_vrr_rate").default_val(-1);
        m_impress = param<bool>("impress").default_val(false);
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        m_frontend = frontend;
        m_cfg.set_device(cast_parent<IDRAMController>());
        init_dram_params(m_cfg.m_dram);
        m_nRC = m_cfg.m_dram->m_timing_vals("nRC");
        m_nRP = m_cfg.m_dram->m_timing_vals("nRP");

        m_is_abo_needed = false;
        m_abo_act_cycles = m_abo_act_ns / ((float) m_cfg.m_dram->m_timing_vals("tCK_ps") / 1000.0f);

        m_bank_counters.reserve(m_cfg.m_num_banks);
        for (int i = 0; i < m_cfg.m_num_banks; i++) {
            m_bank_counters.emplace_back(
                i, m_cfg, m_is_abo_needed, m_abo_from_threshold,
                [this](int flat_bank_id) {
                    mark_abo_trigger(ABOTrigger::Chronus, flat_bank_id);
                },
                m_abo_thresh, m_trefi_vrr_rate, m_debug, s_borrowed_refreshes, s_rfm_refreshes,
                m_clk, m_impress, m_nRC, m_nRP);
        }

        register_stat(s_num_recovery).name("prac_num_recovery");
        register_stat(s_abo_by_chronus_threshold).name("chronus_abo_by_threshold")
            .desc("ABO triggered by Chronus counter reaching threshold");
        register_stat(s_borrowed_refreshes).name("chronus_borrowed_refreshes")
            .desc("Max-count rows refreshed in time borrowed from periodic REF (trefi_vrr_rate)");
        register_stat(s_rfm_refreshes).name("chronus_rfm_refreshes")
            .desc("Max-count rows refreshed by RFM");
    }

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
        m_clk++;

        update_state_machine(request_found, *req_it);

        if (!request_found) {
            return;
        }

        auto& req = *req_it;
        auto& req_meta = m_cfg.m_dram->m_command_meta(req.command);
        auto& req_scope = m_cfg.m_dram->m_command_scopes(req.command);

        bool has_bank_wildcard = req.addr_vec[m_cfg.m_bank_level] == -1;
        bool has_bankgroup_wildcard = req.addr_vec[m_cfg.m_bankgroup_level] == -1;
        if (has_bankgroup_wildcard && has_bank_wildcard) { // All BG, All Bank
            int offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
            for (int i = 0; i < m_cfg.m_num_banks_per_rank; i++) {
                m_bank_counters[offset + i].on_request(req);
            }
            req.addr_vec[m_cfg.m_bank_level] = -1;
        }
        else if (has_bankgroup_wildcard) { // All BG, Single Bank
            int rank_offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
            int bank_offset = req.addr_vec[m_cfg.m_bank_level];
            for (int i = 0; i < m_cfg.m_num_bankgroups; i++) {
                int bg_offset = i * m_cfg.m_num_banks_per_bankgroup;
                m_bank_counters[rank_offset + bg_offset + bank_offset].on_request(req);
            }
        }
        else if (has_bank_wildcard) { // Single BG, All Bank
            int rank_offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
            int bg_offset = req.addr_vec[m_cfg.m_bankgroup_level] * m_cfg.m_num_banks_per_bankgroup; 
            for (int i = 0; i < m_cfg.m_num_banks_per_bankgroup; i++) {
                m_bank_counters[rank_offset + bg_offset + i].on_request(req);
            }
        }
        else { // Single BG, Single Bank
            auto flat_bank_id = m_cfg.get_flat_bank_id(req);
            m_bank_counters[flat_bank_id].on_request(req);
        }
    }

    void update_state_machine(bool request_found, const Request& req) {
#ifdef WARMUP_TEST
        if (!m_warmup_dump_done && m_frontend && m_frontend->is_warmup_done()) {
            m_warmup_dump_done = true;
            for (int b = 0; b < (int)m_bank_counters.size(); b++) {
                auto crit = m_bank_counters[b].get_critical_row_ids();
                if (!crit.empty())
                    fprintf(stderr, "[WARMUP_TEST] chronus_state: bank=%d critical_rows=%zu\n", b, crit.size());
            }
            fprintf(stderr, "[WARMUP_TEST] chronus: state=%d is_abo_needed=%d num_recovery=%lu\n",
                    (int)m_state, (int)m_is_abo_needed, s_num_recovery);
        }
#endif
        static const std::unordered_map<ABOState, std::string> state_names = {
            {ABOState::NORMAL, "ABOState::NORMAL"},
            {ABOState::PRE_RECOVERY, "ABOState::PRE_RECOVERY"},
            {ABOState::RECOVERY, "ABOState::RECOVERY"}
        };
        auto cmd_prea = m_cfg.m_dram->m_commands("PREA");
        auto cmd_rfmab = m_cfg.m_dram->m_commands("RFMab");
        auto cmd_act = m_cfg.m_dram->m_commands("ACT");
        auto cur_state = m_state;
        switch(m_state) {
        case ABOState::NORMAL:
            if (m_is_abo_needed && m_frontend && m_frontend->is_warmup_done()) {
                if (m_debug) {
                    std::printf("[Chronus] [%lu] <%s> Asserting ALERT_N.\n", m_clk, state_names.at(cur_state).c_str());
                }
                m_state = ABOState::PRE_RECOVERY;
                m_abo_recovery_start = m_clk + m_abo_act_cycles;
                s_num_recovery++;
                if (m_abo_from_threshold) s_abo_by_chronus_threshold++;
                m_abo_from_threshold = false;
                if (m_abo_event_callback) {
                    uint32_t triggers = m_pending_abo_triggers;
                    if (triggers == 0) {
                        triggers = static_cast<uint32_t>(ABOTrigger::External);
                    }
                    m_abo_event_callback(triggers, m_pending_abo_bank);
                }
                m_pending_abo_triggers = 0;
                m_pending_abo_bank = -1;
            } else if (m_frontend && !m_frontend->is_warmup_done()) {
                if (m_shadow_wait) {
                    if (m_clk >= m_shadow_start) shadow_recovery();
                } else if (m_is_abo_needed) {
                    m_shadow_wait = true;
                    m_shadow_start = m_clk + m_abo_act_cycles;
                }
            } else {
                m_shadow_wait = false;
            }
            break;
        case ABOState::PRE_RECOVERY:
            if (request_found && req.command == cmd_prea) {
                if (m_debug) {
                    std::printf("[Chronus] [%lu] <%s> Received PREA.\n", m_clk, state_names.at(cur_state).c_str());
                }
            }
            if (m_clk == m_abo_recovery_start) {
                m_state = ABOState::RECOVERY;
                m_abo_recovery_start = std::numeric_limits<Clk_t>::max();
            }
            break;
        case ABOState::RECOVERY:
            if (request_found && req.command == cmd_rfmab) {
                int rank_begin = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
                int rank_end = (req.addr_vec[m_cfg.m_rank_level] + 1) * m_cfg.m_num_banks_per_rank;
                m_is_abo_needed = false;
                for (int i = 0; i < m_cfg.m_num_banks; i++) {
                    int num_effective_critical = m_bank_counters[i].num_critical();
                    // The RFMab resets one row in each bank of its rank.
                    if (i >= rank_begin && i < rank_end) {
                        num_effective_critical--;
                    }
                    m_is_abo_needed |= num_effective_critical > 0;
                }
                if (!m_is_abo_needed && m_pending_work_fn && m_pending_work_fn()) {
                    m_is_abo_needed = true;
                }
                if (!m_is_abo_needed) {
                    m_state = ABOState::NORMAL;
                    // These triggers belong to the recovery that just ended.
                    m_pending_abo_triggers = 0;
                    m_pending_abo_bank = -1;
                }
            }
            break;
        }
        if (m_debug && cur_state != m_state) {
            std::printf("[Chronus] [%lu] <%s> -> <%s>\n", m_clk, state_names.at(cur_state).c_str(), state_names.at(m_state).c_str());
        }
    }

    Clk_t next_recovery_cycle() override {
        return m_abo_recovery_start;
    }

    int get_num_abo_recovery_refs() override {
        return m_abo_recovery_refs;
    }

    ABOState get_state() override {
        return m_state;
    }

    void set_abo_needed(bool val) override {
        set_abo_needed(val, ABOTrigger::External, -1);
    }

    void set_abo_needed(bool val, ABOTrigger trigger,
                        int flat_bank_id) override {
        if (val) {
            mark_abo_trigger(trigger, flat_bank_id);
        } else {
            m_is_abo_needed = false;
            m_pending_abo_triggers = 0;
            m_pending_abo_bank = -1;
        }
    }

    void set_abo_event_callback(ABOEventCallback callback) override {
        m_abo_event_callback = std::move(callback);
    }

    bool is_abo_active() const override {
        return m_state == ABOState::PRE_RECOVERY || m_state == ABOState::RECOVERY;
    }

    uint32_t get_act_count(int flat_bank_id, int row_id) override {
        if (flat_bank_id < 0 || flat_bank_id >= (int)m_bank_counters.size())
            return 0;
        return m_bank_counters[flat_bank_id].get_count(row_id);
    }

    void set_pending_work_fn(std::function<bool()> fn) override {
        m_pending_work_fn = fn;
    }

    void shadow_recovery() {
        int nranks = m_cfg.m_num_ranks;
        int bpr = m_cfg.m_num_banks_per_rank;
        int rounds = 0;
        bool ended = false;
        while (!ended && rounds < 100000) {
            rounds++;
            for (int r = 0; r < nranks; r++) {
                for (int b = r * bpr; b < (r + 1) * bpr; b++) m_bank_counters[b].shadow_rfm();
                if (ended) continue;
                bool need = false;
                for (auto& bc : m_bank_counters) need |= bc.num_critical() > 0;
                if (!need && m_pending_work_fn && m_pending_work_fn()) need = true;
                if (!need) ended = true;
            }
        }
        if (m_debug)
            std::printf("[Chronus] [%lu] SHADOW-RECOVERY rounds=%d\n", m_clk, rounds);
        m_shadow_wait = false;
        m_is_abo_needed = false;
        m_abo_from_threshold = false;
        m_pending_abo_triggers = 0;
        m_pending_abo_bank = -1;
    }

private:
    void mark_abo_trigger(ABOTrigger trigger, int flat_bank_id) {
        m_is_abo_needed = true;
        m_pending_abo_triggers |= static_cast<uint32_t>(trigger);
        if (m_pending_abo_bank < 0 && flat_bank_id >= 0) {
            m_pending_abo_bank = flat_bank_id;
        }
    }

    class PerBankCounters {
    public: 
        PerBankCounters(int bank_id, DeviceConfig& cfg,
                        bool& is_abo_needed, bool& abo_from_threshold,
                        std::function<void(int)> on_threshold,
                        int alert_thresh, int trefi_vrr_rate, bool debug,
                        uint64_t& borrowed_refreshes, uint64_t& rfm_refreshes,
                        const Clk_t& clk, bool impress, int nRC, int nRP)
        : m_bank_id(bank_id), m_cfg(cfg), m_is_abo_needed(is_abo_needed), m_abo_from_threshold(abo_from_threshold),
        m_on_threshold(std::move(on_threshold)), m_alert_thresh(alert_thresh),
        m_trefi_vrr_rate(trefi_vrr_rate), m_debug(debug), m_borrowed_refreshes(borrowed_refreshes),
        m_rfm_refreshes(rfm_refreshes), m_clk(clk), m_impress(impress),
        m_nRC(nRC), m_nRP(nRP) {
            m_rda_close_delay = m_cfg.m_dram->m_timing_vals("nRTP");
            m_wra_close_delay = m_cfg.m_dram->m_timing_vals("nCWL") + m_cfg.m_dram->m_timing_vals("nBL")
                              + m_cfg.m_dram->m_timing_vals("nWR");
            init_handlers(m_cfg.m_dram);
            reset();
        }

        ~PerBankCounters() {
            m_counters.clear();
        }

        void on_request(const Request& req) {
            if (m_handlertable.find(req.command) != m_handlertable.end()) {
                m_handlertable[req.command].handler(req);
            }
        }

        void init_handlers(IDRAM* dram) {
            CommandHandler handlers[] = {
                {std::string("ACT"), std::bind(&PerBankCounters::process_act, this, std::placeholders::_1)},
                {std::string("PRE"), std::bind(&PerBankCounters::process_pre, this, std::placeholders::_1)},
                {std::string("PREA"), std::bind(&PerBankCounters::process_pre, this, std::placeholders::_1)},
                {std::string("PREsb"), std::bind(&PerBankCounters::process_pre, this, std::placeholders::_1)},
                {std::string("RDA"), std::bind(&PerBankCounters::process_rda, this, std::placeholders::_1)},
                {std::string("WRA"), std::bind(&PerBankCounters::process_wra, this, std::placeholders::_1)},
                {std::string("RFMab"), std::bind(&PerBankCounters::process_rfm, this, std::placeholders::_1)},
                {std::string("RFMsb"), std::bind(&PerBankCounters::process_rfm, this, std::placeholders::_1)},
                {std::string("REFab"), std::bind(&PerBankCounters::process_ref, this, std::placeholders::_1)},
                {std::string("REFsb"), std::bind(&PerBankCounters::process_ref, this, std::placeholders::_1)}
            };
            for (auto& h : handlers) {
                if (!dram->m_commands.contains(h.cmd_name)) {
                    std::cout << "[Chronus] Command " << h.cmd_name << "does not exist." << std::endl;
                    exit(0);
                }
                m_handlertable[dram->m_commands(h.cmd_name)] = h;
            }
        }

        void reset() {
            m_counters.clear();
            m_critical_rows.clear();
        }

        bool is_critical() {
            return m_critical_rows.size() > 0;
        }

        int num_critical() {
            return m_critical_rows.size();
        }

        uint32_t get_count(int row_id) const {
            auto it = m_counters.find(row_id);
            return it != m_counters.end() ? static_cast<uint32_t>(it->second) : 0;
        }

        std::vector<int> get_critical_row_ids() const {
            std::vector<int> result;
            result.reserve(m_critical_rows.size());
            for (auto& [row, count] : m_critical_rows)
                result.push_back(row);
            return result;
        }

        void count_row(int row_addr, const char* tag, double amount = 1.0) {
            if (m_counters.find(row_addr) == m_counters.end()) {
                m_counters[row_addr] = 0;
            }
            m_counters[row_addr] += amount;
            if (m_debug) {
                std::printf("[Chronus] [%d] [%s] Row: %d Act: %.2f  thresh=%d  crit_now=%zu  abo_needed=%d\n",
                    m_bank_id, tag, row_addr, m_counters[row_addr],
                    m_alert_thresh, m_critical_rows.size(), (int)m_is_abo_needed);
            }
            if (m_counters[row_addr] >= m_alert_thresh) {
                bool first_crit = (m_critical_rows.find(row_addr) == m_critical_rows.end());
                m_critical_rows[row_addr] = m_counters[row_addr];
                if (!m_is_abo_needed) m_abo_from_threshold = true;
                m_is_abo_needed = true;
                if (first_crit && m_on_threshold) {
                    m_on_threshold(m_bank_id);
                }
                if (m_debug && first_crit) {
                    std::printf("[Chronus] [%d] [CRITICAL] Row %d crossed threshold (count=%.2f >= %d) -> ABO asserted (abo_from_threshold=%d)\n",
                        m_bank_id, row_addr, m_counters[row_addr], m_alert_thresh,
                        (int)m_abo_from_threshold);
                }
            }
        }

        void shadow_rfm() {
            if (reset_max_row("RFM")) m_rfm_refreshes++;
        }

        // Row with the highest non-zero activation count, -1 if none.
        int max_row() const {
            int best = -1;
            double best_cnt = 0;
            for (auto& [row, cnt] : m_counters) {
                if (cnt > best_cnt) { best = row; best_cnt = cnt; }
            }
            return best;
        }

    private:
        struct CommandHandler {
            std::string cmd_name;
            std::function<void(const Request&)> handler;
        };

        DeviceConfig& m_cfg;
        bool& m_is_abo_needed;
        bool& m_abo_from_threshold;
        std::function<void(int)> m_on_threshold;

        std::unordered_map<int, double> m_counters;
        std::unordered_map<int, double> m_critical_rows;
        std::unordered_map<int, CommandHandler> m_handlertable;

        int m_alert_thresh = -1;
        bool m_debug = false;
        int m_bank_id = -1;
        int m_trefi_vrr_rate = -1;
        int m_trefi_ctr = -1;
        int m_open_row = -1;
        uint64_t& m_borrowed_refreshes;
        uint64_t& m_rfm_refreshes;
        const Clk_t& m_clk;
        bool m_impress = false;
        int m_nRC = -1;
        int m_nRP = -1;
        Clk_t m_act_clk = 0;
        int m_rda_close_delay = 0;
        int m_wra_close_delay = 0;

        // Refresh (reset) the max-count row. Returns false if every counter is 0.
        bool reset_max_row(const char* tag) {
            int row = max_row();
            if (row < 0) {
                if (m_debug) {
                    std::printf("[Chronus] [%d] [%s] No critical row.\n", m_bank_id, tag);
                }
                return false;
            }
            if (m_debug) {
                std::printf("[Chronus] [%d] [%s] Row: %d Act: %.2f\n", m_bank_id, tag, row, m_counters[row]);
            }
            m_counters[row] = 0;
            m_critical_rows.erase(row);
            return true;
        }

        void process_act(const Request& req) {
            m_open_row = req.addr_vec[m_cfg.m_row_level];
            m_act_clk = m_clk;
        }

        double eact(Clk_t t_close) const {
            if (!m_impress) return 1.0;
            return std::max(1.0, double(t_close - m_act_clk + m_nRP) / m_nRC);
        }

        void close_row(Clk_t t_close) {
            if (m_open_row >= 0) {
                double e = eact(t_close);
                if (m_debug && m_impress) {
                    double pressed = double(t_close - m_act_clk);
                    std::printf("[Chronus-EACT] [%d] row=%d act=%ld close=%ld pressed=%.1f nRC=%d nRP=%d eact=%.4f\n",
                                m_bank_id, m_open_row, (long)m_act_clk, (long)t_close, pressed, m_nRC, m_nRP, e);
                }
                count_row(m_open_row, "ACT", e);
            }
            m_open_row = -1;
        }

        void process_pre(const Request& req) {
            close_row(m_clk);
        }

        void process_rda(const Request& req) {
            close_row(m_clk + m_rda_close_delay);
        }

        void process_wra(const Request& req) {
            close_row(m_clk + m_wra_close_delay);
        }

        void process_rfm(const Request& req) {
            if (reset_max_row("RFM")) m_rfm_refreshes++;
        }

        void process_ref(const Request& req) {
            m_trefi_ctr++;
            if (m_trefi_vrr_rate < 0 || m_trefi_ctr < m_trefi_vrr_rate) {
                return;
            }
            if (m_debug) {
                std::printf("[Chronus] [%d] [REF] Borrowing time for victim refresh\n", m_bank_id);
            }
            m_trefi_ctr = 0;
            // Borrowed refresh (Chronus paper §5/§7.1) is transparent REF time, not an RFM window.
            if (reset_max_row("REF")) m_borrowed_refreshes++;
        }
    };  // class PerBankCounters

};      // class Chronus

}       // namespace Ramulator
