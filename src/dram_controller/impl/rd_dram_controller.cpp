#include <limits>

#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"
#include "dram_controller/impl/plugin/prac.h"
#include "frontend/frontend.h"

namespace Ramulator{
class ReadDisturbDRAMController final : public IDRAMController, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, ReadDisturbDRAMController, "ReadDisturb",
                                         "DRAM controller with PRAC/ABO recovery and maintenance-request support for read disturbance mitigations")
    
    private:

        // Write-mode watermarks (read/write buffer toggle)
        float m_wr_low_watermark = 0.2f;
        float m_wr_high_watermark = 0.8f;

        bool m_is_debug = false;

        // Request buffers
        std::deque<Request> pending; // completed reads waiting to depart (served after depart clk)
        ReqBuffer m_active_buffer; // requests with opened row, in progress
        ReqBuffer m_prac_buffer; // PRAC ABO recovery: PREA + RFMab (rank-wide)
        ReqBuffer m_priority_buffer; // maintenance requests (priority_send)
        ReqBuffer m_read_buffer;
        ReqBuffer m_write_buffer;
        Clk_t m_watchdog_max_wait = 0;
        size_t s_watchdog_max_pending_age = 0;

        bool m_is_write_mode = false;

        // Plugin / Frontend HAndles
        IPRAC* m_prac = nullptr;
        IFrontEnd* m_frontend = nullptr;

        // PRAC ABO template commands
        Request* m_prea_template  = nullptr;
        Request* m_rfmab_template = nullptr;

        int m_bank_addr_idx = -1;        // bank level, for active-buffer conflict check

        // Warmup end: reset accumulated stats once
        bool m_warmup_stats_reset = false;

        int m_level_bank = -1, m_level_rank = -1, m_level_bg = -1;

        // ── Statistics ──
        size_t s_num_read_reqs = 0, s_num_write_reqs = 0, s_num_other_reqs=0;
        size_t s_read_latency = 0;   float s_avg_read_latency = 0;
        size_t s_prac_recoveries = 0;
        size_t s_total_rfm_cycles = 0;
        size_t s_dram_read_count = 0;
        size_t s_measurement_cycles = 0;   // clocks in the measurement window (Mess full-BW window)
        std::vector<int> s_core_row_hits, s_core_row_misses, s_core_row_conflicts;
        int s_num_row_hits = 0, s_num_row_misses = 0, s_num_row_conflicts = 0;
        size_t s_queue_len = 0, s_read_queue_len = 0, s_write_queue_len = 0, s_priority_queue_len = 0;
        float  s_queue_len_avg = 0, s_read_queue_len_avg = 0, s_write_queue_len_avg = 0, s_priority_queue_len_avg = 0;
        Clk_t m_clk_at_warmup = 0;   // clock at warmup->measurement boundary; divisor baseline for queue-len averages



        // RowPress tmro cap (max row open time), baseline-only, default disabled
        int m_max_row_open_ns = -1;
        uint64_t m_max_row_open_clk = 0;
        int m_PRE_req_id_mro = -1;
        std::vector<int64_t> m_row_opened_at;
        std::vector<std::vector<int>> m_open_addr_vec;
        int m_mro_num_ranks = -1, m_mro_num_bgs = -1, m_mro_num_banks = -1;
        int m_mro_num_banks_total = -1;



    public:
        void init() override {
            m_wr_low_watermark = param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
            m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching back to write mode.").default_val(0.8f);

            m_is_debug = param<bool>("debug").default_val(false);

            m_watchdog_max_wait = param<Clk_t>("watchdog_max_wait_cycles")
                .desc("Livelock/starvation safeguard: max cycles a request may wait before aborting. "
                    "PLACEHOLDER — will be calibrated empirically.")
                .default_val(100000000);


            m_max_row_open_ns = param<int>("max_open_ns")
                .desc("Max time (ns) a row can stay open before being force-closed (RowPress tmro cap). -1 = disabled.")
                .default_val(-1);



            m_scheduler = create_child_ifce<IScheduler>();
            m_refresh = create_child_ifce<IRefreshManager>();
            m_rowpolicy = create_child_ifce<IRowPolicy>();

            if (m_config["plugins"]){
                YAML::Node plugin_configs = m_config["plugins"];
                for(YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it){
                    m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
                }
            }

        }

        void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override{
            m_frontend = frontend;
            m_dram = memory_system->get_ifce<IDRAM>();
            m_bank_addr_idx = m_dram->m_levels("bank");

            m_prac_buffer.max_size     = 512 * 3 + 32;
            m_priority_buffer.max_size = std::numeric_limits<size_t>::max();   // unlimited

            register_stat(s_watchdog_max_pending_age).name("watchdog_max_pending_age_{}", m_channel_id);



            // RowPress tmro cap setup — no-op unless max_open_ns was explicitly configured
            if (m_max_row_open_ns > 0) {
                m_max_row_open_clk = (uint64_t)(m_max_row_open_ns / ((float) m_dram->m_timing_vals("tCK_ps") / 1000.0f));

                m_mro_num_ranks = m_dram->get_level_size("rank");
                m_mro_num_bgs   = m_dram->get_level_size("bankgroup");
                m_mro_num_banks = m_dram->get_level_size("bank");
                m_mro_num_banks_total = m_mro_num_ranks * m_mro_num_bgs * m_mro_num_banks;

                m_PRE_req_id_mro = m_dram->m_requests("close-row");

                m_row_opened_at.assign(m_mro_num_banks_total, -1);
                m_open_addr_vec.assign(m_mro_num_banks_total, std::vector<int>());
            }



            for (auto* p : m_plugins) {
                if (!m_prac) m_prac = dynamic_cast<IPRAC*>(p);
            }

            if (m_prac) {
                std::vector<int> all_bank_addr(m_dram->m_levels.size(), -1);
                all_bank_addr[m_dram->m_levels("channel")] = m_channel_id;

                m_prea_template = new Request(all_bank_addr, m_dram->m_requests("close-all-bank"));
                m_prea_template->command = m_prea_template->final_command = m_dram->m_commands("PREA");

                m_rfmab_template = new Request(all_bank_addr, m_dram->m_requests("rfm"));
                m_rfmab_template->command = m_rfmab_template->final_command = m_dram->m_commands("RFMab");
            }

            m_level_bank = m_dram->m_levels("bank");
            m_level_rank = m_dram->m_levels("rank");
            m_level_bg   = m_dram->m_levels("bankgroup");

            // ── Per-core row hit/miss/conflict stats ──
            int num_cores = m_frontend ? m_frontend->get_num_cores() : 1;
            s_core_row_hits.resize(num_cores, 0);
            s_core_row_misses.resize(num_cores, 0);
            s_core_row_conflicts.resize(num_cores, 0);
            for (int i = 0; i < num_cores; i++) {
                register_stat(s_core_row_hits[i]).name("controller_core_row_hits_{}", i);
                register_stat(s_core_row_misses[i]).name("controller_core_row_misses_{}", i);
                register_stat(s_core_row_conflicts[i]).name("controller_core_row_conflicts_{}", i);
            }
            register_stat(s_num_row_hits).name("controller_num_row_hits");
            register_stat(s_num_row_misses).name("controller_num_row_misses");
            register_stat(s_num_row_conflicts).name("controller_num_row_conflicts");

            // ── Request + latency stats ──
            register_stat(s_num_read_reqs).name("rd_ctrl_num_read_reqs_{}", m_channel_id);
            register_stat(s_num_write_reqs).name("rd_ctrl_num_write_reqs_{}", m_channel_id);
            register_stat(s_num_other_reqs).name("rd_ctrl_num_other_reqs_{}", m_channel_id);
            register_stat(s_dram_read_count).name("rd_ctrl_dram_read_count_{}", m_channel_id);
            register_stat(s_read_latency).name("rd_ctrl_read_latency_{}", m_channel_id);
            register_stat(s_avg_read_latency).name("rd_ctrl_avg_read_latency_{}", m_channel_id);
            register_stat(s_prac_recoveries).name("rd_ctrl_prac_recoveries_{}", m_channel_id);
            register_stat(s_total_rfm_cycles).name("rd_ctrl_total_rfm_cycles_{}", m_channel_id);
            register_stat(s_measurement_cycles).name("rd_ctrl_measurement_cycles_{}", m_channel_id);

            // ── Queue-length diagnostics ──
            register_stat(s_queue_len).name("queue_len_{}", m_channel_id);
            register_stat(s_read_queue_len).name("read_queue_len_{}", m_channel_id);
            register_stat(s_write_queue_len).name("write_queue_len_{}", m_channel_id);
            register_stat(s_priority_queue_len).name("priority_queue_len_{}", m_channel_id);
            register_stat(s_queue_len_avg).name("queue_len_avg_{}", m_channel_id);
            register_stat(s_read_queue_len_avg).name("read_queue_len_avg_{}", m_channel_id);
            register_stat(s_write_queue_len_avg).name("write_queue_len_avg_{}", m_channel_id);
            register_stat(s_priority_queue_len_avg).name("priority_queue_len_avg_{}", m_channel_id);

        }

        bool send(Request& req) override{
            req.final_command = m_dram->m_request_translations(req.type_id);

            switch(req.type_id){
                case Request::Type::Read:{
                    s_num_read_reqs++;
                    break;
                }
                case Request::Type::Write:{
                    s_num_write_reqs++;
                    break;
                }
                default:{
                    s_num_other_reqs++;
                }
            }

            // Read that hits a pending write in the write buffer → serve from buffer, bypass DRAM
            if (req.type_id == Request::Type::Read) {
                auto same_addr = [&req](const Request& w) { return w.addr == req.addr; };
                if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(), same_addr) != m_write_buffer.end()) {
                    req.arrive  = m_clk;
                    req.depart  = m_clk + 1;
                    req.from_write_buffer = true;
                    pending.push_back(req);
                    return true;
                }
            }

            req.arrive = m_clk;
            bool ok = false;
            if      (req.type_id == Request::Type::Read)  ok = m_read_buffer.enqueue(req);
            else if (req.type_id == Request::Type::Write) ok = m_write_buffer.enqueue(req);
            else throw std::runtime_error("ReadDisturbDRAMController: invalid request type");

            if (!ok) { req.arrive = -1; return false; }
            return true;

        }

        bool priority_send(Request& req) override{
            req.final_command = m_dram->m_request_translations(req.type_id);
            req.arrive = m_clk;
            return m_priority_buffer.enqueue(req);
        }

        void tick() override{
            m_clk++;

            Clk_t pending_age = oldest_pending_age();
            if (pending_age > (Clk_t)s_watchdog_max_pending_age) s_watchdog_max_pending_age = pending_age;
            check_livelock_watchdog(pending_age, m_watchdog_max_wait);

            // update stats
             s_queue_len          += m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size()
                                    + m_active_buffer.size() + m_prac_buffer.size() + pending.size();
            s_read_queue_len     += m_read_buffer.size() + pending.size();
            s_write_queue_len    += m_write_buffer.size();
            s_priority_queue_len += m_priority_buffer.size();

            serve_completed_reads();

            m_refresh->tick();

            bool warmup_active = m_frontend && !m_frontend->is_warmup_done();

            // Warmup -> measurement transition: reset stats accumulated during warmup
            if (!m_warmup_stats_reset && !warmup_active) {
                m_warmup_stats_reset = true;

                // Reset stats that accumulated during warmup (so measured stats are clean)
                s_num_read_reqs = 0;
                s_num_write_reqs = 0;
                s_num_other_reqs = 0;
                s_read_latency = 0;
                s_dram_read_count = 0;
                s_num_row_hits = 0;
                s_num_row_misses = 0;
                s_num_row_conflicts = 0;
                std::fill(s_core_row_hits.begin(),      s_core_row_hits.end(),      0);
                std::fill(s_core_row_misses.begin(),    s_core_row_misses.end(),    0);
                std::fill(s_core_row_conflicts.begin(), s_core_row_conflicts.end(), 0);
            }

            // PRAC ABO recovery injection (drive Chronus' ABO state machine)
            if (m_prac && !warmup_active) {
                Clk_t nrc = m_prac->next_recovery_cycle();
                int   nRP = m_dram->m_timing_vals("nRP");

                // Recovery deadline within nRP+5 cycles, and we haven't queued it yet?
                bool recovery_near = (nrc != std::numeric_limits<Clk_t>::max())
                                    && (nrc - m_clk <= (Clk_t)(nRP + 5));
                bool already_queued = (m_prac_buffer.size() != 0);

                if (recovery_near && !already_queued) {
                    int nranks = m_dram->get_level_size("rank");
                    // PREA for every rank first...
                    for (int r = 0; r < nranks; r++) {
                        m_prea_template->addr_vec[m_level_rank] = r;
                        m_prea_template->arrive = m_clk;
                        m_prac_buffer.enqueue(*m_prea_template);
                    }
                    // ...then RFMab for every rank
                    for (int r = 0; r < nranks; r++) {
                        m_rfmab_template->addr_vec[m_level_rank] = r;
                        m_rfmab_template->arrive = m_clk;
                        m_prac_buffer.enqueue(*m_rfmab_template);
                    }
                    s_prac_recoveries++;
                }

                // Still in RECOVERY and the buffer drained → inject more RFMab to keep recovering
                if (m_prac->get_state() == IPRAC::ABOState::RECOVERY && m_prac_buffer.size() == 0) {
                    int nranks = m_dram->get_level_size("rank");
                    for (int r = 0; r < nranks; r++) {
                        m_rfmab_template->addr_vec[m_level_rank] = r;
                        m_rfmab_template->arrive = m_clk;
                        m_prac_buffer.enqueue(*m_rfmab_template);
                    }
                }
            }
                // Schedule a request
            ReqBuffer::iterator req_it;
            ReqBuffer* buffer = nullptr;
            bool request_found = schedule_request(req_it, buffer);
            //  Row policy + plugins
            m_rowpolicy->update(request_found, req_it);
            for (auto* p : m_plugins) p->update(request_found, req_it);
            // Issue the command
            if (request_found) {
                // Track total RFMab cycles
                if (m_rfmab_template && req_it->command == m_rfmab_template->command)
                    s_total_rfm_cycles += m_dram->m_timing_vals("nRFM1");
                m_dram->issue_command(req_it->command, req_it->addr_vec);



                if (m_max_row_open_ns > 0) {
                    if (m_dram->m_command_meta(req_it->command).is_closing ||
                        m_dram->m_command_meta(req_it->command).is_refreshing) {

                        if (req_it->addr_vec[m_level_bg] == -1 && req_it->addr_vec[m_level_bank] == -1) {
                            for (int b = 0; b < m_mro_num_banks; b++) {
                                for (int bg = 0; bg < m_mro_num_bgs; bg++) {
                                    int rank_id = req_it->addr_vec[m_level_rank];
                                    int flat_bank_id = b + bg * m_mro_num_banks + rank_id * m_mro_num_banks * m_mro_num_bgs;
                                    m_row_opened_at[flat_bank_id] = -1;
                                }
                            }
                        } else if (req_it->addr_vec[m_level_bg] == -1) {
                            for (int bg = 0; bg < m_mro_num_bgs; bg++) {
                                int bank_id = req_it->addr_vec[m_level_bank];
                                int rank_id = req_it->addr_vec[m_level_rank];
                                int flat_bank_id = bank_id + bg * m_mro_num_banks + rank_id * m_mro_num_banks * m_mro_num_bgs;
                                m_row_opened_at[flat_bank_id] = -1;
                            }
                        } else {
                            int rank_id = req_it->addr_vec[m_level_rank];
                            int bg_id   = req_it->addr_vec[m_level_bg];
                            int bank_id = req_it->addr_vec[m_level_bank];
                            int flat_bank_id = bank_id + bg_id * m_mro_num_banks + rank_id * m_mro_num_banks * m_mro_num_bgs;
                            m_row_opened_at[flat_bank_id] = -1;
                        }

                    } else if (m_dram->m_command_meta(req_it->command).is_opening &&
                            m_dram->m_command_scopes(req_it->command) == m_dram->m_levels("row")) {
                        int rank_id = req_it->addr_vec[m_level_rank];
                        int bg_id   = req_it->addr_vec[m_level_bg];
                        int bank_id = req_it->addr_vec[m_level_bank];
                        int flat_bank_id = bank_id + bg_id * m_mro_num_banks + rank_id * m_mro_num_banks * m_mro_num_bgs;
                        m_row_opened_at[flat_bank_id] = m_clk;
                        m_open_addr_vec[flat_bank_id] = req_it->addr_vec;
                    }
                }








                if (m_is_debug) {
                    int c = req_it->command;
                    // print EVERY issued command (ACT/RD/WR/PRE/PREsb/PREA/REF/RFMab/...)
                    auto nm = m_dram->m_commands(c);
                    int ra  = req_it->addr_vec[m_level_rank];
                    int bg  = req_it->addr_vec[m_level_bg];
                    int ba  = req_it->addr_vec[m_level_bank];
                    int row = req_it->addr_vec[m_dram->m_levels("row")];
                    std::printf("[CMD] [%ld] %-5.*s ra=%d bg=%d ba=%d row=%d type_id=%d\n",
                                (long)m_clk, (int)nm.size(), nm.data(), ra, bg, ba, row, req_it->type_id);
                }
                if (req_it->command == req_it->final_command) {
                    // Last command of the request
                    bool is_maintenance = (buffer == &m_prac_buffer);
                    if (!is_maintenance && req_it->type_id == Request::Type::Read) {
                        req_it->depart = m_clk + m_dram->m_read_latency;
                        pending.push_back(*req_it);
                    }
                    buffer->remove(req_it);
                } else if (m_dram->m_command_meta(req_it->command).is_opening) {
                    // Not final, but an opening (ACT) -> move to active buffer to continue
                    if (m_active_buffer.enqueue(*req_it))
                        buffer->remove(req_it);
                }
            }

            if (m_max_row_open_ns > 0) {
                for (int flat_bank_id = 0; flat_bank_id < m_mro_num_banks_total; flat_bank_id++) {
                    if (m_row_opened_at[flat_bank_id] == -1) continue;

                    int64_t open_for = (int64_t)m_clk - m_row_opened_at[flat_bank_id];
                    if (open_for <= (int64_t)m_max_row_open_clk) continue;

                    auto& target = m_open_addr_vec[flat_bank_id];
                    int target_rank = target[m_level_rank];
                    int target_bg   = target[m_level_bg];
                    int target_bank = target[m_level_bank];

                    auto same_bank = [&](const Request& r) {
                        return r.addr_vec[m_level_rank] == target_rank &&
                            r.addr_vec[m_level_bg]   == target_bg   &&
                            r.addr_vec[m_level_bank] == target_bank;
                    };

                    bool pending_same_bank =
                        std::find_if(m_active_buffer.begin(),   m_active_buffer.end(),   same_bank) != m_active_buffer.end();

                    if (pending_same_bank) continue;
                    
                
                    Request pre_req(target, m_PRE_req_id_mro);
                    if (priority_send(pre_req)) {
                        if (m_is_debug) {
                            std::printf("[MaxRowOpen] [%ld] flat_bank=%d forced PRE after %lld clk (cap=%lu)\n",
                                        (long)m_clk, flat_bank_id, (long long)open_for, (unsigned long)m_max_row_open_clk);
                        }
                        m_row_opened_at[flat_bank_id] = -1;
                    }
                    
                }
            }

        }

        void reset_stats() override {
            Implementation::reset_stats();   // zero registered stats + recurse into child plugins
            m_clk_at_warmup = m_clk;         // capture the measurement-window start clock
        }

        void finalize() override{
            // Average read latency over real DRAM reads only (write-buffer hits excluded)
            s_avg_read_latency = s_dram_read_count ? (float)s_read_latency / (float)s_dram_read_count : 0.0f;

            // Queue-length averages over the post-warmup measurement window only
            Clk_t measured_cycles = m_clk - m_clk_at_warmup;
            s_measurement_cycles = (m_clk > m_clk_at_warmup) ? (size_t)measured_cycles : 0;
            if (measured_cycles > 0) {
                s_queue_len_avg          = (float)s_queue_len          / (float)measured_cycles;
                s_read_queue_len_avg     = (float)s_read_queue_len     / (float)measured_cycles;
                s_write_queue_len_avg    = (float)s_write_queue_len    / (float)measured_cycles;
                s_priority_queue_len_avg = (float)s_priority_queue_len / (float)measured_cycles;
            }

            // Finalize plugins
            for (auto* p : m_plugins) {
                auto* impl = dynamic_cast<Implementation*>(p);
                if (impl) impl->finalize();
            }
        }

        // --- Mess request generator hooks (queue inspection) ---
        size_t get_read_queue_length() override    { return m_read_buffer.size() + pending.size(); }
        size_t get_write_queue_length() override   { return m_write_buffer.size(); }
        size_t get_active_buffer_length() override { return m_active_buffer.size() + m_prac_buffer.size(); }

        bool is_req_in_read_queue(const Request& req) override {
            auto same_addr = [&](const Request& r){ return r.addr == req.addr; };
            return std::find_if(m_read_buffer.begin(), m_read_buffer.end(), same_addr) != m_read_buffer.end();
        }

        bool is_req_in_pending_queue(const Request& req) override {
            auto same_addr = [&](const Request& r){ return r.addr == req.addr; };
            if (std::find_if(m_active_buffer.begin(), m_active_buffer.end(), same_addr) != m_active_buffer.end())
                return true;
            return std::find_if(pending.begin(), pending.end(), same_addr) != pending.end();
        }


    private:

        void set_write_mode() {
            if (!m_is_write_mode) {
                if (m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size
                        || m_read_buffer.size() == 0)
                    m_is_write_mode = true;
            } else {
                if (m_write_buffer.size() < m_wr_low_watermark * m_write_buffer.max_size
                        && m_read_buffer.size() != 0)
                    m_is_write_mode = false;
            }
        }

        // A closing command (PRE/PREsb) conflicts if any open row in the active buffer
        // shares the same bank — we must defer it so we don't close a row in use.
        bool active_buffer_conflict(ReqBuffer::iterator req_it) {
            if (!m_dram->m_command_meta(req_it->command).is_closing)
                return false;
            auto& rowgroup = req_it->addr_vec;
            for (auto it = m_active_buffer.begin(); it != m_active_buffer.end(); ++it) {
                bool match = true;
                for (int i = 0; i <= m_bank_addr_idx; i++) {
                    if (it->addr_vec[i] != rowgroup[i]
                            && it->addr_vec[i] != -1 && rowgroup[i] != -1) {
                        match = false;
                        break;
                    }
                }
                if (match) return true;
            }
            return false;
        }

        bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
            bool request_found = false;

            // 1. Active buffer (highest priority — already-opened rows in progress)
            req_it = m_scheduler->get_best_request(m_active_buffer);
            if (req_it != m_active_buffer.end()) {
                bool fits = !m_prac
                    || m_clk + m_prac->min_cycles_with_preall(req_it) < m_prac->next_recovery_cycle();
                if (fits && m_dram->check_ready(req_it->command, req_it->addr_vec)) {
                    req_buffer    = &m_active_buffer;
                    request_found = true;
                }
            }

            if (!request_found) {
                // 2. PRAC buffer (ABO recovery — rank-wide, top priority over normal traffic)
                if (m_prac_buffer.size() != 0) {
                    req_buffer = &m_prac_buffer;
                    req_it     = m_prac_buffer.begin();
                    req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
                    bool is_rfm     = req_it->command == m_dram->m_commands("RFMab");
                    bool is_pre_rec = m_prac && m_prac->get_state() == IPRAC::ABOState::PRE_RECOVERY;
                    bool is_ready   = m_dram->check_ready(req_it->command, req_it->addr_vec);
                    request_found   = (!is_rfm || !is_pre_rec) && is_ready;
                    if (!request_found && m_prac_buffer.size() != 0)
                        return false;   // PRAC waiting → block everything else this cycle
                }

                // 3. Priority buffer (RefreshManager maintenance) (periodic refresh)
                if (!request_found && m_priority_buffer.size() != 0) {
                    req_buffer = &m_priority_buffer;
                    req_it     = m_priority_buffer.begin();
                    req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
                    bool fits = !m_prac
                        || m_clk + m_prac->min_cycles_with_preall(req_it) < m_prac->next_recovery_cycle();
                    request_found = fits && m_dram->check_ready(req_it->command, req_it->addr_vec);
                    if (!request_found && m_priority_buffer.size() != 0) {
                        return false;
                    }
                }

                // 4. Read / write buffer (normal traffic)
                if (!request_found) {
                    set_write_mode();
                    auto& buf = m_is_write_mode ? m_write_buffer : m_read_buffer;
                    req_it = m_scheduler->get_best_request(buf);
                    if (req_it != buf.end()) {
                        bool fits = !m_prac
                            || m_clk + m_prac->min_cycles_with_preall(req_it) < m_prac->next_recovery_cycle();
                        if (fits && m_dram->check_ready(req_it->command, req_it->addr_vec)) {
                            req_buffer    = &buf;
                            request_found = true;
                        }
                    }
                }
            }

            // Defer a closing command that would close a row still open in the active buffer
            if (request_found && active_buffer_conflict(req_it)) {
                request_found = false;
            }

            // Per-core + aggregate row hit/miss/conflict tracking (skip active buffer; counted on first issue)
            if (request_found && req_buffer != &m_active_buffer) {
                if (req_it->type_id == Request::Type::Read || req_it->type_id == Request::Type::Write) {
                    auto& req_meta = m_dram->m_command_meta(req_it->command);
                    int source_id = req_it->source_id >= 0 ? req_it->source_id : 0;
                    int increment = req_it->source_id >= 0 ? 1 : 0;
                    bool core_done = increment && m_frontend && m_frontend->is_core_finished(source_id);
                    if (req_meta.is_accessing) { if (!core_done) s_core_row_hits[source_id]      += increment; s_num_row_hits++; }
                    if (req_meta.is_opening)   { if (!core_done) s_core_row_misses[source_id]    += increment; s_num_row_misses++; }
                    if (req_meta.is_closing)   { if (!core_done) s_core_row_conflicts[source_id] += increment; s_num_row_conflicts++; }
                }
            }

            return request_found;
        }

        void serve_completed_reads(){
            if(!pending.empty() && pending[0].depart <= m_clk){
                auto& req = pending[0];
                size_t lat = req.depart - req.arrive;

                if (!req.from_write_buffer){
                    s_dram_read_count++;
                    s_read_latency += lat;
                }
                if (req.callback) {
                    // If the request comes from outside (e.g., processor), call its callback
                    req.callback(req);
                }
                pending.pop_front();
            }
        }


        Clk_t oldest_pending_age() const override {
            bool found = false;
            Clk_t oldest_arrive = 0;

            auto consider = [&](Clk_t arrive) {
                if (!found || arrive < oldest_arrive) { oldest_arrive = arrive; found = true; }
            };

            if (!pending.empty())                  consider(pending.front().arrive);
            if (!m_active_buffer.buffer.empty())   consider(m_active_buffer.buffer.front().arrive);
            if (!m_prac_buffer.buffer.empty())     consider(m_prac_buffer.buffer.front().arrive);
            if (!m_priority_buffer.buffer.empty()) consider(m_priority_buffer.buffer.front().arrive);
            if (!m_read_buffer.buffer.empty())     consider(m_read_buffer.buffer.front().arrive);
            if (!m_write_buffer.buffer.empty())    consider(m_write_buffer.buffer.front().arrive);

            return found ? (m_clk - oldest_arrive) : 0;
        }




};
}