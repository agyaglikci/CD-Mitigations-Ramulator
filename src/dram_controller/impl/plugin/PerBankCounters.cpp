#include "dram_controller/impl/plugin/PerBankCounters.h"

namespace Ramulator {

    PerBankCounters::PerBankCounters(DeviceConfig &cfg) : m_cfg(cfg) {
        m_rolling_pointers.resize(m_cfg.m_num_subarrays_per_bank, 0);
    }

    SingleCounters::SingleCounters(DeviceConfig &cfg, int alert_thresh): PerBankCounters(cfg), m_alert_thresh(alert_thresh) {
        m_counters.resize(m_cfg.m_num_subarrays_per_bank, 0);
        init_dram_params(m_cfg.m_dram);
    }

    DualCounters::DualCounters(DeviceConfig &cfg, int alert_thresh): PerBankCounters(cfg), m_alert_thresh(alert_thresh) {
        m_counters_odd.resize(m_cfg.m_num_subarrays_per_bank, 0);
        m_counters_even.resize(m_cfg.m_num_subarrays_per_bank, 0);
        init_dram_params(m_cfg.m_dram);
    }

    ProbabilisticCounters::ProbabilisticCounters(DeviceConfig &cfg, double progress_probability): PerBankCounters(cfg), m_progress_probability(progress_probability) {
        m_need_progress.resize(m_cfg.m_num_subarrays_per_bank, false);
        init_dram_params(m_cfg.m_dram);
    }

    NoneCounters::~NoneCounters() { return; }

    SingleCounters::~SingleCounters() { return; }

    DualCounters::~DualCounters() { return; }

    ProbabilisticCounters::~ProbabilisticCounters() { return; }

    void PerBankCounters::on_request(const Request& req) {
        if (m_handlertable.find(req.command) != m_handlertable.end())
            m_handlertable[req.command].handler(req);
    }

    int PerBankCounters::get_and_inc_rolling_pointer(int subarray_id) {
        m_rolling_pointers[subarray_id] = (m_rolling_pointers[subarray_id] + 1) % m_cfg.m_num_rows_per_subarray;
        if (m_cfg.is_salp)
            return m_rolling_pointers[subarray_id];
        else
            return m_rolling_pointers[subarray_id] + subarray_id * m_cfg.m_num_rows_per_subarray;
        
    }

    void PerBankCounters::init_dram_params(IDRAM* dram) {

        CommandHandler handlers[] = {
            {std::string("ACT"), std::bind(&PerBankCounters::process_act, this, std::placeholders::_1, 1)},
            {std::string("VRR"), std::bind(&PerBankCounters::process_act, this, std::placeholders::_1, 4)},
            {std::string("RFMsb"), std::bind(&PerBankCounters::process_rfm, this, std::placeholders::_1)},
            {std::string("RFMab"), std::bind(&PerBankCounters::process_rfm, this, std::placeholders::_1)}

        };

        for (auto& h : handlers) {
            if (!dram->m_commands.contains(h.cmd_name))
                std::cout << "[PRAC] Command " << h.cmd_name << "does not exist. Hope you know what you are doing!" << std::endl;
            else 
                m_handlertable[dram->m_commands(h.cmd_name)] = h;
        }
    }

    void SingleCounters::process_act(const Request& req, int activations) {
        if (m_alert_thresh < 0) return;
        auto subarray_id = m_cfg.get_subarray_id(req);

        m_counters[subarray_id] += activations;
        if (subarray_id > 0) m_counters[subarray_id - 1] += activations;
        if (subarray_id < (m_cfg.m_num_subarrays_per_bank - 1)) m_counters[subarray_id + 1] += activations;
    }

    void SingleCounters::process_rfm(const Request& req) {
        if (m_alert_thresh < 0) return;
        int increment = 2 * m_brc;
        for (int i = 0; i < m_cfg.m_num_subarrays_per_bank; i++)
            m_counters[i] += increment;
    }

    void DualCounters::process_act(const Request& req, int activations) {
        if (m_alert_thresh < 0) return;
        auto subarray_id = m_cfg.get_subarray_id(req);
        
        m_counters_odd[subarray_id] += activations;
        m_counters_even[subarray_id] += activations;
        
        if (subarray_id > 0) m_counters_odd[subarray_id - 1] += activations;
        if (subarray_id < (m_cfg.m_num_subarrays_per_bank - 1)) m_counters_even[subarray_id + 1] += activations;
    }

    void DualCounters::process_rfm(const Request& req) {
        if (m_alert_thresh < 0) return;
        int increment = 2 * m_brc;
        for (int i = 0; i < m_cfg.m_num_subarrays_per_bank; i++) {
            m_counters_odd[i] += increment;
            m_counters_even[i] += increment;
        }
    }

    void ProbabilisticCounters::process_act(const Request& req, int activations) {
        if (m_progress_probability <= 0.0) return;
        auto subarray_id = m_cfg.get_subarray_id(req);
        bool result = false;
        for (int i = 0; i < activations; i++)
            result = result || ProbabilisticCounters::distribution(ProbabilisticCounters::generator) < m_progress_probability;
        m_need_progress[subarray_id] = result;
        if (subarray_id > 0) 
            m_need_progress[subarray_id - 1] = result || m_need_progress[subarray_id - 1];
        if (subarray_id < (m_cfg.m_num_subarrays_per_bank - 1)) 
            m_need_progress[subarray_id + 1] = result || m_need_progress[subarray_id + 1];
    }

    void ProbabilisticCounters::process_rfm(const Request& req) {
        if (m_progress_probability <= 0.0) return;
        int samples = 2 * m_brc;
        for (int i = 0; i < m_cfg.m_num_subarrays_per_bank; i++) {
            bool result = false;
            for (int j = 0; j < samples; j++)
                result = result || ProbabilisticCounters::distribution(ProbabilisticCounters::generator) < m_progress_probability;
            m_need_progress[i] = result || m_need_progress[i];
        }
    }

    bool SingleCounters::need_progress(int subarray_id) const {
        if (m_alert_thresh < 0) return false;
        return m_counters[subarray_id] >= m_alert_thresh;
    }

    bool DualCounters::need_progress(int subarray_id) const {
        if (m_alert_thresh < 0) return false;
        return std::max(m_counters_odd[subarray_id], m_counters_even[subarray_id]) >= m_alert_thresh;
    }

    bool ProbabilisticCounters::need_progress(int subarray_id) const {
        return m_need_progress[subarray_id];
    }

    void SingleCounters::zero_counters(int subarray_id) {
        if (m_alert_thresh < 0) return;
        if (m_counters[subarray_id] > m_alert_thresh)
            m_counters[subarray_id] -= m_alert_thresh;
        else
            m_counters[subarray_id] = 0;
    }

    void DualCounters::zero_counters(int subarray_id) {
        if (m_alert_thresh < 0) return;
        
        if (m_counters_odd[subarray_id] > m_alert_thresh)
            m_counters_odd[subarray_id] -= m_alert_thresh;
        else
            m_counters_odd[subarray_id] = 0;

        if (m_counters_even[subarray_id] > m_alert_thresh)
            m_counters_even[subarray_id] -= m_alert_thresh;
        else
            m_counters_even[subarray_id] = 0;
    }

    void ProbabilisticCounters::zero_counters(int subarray_id) {
        m_need_progress[subarray_id] = false;
    }

    std::mt19937 ProbabilisticCounters::generator(std::random_device{}());
    std::uniform_real_distribution<double> ProbabilisticCounters::distribution(0.0, 1.0);
}

