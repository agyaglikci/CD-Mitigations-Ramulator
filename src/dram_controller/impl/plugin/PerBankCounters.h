#pragma once
#include "dram_controller/impl/pluginutil/device_config.h"
#include <algorithm>
#include <limits>
#include <vector>
#include <functional>
#include <random>
#include <unordered_map>

enum class CounterType {
    SINGLE,
    DUAL,
    PROBABILISTIC,
    NONE
};

union trigger_thresh {
    int int_mtg; // for deterministic implementation
    float float_mtg; // for probabilistic implementation
};

namespace Ramulator {

    class PerBankCounters{
        protected:

            struct CommandHandler {
                std::string cmd_name;
                std::function<void(const Request&)> handler;
            };

            std::unordered_map<int, CommandHandler> m_handlertable;
            std::vector<uint32_t> m_rolling_pointers;
            DeviceConfig &m_cfg;
            const int m_brc = 2;


        public:
            PerBankCounters(DeviceConfig &cfg);
            ~PerBankCounters() {return;};
            void on_request(const Request& req);
            int get_and_inc_rolling_pointer(int subarray_id);

            void init_dram_params(IDRAM* dram);
            virtual bool need_progress(int subarray_id) const = 0;
            virtual void zero_counters(int subarray_id) = 0;
            virtual void process_act(const Request& req, int activations) = 0;
            virtual void process_rfm(const Request& req) = 0;
            virtual uint32_t get_count(int subarray_id) const { return 0; }
    };

    class NoneCounters: public PerBankCounters {
        public:
            NoneCounters(DeviceConfig &cfg) : PerBankCounters(cfg) {return;};
            ~NoneCounters();
            bool need_progress(int subarray_id) const {return false;};
            void zero_counters(int subarray_id) override {return;};
            void process_act(const Request& req, int activations) override {return;};
            void process_rfm(const Request& req) override {return;};
    };

    class SingleCounters: public PerBankCounters {

        private:
            std::vector<uint32_t> m_counters;
            int m_alert_thresh;

        public:
            SingleCounters(DeviceConfig &cfg, int alert_thresh);
            ~SingleCounters();
            uint32_t get_count(int subarray_id) const override { return m_counters[subarray_id]; }
            bool need_progress(int subarray_id) const override;
            void zero_counters(int subarray_id) override;
            void process_act(const Request& req, int activations) override;
            void process_rfm(const Request& req) override;
    };

    class DualCounters: public PerBankCounters {

        private:
            std::vector<uint32_t> m_counters_odd;
            std::vector<uint32_t> m_counters_even;
            int m_alert_thresh;

        public:
            DualCounters(DeviceConfig &cfg, int alert_thresh);
            ~DualCounters();
            uint32_t get_count(int subarray_id) const override { return std::max(m_counters_odd[subarray_id], m_counters_even[subarray_id]); }
            bool need_progress(int subarray_id) const override;
            void zero_counters(int subarray_id) override;
            void process_act(const Request& req, int activations) override;
            void process_rfm(const Request& req) override;
    };

    class ProbabilisticCounters: public PerBankCounters {

        private:
            std::vector<bool> m_need_progress;
            double m_progress_probability;
            static std::mt19937 generator;
            static std::uniform_real_distribution<double> distribution;

        public:
            ProbabilisticCounters(DeviceConfig &cfg, double progress_probability);
            ~ProbabilisticCounters();
            bool need_progress(int subarray_id) const override;
            void zero_counters(int subarray_id) override;
            void process_act(const Request& req, int activations) override;
            void process_rfm(const Request& req) override;
    };
}

