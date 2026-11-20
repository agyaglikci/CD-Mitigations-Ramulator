#include <functional>
#include <iostream>
#include <filesystem>

#include "base/utils.h"
#include "frontend/frontend.h"
#include "translation/translation.h"
#include "frontend/impl/processor/simpleO3/core.h"
#include "frontend/impl/processor/simpleO3/llc.h"


namespace Ramulator {

class SimpleO3 final : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, SimpleO3, "SimpleO3", "Simple timing model OoO processor frontend.")

  private:
    ITranslation*  m_translation;

    int m_num_cores = -1;
    std::vector<SimpleO3Core*> m_cores;
    SimpleO3LLC* m_llc;

    // Cores flagged as attackers are excluded from the is_finished() check, so
    // the simulation ends once all benign cores complete their instructions and
    // the attacker free-runs throughout (BreakHammer evaluation methodology).
    std::vector<bool> m_core_is_attacker;

    size_t m_num_expected_insts = 0;
    size_t m_num_warmup_insts = 0;

    bool llc_serialize = false;
    std::string llc_serialization_filename;
    bool llc_deserialize = false;
    std::string llc_deserialization_filename;


  public:
    void init() override {
      m_clock_ratio = param<uint>("clock_ratio").required();
      
      // Core params
      std::vector<std::string> trace_list = param<std::vector<std::string>>("traces").desc("A list of traces.").required();
      m_num_cores = trace_list.size();

      const std::vector<int> no_attackers{};
      std::vector<int> attacker_cores =
          param<std::vector<int>>("attacker_cores")
              .desc("Core ids running an attacker trace. These cores are "
                    "excluded from is_finished(), so the run ends when all "
                    "benign cores complete and the attacker free-runs "
                    "throughout (BreakHammer methodology).")
              .default_val(no_attackers);
      m_core_is_attacker.assign(m_num_cores, false);
      for (int cid : attacker_cores) {
        if (cid >= 0 && cid < m_num_cores) m_core_is_attacker[cid] = true;
      }

      int ipc   = param<int>("ipc").desc("IPC of the SimpleO3 core.").default_val(4);
      int depth = param<int>("inst_window_depth").desc("Instruction window size of the SimpleO3 core.").default_val(128);

      // LLC params
      int llc_latency           = param<int>("llc_latency").desc("Aggregated latency of the LLC.").default_val(47);
      int llc_linesize_bytes    = param<int>("llc_linesize").desc("LLC cache line size in bytes.").default_val(64);
      int llc_associativity     = param<int>("llc_associativity").desc("LLC set associativity.").default_val(8);
      int llc_capacity_per_core = parse_capacity_str(param<std::string>("llc_capacity_per_core").desc("LLC capacity per core.").default_val("2MB"));
      int llc_num_mshr_per_core = param<int>("llc_num_mshr_per_core").desc("Number of LLC MSHR entries per core.").default_val(16);
      llc_serialize = param<bool>("llc_serialize").desc("Whether to serialize the LLC.").default_val(false);
      llc_serialization_filename = param<std::string>("llc_serialization_filename").desc("Filename to serialize the LLC.").default_val("llc_serialization");
      llc_deserialize = param<bool>("llc_deserialize").desc("Whether to deserialize the LLC.").default_val(false);
      llc_deserialization_filename = param<std::string>("llc_deserialization_filename").desc("Filename to deserialize the LLC.").default_val("llc_serialization");

      // Simulation parameters
      m_num_expected_insts = param<int>("num_expected_insts").desc("Number of instructions that the frontend should execute.").required();
      m_num_warmup_insts = param<int>("num_warmup_insts").desc("Number of warmup instructions before measurement begins.").default_val(0);
      if (m_num_warmup_insts == 0) {
        set_warmup_done();
      }

      // Create address translation module
      m_translation = create_child_ifce<ITranslation>();

      // Create the LLC
      m_llc = new SimpleO3LLC(llc_latency, llc_capacity_per_core * m_num_cores, llc_linesize_bytes, llc_associativity, llc_num_mshr_per_core * m_num_cores);
      m_llc->initialize_breakhammer_state(m_num_cores);
      if (llc_deserialize) {
        if (!std::filesystem::exists(llc_deserialization_filename)) {
          throw std::runtime_error("LLC deserialization file not found.");
        }
        m_llc->deserialize(llc_deserialization_filename);
      }

      // Create the cores
      for (int id = 0; id < m_num_cores; id++) {
        SimpleO3Core* core = new SimpleO3Core(id, ipc, depth, m_num_expected_insts, trace_list[id], m_translation, m_llc);
        core->m_callback = [this](Request& req){return this->receive(req);} ;
        core->m_measuring = (m_num_warmup_insts == 0);
        m_cores.push_back(core);
      }

      m_logger = Logging::create_logger("SimpleO3");

      // Register the stats
      register_stat(m_llc->s_llc_eviction).name("llc_eviction");
      register_stat(m_llc->s_llc_read_access).name("llc_read_access");
      register_stat(m_llc->s_llc_write_access).name("llc_write_access");
      register_stat(m_llc->s_llc_read_misses).name("llc_read_misses");
      register_stat(m_llc->s_llc_write_misses).name("llc_write_misses");
      register_stat(m_llc->s_llc_mshr_unavailable).name("llc_mshr_unavailable");
      register_stat(m_llc->s_llc_mshr_blacklisted).name("llc_mshr_blacklisted");
      register_stat(m_llc->s_llc_attacker_flushes).name("llc_attacker_flushes");
      
      for (int core_id = 0; core_id < m_cores.size(); core_id++) {
        // register_stat(m_cores[core_id]->s_insts_retired).name("cycles_retired_core_{}", core_id);
        register_stat(m_cores[core_id]->s_cycles_recorded).name("cycles_recorded_core_{}", core_id);
        register_stat(m_cores[core_id]->s_mem_access_cycles).name("memory_access_cycles_recorded_core_{}", core_id);
      }
    }

    void tick() override {
      m_clk++;

      if(m_clk % 10000000 == 0) {
        size_t total_retired = 0;
        for (size_t core_id = 0; core_id < m_cores.size(); core_id++) {
          size_t retired = m_cores[core_id]->s_insts_retired;
          total_retired += retired;
          m_logger->info("Processor Heartbeat {} cycles, core {} retired {} instructions.", m_clk, core_id, retired);
        }
        m_logger->info("Processor Heartbeat {} cycles, total retired {} instructions.", m_clk, total_retired);
      }

      // Check if warmup is done — every core must individually reach num_warmup_insts.
      // All cores keep running; reset fires when the slowest core crosses the threshold.
      // s_insts_retired is reset to 0 so num_expected_insts counts measurement-only instructions.
      if (!m_warmup_done && m_num_warmup_insts > 0) {
        bool all_warmed_up = true;
        for (size_t i = 0; i < m_cores.size(); i++) {
          if (i < m_core_is_attacker.size() && m_core_is_attacker[i]) continue;
          if (m_cores[i]->s_insts_retired < m_num_warmup_insts) {
            all_warmed_up = false;
            break;
          }
        }
        if (all_warmed_up) {
          set_warmup_done();
          for (auto core : m_cores) {
            core->m_warmup_cycle_offset = core->m_clk;
            core->reached_expected_num_insts = false;
            core->m_last_mem_cycle = 0;
            core->s_insts_retired = 0;
            core->m_measuring = true;
          }
          m_logger->info("Warmup done at {} cycles.", m_clk);
#ifdef WARMUP_TEST
          fprintf(stderr, "[WARMUP_TEST] warmup_done at cycle=%lu\n", m_clk);
#endif
        }
      }

      m_llc->tick();
      for (auto core : m_cores) {
        core->tick();
      }
    }

    void receive(Request& req) {
      m_llc->receive(req);

      // TODO: LLC latency for the core to receive the request?
      for (auto r : m_llc->m_receive_requests[req.addr]) {
        r.arrive = req.arrive;   // copy DRAM-set arrive/depart so core can track memory latency
        r.depart = req.depart;
        m_cores[r.source_id]->receive(r);
        // Attacker flushes the line after each access (load + clflush loop).
        if (r.source_id < (int)m_core_is_attacker.size() && m_core_is_attacker[r.source_id])
          m_llc->clflush(r.addr);
      }
      m_llc->m_receive_requests[req.addr].clear();
    };

    void finalize() override {
      for (auto core : m_cores) {
        if (core->m_measuring && !core->reached_expected_num_insts)
          core->s_cycles_recorded = core->m_clk - core->m_warmup_cycle_offset;
      }
      IFrontEnd::finalize();
    }

    bool is_finished() override {
      for (size_t i = 0; i < m_cores.size(); i++) {
        // Do not wait for attacker cores; they free-run until the benign cores
        // finish, matching the BreakHammer evaluation methodology.
        if (i < m_core_is_attacker.size() && m_core_is_attacker[i]) continue;
        if (!(m_cores[i]->reached_expected_num_insts)){
          return false;
        }
      }
      if (llc_serialize)
        m_llc->serialize(llc_serialization_filename);
      return true;
    }

    void connect_memory_system(IMemorySystem* memory_system) override {
      m_memory_system = memory_system;
      m_impl->setup(this, memory_system);
      for (auto component : m_components) {
        component->setup(this, memory_system);
      }
      m_llc->connect_memory_system(memory_system);
    };

    int get_num_cores() override {
      return m_num_cores;
    };

    bool is_core_finished(int core_id) override {
      if (core_id < 0 || core_id >= (int)m_cores.size()) return false;
      return m_cores[core_id]->reached_expected_num_insts;
    };

    bool apply_breakhammer_throttle(int source_id, int max_mshrs) override {
      return m_llc->apply_source_mshr_limit(source_id, max_mshrs);
    }

    bool release_breakhammer_throttle(int source_id) override {
      return m_llc->release_source_mshr_limit(source_id);
    }

    // BH Changes Begin
    SimpleO3LLC* get_llc() {
      return m_llc;
    }
    // BH Changes End
};

}        // namespace Ramulator
