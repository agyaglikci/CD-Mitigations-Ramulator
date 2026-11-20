#ifndef     RAMULATOR_FRONTEND_FRONTEND_H
#define     RAMULATOR_FRONTEND_FRONTEND_H

#include <vector>
#include <string>
#include <functional>

#include "base/base.h"
#include "memory_system/memory_system.h"


namespace Ramulator {

class IFrontEnd : public Clocked<IFrontEnd>, public TopLevel<IFrontEnd> {
  RAMULATOR_REGISTER_INTERFACE(IFrontEnd, "Frontend", "The frontend that drives the simulation.");

  friend class Factory;

  protected:
    IMemorySystem* m_memory_system;
    uint m_clock_ratio = 1;

  public:
    virtual void connect_memory_system(IMemorySystem* memory_system) { 
      m_memory_system = memory_system; 
      m_impl->setup(this, memory_system);
      for (auto component : m_components) {
        component->setup(this, memory_system);
      }
    };

    virtual bool is_finished() = 0;

    // Warmup support
    bool m_warmup_done = false;
    bool is_warmup_done() const { return m_warmup_done; }
    void set_warmup_done() { m_warmup_done = true; }

    virtual void reset_all_stats() {
      m_impl->reset_stats();
      for (auto component : m_components) {
        component->reset_stats();
      }
    }

    virtual void finalize() {
      for (auto component : m_components) {
        component->finalize();
      }

      YAML::Emitter emitter;
      emitter << YAML::BeginMap;
      m_impl->print_stats(emitter);
      emitter << YAML::EndMap;
      std::cout << emitter.c_str() << std::endl;
    };

    virtual int get_num_cores() { return 1; };

    virtual bool is_core_finished(int core_id) { return false; };

    // BreakHammer-style bandwidth control. Frontends that support per-source
    // MSHR limiting override these hooks; other frontends reject the request.
    virtual bool apply_breakhammer_throttle(int source_id, int max_mshrs) { return false; }
    virtual bool release_breakhammer_throttle(int source_id) { return false; }

    int get_clock_ratio() { return m_clock_ratio; };

    /**
     * @brief    Receives memory requests from external sources (e.g., coming from a full system simulator like GEM5)
     * 
     * @details
     * This functions should take memory requests from external sources (e.g., coming from GEM5), generate Ramulator 2 Requests,
     * (tries to) send to the memory system, and return if this is successful
     * 
     */
    virtual bool receive_external_requests(int req_type_id, Addr_t addr, int source_id, std::function<void(Request&)> callback) { return false; }
};

}        // namespace Ramulator


#endif   // RAMULATOR_FRONTEND_FRONTEND_H
