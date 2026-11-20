#ifndef     RAMULATOR_MEMORYSYSTEM_MEMORY_H
#define     RAMULATOR_MEMORYSYSTEM_MEMORY_H

#include <map>
#include <vector>
#include <string>
#include <functional>

#include "base/base.h"
#include "frontend/frontend.h"
#include "dram/dram.h"

namespace Ramulator {

class IMemorySystem : public TopLevel<IMemorySystem> {
  RAMULATOR_REGISTER_INTERFACE(IMemorySystem, "MemorySystem", "Memory system interface (e.g., communicates between processor and memory controller).")

  friend class Factory;

  protected:
    IFrontEnd* m_frontend;
    uint m_clock_ratio = 1;

  public:
    virtual void connect_frontend(IFrontEnd* frontend) { 
      m_frontend = frontend; 
      m_impl->setup(frontend, this);
      for (auto component : m_components) {
        component->setup(frontend, this);
      }
    };

    virtual void reset_all_stats() {
      m_impl->reset_stats();
      for (auto component : m_components) {
        component->reset_stats();
      }
      // Reset DRAM power counters (cmd_counters, active/idle cycles)
      for (auto component : m_components) {
        IDRAM* dram = dynamic_cast<IDRAM*>(component);
        if (dram) { dram->reset_power_counters(); break; }
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

    /**
     * @brief         Tries to send the request to the memory system
     * 
     * @param    req      The request
     * @return   true     Request is accepted by the memory system.
     * @return   false    Request is rejected by the memory system, maybe the memory controller is full?
     */
    virtual bool send(Request req) = 0;

    /**
     * @brief         Ticks the memory system
     * 
     */
    virtual void tick() = 0;

    /**
     * @brief    Returns 
     * 
     * @return   int 
     */
    int get_clock_ratio() { return m_clock_ratio; };

    // /**
    //  * @brief    Get the integer id of the request type from the memory spec
    //  * 
    //  */
    // virtual const SpecDef& get_supported_requests() = 0;

    virtual float get_tCK() { return -1.0f; };

    // --- Mess request generator hooks. Defaulted (not pure-virtual) so other
    //     memory-system implementations (bh_DRAM, dummy) are unaffected. Only
    //     GenericDRAM overrides them. ---
    virtual bool is_finished_ms() { return true; }
    virtual bool is_request_finished(Request req) { return true; }
    virtual int get_total_address_bits() { return 0; }
    virtual int get_shift_amt(int idx) { return -1; }
    virtual size_t get_max(int idx) { return 0; }
    virtual int get_num_channels() { return 1; }
};

}        // namespace Ramulator


#endif   // RAMULATOR_MEMORYSYSTEM_MEMORY_H