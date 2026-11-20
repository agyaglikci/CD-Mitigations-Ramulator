#ifndef RAMULATOR_CONTROLLER_CONTROLLER_H
#define RAMULATOR_CONTROLLER_CONTROLLER_H

#include <vector>
#include <deque>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "base/base.h"
#include "dram/dram.h"
#include "dram_controller/scheduler.h"
#include "dram_controller/plugin.h"
#include "dram_controller/refresh.h"
#include "dram_controller/rowpolicy.h"


namespace Ramulator {

class IDRAMController : public Clocked<IDRAMController> {
  RAMULATOR_REGISTER_INTERFACE(IDRAMController, "Controller", "Memory Controller Interface");

  public:
    IDRAM*  m_dram = nullptr;          
    IScheduler*   m_scheduler = nullptr;
    IRefreshManager*   m_refresh = nullptr;
    IRowPolicy*   m_rowpolicy = nullptr;
    std::vector<IControllerPlugin*> m_plugins;

    int m_channel_id = -1;
  public:
    /**
     * @brief       Send a request to the memory controller.
     * 
     * @param    req        The request to be enqueued.
     * @return   true       Successful.
     * @return   false      Failed (e.g., buffer full).
     */
    virtual bool send(Request& req) = 0;

    /**
     * @brief       Send a high-priority request to the memory controller.
     * 
     */
    virtual bool priority_send(Request& req) = 0;

    /**
     * @brief       Ticks the memory controller.
     * 
     */
    virtual void tick() = 0;

    /**
     * @brief       Finalizes the memory controller.
    */
    virtual void finalize() = 0;

    // --- Mess request generator hooks (queue inspection). Defaulted so
    //     controllers that don't need them are unaffected. ---
    virtual size_t get_read_queue_length()    { return 0; }
    virtual size_t get_write_queue_length()   { return 0; }
    virtual size_t get_active_buffer_length() { return 0; }
    virtual bool is_req_in_read_queue(const Request& req)    { return false; }
    virtual bool is_req_in_pending_queue(const Request& req) { return false; }
    virtual Clk_t oldest_pending_age() const { return 0; }
    void check_livelock_watchdog(Clk_t age, Clk_t threshold) {
      if (age >= threshold) {
        std::cout << "[Ramulator::Controller] [CRITICAL ERROR] channel " << m_channel_id
                  << ": a request has been waiting " << age << " cycles (threshold=" << threshold
                  << ", clk=" << m_clk << "). Likely a runaway refresh/mitigation loop starving "
                  << "real requests. Aborting." << std::endl;
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        m_impl->print_stats(emitter);
        emitter << YAML::EndMap;
        std::cout << emitter.c_str() << std::endl;
        exit(1);
      }
}

};

}       // namespace Ramulator

#endif  // RAMULATOR_CONTROLLER_CONTROLLER_H