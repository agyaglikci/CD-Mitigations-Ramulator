#ifndef RAMULATOR_PLUGIN_PRAC_H_
#define RAMULATOR_PLUGIN_PRAC_H_

#include "dram/dram.h"

#include <functional>
#include <cstdint>
#include <unordered_map>
#include <vector>

#define _CYCLES(timing_name)    dram->m_timing_vals(timing_name)
#define _COMMAND(command_name)  dram->m_commands(command_name)

namespace Ramulator {

class IPRAC {
public:
    enum class ABOState {
        NORMAL,
        PRE_RECOVERY,
        RECOVERY,
        DELAY
    };

    enum class ABOTrigger : uint32_t {
        None          = 0,
        Chronus       = 1u << 0,
        SALTC         = 1u << 2,
        External      = 1u << 3,
        All        = (1u << 0) | (1u << 2) | (1u << 3)
    };

    using ABOEventCallback =
        std::function<void(uint32_t /* trigger mask */, int /* flat bank */)>;

public:
    virtual Clk_t next_recovery_cycle() = 0;
    virtual int get_num_abo_recovery_refs() = 0;
    virtual ABOState get_state() = 0;
    virtual uint32_t get_act_count(int flat_bank_id, int row_id) = 0;

    // Assert ABO from an external source (e.g. a ColumnDisturb mitigation's ATH crossing).
    virtual void set_abo_needed(bool val) = 0;
    virtual void set_abo_needed(bool val, ABOTrigger trigger, int flat_bank_id) {
        set_abo_needed(val);
    }

    // Called once when an actual ABO recovery starts. Implementations that do
    // not expose trigger attribution may leave this as a no-op.
    virtual void set_abo_event_callback(ABOEventCallback callback) {}

    // True when in PRE_RECOVERY or RECOVERY (consuming RFM budget).
    virtual bool is_abo_active() const = 0;

    // Register a callback that returns true while another plugin (e.g. SALT-C or
    // in-DRAM ColumnKeeper) still has rows to mitigate, keeping ABO recovery alive.
    virtual void set_pending_work_fn(std::function<bool()> fn) = 0;

    void init_dram_params(IDRAM* dram) {
        auto write_to_pre_timing = _CYCLES("nCWL") + _CYCLES("nBL") + _CYCLES("nWR");
        read_cycles = _CYCLES("nRAS") + _CYCLES("nRTP") + _CYCLES("nRP");
        write_cycles = _CYCLES("nRAS") + write_to_pre_timing + _CYCLES("nRP");
        cmd_to_min_cycles[_COMMAND("ACT")] = _CYCLES("nRC");
        cmd_to_min_cycles[_COMMAND("RD")] = _CYCLES("nRTP") + _CYCLES("nRP");
        cmd_to_min_cycles[_COMMAND("WR")] = write_to_pre_timing + _CYCLES("nRP");
        cmd_to_min_cycles[_COMMAND("RFMsb")] = _CYCLES("nRFMsb");
        cmd_to_min_cycles[_COMMAND("RFMab")] = _CYCLES("nRFM1");
        cmd_to_min_cycles[_COMMAND("REFsb")] = _CYCLES("nRFCsb");
        cmd_to_min_cycles[_COMMAND("REFab")] = _CYCLES("nRFC1");
    }

    // TODO: Get these vrom m_timing_cons...
    int min_cycles_with_preall(const ReqBuffer::iterator& req) {
        return min_cycles_with_preall(*req);
    }

    int min_cycles_with_preall(const Request& req) {
        if (cmd_to_min_cycles.find(req.command) == cmd_to_min_cycles.end()) {
            return 0;
        }
        return cmd_to_min_cycles[req.command];
    }

private:
    std::unordered_map<int, int> cmd_to_min_cycles;
    int write_cycles = -1;
    int read_cycles = -1;

};      //  class IPRAC

}       //  namespace Ramulator

#endif // RAMULATOR_PLUGIN_PRAC_H_
