#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/plugin/prac.h"
#include "dram_controller/impl/plugin/throttleable.h"
#include "dram_controller/impl/pluginutil/device_config.h"
#include "frontend/frontend.h"

namespace Ramulator {

class ABOBreakHammerBridge : public IControllerPlugin,
                             public Implementation,
                             public IABOThrottleable {
    RAMULATOR_REGISTER_IMPLEMENTATION(
        IControllerPlugin, ABOBreakHammerBridge, "ABOBreakHammerBridge",
        "Attributes selected ABO events to BreakHammer.")

private:
    DeviceConfig m_cfg;
    IPRAC* m_prac = nullptr;
    IFrontEnd* m_frontend = nullptr;

    std::vector<std::string> m_source_names;
    uint32_t m_source_mask = 0;
    bool m_enabled = false;
    bool m_debug = false;

    uint64_t s_abo_seen = 0;
    uint64_t s_abo_matched = 0;
    uint64_t s_abo_chronus = 0;
    uint64_t s_abo_saltc = 0;
    uint64_t s_abo_external = 0;

    std::vector<int> m_ready_events;
    std::vector<int> m_deferred_events;

public:
    void init() override {
        const std::vector<std::string> default_sources{"all"};
        m_source_names =
            param<std::vector<std::string>>("abo_sources")
                .desc("ABO sources observed by BreakHammer: all, chronus, "
                      "saltc, external")
                .default_val(default_sources);
        m_enabled = param<bool>("enabled").default_val(false);
        m_debug = param<bool>("debug").default_val(false);
        m_source_mask = parse_source_mask(m_source_names);
    }

    void setup(IFrontEnd* frontend, IMemorySystem*) override {
        m_frontend = frontend;
        auto* ctrl = cast_parent<IDRAMController>();
        m_cfg.set_device(ctrl);

        for (auto* plugin : ctrl->m_plugins) {
            if (auto* prac = dynamic_cast<IPRAC*>(plugin)) {
                m_prac = prac;
                break;
            }
        }
        if (!m_prac) {
            throw ConfigurationError(
                "[ABOBreakHammerBridge] requires a PRAC/Chronus plugin.");
        }

        throttleable_setup(m_cfg.m_num_banks, 2);
        m_prac->set_abo_event_callback(
            [this](uint32_t trigger_mask, int flat_bank_id) {
                on_abo(trigger_mask, flat_bank_id);
            });

        register_stat(s_abo_seen).name("breakhammer_abo_seen");
        register_stat(s_abo_matched).name("breakhammer_abo_matched");
        register_stat(s_abo_chronus).name("breakhammer_abo_chronus");
        register_stat(s_abo_saltc).name("breakhammer_abo_saltc");
        register_stat(s_abo_external).name("breakhammer_abo_external");
    }

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
        // Delay ABO attribution by one cycle so the ACT that raised the ABO
        // is included in the interval seen by Throttler.
        for (int bank : m_ready_events) {
            increment_operation(bank, -1);
        }
        m_ready_events.clear();
        m_ready_events.swap(m_deferred_events);
    }

private:
    static uint32_t source_bit(IPRAC::ABOTrigger source) {
        return static_cast<uint32_t>(source);
    }

    static std::string normalized(std::string name) {
        std::transform(name.begin(), name.end(), name.begin(),
                       [](unsigned char c) {
                           return static_cast<char>(std::tolower(c));
                       });
        return name;
    }

    static uint32_t parse_source_mask(
        const std::vector<std::string>& source_names) {
        uint32_t mask = 0;
        for (const auto& raw_name : source_names) {
            const std::string name = normalized(raw_name);
            if (name == "all") {
                mask |= source_bit(IPRAC::ABOTrigger::All);
            } else if (name == "chronus") {
                mask |= source_bit(IPRAC::ABOTrigger::Chronus);
            } else if (name == "saltc" || name == "salt-c") {
                mask |= source_bit(IPRAC::ABOTrigger::SALTC);
            } else if (name == "external") {
                mask |= source_bit(IPRAC::ABOTrigger::External);
            } else {
                throw ConfigurationError(
                    "[ABOBreakHammerBridge] unknown abo_sources entry: {}",
                    raw_name);
            }
        }
        return mask;
    }

    void on_abo(uint32_t trigger_mask, int flat_bank_id) {
        s_abo_seen++;
        if (trigger_mask & source_bit(IPRAC::ABOTrigger::Chronus))
            s_abo_chronus++;
        if (trigger_mask & source_bit(IPRAC::ABOTrigger::SALTC))
            s_abo_saltc++;
        if (trigger_mask & source_bit(IPRAC::ABOTrigger::External))
            s_abo_external++;

        if (!m_enabled || (trigger_mask & m_source_mask) == 0) return;

        int bank = flat_bank_id;
        if (bank < 0 || bank >= m_cfg.m_num_banks) bank = 0;
        m_deferred_events.push_back(bank);
        s_abo_matched++;

        if (m_debug) {
            std::printf(
                "[ABOBreakHammerBridge] ABO matched: triggers=0x%x bank=%d\n",
                trigger_mask, bank);
        }
    }
};

}  // namespace Ramulator
