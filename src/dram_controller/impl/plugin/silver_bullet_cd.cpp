#include <vector>
#include <unordered_map>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/pluginutil/device_config.h"
#include "dram_controller/impl/plugin/prac.h"
#include "frontend/frontend.h"

namespace Ramulator {
class SilverBulletCD : public IControllerPlugin, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, SilverBulletCD, "SilverBulletCD", "Silver Bullet Implementation")
    
    private:
        IDRAM* m_dram = nullptr;
        IPRAC* m_prac = nullptr;
        IFrontEnd* m_frontend = nullptr;
        DeviceConfig m_cfg;

        int m_row_level = -1;
        int m_RFMab_id = -1;
        int m_REFab_id = -1;
        int m_num_subbanks = -1;
        int m_rows_per_ref = -1;

        std::vector<int> m_rr_sb;

        int m_D = -1;
        int m_apm = -1;
        int m_bundle = -1;
        int m_subbank_size = -1;
        int m_blast_radius = -1;
        bool m_neighbor_propagation = true;
        bool m_is_debug = false;

        struct SubBankState{
            int frac = 0;
            int pending = 0;
            int rptr = 0;
        };
        std::vector<std::unordered_map<int, SubBankState>> m_state;


        int s_sbcd_RFMab_refreshes = 0;
        int s_sbcd_REFab_refreshes = 0;
        int s_sbcd_RFMab_commands = 0;
        int s_sbcd_pending_produced = 0;

    
        public:
        void init() override {
            m_D = param<int>("D").required();
            m_apm = param<int>("apm").required();
            m_bundle = param<int>("bundle").required();
            m_subbank_size = param<int>("subbank_size").required();
            m_blast_radius = param<int>("blast_radius").default_val(1);
            m_neighbor_propagation = param<bool>("neighbor_propagation").default_val(true);
            m_is_debug = param<bool>("debug").default_val(false);
        }

        void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override{
            m_ctrl = cast_parent<IDRAMController>();
            m_dram = m_ctrl->m_dram;
            m_cfg.set_device(m_ctrl);
            m_frontend = frontend;

            m_num_subbanks = m_dram->get_level_size("row") / m_subbank_size;
            m_row_level = m_dram->m_levels("row");

            m_RFMab_id = m_dram->m_commands("RFMab");
            m_REFab_id = m_dram->m_commands("REFab");
            
            int rows_per_bank = m_dram->get_level_size("row");
            m_rows_per_ref = rows_per_bank / 8192;
            if (m_rows_per_ref < 1) m_rows_per_ref = 1;

            m_rr_sb.resize(m_cfg.m_num_banks);
            m_state.resize(m_cfg.m_num_banks);

            for (auto* plugin : m_ctrl->m_plugins){
                if (auto* p = dynamic_cast<IPRAC*>(plugin)){
                    m_prac = p;
                    break;
                }
            }
            if (!m_prac) throw ConfigurationError("[SilverBullet] requires a PRAC plugin for ABO-based refresh.");

            register_stat(s_sbcd_RFMab_refreshes).name("silverbulletcd_RFMab_refreshes");
            register_stat(s_sbcd_REFab_refreshes).name("silverbulletcd_REFab_refreshes");
            register_stat(s_sbcd_RFMab_commands).name("silverbulletcd_RFMab_commands");
            register_stat(s_sbcd_pending_produced).name("silverbulletcd_pending_produced");
        }

        void update(bool request_found, ReqBuffer::iterator& req_it) override{
            if (!request_found) return;
            auto& req = *req_it;

            if (m_dram->m_command_meta(req.command).is_opening &&
                m_dram->m_command_scopes(req.command) == m_row_level){
                    int flat_bank = m_cfg.get_flat_bank_id(req);
                    int row = req.addr_vec[m_row_level];
                    int sb = row / m_subbank_size;

                    // hammer(flat_bank, sb);

                    if (m_neighbor_propagation){
                    }
                }
        }



};
}