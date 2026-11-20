#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"

namespace Ramulator {

class FRFCFS : public IScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IScheduler, FRFCFS, "FRFCFS", "FRFCFS DRAM Scheduler.")
  private:
    IDRAM* m_dram;

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = cast_parent<IDRAMController>()->m_dram;
    };

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2) override {
      bool ready1 = m_dram->check_ready(req1->command, req1->addr_vec);
      bool ready2 = m_dram->check_ready(req2->command, req2->addr_vec);

      if (ready1 ^ ready2) {
        if (ready1) {
          return req1;
        } else {
          return req2;
        }
      }

      // Fallback to FCFS
      if (req1->arrive <= req2->arrive) {
        return req1;
      } else {
        return req2;
      } 
    }

    ReqBuffer::iterator get_best_request(ReqBuffer& buffer) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }

      for (auto& req : buffer) {
        req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
      }

      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        candidate = compare(candidate, next);
      }
      return candidate;
    }
};

// True FR-FCFS that PRIORITIZES row-buffer hits (port of Ramulator 1.0's
// FRFCFS_PriorHit).  The plain FRFCFS above only checks check_ready() (command
// timing) and therefore does NOT prioritize hits — a row miss whose ACT is
// timing-ready looks just as "ready" as a row hit, so among ready requests it
// behaves like FCFS.  Here a request only "wins" the readiness test if it is
// BOTH timing-ready AND a row-buffer hit (check_rowbuffer_hit) — exactly the
// 1.0 condition (is_ready && is_row_hit).
class FRFCFS_PriorHit : public IScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IScheduler, FRFCFS_PriorHit, "FRFCFS_PriorHit", "True FR-FCFS: prioritizes row-buffer hits, then readiness, then FCFS.")
  private:
    IDRAM* m_dram;

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = cast_parent<IDRAMController>()->m_dram;
    };

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2) override {
      bool hit1 = m_dram->check_ready(req1->command, req1->addr_vec)
               && m_dram->check_rowbuffer_hit(req1->final_command, req1->addr_vec);
      bool hit2 = m_dram->check_ready(req2->command, req2->addr_vec)
               && m_dram->check_rowbuffer_hit(req2->final_command, req2->addr_vec);

      if (hit1 ^ hit2) {
        return hit1 ? req1 : req2;
      }

      // Fallback to FCFS
      if (req1->arrive <= req2->arrive) {
        return req1;
      } else {
        return req2;
      }
    }

    ReqBuffer::iterator get_best_request(ReqBuffer& buffer) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }

      for (auto& req : buffer) {
        req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
      }

      // Pass 1: prefer the OLDEST request that is a ready ROW HIT.
      auto best_hit = buffer.end();
      for (auto it = buffer.begin(); it != buffer.end(); it++) {
        bool ready_hit = m_dram->check_ready(it->command, it->addr_vec)
                      && m_dram->check_rowbuffer_hit(it->final_command, it->addr_vec);
        if (ready_hit && (best_hit == buffer.end() || it->arrive < best_hit->arrive)) {
          best_hit = it;
        }
      }
      if (best_hit != buffer.end()) {
        return best_hit;
      }

      // Pass 2: no schedulable row hit — fall back to plain FRFCFS (ready-first,
      // then FCFS) so a ready miss/conflict is still issued instead of stalling.
      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        bool r_cand = m_dram->check_ready(candidate->command, candidate->addr_vec);
        bool r_next = m_dram->check_ready(next->command, next->addr_vec);
        if (r_cand ^ r_next) {
          candidate = r_cand ? candidate : next;
        } else if (next->arrive < candidate->arrive) {
          candidate = next;
        }
      }
      return candidate;
    }
};

}       // namespace Ramulator