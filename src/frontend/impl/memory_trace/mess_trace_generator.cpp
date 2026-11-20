#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <bitset>

#include "base/exception.h"
#include "base/request.h"
#include "base/type.h"
#include "frontend/frontend.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"

namespace Ramulator {
class MessReqGenerator : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, MessReqGenerator, "MessReqGenerator", "Sequential Random accesses and Stream traces.")

 private:
  std::string mess_trace_path;
  size_t m_total_requests;
  size_t m_issued_requests;
  size_t s_retried_requests;

  size_t s_random_reads;
  size_t s_random_writes;
  size_t s_stride_reads;
  size_t s_stride_writes;
  size_t s_total_numer_of_idle_ticks;
  size_t s_total_number_for_idle_ticks_random_reads;

  // stores the shift amount we need to increment by
  // rank, bankgroup, bank, column, row, channel
  int shift_amts[6] = {-1, -1, -1, -1, -1, -1};
  int64_t max_val[6] = {-1, -1, -1, -1, -1, -1};
  std::vector<int64_t> channel_offsets;
  std::vector<Addr_t> stream_addrs;
  int addr_index;
  int64_t m_num_columns;  // the number of columns we iterate over for STREAM

  size_t random_count;
  bool m_issue_random;
  float m_read_ratio;

  bool m_disable_random;

  bool retry_send_stream;
  bool retry_send_random;
  Request m_curr_stride_req = {0, 0};
  Request m_curr_random_req = {0, 0};
  Request m_last_random_req = {-1, 0};

  struct Trace {
    bool is_write;
    Addr_t addr;
  };

  Logger_t m_logger;

  Addr_t m_min_addr;
  int64_t m_max_addr;

  size_t m_nop_counter;
  size_t m_curr_nop_counter;

  std::mt19937 m_gen;
  std::bernoulli_distribution m_dist_read_write;

  size_t m_frontend_ticks = 0;
  size_t m_max_ticks = 0;   // 0 = disabled; else hard stop (bounds write-heavy runs where
                            // the pointer-chase read is starved by the write flood)
  bool m_linear_stream = false;
  int m_stream_per_tick = 1;
  Addr_t m_stream_next = 0;
  size_t s_probe_count = 0;
  size_t s_probe_latency_sum = 0;
  float s_avg_probe_latency = 0;
  bool m_probe_outstanding = false;
  IAddrMapper* m_addr_mapper = nullptr;
  int m_row_idx = -1;
  int m_col_idx = -1;
  int m_row_columns = 0;
  bool m_per_channel = false;
  int m_ch_idx = -1;
  std::vector<Addr_t> m_ch_next;
  std::vector<Request> m_ch_pending;
  std::vector<bool> m_ch_has_pending;

  Request decode_addr(Addr_t addr) {
    if (m_addr_mapper == nullptr) {
      m_addr_mapper = m_memory_system->get_ifce<IAddrMapper>();
      IDRAM* dram = m_memory_system->get_ifce<IDRAM>();
      m_row_idx = dram->m_levels("row");
      m_col_idx = dram->m_levels("column");
      m_ch_idx = dram->m_levels("channel");
    }
    Request tmp(addr, Request::Type::Read);
    m_addr_mapper->apply(tmp);
    return tmp;
  }

  bool is_write() {
    bool is_read = m_dist_read_write(m_gen);
    return !is_read;
  }

  Request get_random_request() {
    if (m_max_addr == -1) {
      set_max_addr();
    }
    static std::mt19937 gen(1337);
    std::uniform_int_distribution<uint64_t> dist(m_min_addr, m_max_addr);
    Addr_t addr = dist(gen);
    Request res = {addr, false};  // pointer chase just does reads
    if (m_linear_stream) {
      res = Request(addr, Request::Type::Read, -1, [this](Request& r) {
        s_probe_latency_sum += r.depart - r.arrive;
        s_probe_count++;
        m_probe_outstanding = false;
      });
    }
    m_curr_random_req = res;
    return res;
  }

  int64_t get_max_addr() {
    int total_bits = m_memory_system->get_total_address_bits();
    // we will be issueing the request to all channels, so we need to subtract the channel bits
    if (!m_linear_stream) {
      total_bits -= std::log2(m_memory_system->get_num_channels());
    }
    return (1LL << total_bits) - 1;
  }

  void set_max_addr() {
    m_max_addr = get_max_addr();
    if (m_max_addr <= m_min_addr) {
      throw ConfigurationError("Invalid address range: max_addr must be > min_addr");
    }
  }

  void set_up_channel_offsets() {
    int num_channels = m_memory_system->get_num_channels();
    channel_offsets.resize(num_channels);
    channel_offsets[0] = 0;
    for (int i = 0; i < num_channels; i++) {
      channel_offsets[i] = (int64_t)i << m_memory_system->get_shift_amt(5);
    }
    for(int i=0; i<num_channels; i++) {
      std::cout << "channel " << i << " offset: " << std::bitset<40>(channel_offsets[i]) << std::endl;
    }
  }

  void setup_shift_amts() {
    for (int i=0; i< 6; i++) {
      // column and row are reversed:
      if (i == 3) { // col
        shift_amts[i] = m_memory_system->get_shift_amt(4);
        max_val[i] = (1 << m_memory_system->get_max(4)) -1;
      } else if (i == 4) {  // row
        shift_amts[i] = m_memory_system->get_shift_amt(3);
        max_val[i] = (1 << m_memory_system->get_max(3)) -1;
      } else {
        // get the shift amount you need to increase by to increment LSB
        shift_amts[i] = m_memory_system->get_shift_amt(i);
        // get mask for max value
        max_val[i] = (1LL << m_memory_system->get_max(i)) -1;
      }
    }
  }

  enum AddrFields {
    rank = 0,
    bankgroup = 1,
    bank = 2,
    column = 3,
    row = 4,
    channel = 5
  };

  void setup_stream_addresses() {
    setup_shift_amts();
    set_up_channel_offsets();
    Addr_t addr = 0;
    for (int64_t i=0; i < max_val[row]; i += (1LL << shift_amts[row])) {
      for (int64_t j = 0, s = 0; s < m_num_columns; j += (1LL << shift_amts[column]), s++) {
        for (int64_t k = 0; k < max_val[bank]; k += (1LL << shift_amts[bank])) {
          for (int64_t l = 0; l < max_val[bankgroup]; l += (1LL << shift_amts[bankgroup])) {
                addr = addr + i + j + k + l;
                stream_addrs.push_back(addr);
                addr = 0;
          }
        }
      }
    }
  }

  void ensure_channel_streams() {
    if (!m_ch_next.empty()) return;
    if (m_max_addr == -1) set_max_addr();
    int n = m_memory_system->get_num_channels();
    m_ch_next.assign(n, m_min_addr);
    m_ch_pending.assign(n, Request(0, 0));
    m_ch_has_pending.assign(n, false);
  }

  Request next_stream_for_channel(int c) {
    Addr_t addr;
    while (true) {
      addr = m_ch_next[c];
      m_ch_next[c] += 64;
      if (m_ch_next[c] > m_max_addr) m_ch_next[c] = m_min_addr;
      Request tmp = decode_addr(addr);
      if (tmp.addr_vec[m_ch_idx] != c) continue;
      if (m_row_columns > 0 && tmp.addr_vec[m_col_idx] >= m_row_columns) continue;
      break;
    }
    return Request(addr, is_write());
  }

  Request get_next_stream_request() {
    if (m_max_addr == -1) {
      set_max_addr();
    }
    if (m_linear_stream) {
      Addr_t addr;
      while (true) {
        addr = m_stream_next;
        m_stream_next += 64;
        if (m_stream_next > m_max_addr) m_stream_next = m_min_addr;
        if (m_row_columns == 0) break;
        Request tmp = decode_addr(addr);
        if (m_row_columns > 0 && tmp.addr_vec[m_col_idx] >= m_row_columns) continue;
        break;
      }
      Request res = {addr, is_write()};
      m_curr_stride_req = res;
      return res;
    }
    if (stream_addrs.size() == 0) setup_stream_addresses();
    Addr_t addr = stream_addrs[addr_index];
    addr_index = (addr_index + 1) % stream_addrs.size();
    bool is_read = m_dist_read_write(m_gen);
    Request res = {addr, is_write()};
    m_curr_stride_req = res;
    return res;
  }

 public:
  void init() override {
    m_logger = Logging::create_logger("OnTheFlyReqGenerator");
    m_clock_ratio = param<uint>("clock_ratio").required();
    m_total_requests = param<uint>("total_requests").required();
    m_min_addr = param<Addr_t>("start_addr").required();
    m_max_addr = -1; // will be initialized when memory system is connected
    m_read_ratio = param<float>("ratio_reads").default_val(0.5);
    m_disable_random = param<bool>("disable_random").default_val(false);
    if (m_read_ratio > 1 || m_read_ratio < 0) {
      throw ConfigurationError("Read ration must be between 0 and 1!");
    }
    m_issued_requests = 0;
    m_issue_random = false;
    retry_send_stream = false;
    retry_send_random = false;
    m_nop_counter = param<uint>("nop_counter").default_val(1);
    m_max_ticks = param<uint>("max_ticks").default_val(0);
    std::string stream_mode = param<std::string>("stream_mode").default_val("legacy");
    if (stream_mode != "legacy" && stream_mode != "linear") {
      throw ConfigurationError("stream_mode must be 'legacy' or 'linear'!");
    }
    m_linear_stream = (stream_mode == "linear");
    m_stream_per_tick = param<int>("stream_per_tick").default_val(1);
    if (m_stream_per_tick < 1) {
      throw ConfigurationError("stream_per_tick must be >= 1!");
    }
    m_stream_next = m_min_addr;
    m_row_columns = param<int>("row_columns").default_val(0);
    if (m_row_columns < 0) {
      throw ConfigurationError("row_columns must be >= 0!");
    }
    m_per_channel = param<bool>("per_channel_stream").default_val(false);
    if (m_per_channel && !m_linear_stream) {
      throw ConfigurationError("per_channel_stream requires stream_mode: linear!");
    }
    m_curr_nop_counter = 0;

    s_retried_requests = 0;
    s_random_reads = 0;
    s_random_writes = 0;
    s_stride_reads = 0;
    s_stride_writes = 0;
    s_total_numer_of_idle_ticks = 0;
    s_total_number_for_idle_ticks_random_reads = 0;
    m_num_columns = param<int64_t>("num_columns").default_val(8);
    addr_index = 0;

    std::seed_seq seed{342};
    m_gen = std::mt19937(seed);
    m_dist_read_write = std::bernoulli_distribution(m_read_ratio);
    // Do not register runtime parameters as stats: main.cpp's reset_all_stats() at
    // the warmup->measurement transition (tick 0 here) would zero them mid-run.
    register_stat(s_retried_requests).name("total_number_of_retried_requests");
    register_stat(s_random_reads).name("total_number_of_random_reads");
    register_stat(s_random_writes).name("total_number_of_random_writes");
    register_stat(s_stride_reads).name("total_number_of_stride_reads");
    register_stat(s_stride_writes).name("total_number_of_stride_writes");
    register_stat(s_total_numer_of_idle_ticks).name("s_total_numer_of_idle_ticks");
    register_stat(s_total_number_for_idle_ticks_random_reads).name("s_total_number_for_idle_ticks_random_reads");
    register_stat(s_probe_count).name("probe_count");
    register_stat(s_probe_latency_sum).name("probe_latency_sum");
    register_stat(s_avg_probe_latency).name("avg_probe_latency");

    random_count = 0;

    // Mess has no warmup phase (the max_ticks cap bounds the run). Signal warmup-done
    // immediately so warmup-gated paths (e.g. PRAC/ABO recovery) are active for the whole run.
    // (Same pattern as loadstore_trace.cpp when num_warmup_insts == 0.)
    set_warmup_done();
  };


  void update_stats() {
    if (m_curr_random_req.type_id == Request::Type::Read && !m_issue_random) {
      s_random_reads ++;
    } else if (m_curr_stride_req.type_id == Request::Type::Read && m_issue_random) {
      s_stride_reads ++;
    } else if (m_curr_random_req.type_id == Request::Type::Write && !m_issue_random) {
      s_random_writes ++;
    } else {
      s_stride_writes ++;
    }
  }

  // sends a requests to all channels
  // by adding the channel offset to the address
  bool send_request(Request req) {
    if (m_linear_stream) return m_memory_system->send(req);
    bool sent = false;
    for (int i=0; i<channel_offsets.size(); i++) {
      Request req_i = req;
      req_i.addr += channel_offsets[i];
      sent = m_memory_system->send(req_i);
    }
    return sent;
  }

  bool issue_request() {
    bool sent = false;
    if (m_issue_random) {
      Request req = {0, 0};
      if (retry_send_random) {
        req = m_curr_random_req;
      } else {
        req = get_random_request();
      }

      if (m_disable_random)
        sent = true;
      else
        sent = send_request(req);

      if (sent) {  // request accepted by MC
        random_count += 1;
        m_last_random_req = req;
        if (m_linear_stream && !m_disable_random) m_probe_outstanding = true;
        m_issue_random = !m_issue_random;
        retry_send_random = false;
      } else {  // request rejected by MC -> we need to retry sending it
        retry_send_random = true;
      }

    } else {  // issue stream
      Request req = {0, 0};
      if (retry_send_stream) {
        req = m_curr_stride_req;
      } else {  // issue new request
        req = get_next_stream_request();
      }
      sent = send_request(req);
      if (sent) {
        retry_send_stream = false;
        m_issue_random = !m_issue_random;
      } else {
        retry_send_stream = true;
      }
    }
    return sent;
  }

  void issue_per_channel_stream() {
    ensure_channel_streams();
    for (int c = 0; c < (int)m_ch_next.size(); c++) {
      Request req = m_ch_has_pending[c] ? m_ch_pending[c] : next_stream_for_channel(c);
      if (m_memory_system->send(req)) {
        m_ch_has_pending[c] = false;
        m_issued_requests++;
        if (req.type_id == Request::Type::Read) s_stride_reads++;
        else s_stride_writes++;
      } else {
        m_ch_pending[c] = req;
        m_ch_has_pending[c] = true;
        s_retried_requests++;
      }
    }
  }

  // regulate the bandwidth by inserting a number of NOP operations
  // is this function returns true, no operations are issued that cycle
  bool is_idle_tick() {
    bool res = true;
    if ((m_curr_nop_counter == 0) || m_issue_random) res = false;  // if we are mod NOP, or issueing a random req, not idle
    if (!m_issue_random) {
      // if we are in stream mode, increment the NOP counter
      m_curr_nop_counter = (m_curr_nop_counter + 1) % m_nop_counter;
    }
    if (res) s_total_numer_of_idle_ticks ++;
    return res;
  }

  bool can_issue_random_req() {
    if (m_linear_stream) return !m_probe_outstanding;
    if (m_last_random_req.addr == -1)
      return true;
    else if (m_memory_system->is_request_finished(m_last_random_req))
      return true;
    return false;
  }

  void tick() override {
    if ((m_frontend_ticks % 100000) == 0) {
      std::cout << "Frontend: tick " << m_frontend_ticks << ", issued " << random_count << "/" << m_total_requests << "random requests" << std::endl;
    }
    m_frontend_ticks++;

    bool idle = is_idle_tick();
    if (idle || (random_count >= m_total_requests)) {
      m_issue_random = !m_issue_random;
      return;
    }

    // make sure random requests are issued sequentially
    if (m_issue_random && !can_issue_random_req()) {
      s_total_number_for_idle_ticks_random_reads ++;
      m_issue_random = false;
      return;
    }

    if (m_per_channel && !m_issue_random) {
      issue_per_channel_stream();
      m_issue_random = true;
      return;
    }

    bool was_stream = !m_issue_random;
    bool request_sent = issue_request();
    if (request_sent) {
      m_issued_requests++;
      update_stats();
      if (m_linear_stream && was_stream) {
        for (int i = 1; i < m_stream_per_tick; i++) {
          Request req = get_next_stream_request();
          if (!send_request(req)) {
            retry_send_stream = true;
            break;
          }
          m_issued_requests++;
          if (req.type_id == Request::Type::Read) s_stride_reads++;
          else s_stride_writes++;
        }
      }
    } else {
      s_retried_requests ++;
    }
  };

  void finalize() override {
    s_avg_probe_latency = s_probe_count ? (float)s_probe_latency_sum / (float)s_probe_count : 0.0f;
    IFrontEnd::finalize();
  }

  // is_finished looks at only random request counts, with an optional hard tick cap
  // so write-heavy runs (where the read probe starves) still terminate.
  bool is_finished() override {
    if (m_max_ticks && m_frontend_ticks >= m_max_ticks) return true;
    return (random_count >= m_total_requests && m_memory_system->is_finished_ms());
  };
};
}  // end namespace Ramulator
