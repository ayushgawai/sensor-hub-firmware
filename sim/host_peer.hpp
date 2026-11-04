// Host-side peer for a simulated device: a LinkEndpoint wired to the
// SimHal UART, with optional bit errors in both directions.
#pragma once

#include <cstdint>
#include <deque>
#include <random>
#include <vector>

#include "shf/firmware.hpp"
#include "shf/protocol.hpp"
#include "sim_hal.hpp"

namespace shf::sim {

class HostPeer {
 public:
  HostPeer(SimHal& hal, double ber, uint64_t seed)
      : hal_(hal),
        ber_(ber),
        rng_(seed),
        link_(LinkEndpoint::Config{30000, 1000, 7}, &HostPeer::tx, this, &HostPeer::deliver, this) {
    hal_.on_uart_tx = [this](const uint8_t* d, size_t n) {
      for (size_t i = 0; i < n; ++i) rx_q_.push_back(noise(d[i]));
    };
  }
  ~HostPeer() { hal_.on_uart_tx = nullptr; }

  void queue(const std::vector<uint8_t>& msg) { out_.push_back(msg); }
  bool idle() const { return out_.empty() && !link_.busy(); }

  // Call from DeviceSim::on_step.
  void step(uint64_t now) {
    now_ = now;
    while (!rx_q_.empty()) {
      const uint8_t b = rx_q_.front();
      rx_q_.pop_front();
      link_.on_rx_byte(b, now);
    }
    link_.poll(now);
    if (!link_.busy() && !out_.empty()) {
      if (link_.send(out_.front().data(), out_.front().size(), now)) out_.pop_front();
    }
  }

  // Messages received from the device (telemetry and status).
  std::vector<std::vector<uint8_t>> received;
  const LinkEndpoint& link() const { return link_; }

  // Latest STATUS for a request id, or -1.
  int last_status(uint8_t req) const {
    for (auto it = received.rbegin(); it != received.rend(); ++it) {
      if (it->size() >= 3 && (*it)[0] == kMsgStatus && (*it)[1] == req) return (*it)[2];
    }
    return -1;
  }
  // Latest telemetry message, or empty.
  std::vector<uint8_t> last_telemetry() const {
    for (auto it = received.rbegin(); it != received.rend(); ++it) {
      if (!it->empty() && (*it)[0] == kMsgTelemetry) return *it;
    }
    return {};
  }

 private:
  uint8_t noise(uint8_t b) {
    if (ber_ <= 0) return b;
    std::bernoulli_distribution flip(ber_);
    for (int i = 0; i < 8; ++i) {
      if (flip(rng_)) b ^= static_cast<uint8_t>(1u << i);
    }
    return b;
  }
  static void tx(void* ctx, const uint8_t* d, size_t n) {
    auto* self = static_cast<HostPeer*>(ctx);
    std::vector<uint8_t> v(d, d + n);
    for (auto& b : v) b = self->noise(b);
    self->hal_.uart_inject(v.data(), v.size());
  }
  static void deliver(void* ctx, const uint8_t* d, size_t n) {
    static_cast<HostPeer*>(ctx)->received.emplace_back(d, d + n);
  }

  SimHal& hal_;
  double ber_;
  std::mt19937_64 rng_;
  LinkEndpoint link_;
  std::deque<uint8_t> rx_q_;
  std::deque<std::vector<uint8_t>> out_;
  uint64_t now_ = 0;
};

// Message builders.
inline void put_u32(std::vector<uint8_t>& v, uint32_t x) {
  for (int i = 0; i < 4; ++i) v.push_back(static_cast<uint8_t>(x >> (8 * i)));
}
inline std::vector<uint8_t> msg_ota_begin(uint32_t size, uint32_t crc, uint32_t version) {
  std::vector<uint8_t> m{kMsgOtaBegin};
  put_u32(m, size);
  put_u32(m, crc);
  put_u32(m, version);
  return m;
}
inline std::vector<uint8_t> msg_ota_chunk(uint32_t off, const uint8_t* d, size_t n) {
  std::vector<uint8_t> m{kMsgOtaChunk};
  put_u32(m, off);
  m.insert(m.end(), d, d + n);
  return m;
}
inline std::vector<uint8_t> msg_simple(uint8_t id) { return {id}; }

}  // namespace shf::sim
