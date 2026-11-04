// Event-driven model of a full-duplex serial link between two LinkEndpoints,
// with random bit errors, frame drops, frame duplicates and garbage bursts.
// Used by the protocol fuzz test and the goodput bench.
#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <random>
#include <vector>

#include "shf/cobs.hpp"
#include "shf/crc.hpp"
#include "shf/protocol.hpp"

namespace shf::sim {

struct ChannelConfig {
  double ber = 0.0;          // independent bit-flip probability
  double drop_prob = 0.0;    // whole frame lost
  double dup_prob = 0.0;     // frame delivered twice
  double garbage_prob = 0.0; // random burst of 1..32 bytes injected before a frame
  uint32_t byte_time_us = 87;  // 115200 baud 8N1
};

struct ChannelStats {
  uint64_t frames = 0;
  uint64_t bytes = 0;
  uint64_t bit_flips = 0;
  uint64_t frames_damaged = 0;
  uint64_t drops = 0;
  uint64_t dups = 0;
  uint64_t garbage_bursts = 0;
  // Damaged frames that a design with only the CRC-16/CCITT check would
  // have accepted (structure valid, first CRC matches).
  uint64_t ccitt_only_accepts = 0;
};

class NoisyChannel {
 public:
  NoisyChannel(const ChannelConfig& cfg, uint64_t seed) : cfg_(cfg), rng_(seed) {
    next_error_ = draw_gap();
  }

  void send(uint64_t now, const uint8_t* d, size_t n) {
    ++stats_.frames;
    if (cfg_.garbage_prob > 0 && coin(cfg_.garbage_prob)) {
      ++stats_.garbage_bursts;
      std::vector<uint8_t> g(1 + rng_() % 32);
      for (auto& b : g) b = static_cast<uint8_t>(rng_());
      enqueue(now, std::move(g));
    }
    std::vector<uint8_t> f(d, d + n);
    const bool drop = cfg_.drop_prob > 0 && coin(cfg_.drop_prob);
    const bool dup = cfg_.dup_prob > 0 && coin(cfg_.dup_prob);
    if (drop) {
      ++stats_.drops;
      occupy(now, n);  // the bytes still used the wire
    } else {
      enqueue(now, f);
    }
    if (dup) {
      ++stats_.dups;
      enqueue(now, std::move(f));
    }
  }

  uint64_t next_arrival() const { return q_.empty() ? UINT64_MAX : q_.front().arrive; }

  template <typename Sink>
  void deliver_due(uint64_t now, Sink&& sink) {
    while (!q_.empty() && q_.front().arrive <= now) {
      Pending p = std::move(q_.front());
      q_.pop_front();
      for (uint8_t b : p.bytes) sink(b, p.arrive);
    }
  }

  const ChannelStats& stats() const { return stats_; }

 private:
  struct Pending {
    uint64_t arrive;
    std::vector<uint8_t> bytes;
  };

  bool coin(double p) { return std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < p; }

  uint64_t draw_gap() {
    if (cfg_.ber <= 0) return UINT64_MAX;
    return std::geometric_distribution<uint64_t>(cfg_.ber)(rng_);
  }

  uint64_t occupy(uint64_t now, size_t n) {
    const uint64_t start = std::max(now, busy_until_);
    busy_until_ = start + n * cfg_.byte_time_us;
    stats_.bytes += n;
    return busy_until_;
  }

  // Splits a received byte run on delimiters and counts segments whose
  // header+payload differ from what was sent, yet pass the header/length
  // checks and CRC-16/CCITT: the damaged frames a single-CRC-16 design would
  // have accepted.
  static size_t decode_body(const uint8_t* enc, size_t len, uint8_t* raw) {
    size_t m = 0;
    if (!cobs_decode(enc, len, raw, kMaxRawFrame, m) || m < kFrameOverhead) return 0;
    const size_t body = m - 4;
    const uint8_t hdr = raw[0];
    if ((hdr & 0x7Cu) != 0 || (hdr & 3u) == 0 || raw[2] != body - 3) return 0;
    const uint16_t a = static_cast<uint16_t>((raw[body] << 8) | raw[body + 1]);
    return crc16_ccitt(raw, body) == a ? body : 0;
  }
  static uint64_t ccitt_only_accepts(const std::vector<uint8_t>& sent, const std::vector<uint8_t>& got) {
    uint8_t ref[kMaxRawFrame];
    const size_t ref_len = sent.size() > 2 ? decode_body(sent.data() + 1, sent.size() - 2, ref) : 0;
    uint64_t n = 0;
    size_t start = 0;
    for (size_t i = 0; i <= got.size(); ++i) {
      if (i < got.size() && got[i] != 0) continue;
      uint8_t raw[kMaxRawFrame];
      const size_t body = i > start ? decode_body(got.data() + start, i - start, raw) : 0;
      if (body > 0 && !(body == ref_len && std::equal(raw, raw + body, ref))) ++n;
      start = i + 1;
    }
    return n;
  }

  void enqueue(uint64_t now, std::vector<uint8_t> f) {
    // Bit errors: the gap to the next flipped bit is geometric, carried
    // across frames so the stream sees an exact per-bit error rate.
    const std::vector<uint8_t> sent = cfg_.ber > 0 ? f : std::vector<uint8_t>{};
    bool damaged = false;
    const uint64_t bits = f.size() * 8;
    uint64_t pos = 0;
    while (next_error_ < bits - pos) {
      pos += next_error_;
      f[pos / 8] ^= static_cast<uint8_t>(1u << (pos % 8));
      ++stats_.bit_flips;
      damaged = true;
      ++pos;
      next_error_ = draw_gap();
    }
    if (next_error_ != UINT64_MAX) next_error_ -= bits - pos;
    if (damaged) {
      ++stats_.frames_damaged;
      stats_.ccitt_only_accepts += ccitt_only_accepts(sent, f);
    }
    const uint64_t arrive = occupy(now, f.size());
    q_.push_back(Pending{arrive, std::move(f)});
  }

  ChannelConfig cfg_;
  std::mt19937_64 rng_;
  uint64_t next_error_;
  uint64_t busy_until_ = 0;
  std::deque<Pending> q_;
  ChannelStats stats_;
};

// Two endpoints joined by two noisy channels, driven in virtual time.
class LinkPair {
 public:
  struct Side {
    LinkPair* pair;
    NoisyChannel* out;
    std::function<void(const uint8_t*, size_t)> on_deliver;
  };

  LinkPair(const LinkEndpoint::Config& ecfg, const ChannelConfig& ccfg, uint64_t seed)
      : ab_(ccfg, seed * 3 + 1),
        ba_(ccfg, seed * 3 + 2),
        sa_{this, &ab_, {}},
        sb_{this, &ba_, {}},
        a_(ecfg, &LinkPair::tx, &sa_, &LinkPair::deliver, &sa_),
        b_(LinkEndpoint::Config{ecfg.retx_timeout_us, ecfg.max_retries,
                                static_cast<uint8_t>(ecfg.initial_seq + 101)},
           &LinkPair::tx, &sb_, &LinkPair::deliver, &sb_) {}

  LinkEndpoint& a() { return a_; }
  LinkEndpoint& b() { return b_; }
  Side& side_a() { return sa_; }  // payloads delivered to A
  Side& side_b() { return sb_; }  // payloads delivered to B
  const NoisyChannel& ab() const { return ab_; }
  const NoisyChannel& ba() const { return ba_; }
  uint64_t now() const { return now_; }

  // Advances to the next event (frame arrival or retransmit deadline).
  // Returns false if nothing is pending.
  bool step() {
    const uint64_t t = std::min(std::min(ab_.next_arrival(), ba_.next_arrival()),
                                std::min(deadline(a_), deadline(b_)));
    if (t == UINT64_MAX) return false;
    now_ = std::max(now_, t);
    ab_.deliver_due(now_, [this](uint8_t byte, uint64_t at) { b_.on_rx_byte(byte, at); });
    ba_.deliver_due(now_, [this](uint8_t byte, uint64_t at) { a_.on_rx_byte(byte, at); });
    a_.poll(now_);
    b_.poll(now_);
    return true;
  }

 private:
  static uint64_t deadline(const LinkEndpoint& e) { return e.retx_deadline(); }
  static void tx(void* ctx, const uint8_t* d, size_t n) {
    auto* s = static_cast<Side*>(ctx);
    s->out->send(s->pair->now_, d, n);
  }
  static void deliver(void* ctx, const uint8_t* d, size_t n) {
    auto* s = static_cast<Side*>(ctx);
    if (s->on_deliver) s->on_deliver(d, n);
  }

  uint64_t now_ = 0;
  NoisyChannel ab_, ba_;
  Side sa_, sb_;
  LinkEndpoint a_, b_;
};

}  // namespace shf::sim
