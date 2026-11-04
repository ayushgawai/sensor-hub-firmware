#include <cstring>
#include <random>
#include <vector>

#include "link_sim.hpp"
#include "shf/cobs.hpp"
#include "shf/crc.hpp"
#include "shf/protocol.hpp"
#include "test.hpp"

using namespace shf;

TEST(crc_check_values) {
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc16_ccitt(s, 9) == 0x29B1);
  CHECK(crc16_t10dif(s, 9) == 0xD0DB);
  CHECK(crc32(s, 9) == 0xCBF43926u);
  CHECK(crc32(s + 4, 5, crc32(s, 4)) == 0xCBF43926u);  // incremental
}

TEST(cobs_roundtrip_random) {
  std::mt19937 rng(1);
  std::vector<uint8_t> src, enc, dec;
  for (int iter = 0; iter < 20000; ++iter) {
    const size_t n = rng() % 600;
    src.resize(n);
    const uint32_t zero_rate = rng() % 4;  // vary density of zeros
    for (auto& b : src) b = (rng() % 4 < zero_rate) ? 0 : static_cast<uint8_t>(rng());
    enc.assign(cobs_max_encoded(n), 0xAA);
    const size_t e = cobs_encode(src.data(), n, enc.data(), enc.size());
    REQUIRE(e > 0 && e <= cobs_max_encoded(n));
    for (size_t i = 0; i < e; ++i) CHECK(enc[i] != 0);
    dec.assign(n + 1, 0);
    size_t d = 0;
    REQUIRE(cobs_decode(enc.data(), e, dec.data(), dec.size(), d));
    CHECK(d == n);
    CHECK(std::memcmp(dec.data(), src.data(), n) == 0);
  }
  uint8_t small[2];
  const uint8_t three[3] = {1, 2, 3};
  CHECK(cobs_encode(three, 3, small, sizeof(small)) == 0);  // too small
}

TEST(frame_roundtrip_and_stream) {
  Deframer df;
  Frame f{}, out{};
  std::vector<uint8_t> wire;
  uint8_t buf[kMaxEncodedFrame];
  for (int i = 0; i <= static_cast<int>(kMaxPayload); ++i) {
    f.type = FrameType::Data;
    f.flags = (i % 3 == 0) ? kFlagSyn : 0;
    f.seq = static_cast<uint8_t>(i * 7);
    f.len = static_cast<uint8_t>(i);
    for (int j = 0; j < i; ++j) f.payload[j] = static_cast<uint8_t>(j * 31 + i);
    const size_t n = encode_frame(f, buf, sizeof(buf));
    REQUIRE(n > 0 && n <= kMaxEncodedFrame);
    wire.insert(wire.end(), buf, buf + n);
    wire.push_back(0);  // idle delimiters must be ignored
  }
  int got = 0;
  for (uint8_t b : wire) {
    const auto r = df.push(b, out);
    CHECK(r != Deframer::Result::Bad);
    if (r == Deframer::Result::Ok) {
      CHECK(out.len == got);
      CHECK(out.seq == static_cast<uint8_t>(got * 7));
      CHECK(out.flags == ((got % 3 == 0) ? kFlagSyn : 0));
      for (int j = 0; j < got; ++j) CHECK(out.payload[j] == static_cast<uint8_t>(j * 31 + got));
      ++got;
    }
  }
  CHECK(got == static_cast<int>(kMaxPayload) + 1);
}

namespace {

int count_accepted(const std::vector<uint8_t>& wire) {
  Deframer df;
  Frame out{};
  int ok = 0;
  for (uint8_t b : wire) ok += df.push(b, out) == Deframer::Result::Ok;
  ok += df.push(0, out) == Deframer::Result::Ok;  // flush
  return ok;
}

}  // namespace

TEST(frame_rejects_all_1_2_3_bit_errors) {
  // Exhaustive 1- and 2-bit errors, 200k random 3-bit errors, on a 24-byte
  // payload frame. Every one must be rejected.
  Frame f{};
  f.type = FrameType::Data;
  f.seq = 42;
  f.len = 24;
  for (int i = 0; i < 24; ++i) f.payload[i] = static_cast<uint8_t>(i * 13 + 1);
  uint8_t buf[kMaxEncodedFrame];
  const size_t n = encode_frame(f, buf, sizeof(buf));
  std::vector<uint8_t> base(buf, buf + n);
  const size_t bits = n * 8;  // includes the delimiters
  int accepted = 0;
  long tried = 0;
  std::vector<uint8_t> w;
  for (size_t i = 0; i < bits; ++i) {
    w = base;
    w[i / 8] ^= static_cast<uint8_t>(1u << (i % 8));
    accepted += count_accepted(w);
    ++tried;
    for (size_t j = i + 1; j < bits; ++j) {
      std::vector<uint8_t> w2 = w;
      w2[j / 8] ^= static_cast<uint8_t>(1u << (j % 8));
      accepted += count_accepted(w2);
      ++tried;
    }
  }
  std::mt19937 rng(3);
  for (int k = 0; k < 200000; ++k) {
    w = base;
    size_t a = rng() % bits, b, c;
    do b = rng() % bits; while (b == a);
    do c = rng() % bits; while (c == a || c == b);
    for (size_t p : {a, b, c}) w[p / 8] ^= static_cast<uint8_t>(1u << (p % 8));
    accepted += count_accepted(w);
    ++tried;
  }
  CHECK(accepted == 0);
  NOTE("%ld corrupted frames, %d accepted", tried, accepted);
}

TEST(deframer_random_garbage) {
  std::mt19937 rng(9);
  Deframer df;
  Frame out{};
  int ok = 0;
  for (int i = 0; i < 2000000; ++i) {
    // Bias toward zeros so many short "frames" are attempted.
    const uint8_t b = (rng() % 16 == 0) ? 0 : static_cast<uint8_t>(rng());
    ok += df.push(b, out) == Deframer::Result::Ok;
  }
  CHECK(ok == 0);
  NOTE("2M garbage bytes: %d accepted, %u rejected by CRC, %u by COBS/format checks", ok,
       df.stats().crc_errors, df.stats().cobs_errors + df.stats().format_errors);
}

namespace {

struct FuzzResult {
  uint64_t delivered = 0;   // both directions
  uint64_t corrupted = 0;   // payload differs from what was sent: must be 0
  uint64_t misordered = 0;  // duplicate or out of order: must be 0
  uint64_t gaps = 0;        // packets skipped; only allowed for abandoned frames
  uint64_t abandoned = 0;
  uint64_t frames_damaged = 0;
  uint64_t ccitt_only = 0;
  bool completed = false;
};

struct Stream {
  std::vector<std::vector<uint8_t>> sent;
  int64_t last = -1;
};

// Each payload starts with its 32-bit index, so the checker can tell
// corruption, duplication, reordering and loss apart.
void check_delivery(Stream& s, FuzzResult& r, const uint8_t* d, size_t n) {
  if (n < 4) {
    ++r.corrupted;
    return;
  }
  const uint32_t idx = static_cast<uint32_t>(d[0]) | (static_cast<uint32_t>(d[1]) << 8) |
                       (static_cast<uint32_t>(d[2]) << 16) | (static_cast<uint32_t>(d[3]) << 24);
  if (idx >= s.sent.size() || s.sent[idx].size() != n || std::memcmp(s.sent[idx].data(), d, n) != 0) {
    ++r.corrupted;
    return;
  }
  if (static_cast<int64_t>(idx) <= s.last) {
    ++r.misordered;
    return;
  }
  r.gaps += static_cast<uint64_t>(static_cast<int64_t>(idx) - s.last - 1);
  s.last = idx;
  ++r.delivered;
}

// Full duplex: both sides stream random payloads at the same time.
FuzzResult fuzz(const sim::ChannelConfig& cc, uint32_t packets, size_t max_payload, uint64_t seed) {
  FuzzResult res;
  sim::LinkPair lp(LinkEndpoint::Config{8000, 2000, static_cast<uint8_t>(seed)}, cc, seed);
  std::mt19937_64 rng(seed);
  Stream ab, ba;
  for (Stream* s : {&ab, &ba}) {
    for (uint32_t i = 0; i < packets; ++i) {
      std::vector<uint8_t> p(4 + rng() % (max_payload - 3));
      for (auto& b : p) b = static_cast<uint8_t>(rng());
      for (int k = 0; k < 4; ++k) p[k] = static_cast<uint8_t>(i >> (8 * k));
      s->sent.push_back(std::move(p));
    }
  }
  lp.side_b().on_deliver = [&](const uint8_t* d, size_t n) { check_delivery(ab, res, d, n); };
  lp.side_a().on_deliver = [&](const uint8_t* d, size_t n) { check_delivery(ba, res, d, n); };
  size_t next_a = 0, next_b = 0;
  for (;;) {
    if (!lp.a().busy() && next_a < packets) {
      lp.a().send(ab.sent[next_a].data(), ab.sent[next_a].size(), lp.now());
      ++next_a;
    }
    if (!lp.b().busy() && next_b < packets) {
      lp.b().send(ba.sent[next_b].data(), ba.sent[next_b].size(), lp.now());
      ++next_b;
    }
    if (next_a == packets && next_b == packets && !lp.a().busy() && !lp.b().busy()) {
      res.completed = true;
      break;
    }
    if (!lp.step()) break;
  }
  while (lp.step()) {  // drain frames still on the wire
  }
  res.frames_damaged = lp.ab().stats().frames_damaged + lp.ba().stats().frames_damaged;
  res.ccitt_only = lp.ab().stats().ccitt_only_accepts + lp.ba().stats().ccitt_only_accepts;
  res.abandoned = lp.a().stats().abandoned + lp.b().stats().abandoned;
  return res;
}

}  // namespace

TEST(protocol_fuzz_errors_drops_duplicates) {
  struct Cfg {
    const char* name;
    sim::ChannelConfig cc;
    uint32_t packets;
    size_t max_payload;
    bool expect_all;  // low enough noise that no frame should be abandoned
  };
  const Cfg cfgs[] = {
      {"ber=1e-3 drop=5% dup=5% garbage=2%", {1e-3, 0.05, 0.05, 0.02, 87}, 20000, 64, true},
      {"ber=0 drop=30% dup=30% garbage=10%", {0.0, 0.30, 0.30, 0.10, 87}, 10000, 64, true},
      {"ber=1e-2 drop=10% dup=10% garbage=5%", {1e-2, 0.10, 0.10, 0.05, 87}, 3000, 64, false},
      {"ber=3e-2, short payloads", {3e-2, 0.0, 0.0, 0.0, 87}, 1000, 8, false},
  };
  uint64_t seed = 100;
  for (const Cfg& c : cfgs) {
    const FuzzResult r = fuzz(c.cc, c.packets, c.max_payload, ++seed);
    CHECK(r.completed);
    CHECK(r.corrupted == 0);
    CHECK(r.misordered == 0);
    CHECK(r.gaps <= r.abandoned);
    if (c.expect_all) {
      CHECK(r.abandoned == 0);
      CHECK(r.delivered == 2ull * c.packets);
    }
    NOTE("%-38s %llu/%u delivered, %llu damaged frames, 0 corrupted accepted: %s, "
         "abandoned %llu, CRC-16/CCITT alone would have accepted %llu",
         c.name, static_cast<unsigned long long>(r.delivered), 2 * c.packets,
         static_cast<unsigned long long>(r.frames_damaged), r.corrupted == 0 ? "yes" : "NO",
         static_cast<unsigned long long>(r.abandoned), static_cast<unsigned long long>(r.ccitt_only));
  }
}

TEST(protocol_resync_after_abandon) {
  // The link goes dark long enough for the sender to give up on a frame.
  // Afterwards SYN resynchronizes and nothing is delivered twice.
  // Driven by hand so the test controls exactly which direction is lost.
  struct Wire {
    std::vector<uint8_t> bytes;
  } a_to_b, b_to_a;
  std::vector<std::vector<uint8_t>> got;
  LinkEndpoint a(
      LinkEndpoint::Config{1000, 3, 10},
      [](void* c, const uint8_t* d, size_t n) {
        auto* w = static_cast<Wire*>(c);
        w->bytes.insert(w->bytes.end(), d, d + n);
      },
      &a_to_b, [](void*, const uint8_t*, size_t) {}, nullptr);
  LinkEndpoint b(
      LinkEndpoint::Config{1000, 3, 50},
      [](void* c, const uint8_t* d, size_t n) {
        auto* w = static_cast<Wire*>(c);
        w->bytes.insert(w->bytes.end(), d, d + n);
      },
      &b_to_a,
      [](void* c, const uint8_t* d, size_t n) {
        static_cast<std::vector<std::vector<uint8_t>>*>(c)->emplace_back(d, d + n);
      },
      &got);
  auto pump = [&](bool deliver_ab, bool deliver_ba, uint64_t now) {
    if (deliver_ab) {
      for (uint8_t x : a_to_b.bytes) b.on_rx_byte(x, now);
    }
    a_to_b.bytes.clear();
    if (deliver_ba) {
      for (uint8_t x : b_to_a.bytes) a.on_rx_byte(x, now);
    }
    b_to_a.bytes.clear();
  };
  uint64_t now = 0;
  const uint8_t m1[] = {1}, m2[] = {2}, m3[] = {3};
  CHECK(a.send(m1, 1, now));
  pump(true, true, now);
  CHECK(!a.busy());
  // m2 reaches B but every ACK is lost; A gives up after 3 retries.
  CHECK(a.send(m2, 1, now));
  for (int i = 0; i < 10; ++i) {
    pump(true, false, now);
    now += 2000;  // > frame wire time + 1 ms timeout
    a.poll(now);
  }
  CHECK(!a.busy());
  CHECK(a.stats().abandoned == 1);
  // m3 carries SYN; B must accept it once.
  CHECK(a.send(m3, 1, now));
  pump(true, true, now);
  CHECK(!a.busy());
  REQUIRE(got.size() == 3);
  CHECK(got[0][0] == 1 && got[1][0] == 2 && got[2][0] == 3);
  CHECK(b.stats().duplicates >= 3);
}
