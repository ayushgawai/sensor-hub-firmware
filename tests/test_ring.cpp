#include <atomic>
#include <cstdint>
#include <thread>

#include "shf/ring_buffer.hpp"
#include "test.hpp"

using shf::SpscRing;

TEST(ring_basic_fifo_and_full) {
  SpscRing<uint32_t, 8> r;
  CHECK(r.empty());
  for (uint32_t i = 0; i < 8; ++i) CHECK(r.push(i));
  CHECK(!r.push(99));  // full: all 8 slots usable
  CHECK(r.overflows() == 1);
  CHECK(r.size() == 8);
  uint32_t v = 0;
  for (uint32_t i = 0; i < 8; ++i) {
    CHECK(r.pop(v));
    CHECK(v == i);
  }
  CHECK(!r.pop(v));
  // Many wraps of the index.
  for (uint32_t i = 0; i < 100000; ++i) {
    CHECK(r.push(i));
    CHECK(r.pop(v));
    CHECK(v == i);
  }
}

namespace {

struct Item {
  uint32_t seq;
  uint32_t inv;  // ~seq, detects torn reads
  uint64_t pad;
};

}  // namespace

TEST(ring_concurrent_producer_consumer) {
  // The producer spins when full, so every item must arrive exactly once,
  // in order and untorn.
  constexpr uint32_t kItems = 2000000;
  static SpscRing<Item, 64> r;
  std::atomic<bool> start{false};
  std::thread producer([&] {
    while (!start.load()) {
    }
    for (uint32_t i = 0; i < kItems;) {
      if (r.push(Item{i, ~i, i * 3ull})) ++i;
    }
  });
  start = true;
  uint32_t expect = 0;
  uint32_t bad = 0;
  Item it{};
  while (expect < kItems) {
    if (!r.pop(it)) continue;
    if (it.seq != expect || it.inv != ~expect || it.pad != expect * 3ull) ++bad;
    expect = it.seq + 1;
  }
  producer.join();
  CHECK(bad == 0);
  CHECK(expect == kItems);
  CHECK(r.empty());
  NOTE("%u items through a 64-slot ring, %u out-of-order or torn", kItems, bad);
}

TEST(ring_isr_style_drop_on_full) {
  // Like a UART ISR: the producer never waits, it drops on overflow. The
  // consumer must see a strictly increasing subsequence and
  // received + dropped == produced.
  constexpr uint32_t kItems = 1000000;
  static SpscRing<uint32_t, 16> r;
  std::atomic<bool> done{false};
  std::thread producer([&] {
    for (uint32_t i = 0; i < kItems; ++i) r.push(i);
    done = true;
  });
  uint32_t received = 0;
  uint32_t last = 0;
  bool first = true;
  bool ordered = true;
  uint32_t v;
  for (;;) {
    if (r.pop(v)) {
      if (!first && v <= last) ordered = false;
      last = v;
      first = false;
      ++received;
    } else if (done.load() && r.empty()) {
      break;
    }
  }
  producer.join();
  CHECK(ordered);
  CHECK(received + r.overflows() == kItems);
  NOTE("received %u, dropped %u of %u", received, r.overflows(), kItems);
}
