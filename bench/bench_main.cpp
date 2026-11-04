// Host-simulator sanity bench. Numbers come from a laptop running the
// simulated HAL, not from an MCU.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <deque>
#include <optional>
#include <random>
#include <thread>

#include "device_sim.hpp"
#include "link_sim.hpp"
#include "ota_sim.hpp"
#include "shf/firmware.hpp"
#include "sim_hal.hpp"

using namespace shf;
using namespace shf::sim;

namespace {

void section(const char* title) { std::printf("\n== %s\n", title); }

// ------------------------------------------------------------------ jitter

void bench_jitter(double seconds) {
  section("Scheduler loop jitter (real-time host clock)");
  SimHal hal(SimHal::ClockMode::RealTime, 1);
  provision(hal, make_image(1, 4096));
  hal.power_cycle();
  const BootDecision boot = bootloader_run(hal, kDefaultLayout);
  static std::optional<Firmware> fw;  // static storage, as on the target
  fw.emplace(hal, BootInfo{boot.slot, boot.version, boot.trial});

  // Host peer on its own thread. It feeds the device UART RX "interrupt"
  // at the 115200-baud byte rate, so the firmware's SPSC ring sees a real
  // concurrent producer, and it ACKs the device's telemetry.
  static SpscRing<uint8_t, 4096> dev_to_host;
  std::atomic<uint32_t> host_drops{0};
  hal.on_uart_tx = [&](const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      if (!dev_to_host.push(d[i])) host_drops.fetch_add(1);
    }
  };
  std::atomic<bool> stop{false};
  std::atomic<uint32_t> host_rx_msgs{0};
  std::atomic<uint32_t> host_tx_msgs{0};
  std::thread host([&] {
    std::deque<uint8_t> wire;
    struct Ctx {
      std::deque<uint8_t>* wire;
      std::atomic<uint32_t>* rx;
    } ctx{&wire, &host_rx_msgs};
    LinkEndpoint ep(
        LinkEndpoint::Config{30000, 50, 3},
        [](void* c, const uint8_t* d, size_t n) {
          auto* w = static_cast<Ctx*>(c)->wire;
          w->insert(w->end(), d, d + n);
        },
        &ctx, [](void* c, const uint8_t*, size_t) { static_cast<Ctx*>(c)->rx->fetch_add(1); }, &ctx);
    uint64_t next_byte = hal.micros();
    uint64_t next_cmd = hal.micros();
    while (!stop.load(std::memory_order_relaxed)) {
      const uint64_t now = hal.micros();
      uint8_t b;
      while (dev_to_host.pop(b)) ep.on_rx_byte(b, now);
      ep.poll(now);
      if (now >= next_cmd && !ep.busy()) {
        const uint8_t cmd = kMsgClearFault;  // harmless request, device replies STATUS
        if (ep.send(&cmd, 1, now)) host_tx_msgs.fetch_add(1);
        next_cmd = now + 20000;
      }
      if (!wire.empty() && now >= next_byte) {
        hal.uart_isr_byte(wire.front());
        wire.pop_front();
        next_byte = now + SimHal::kUartByteUs;
      }
    }
  });

  const uint64_t warm_end = hal.micros() + 300000;
  while (hal.micros() < warm_end) {
    hal.service_real_time();
    fw->poll();
  }
  fw->scheduler().clear_jitter();
  const uint64_t t0 = hal.micros();
  const uint64_t end = t0 + static_cast<uint64_t>(seconds * 1e6);
  while (hal.micros() < end) {
    hal.service_real_time();
    fw->poll();
  }
  stop = true;
  host.join();
  hal.on_uart_tx = nullptr;

  const Scheduler& s = fw->scheduler();
  const JitterHistogram& h = s.jitter();
  std::printf("run: %.1f s, %llu releases of the 1 kHz imu/fusion task (probe)\n", seconds,
              static_cast<unsigned long long>(h.count()));
  const TaskStats& imu = s.stats(fw->imu_task_id());
  std::printf("imu task release jitter: p50 %u us, p99 %u us, max %u us (mean %.2f us, 1 us timer resolution)\n",
              h.percentile(500), h.percentile(990), h.max(),
              imu.runs ? static_cast<double>(imu.sum_jitter_us) / imu.runs : 0.0);
  for (size_t i = 0; i < s.task_count(); ++i) {
    const TaskStats& ts = s.stats(static_cast<int>(i));
    std::printf("  task %-9s runs %7u  max jitter %5u us  max exec %4u us  overruns %u\n", s.name(static_cast<int>(i)),
                ts.runs, ts.max_jitter_us, ts.max_exec_us, ts.overruns);
  }
  std::printf("uart traffic: host->device %u commands, device->host %u messages, rx ring overflows %u\n",
              host_tx_msgs.load(), host_rx_msgs.load(), fw->uart_overflows());
  std::printf("state %s, watchdog resets %u\n", fw_state_name(fw->state()), hal.watchdog_resets());
  fw.reset();
}

// ---------------------------------------------------------------- goodput

struct GoodputResult {
  uint64_t delivered = 0, corrupted = 0, misordered = 0;
  LinkStats tx{};
  ChannelStats ab{}, ba{};
  uint64_t sim_us = 0;
};

void fill_payload(uint32_t idx, uint8_t* p, size_t n) {
  uint32_t x = idx * 2654435761u + 12345u;
  for (size_t i = 0; i < n; ++i) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    p[i] = static_cast<uint8_t>(x);
  }
  for (int k = 0; k < 4; ++k) p[k] = static_cast<uint8_t>(idx >> (8 * k));
}

GoodputResult run_goodput(double ber, uint32_t packets, size_t payload) {
  GoodputResult r;
  LinkPair lp(LinkEndpoint::Config{3000, 1000, 1}, ChannelConfig{ber, 0, 0, 0, 87}, 2024);
  int64_t last = -1;
  lp.side_b().on_deliver = [&](const uint8_t* d, size_t n) {
    uint8_t expect[kMaxPayload];
    const uint32_t idx = n >= 4 ? (static_cast<uint32_t>(d[0]) | (static_cast<uint32_t>(d[1]) << 8) |
                                   (static_cast<uint32_t>(d[2]) << 16) | (static_cast<uint32_t>(d[3]) << 24))
                                : 0;
    fill_payload(idx, expect, payload);
    if (n != payload || idx >= packets || std::memcmp(d, expect, n) != 0) {
      ++r.corrupted;
    } else if (static_cast<int64_t>(idx) <= last) {
      ++r.misordered;
    } else {
      last = idx;
      ++r.delivered;
    }
  };
  uint8_t p[kMaxPayload];
  uint32_t next = 0;
  for (;;) {
    if (!lp.a().busy() && next < packets) {
      fill_payload(next, p, payload);
      lp.a().send(p, payload, lp.now());
      ++next;
    }
    if (next == packets && !lp.a().busy()) break;
    if (!lp.step()) break;
  }
  r.sim_us = lp.now();
  while (lp.step()) {
  }
  r.tx = lp.a().stats();
  r.ab = lp.ab().stats();
  r.ba = lp.ba().stats();
  return r;
}

void print_goodput(double ber, uint32_t packets, size_t payload, const GoodputResult& r) {
  const double secs = r.sim_us / 1e6;
  const double goodput = r.delivered * payload / secs;
  const double line = 1e6 / SimHal::kUartByteUs;  // bytes/s at 115200 8N1
  const double retx_rate = r.tx.data_first_tx ? static_cast<double>(r.tx.data_retx) / r.tx.data_first_tx : 0;
  std::printf("BER %.0e, %u packets x %zu B payload:\n", ber, packets, payload);
  std::printf("  delivered %llu, corrupted accepted %llu, duplicated/reordered %llu, abandoned %u\n",
              static_cast<unsigned long long>(r.delivered), static_cast<unsigned long long>(r.corrupted),
              static_cast<unsigned long long>(r.misordered), r.tx.abandoned);
  std::printf("  goodput %.0f B/s over %.0f s simulated = %.1f%% of the %.0f B/s line rate\n", goodput, secs,
              100.0 * goodput / line, line);
  const uint64_t frames = static_cast<uint64_t>(r.tx.data_first_tx) + r.tx.data_retx;
  std::printf("  data frames sent %llu, retransmissions %u = %.2f per packet, %.1f%% of data frames (%u on NAK, %u on timeout)\n",
              static_cast<unsigned long long>(frames), r.tx.data_retx, retx_rate,
              frames ? 100.0 * r.tx.data_retx / frames : 0.0, r.tx.retx_on_nak, r.tx.retx_on_timeout);
  std::printf("  frames damaged in transit %llu (data %llu, ack/nak %llu)\n",
              static_cast<unsigned long long>(r.ab.frames_damaged + r.ba.frames_damaged),
              static_cast<unsigned long long>(r.ab.frames_damaged),
              static_cast<unsigned long long>(r.ba.frames_damaged));
  std::printf("  damaged frames a CRC-16/CCITT-only check would have accepted: %llu (dual CRC-16 accepted 0)\n",
              static_cast<unsigned long long>(r.ab.ccitt_only_accepts + r.ba.ccitt_only_accepts));
}

void bench_goodput() {
  section("Protocol goodput (virtual time, 115200 baud, stop-and-wait ARQ, 3 ms ACK timeout)");
  const size_t payload = 16;
  for (double ber : {0.0, 1e-3}) {
    const uint32_t n = 100000;
    print_goodput(ber, n, payload, run_goodput(ber, n, payload));
  }
  const uint32_t n = 1000000;
  const auto t0 = std::chrono::steady_clock::now();
  const GoodputResult r = run_goodput(1e-2, n, payload);
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  print_goodput(1e-2, n, payload, r);
  std::printf("  (wall time %.1f s)\n", wall);
}

// -------------------------------------------------------------------- OTA

void bench_ota() {
  section("OTA power-loss recovery (500 interrupted updates)");
  OtaRig rig(31337, 1, 20000);
  std::mt19937 rng(8);
  uint32_t good = 0, bad = 0, finished = 0, rolled = 0, invalid = 0, unexpected = 0;
  uint32_t old_after_cut = 0, new_after_cut = 0, cuts = 0, boots = 0;
  uint64_t ops = 0;
  uint32_t regions[static_cast<int>(CutRegion::Count)] = {};
  uint32_t version = 1;
  for (int i = 0; i < 500; ++i) {
    OtaExperiment e;
    e.image_fails_self_test = (i % 5 == 4);  // every 5th image never confirms
    e.new_version = e.image_fails_self_test ? 100000 + static_cast<uint32_t>(i) : ++version;
    e.image_size = 2000 + rng() % 58000;
    e.extra_cut_prob = 0.25;
    const uint32_t before = rig.running_version();
    rig.run(e);
    (e.image_fails_self_test ? bad : good) += 1;
    finished += e.finished_new;
    rolled += e.rolled_back && e.final_version == before;
    invalid += e.invalid_boots;
    unexpected += e.unexpected_errors;
    cuts += e.cuts;
    boots += e.boots;
    ops += e.clean_ops;
    ++regions[static_cast<int>(e.first_cut)];
    (e.version_after_first_cut == before ? old_after_cut : new_after_cut) += 1;
    if (e.image_fails_self_test) {
      version = rig.running_version();
    }
  }
  std::printf("updates: %u good images, %u images that fail their self-test; sizes 2-60 KB\n", good, bad);
  std::printf("power cuts injected: %u (one at a uniformly random flash op per update, plus 25%% chance of another on each retry)\n",
              cuts);
  std::printf("first cut landed in:");
  for (int k = 1; k < static_cast<int>(CutRegion::Count); ++k) {
    std::printf(" %s %u%s", cut_region_name(static_cast<CutRegion>(k)), regions[k],
                k + 1 < static_cast<int>(CutRegion::Count) ? "," : "\n");
  }
  std::printf("boot right after the first cut: previous image %u, new image (trial) %u\n", old_after_cut,
              new_after_cut);
  std::printf("boots checked: %u, boots without a valid image: %u\n", boots, invalid);
  std::printf("good images running and confirmed after recovery: %u/%u\n", finished, good);
  std::printf("bad images rolled back to the previous version: %u/%u\n", rolled, bad);
  std::printf("unexpected OTA errors: %u, NOR write violations: %llu, mean flash ops per clean update: %.0f\n",
              unexpected, static_cast<unsigned long long>(rig.hal().flash().nor_violations()),
              static_cast<double>(ops) / 500.0);
}

// --------------------------------------------------------------- footprint

void bench_ram() {
  section("Static RAM footprint (sizeof, host build)");
  std::printf("Firmware instance (everything below + glue): %zu B\n", sizeof(Firmware));
  std::printf("  SpscRing<uint8_t,256> UART RX:  %zu B (64-byte aligned counters on the host; build with -DSHF_RING_ALIGN=4 on an MCU)\n",
              sizeof(SpscRing<uint8_t, Firmware::kUartRingSize>));
  std::printf("  LinkEndpoint:                   %zu B\n", sizeof(LinkEndpoint));
  std::printf("  Scheduler (8 tasks + histogram): %zu B (jitter histogram %zu B)\n", sizeof(Scheduler),
              sizeof(JitterHistogram));
  std::printf("  ComplementaryFilter:            %zu B\n", sizeof(ComplementaryFilter));
  std::printf("  FaultManager:                   %zu B\n", sizeof(FaultManager));
  std::printf("  OtaManager:                     %zu B\n", sizeof(OtaManager));
}

}  // namespace

int main() {
  std::printf("sensor-hub-firmware bench: HOST-SIMULATOR SANITY RUN (simulated HAL on a laptop, not an MCU)\n");
  bench_jitter(10.0);
  bench_goodput();
  bench_ota();
  bench_ram();
  return 0;
}
