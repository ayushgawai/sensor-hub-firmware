// Sensor-hub firmware application: wires the HAL, scheduler, ring buffer,
// link protocol, fusion, fault handling and OTA together. One instance is
// statically allocated by the target's main() (or by the simulator). No heap.
//
// Tasks (lower number = higher priority):
//   0 imu        1 kHz   read IMU, stuck detection, fusion, watchdog check-in
//   1 comms      1 kHz   drain UART ring into the link, retransmit timer, TX queue
//   2 telemetry  50 Hz   queue a tilt/status message
//   3 health     100 Hz  fault state machine, watchdog kick, OTA confirm
//
// Application messages (first payload byte):
//   0x01 TELEMETRY   dev->host  uptime_ms u32, roll q15, pitch q15, state, flags, version u32
//   0x02 STATUS      dev->host  request id, status code
//   0x10 OTA_BEGIN   host->dev  size u32, crc32 u32, version u32
//   0x11 OTA_CHUNK   host->dev  offset u32, data
//   0x12 OTA_END     host->dev
//   0x20 CLEAR_FAULT host->dev
//   0x21 REBOOT      host->dev
#pragma once

#include <atomic>
#include <cstdint>

#include "shf/fault.hpp"
#include "shf/fusion.hpp"
#include "shf/hal.hpp"
#include "shf/ota.hpp"
#include "shf/protocol.hpp"
#include "shf/ring_buffer.hpp"
#include "shf/scheduler.hpp"

namespace shf {

enum MsgId : uint8_t {
  kMsgTelemetry = 0x01,
  kMsgStatus = 0x02,
  kMsgOtaBegin = 0x10,
  kMsgOtaChunk = 0x11,
  kMsgOtaEnd = 0x12,
  kMsgClearFault = 0x20,
  kMsgReboot = 0x21,
};
constexpr size_t kOtaChunkMax = kMaxPayload - 5;

struct BootInfo {
  uint8_t slot;
  uint32_t version;
  bool trial;
};

struct FirmwareConfig {
  uint32_t telemetry_period_us = 20000;
  uint32_t confirm_after_us = 2000000;  // healthy time before a trial image confirms
  uint32_t watchdog_timeout_us = 100000;
  uint32_t link_timeout_us = 30000;
  uint16_t link_max_retries = 50;
  FaultConfig faults{};
};

struct FirmwareCounters {
  uint32_t imu_samples;
  uint32_t telemetry_queued;
  uint32_t telemetry_skipped;
  uint32_t messages_rx;
  uint32_t ota_errors;
};

class Firmware {
 public:
  static constexpr size_t kUartRingSize = 256;
  static constexpr uint32_t kCheckinImu = 1u << 0;
  static constexpr uint32_t kCheckinComms = 1u << 1;

  Firmware(Hal& hal, const BootInfo& boot, const FirmwareConfig& cfg = FirmwareConfig{});
  Firmware(const Firmware&) = delete;
  Firmware& operator=(const Firmware&) = delete;

  // One main-loop iteration. Returns true if a task ran.
  bool poll() { return sched_.run_once(); }
  uint64_t next_wakeup() const { return sched_.next_release(); }

  FwState state() const { return faults_.state(); }
  uint32_t fault_flags() const { return faults_.flags(); }
  const BootInfo& boot_info() const { return boot_; }
  bool trial() const { return ota_.trial(); }
  q15_t roll_q15() const { return fusion_.roll_q15(); }
  q15_t pitch_q15() const { return fusion_.pitch_q15(); }
  uint32_t roll_bam() const { return fusion_.roll_bam(); }
  uint32_t pitch_bam() const { return fusion_.pitch_bam(); }
  const Scheduler& scheduler() const { return sched_; }
  Scheduler& scheduler() { return sched_; }
  const LinkEndpoint& link() const { return link_; }
  const FirmwareCounters& counters() const { return counters_; }
  const FaultManager& faults() const { return faults_; }
  uint32_t uart_overflows() const { return uart_rx_.overflows(); }
  uint32_t watchdog_misses() const { return wdt_.missed(); }
  int imu_task_id() const { return t_imu_; }

  // Debug hook: the comms task stops running its body (and checking in),
  // as if it were stuck waiting on something.
  void debug_stall_comms(bool stall) { stall_comms_ = stall; }

 private:
  static uint64_t clock_fn(void* ctx);
  static void tick_isr(void* ctx);
  static void uart_rx_isr(void* ctx, uint8_t b);
  static void link_tx(void* ctx, const uint8_t* bytes, size_t n);
  static void link_deliver(void* ctx, const uint8_t* payload, size_t n);
  static void imu_task(void* ctx, uint64_t now);
  static void comms_task(void* ctx, uint64_t now);
  static void telemetry_task(void* ctx, uint64_t now);
  static void health_task(void* ctx, uint64_t now);

  void handle_message(const uint8_t* p, size_t n);
  void reply_status(uint8_t req, uint8_t status);
  bool enqueue(const uint8_t* p, size_t n);

  struct Msg {
    uint8_t len;
    uint8_t data[kMaxPayload];
  };

  Hal& hal_;
  BootInfo boot_;
  FirmwareConfig cfg_;
  Scheduler sched_;
  int t_imu_ = -1;
  SpscRing<uint8_t, kUartRingSize> uart_rx_;
  LinkEndpoint link_;
  SpscRing<Msg, 4> txq_;
  ComplementaryFilter fusion_;
  FaultManager faults_;
  WatchdogMonitor wdt_;
  OtaManager ota_;
  std::atomic<uint32_t> uptime_ms_{0};
  uint64_t now_us_ = 0;
  uint64_t healthy_since_us_ = 0;
  uint64_t reset_at_us_ = 0;
  FwState last_state_ = FwState::Normal;
  bool stall_comms_ = false;
  FirmwareCounters counters_{};
};

}  // namespace shf
