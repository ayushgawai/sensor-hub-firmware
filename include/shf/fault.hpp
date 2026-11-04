// Fault state machine and watchdog check-in monitor.
//
//            sensor stuck / no data                 retries exhausted
//   NORMAL ---------------------------> DEGRADED ---------------------> SAFE_MODE
//     ^  ^        sensor healthy 50 ms     |                              ^   |
//     |  +---------------------------------+                              |   | clear command
//     |                                                                   |   v
//     |   supply < 3.0 V (3 checks)                          3 watchdog resets  NORMAL
//     +----- BROWNOUT <---- NORMAL/DEGRADED                  in a row at boot
//      supply >= 3.1 V for 100 ms
//
// Brownout is tracked independently of the state so flash writes stay
// blocked during a brownout even in SAFE_MODE.
#pragma once

#include <cstdint>

#include "shf/hal.hpp"

namespace shf {

enum class FwState : uint8_t { Normal = 0, Degraded = 1, Brownout = 2, SafeMode = 3 };
const char* fw_state_name(FwState s);

enum FaultFlag : uint32_t {
  kFaultImuStuck = 1u << 0,
  kFaultImuNoData = 1u << 1,
  kFaultImuRecoveryFailed = 1u << 2,
  kFaultBrownout = 1u << 3,
  kFaultWatchdogLoop = 1u << 4,
};

struct FaultConfig {
  uint32_t stuck_identical_samples = 50;   // identical consecutive samples => frozen
  uint32_t no_data_timeout_us = 20000;     // no data-ready for 20 ms => dead
  uint32_t imu_retry_interval_us = 100000;
  uint8_t imu_max_retries = 5;
  uint32_t imu_good_us = 50000;            // healthy data needed to leave DEGRADED
  uint16_t brownout_enter_mv = 3000;
  uint16_t brownout_exit_mv = 3100;
  uint8_t brownout_debounce = 3;           // consecutive low readings
  uint32_t brownout_exit_us = 100000;
  uint8_t safe_mode_wdt_resets = 3;        // consecutive watchdog resets => SAFE_MODE
  uint32_t wdt_count_clear_us = 10000000;  // 10 s in NORMAL clears the count
};

class FaultManager {
 public:
  struct Actions {
    bool reset_imu;
    bool abort_flash_ops;
  };

  explicit FaultManager(const FaultConfig& cfg = FaultConfig{}) : cfg_(cfg) {}

  void on_boot(RetainedState& r, ResetCause cause, uint64_t now_us);
  void on_imu_sample(const ImuRaw& s, uint64_t now_us);
  Actions step(uint64_t now_us, uint16_t supply_mv);
  // Operator command. Leaves SAFE_MODE if the supply is healthy.
  bool clear_safe_mode(uint64_t now_us);

  FwState state() const { return state_; }
  uint32_t flags() const { return flags_; }
  bool flash_writes_allowed() const { return !brownout_; }
  bool fusion_enabled() const { return state_ == FwState::Normal || state_ == FwState::Brownout; }
  bool imu_active() const { return state_ != FwState::SafeMode; }
  uint32_t imu_resets() const { return imu_resets_total_; }

 private:
  bool sensor_bad(uint64_t now_us) const;
  void enter(FwState s, uint64_t now_us);

  FaultConfig cfg_;
  RetainedState* retained_ = nullptr;
  FwState state_ = FwState::Normal;
  uint32_t flags_ = 0;
  uint64_t state_since_us_ = 0;

  ImuRaw last_{};
  bool have_last_ = false;
  uint32_t identical_ = 0;
  uint64_t last_sample_us_ = 0;
  uint8_t retries_ = 0;
  uint64_t last_retry_us_ = 0;
  uint64_t good_since_us_ = 0;
  uint32_t imu_resets_total_ = 0;

  bool brownout_ = false;
  uint8_t low_count_ = 0;
  uint64_t high_since_us_ = 0;
  uint16_t last_mv_ = 0;
};

// Each critical task sets its bit every time it runs. The health task kicks
// the hardware watchdog only when every required bit is set, then clears
// them. A hung main loop or a stalled task therefore stops the kicks.
class WatchdogMonitor {
 public:
  void set_required(uint32_t mask) { required_ = mask; }
  void checkin(uint32_t bit) { seen_ |= bit; }
  // Returns true if it kicked.
  bool service(Hal& hal);
  uint32_t missed() const { return missed_; }

 private:
  uint32_t required_ = 0;
  uint32_t seen_ = 0;
  uint32_t missed_ = 0;
};

}  // namespace shf
