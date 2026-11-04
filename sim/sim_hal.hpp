// Host-side simulated hardware. Not part of the firmware core: uses the heap,
// std::function and <random> freely.
#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <random>
#include <vector>

#include "shf/hal.hpp"

namespace shf::sim {

// NOR flash with power-cut injection. A "mutating op" is one erase or write
// call. When armed, the selected op is torn (partially applied) and the
// flash then fails every call until power is restored.
class SimFlash {
 public:
  enum class OpKind : uint8_t { None, Erase, Write };

  SimFlash(uint32_t size, uint32_t sector, uint64_t seed);

  bool read(uint32_t addr, uint8_t* dst, size_t n);
  bool erase_sector(uint32_t addr);
  bool write(uint32_t addr, const uint8_t* src, size_t n);

  // Tear the op with index mutating_ops() + ops_from_now.
  void arm_power_cut(uint64_t ops_from_now);
  void disarm() { cut_at_ = UINT64_MAX; }
  bool power_lost() const { return power_lost_; }
  void restore_power() { power_lost_ = false; }

  uint64_t mutating_ops() const { return ops_; }
  uint64_t nor_violations() const { return nor_violations_; }  // writes that tried 0 -> 1
  OpKind last_cut_kind() const { return cut_kind_; }
  uint32_t last_cut_addr() const { return cut_addr_; }

  uint32_t size() const { return static_cast<uint32_t>(mem_.size()); }
  uint32_t sector_size() const { return sector_; }
  std::vector<uint8_t>& raw() { return mem_; }

 private:
  bool take_cut(OpKind k, uint32_t addr);

  std::vector<uint8_t> mem_;
  uint32_t sector_;
  std::mt19937_64 rng_;
  uint64_t ops_ = 0;
  uint64_t cut_at_ = UINT64_MAX;
  bool power_lost_ = false;
  uint64_t nor_violations_ = 0;
  OpKind cut_kind_ = OpKind::None;
  uint32_t cut_addr_ = 0;
};

// Simulated 6-axis IMU sampling a scripted motion. Produces quantized raw
// samples plus the ground-truth angles.
class SimImu {
 public:
  struct Motion {
    double roll_offset_deg = 10.0, roll_amp_deg = 25.0, roll_freq_hz = 0.5;
    double pitch_offset_deg = -5.0, pitch_amp_deg = 20.0, pitch_freq_hz = 0.3;
    double roll_spin_dps = 0.0;  // constant extra roll rate (exercises +/-180 wrap)
  };
  struct Noise {
    double accel_g = 0.004;   // 1-sigma
    double gyro_dps = 0.05;   // 1-sigma
    double gyro_bias_dps = 0.2;
  };
  enum class Fault : uint8_t { None, Frozen, NoData };

  explicit SimImu(uint64_t seed) : rng_(seed) {}
  void set_motion(const Motion& m) { motion_ = m; }
  void set_noise(const Noise& n) { noise_ = n; }
  void set_fault(Fault f, bool clears_on_reset) {
    fault_ = f;
    clears_on_reset_ = clears_on_reset;
  }
  Fault fault() const { return fault_; }
  void reset_requested() {
    ++resets_;
    if (clears_on_reset_) fault_ = Fault::None;
  }
  uint32_t resets() const { return resets_; }

  // Sample at time t (seconds). Truth angles in radians.
  ImuRaw sample(double t, double* roll_true, double* pitch_true);

 private:
  Motion motion_{};
  Noise noise_{};
  std::mt19937_64 rng_;
  Fault fault_ = Fault::None;
  bool clears_on_reset_ = true;
  uint32_t resets_ = 0;
};

class SimHal final : public Hal {
 public:
  enum class ClockMode { Virtual, RealTime };
  static constexpr uint32_t kUartByteUs = 87;  // 115200 baud, 8N1

  explicit SimHal(ClockMode mode = ClockMode::Virtual, uint64_t seed = 1);

  // --- Hal ---
  uint64_t micros() override;
  void set_tick_handler(IsrHandler fn, void* ctx) override;
  void set_uart_rx_handler(UartRxHandler fn, void* ctx) override;
  void uart_write(const uint8_t* data, size_t n) override;
  bool imu_read(ImuRaw& out) override;
  void imu_reset() override { imu_.reset_requested(); }
  uint32_t flash_size() const override { return flash_.size(); }
  uint32_t flash_sector_size() const override { return flash_.sector_size(); }
  bool flash_read(uint32_t a, uint8_t* d, size_t n) override { return flash_.read(a, d, n); }
  bool flash_erase_sector(uint32_t a) override { return flash_.erase_sector(a); }
  bool flash_write(uint32_t a, const uint8_t* s, size_t n) override { return flash_.write(a, s, n); }
  uint16_t supply_mv() override { return supply_mv_.load(std::memory_order_relaxed); }
  void watchdog_start(uint32_t timeout_us) override;
  void watchdog_kick() override;
  ResetCause reset_cause() const override { return reset_cause_; }
  RetainedState& retained() override { return retained_; }
  void system_reset() override { request_reset(ResetCause::Software); }

  // --- simulation control (virtual clock) ---
  // Moves time forward, firing tick / UART RX / watchdog events in order.
  // Stops early if a reset becomes pending.
  void advance_to(uint64_t t_us);
  uint64_t next_event_us() const;
  // --- simulation control (real-time clock) ---
  void service_real_time();  // fire due tick ISRs

  // Host -> device bytes, delivered to the RX ISR at the UART byte rate.
  void uart_inject(const uint8_t* d, size_t n);
  // Calls the RX ISR handler immediately (for an "interrupt" thread).
  void uart_isr_byte(uint8_t b) {
    if (uart_rx_) uart_rx_(uart_ctx_, b);
  }
  std::function<void(const uint8_t*, size_t)> on_uart_tx;

  void set_supply_mv(uint16_t mv) { supply_mv_.store(mv, std::memory_order_relaxed); }
  SimImu& imu() { return imu_; }
  SimFlash& flash() { return flash_; }

  bool reset_pending() const { return reset_pending_; }
  ResetCause pending_cause() const { return pending_cause_; }
  // Applies the pending reset: peripherals and handlers are reset, retained
  // RAM and flash survive.
  void apply_reset();
  // Full power cycle: retained RAM is lost.
  void power_cycle();
  uint32_t watchdog_resets() const { return wdt_resets_; }
  uint64_t last_kick_us() const { return last_kick_us_; }

 private:
  void request_reset(ResetCause c);
  uint64_t real_now() const;

  ClockMode mode_;
  uint64_t now_us_ = 0;
  uint64_t t0_ns_ = 0;

  IsrHandler tick_ = nullptr;
  void* tick_ctx_ = nullptr;
  uint64_t next_tick_us_ = 1000;

  UartRxHandler uart_rx_ = nullptr;
  void* uart_ctx_ = nullptr;
  std::deque<uint8_t> uart_q_;
  uint64_t uart_next_us_ = 0;

  SimImu imu_;
  int64_t imu_last_index_ = -1;
  ImuRaw imu_frozen_{};

  SimFlash flash_;
  std::atomic<uint16_t> supply_mv_{3300};

  bool wdt_running_ = false;
  uint32_t wdt_timeout_us_ = 0;
  uint64_t last_kick_us_ = 0;
  uint32_t wdt_resets_ = 0;

  RetainedState retained_{};
  ResetCause reset_cause_ = ResetCause::PowerOn;
  bool reset_pending_ = false;
  ResetCause pending_cause_ = ResetCause::PowerOn;
};

}  // namespace shf::sim
