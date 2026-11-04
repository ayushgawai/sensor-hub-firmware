#include "sim_hal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include "shf/ota.hpp"

namespace shf::sim {

// ---------------------------------------------------------------- SimFlash

SimFlash::SimFlash(uint32_t size, uint32_t sector, uint64_t seed)
    : mem_(size, 0xFF), sector_(sector), rng_(seed) {}

bool SimFlash::read(uint32_t addr, uint8_t* dst, size_t n) {
  if (power_lost_ || addr + n > mem_.size()) return false;
  std::memcpy(dst, mem_.data() + addr, n);
  return true;
}

bool SimFlash::take_cut(OpKind k, uint32_t addr) {
  const uint64_t idx = ops_++;
  if (idx != cut_at_) return false;
  cut_at_ = UINT64_MAX;
  power_lost_ = true;
  cut_kind_ = k;
  cut_addr_ = addr;
  return true;
}

bool SimFlash::erase_sector(uint32_t addr) {
  if (power_lost_ || addr % sector_ != 0 || addr + sector_ > mem_.size()) return false;
  uint8_t* p = mem_.data() + addr;
  if (take_cut(OpKind::Erase, addr)) {
    // Torn erase: each byte is fully erased with probability q, otherwise
    // some of its bits have been pulled up to 1.
    const double q = std::uniform_real_distribution<double>(0.0, 1.0)(rng_);
    std::bernoulli_distribution full(q);
    for (uint32_t i = 0; i < sector_; ++i) {
      p[i] = full(rng_) ? 0xFF : static_cast<uint8_t>(p[i] | (rng_() & 0xFF));
    }
    return false;
  }
  std::memset(p, 0xFF, sector_);
  return true;
}

bool SimFlash::write(uint32_t addr, const uint8_t* src, size_t n) {
  if (power_lost_ || addr + n > mem_.size()) return false;
  uint8_t* p = mem_.data() + addr;
  for (size_t i = 0; i < n; ++i) {
    if ((src[i] & ~p[i]) != 0) ++nor_violations_;
  }
  if (take_cut(OpKind::Write, addr)) {
    // Torn write: a prefix lands, the next byte lands partially.
    const size_t k = std::uniform_int_distribution<size_t>(0, n)(rng_);
    for (size_t i = 0; i < k; ++i) p[i] &= src[i];
    if (k < n) p[k] &= static_cast<uint8_t>(src[k] | (rng_() & 0xFF));
    return false;
  }
  for (size_t i = 0; i < n; ++i) p[i] &= src[i];  // NOR: bits only go 1 -> 0
  return true;
}

void SimFlash::arm_power_cut(uint64_t ops_from_now) { cut_at_ = ops_ + ops_from_now; }

// ------------------------------------------------------------------ SimImu

ImuRaw SimImu::sample(double t, double* roll_true, double* pitch_true) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double d2r = kPi / 180.0;
  const Motion& m = motion_;
  const double wr = 2 * kPi * m.roll_freq_hz;
  const double wp = 2 * kPi * m.pitch_freq_hz;
  const double roll = (m.roll_offset_deg + m.roll_amp_deg * std::sin(wr * t) + m.roll_spin_dps * t) * d2r;
  const double pitch = (m.pitch_offset_deg + m.pitch_amp_deg * std::sin(wp * t)) * d2r;
  const double roll_rate = (m.roll_amp_deg * wr * std::cos(wr * t) + m.roll_spin_dps);  // deg/s
  const double pitch_rate = m.pitch_amp_deg * wp * std::cos(wp * t);                     // deg/s
  if (roll_true) *roll_true = std::remainder(roll, 2 * kPi);
  if (pitch_true) *pitch_true = pitch;

  // Body rates for ZYX Euler angles with zero yaw.
  const double p = roll_rate;
  const double q = pitch_rate * std::cos(roll);
  const double r = -pitch_rate * std::sin(roll);
  // Specific force at rest (gravity reaction) in the body frame.
  const double ax = -std::sin(pitch);
  const double ay = std::sin(roll) * std::cos(pitch);
  const double az = std::cos(roll) * std::cos(pitch);

  std::normal_distribution<double> na(0.0, noise_.accel_g);
  std::normal_distribution<double> ng(0.0, noise_.gyro_dps);
  auto q16 = [](double v) {
    const double r = std::nearbyint(v);
    return static_cast<int16_t>(std::max(-32768.0, std::min(32767.0, r)));
  };
  const double ga = kAccelLsbPerG;
  const double gg = kGyroLsbPerDpsX10 / 10.0;
  ImuRaw s;
  s.ax = q16((ax + na(rng_)) * ga);
  s.ay = q16((ay + na(rng_)) * ga);
  s.az = q16((az + na(rng_)) * ga);
  s.gx = q16((p + noise_.gyro_bias_dps + ng(rng_)) * gg);
  s.gy = q16((q - noise_.gyro_bias_dps + ng(rng_)) * gg);
  s.gz = q16((r + ng(rng_)) * gg);
  return s;
}

// ------------------------------------------------------------------ SimHal

SimHal::SimHal(ClockMode mode, uint64_t seed)
    : mode_(mode),
      imu_(seed ^ 0x1111),
      flash_(kDefaultLayout.slot_addr[1] + kDefaultLayout.slot_size, kDefaultLayout.sector_size,
             seed ^ 0x2222) {
  t0_ns_ = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

uint64_t SimHal::real_now() const {
  const uint64_t ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
  return (ns - t0_ns_) / 1000;
}

uint64_t SimHal::micros() { return mode_ == ClockMode::Virtual ? now_us_ : real_now(); }

void SimHal::set_tick_handler(IsrHandler fn, void* ctx) {
  tick_ = fn;
  tick_ctx_ = ctx;
}

void SimHal::set_uart_rx_handler(UartRxHandler fn, void* ctx) {
  uart_rx_ = fn;
  uart_ctx_ = ctx;
}

void SimHal::uart_write(const uint8_t* data, size_t n) {
  if (on_uart_tx) on_uart_tx(data, n);
}

bool SimHal::imu_read(ImuRaw& out) {
  // Samples are generated lazily: the latest 1 kHz sample at or before now.
  const int64_t idx = static_cast<int64_t>(micros() / (1000000 / kImuRateHz));
  if (idx <= imu_last_index_) return false;
  switch (imu_.fault()) {
    case SimImu::Fault::NoData:
      return false;
    case SimImu::Fault::Frozen:
      imu_last_index_ = idx;
      out = imu_frozen_;
      return true;
    case SimImu::Fault::None:
      break;
  }
  imu_last_index_ = idx;
  out = imu_.sample(static_cast<double>(idx) / kImuRateHz, nullptr, nullptr);
  imu_frozen_ = out;
  return true;
}

void SimHal::watchdog_start(uint32_t timeout_us) {
  wdt_running_ = true;
  wdt_timeout_us_ = timeout_us;
  last_kick_us_ = micros();
}

void SimHal::watchdog_kick() { last_kick_us_ = micros(); }

void SimHal::request_reset(ResetCause c) {
  if (reset_pending_) return;
  reset_pending_ = true;
  pending_cause_ = c;
}

uint64_t SimHal::next_event_us() const {
  uint64_t t = tick_ ? next_tick_us_ : UINT64_MAX;
  if (!uart_q_.empty()) t = std::min(t, uart_next_us_);
  if (wdt_running_) t = std::min(t, last_kick_us_ + wdt_timeout_us_ + 1);
  return t;
}

void SimHal::advance_to(uint64_t t_us) {
  while (!reset_pending_) {
    const uint64_t e = next_event_us();
    if (e > t_us) break;
    if (e > now_us_) now_us_ = e;
    if (wdt_running_ && now_us_ > last_kick_us_ + wdt_timeout_us_) {
      ++wdt_resets_;
      request_reset(ResetCause::Watchdog);
      return;
    }
    if (tick_ && next_tick_us_ <= now_us_) {
      next_tick_us_ += 1000;
      tick_(tick_ctx_);
    }
    if (!uart_q_.empty() && uart_next_us_ <= now_us_) {
      const uint8_t b = uart_q_.front();
      uart_q_.pop_front();
      uart_next_us_ = now_us_ + kUartByteUs;
      if (uart_rx_) uart_rx_(uart_ctx_, b);
    }
  }
  if (!reset_pending_ && t_us > now_us_) now_us_ = t_us;
}

void SimHal::service_real_time() {
  const uint64_t now = real_now();
  if (wdt_running_ && now > last_kick_us_ + wdt_timeout_us_) {
    ++wdt_resets_;
    request_reset(ResetCause::Watchdog);
  }
  while (tick_ && next_tick_us_ <= now) {
    next_tick_us_ += 1000;
    tick_(tick_ctx_);
  }
}

void SimHal::uart_inject(const uint8_t* d, size_t n) {
  if (uart_q_.empty()) uart_next_us_ = std::max(uart_next_us_, now_us_ + kUartByteUs);
  uart_q_.insert(uart_q_.end(), d, d + n);
}

void SimHal::apply_reset() {
  reset_cause_ = pending_cause_;
  reset_pending_ = false;
  tick_ = nullptr;
  uart_rx_ = nullptr;
  uart_q_.clear();
  wdt_running_ = false;
  next_tick_us_ = micros() + 1000;
}

void SimHal::power_cycle() {
  retained_ = RetainedState{};
  pending_cause_ = ResetCause::PowerOn;
  reset_pending_ = true;
  apply_reset();
  flash_.restore_power();
}

}  // namespace shf::sim
