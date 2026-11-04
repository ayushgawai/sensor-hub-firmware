// Fixed-point helpers and the Q15 complementary filter.
//
// Angle formats:
//   BAM32: uint32_t binary angle, 2^32 = one full turn. Wraps for free.
//   Q15 angle: int16_t, 32768 = pi rad (180 deg). Top 16 bits of BAM32.
// Coefficients are Q15 (32768 = 1.0).
//
// The filter keeps its state in BAM32. At 1 kHz a 1 deg/s rotation moves the
// angle 0.001 deg per sample, which is below the 0.0055 deg step of a Q15
// angle; a Q15-only state would round slow rotations away. Inputs,
// coefficients and outputs are Q15; the accumulator is 32-bit, the same
// split CMSIS-DSP uses for its Q15 filters.
#pragma once

#include <cstdint>

#include "shf/hal.hpp"

namespace shf {

using q15_t = int16_t;

constexpr q15_t q15_from_double(double v) {
  const double s = v * 32768.0;
  return s >= 32767.0 ? 32767 : (s <= -32768.0 ? -32768 : static_cast<q15_t>(s < 0 ? s - 0.5 : s + 0.5));
}

inline q15_t q15_mul(q15_t a, q15_t b) {
  const int32_t p = (static_cast<int32_t>(a) * b + (1 << 14)) >> 15;
  return static_cast<q15_t>(p > 32767 ? 32767 : (p < -32768 ? -32768 : p));
}

// Integer square root, floor(sqrt(v)).
uint32_t isqrt32(uint32_t v);

// atan2(y, x) as BAM32 via 24-iteration CORDIC. Error < 1e-6 rad.
uint32_t atan2_bam(int32_t y, int32_t x);

inline q15_t bam_to_q15(uint32_t bam) {
  return static_cast<q15_t>(static_cast<uint16_t>(bam >> 16));
}
inline int32_t bam_signed(uint32_t bam) {
  // Two's-complement reinterpretation: defined by C++20, and what every
  // C++17 compiler does in practice.
  return static_cast<int32_t>(bam);
}

class ComplementaryFilter {
 public:
  struct Config {
    q15_t alpha;               // gyro weight, e.g. 0.98
    int64_t gyro_bam_per_lsb_q16;  // BAM32 per gyro LSB per sample, Q16
  };

  static constexpr Config default_config() {
    // 2^32 BAM / 360 deg / (65.5 LSB per deg/s) / 1000 samples per s, as Q16.
    return Config{q15_from_double(0.98),
                  ((int64_t{1} << 48) * 10 + (int64_t{360} * kGyroLsbPerDpsX10 * kImuRateHz) / 2) /
                      (int64_t{360} * kGyroLsbPerDpsX10 * kImuRateHz)};
  }

  explicit ComplementaryFilter(const Config& cfg = default_config()) : cfg_(cfg) {}

  void reset() { initialized_ = false; }
  void update(const ImuRaw& s);

  uint32_t roll_bam() const { return roll_; }
  uint32_t pitch_bam() const { return pitch_; }
  q15_t roll_q15() const { return bam_to_q15(roll_); }
  q15_t pitch_q15() const { return bam_to_q15(pitch_); }
  bool initialized() const { return initialized_; }

  // Tilt from the accelerometer alone (BAM32).
  static uint32_t accel_roll(const ImuRaw& s);
  static uint32_t accel_pitch(const ImuRaw& s);

 private:
  uint32_t blend(uint32_t angle, int16_t gyro_lsb, uint32_t accel_angle) const;

  Config cfg_;
  bool initialized_ = false;
  uint32_t roll_ = 0;
  uint32_t pitch_ = 0;
};

}  // namespace shf
