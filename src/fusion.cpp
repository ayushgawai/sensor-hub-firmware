#include "shf/fusion.hpp"

namespace shf {
namespace {

// round(atan(2^-i) / (2*pi) * 2^32)
constexpr uint32_t kAtanBam[24] = {
    0x20000000u, 0x12E4051Eu, 0x09FB385Bu, 0x051111D4u, 0x028B0D43u, 0x0145D7E1u,
    0x00A2F61Eu, 0x00517C55u, 0x0028BE53u, 0x00145F2Fu, 0x000A2F98u, 0x000517CCu,
    0x00028BE6u, 0x000145F3u, 0x0000A2FAu, 0x0000517Du, 0x000028BEu, 0x0000145Fu,
    0x00000A30u, 0x00000518u, 0x0000028Cu, 0x00000146u, 0x000000A3u, 0x00000051u,
};

// Arithmetic right shift that is well defined for negative values.
inline int32_t asr(int32_t v, int s) { return v >= 0 ? (v >> s) : ~((~v) >> s); }

}  // namespace

uint32_t isqrt32(uint32_t v) {
  uint32_t res = 0;
  uint32_t bit = 1u << 30;
  while (bit > v) bit >>= 2;
  while (bit != 0) {
    if (v >= res + bit) {
      v -= res + bit;
      res = (res >> 1) + bit;
    } else {
      res >>= 1;
    }
    bit >>= 2;
  }
  return res;
}

uint32_t atan2_bam(int32_t y, int32_t x) {
  if (x == 0 && y == 0) return 0;
  uint32_t angle = 0;
  if (x < 0) {  // atan2(y, x) = pi + atan2(-y, -x)
    x = -x;
    y = -y;
    angle = 0x80000000u;
  }
  // Normalize so the largest magnitude sits in [2^27, 2^28). CORDIC gain
  // (1.65) times sqrt(2) then stays below 2^31.
  int32_t m = (x > (y < 0 ? -y : y)) ? x : (y < 0 ? -y : y);
  while (m < (1 << 27)) {
    x *= 2;
    y *= 2;
    m *= 2;
  }
  while (m >= (1 << 28)) {
    x = asr(x, 1);
    y = asr(y, 1);
    m >>= 1;
  }
  for (int i = 0; i < 24; ++i) {
    const int32_t xs = asr(x, i);
    const int32_t ys = asr(y, i);
    if (y > 0) {
      x += ys;
      y -= xs;
      angle += kAtanBam[i];
    } else {
      x -= ys;
      y += xs;
      angle -= kAtanBam[i];
    }
  }
  return angle;
}

uint32_t ComplementaryFilter::accel_roll(const ImuRaw& s) { return atan2_bam(s.ay, s.az); }

uint32_t ComplementaryFilter::accel_pitch(const ImuRaw& s) {
  const int32_t ay = s.ay;
  const int32_t az = s.az;
  const uint32_t mag = isqrt32(static_cast<uint32_t>(ay * ay) + static_cast<uint32_t>(az * az));
  return atan2_bam(-static_cast<int32_t>(s.ax), static_cast<int32_t>(mag));
}

uint32_t ComplementaryFilter::blend(uint32_t angle, int16_t gyro_lsb, uint32_t accel_angle) const {
  // Predict with the gyro.
  const int64_t delta = (static_cast<int64_t>(gyro_lsb) * cfg_.gyro_bam_per_lsb_q16 + (1 << 15)) >> 16;
  const uint32_t pred = angle + static_cast<uint32_t>(static_cast<int32_t>(delta));
  // Correct toward the accelerometer by (1 - alpha). The error is a wrapped
  // difference, so the filter behaves at +/-180 deg.
  const int32_t err = bam_signed(accel_angle - pred);
  const int64_t k = 32768 - cfg_.alpha;
  const int64_t corr = (static_cast<int64_t>(err) * k + (1 << 14)) >> 15;
  return pred + static_cast<uint32_t>(static_cast<int32_t>(corr));
}

void ComplementaryFilter::update(const ImuRaw& s) {
  const uint32_t ar = accel_roll(s);
  const uint32_t ap = accel_pitch(s);
  if (!initialized_) {
    roll_ = ar;
    pitch_ = ap;
    initialized_ = true;
    return;
  }
  roll_ = blend(roll_, s.gx, ar);
  pitch_ = blend(pitch_, s.gy, ap);
}

}  // namespace shf
