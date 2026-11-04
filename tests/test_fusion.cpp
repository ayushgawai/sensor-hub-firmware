#include <cmath>
#include <random>

#include "shf/fusion.hpp"
#include "sim_hal.hpp"
#include "test.hpp"

using namespace shf;

namespace {

constexpr double kPi = 3.14159265358979323846;

double bam_to_rad(uint32_t bam) { return static_cast<int32_t>(bam) * (kPi / 2147483648.0); }
double wrap_rad(double a) { return std::remainder(a, 2 * kPi); }
double deg(double rad) { return rad * 180.0 / kPi; }

// Double-precision reference with the same equations and nominal constants.
struct FloatFilter {
  double alpha = 0.98;
  double roll = 0, pitch = 0;
  bool init = false;
  void update(const ImuRaw& s) {
    const double ar = std::atan2(static_cast<double>(s.ay), static_cast<double>(s.az));
    const double ap = std::atan2(-static_cast<double>(s.ax),
                                 std::sqrt(static_cast<double>(s.ay) * s.ay + static_cast<double>(s.az) * s.az));
    if (!init) {
      roll = ar;
      pitch = ap;
      init = true;
      return;
    }
    const double k = 1.0 / 65.5 / 1000.0 * kPi / 180.0;  // rad per LSB per sample
    roll = blend(roll, s.gx * k, ar);
    pitch = blend(pitch, s.gy * k, ap);
  }
  double blend(double a, double d, double acc) const {
    const double pred = a + d;
    return wrap_rad(pred + (1 - alpha) * wrap_rad(acc - pred));
  }
};

}  // namespace

TEST(fusion_cordic_atan2_and_isqrt) {
  double max_err = 0;
  for (int i = 0; i < 3600; ++i) {
    const double a = (i / 10.0 - 180.0) * kPi / 180.0;
    for (double r : {1.0, 7.0, 300.0, 8192.0, 32767.0, 46000.0}) {
      const int32_t x = static_cast<int32_t>(std::lround(r * std::cos(a)));
      const int32_t y = static_cast<int32_t>(std::lround(r * std::sin(a)));
      if (x == 0 && y == 0) continue;
      const double ref = std::atan2(static_cast<double>(y), static_cast<double>(x));
      const double err = std::fabs(wrap_rad(bam_to_rad(atan2_bam(y, x)) - ref));
      if (err > max_err) max_err = err;
    }
  }
  CHECK(max_err < 2e-6);
  NOTE("CORDIC atan2 max error %.2e rad (%.2e deg)", max_err, deg(max_err));

  std::mt19937 rng(5);
  for (int i = 0; i < 200000; ++i) {
    const uint32_t v = rng();
    const uint32_t s = isqrt32(v);
    CHECK(static_cast<uint64_t>(s) * s <= v && static_cast<uint64_t>(s + 1) * (s + 1) > v);
  }
  CHECK(isqrt32(0) == 0 && isqrt32(1) == 1 && isqrt32(0xFFFFFFFFu) == 65535);
}

TEST(fusion_q15_vs_float_reference) {
  struct Scenario {
    const char* name;
    sim::SimImu::Motion m;
  };
  sim::SimImu::Motion slow{};
  sim::SimImu::Motion spin{};
  spin.roll_amp_deg = 0;
  spin.roll_spin_dps = 90;  // crosses +/-180 every 4 s
  spin.pitch_amp_deg = 10;
  sim::SimImu::Motion fast{};
  fast.roll_amp_deg = 60;
  fast.roll_freq_hz = 2.0;
  fast.pitch_amp_deg = 45;
  fast.pitch_freq_hz = 1.5;
  sim::SimImu::Motion still{};
  still.roll_amp_deg = 0;
  still.pitch_amp_deg = 0;
  still.roll_offset_deg = 30;
  still.pitch_offset_deg = -60;
  const Scenario sc[] = {{"slow sway", slow}, {"roll spin 90 dps", spin}, {"fast 2 Hz", fast}, {"static", still}};

  for (const Scenario& s : sc) {
    sim::SimImu imu(77);
    imu.set_motion(s.m);
    ComplementaryFilter fx;
    FloatFilter fl;
    double max_fx_fl = 0, sum_sq = 0, max_fx_truth = 0, max_fl_truth = 0;
    const int n = 60 * 1000;  // 60 s at 1 kHz
    for (int i = 0; i < n; ++i) {
      double rt, pt;
      const ImuRaw raw = imu.sample(i / 1000.0, &rt, &pt);
      fx.update(raw);
      fl.update(raw);
      const double er = std::fabs(wrap_rad(bam_to_rad(fx.roll_bam()) - fl.roll));
      const double ep = std::fabs(wrap_rad(bam_to_rad(fx.pitch_bam()) - fl.pitch));
      max_fx_fl = std::max(max_fx_fl, std::max(er, ep));
      sum_sq += er * er + ep * ep;
      if (i > 2000) {  // after the filter settles
        max_fx_truth = std::max(max_fx_truth, std::fabs(wrap_rad(bam_to_rad(fx.roll_bam()) - rt)));
        max_fx_truth = std::max(max_fx_truth, std::fabs(wrap_rad(bam_to_rad(fx.pitch_bam()) - pt)));
        max_fl_truth = std::max(max_fl_truth, std::fabs(wrap_rad(fl.roll - rt)));
        max_fl_truth = std::max(max_fl_truth, std::fabs(wrap_rad(fl.pitch - pt)));
      }
    }
    const double rms = std::sqrt(sum_sq / (2.0 * n));
    CHECK(deg(max_fx_fl) < 0.01);
    // The Q15 output (top 16 bits) adds at most one LSB = 0.0055 deg.
    CHECK(std::fabs(std::remainder(fx.roll_q15() / 32768.0 * 180.0 - deg(bam_to_rad(fx.roll_bam())), 360.0)) <
          0.0056);
    NOTE("%-17s fixed vs float: max %.5f deg, rms %.5f deg | vs truth: fixed %.2f deg, float %.2f deg",
         s.name, deg(max_fx_fl), deg(rms), deg(max_fx_truth), deg(max_fl_truth));
  }
}
