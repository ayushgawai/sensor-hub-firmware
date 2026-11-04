#include "shf/fault.hpp"

namespace shf {

const char* fw_state_name(FwState s) {
  switch (s) {
    case FwState::Normal: return "NORMAL";
    case FwState::Degraded: return "DEGRADED";
    case FwState::Brownout: return "BROWNOUT";
    case FwState::SafeMode: return "SAFE_MODE";
  }
  return "?";
}

void FaultManager::on_boot(RetainedState& r, ResetCause cause, uint64_t now_us) {
  retained_ = &r;
  if (r.magic != kRetainedMagic || cause == ResetCause::PowerOn) {
    r.magic = kRetainedMagic;
    r.consecutive_wdt_resets = 0;
    r.total_resets = 0;
  } else {
    ++r.total_resets;
  }
  if (cause == ResetCause::Watchdog) ++r.consecutive_wdt_resets;
  last_sample_us_ = now_us;  // grace period for the first sample
  if (r.consecutive_wdt_resets >= cfg_.safe_mode_wdt_resets) {
    flags_ |= kFaultWatchdogLoop;
    enter(FwState::SafeMode, now_us);
  } else {
    enter(FwState::Normal, now_us);
  }
}

void FaultManager::on_imu_sample(const ImuRaw& s, uint64_t now_us) {
  const bool same = have_last_ && s.ax == last_.ax && s.ay == last_.ay && s.az == last_.az &&
                    s.gx == last_.gx && s.gy == last_.gy && s.gz == last_.gz;
  identical_ = same ? identical_ + 1 : 0;
  last_ = s;
  have_last_ = true;
  last_sample_us_ = now_us;
}

bool FaultManager::sensor_bad(uint64_t now_us) const {
  return identical_ >= cfg_.stuck_identical_samples ||
         now_us - last_sample_us_ > cfg_.no_data_timeout_us;
}

void FaultManager::enter(FwState s, uint64_t now_us) {
  state_ = s;
  state_since_us_ = now_us;
}

FaultManager::Actions FaultManager::step(uint64_t now_us, uint16_t supply_mv) {
  Actions a{false, false};
  last_mv_ = supply_mv;

  // Brownout detector: debounced entry, hysteresis plus hold time on exit.
  if (supply_mv < cfg_.brownout_enter_mv) {
    if (low_count_ < 255) ++low_count_;
  } else {
    low_count_ = 0;
  }
  if (!brownout_ && low_count_ >= cfg_.brownout_debounce) {
    brownout_ = true;
    flags_ |= kFaultBrownout;
    high_since_us_ = 0;
    a.abort_flash_ops = true;
  } else if (brownout_) {
    if (supply_mv >= cfg_.brownout_exit_mv) {
      if (high_since_us_ == 0) high_since_us_ = now_us;
      if (now_us - high_since_us_ >= cfg_.brownout_exit_us) {
        brownout_ = false;
        flags_ &= ~kFaultBrownout;
      }
    } else {
      high_since_us_ = 0;
    }
  }

  if (brownout_ && (state_ == FwState::Normal || state_ == FwState::Degraded)) {
    enter(FwState::Brownout, now_us);
    return a;
  }
  if (state_ == FwState::Brownout) {
    if (brownout_) return a;
    enter(FwState::Normal, now_us);
  }

  const bool bad = sensor_bad(now_us);
  switch (state_) {
    case FwState::Normal:
      if (bad) {
        flags_ |= (identical_ >= cfg_.stuck_identical_samples) ? kFaultImuStuck : kFaultImuNoData;
        enter(FwState::Degraded, now_us);
        retries_ = 1;
        last_retry_us_ = now_us;
        good_since_us_ = 0;
        a.reset_imu = true;
        ++imu_resets_total_;
      } else if (retained_ && retained_->consecutive_wdt_resets != 0 &&
                 now_us - state_since_us_ >= cfg_.wdt_count_clear_us) {
        retained_->consecutive_wdt_resets = 0;
      }
      break;
    case FwState::Degraded:
      if (!bad) {
        if (good_since_us_ == 0) good_since_us_ = now_us;
        if (now_us - good_since_us_ >= cfg_.imu_good_us) {
          flags_ &= ~(kFaultImuStuck | kFaultImuNoData);
          enter(FwState::Normal, now_us);
        }
      } else {
        good_since_us_ = 0;
        if (now_us - last_retry_us_ >= cfg_.imu_retry_interval_us) {
          if (retries_ >= cfg_.imu_max_retries) {
            flags_ |= kFaultImuRecoveryFailed;
            enter(FwState::SafeMode, now_us);
          } else {
            ++retries_;
            last_retry_us_ = now_us;
            a.reset_imu = true;
            ++imu_resets_total_;
          }
        }
      }
      break;
    case FwState::Brownout:
    case FwState::SafeMode:
      break;
  }
  return a;
}

bool FaultManager::clear_safe_mode(uint64_t now_us) {
  if (state_ != FwState::SafeMode || brownout_ || last_mv_ < cfg_.brownout_exit_mv) return false;
  flags_ = 0;
  identical_ = 0;
  have_last_ = false;
  last_sample_us_ = now_us;
  if (retained_) retained_->consecutive_wdt_resets = 0;
  enter(FwState::Normal, now_us);
  return true;
}

bool WatchdogMonitor::service(Hal& hal) {
  const bool all = (seen_ & required_) == required_;
  if (all) {
    hal.watchdog_kick();
  } else {
    ++missed_;
  }
  seen_ = 0;
  return all;
}

}  // namespace shf
