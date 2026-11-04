#include "shf/scheduler.hpp"

namespace shf {

void JitterHistogram::record(uint32_t us) {
  if (us < kBuckets) {
    ++counts_[us];
  } else {
    ++overflow_;
  }
  if (us > max_) max_ = us;
  ++count_;
}

uint32_t JitterHistogram::percentile(uint32_t per_mille) const {
  if (count_ == 0) return 0;
  // Smallest value v such that at least per_mille/1000 of samples are <= v.
  const uint64_t target = (count_ * per_mille + 999) / 1000;
  uint64_t seen = 0;
  for (uint32_t i = 0; i < kBuckets; ++i) {
    seen += counts_[i];
    if (seen >= target) return i;
  }
  return max_;
}

void JitterHistogram::clear() {
  for (auto& c : counts_) c = 0;
  overflow_ = 0;
  max_ = 0;
  count_ = 0;
}

int Scheduler::add_task(const char* name, TaskFn fn, void* ctx, uint32_t period_us,
                        uint8_t priority, uint64_t first_release_us) {
  if (count_ >= kMaxTasks || period_us == 0 || fn == nullptr) return -1;
  Task& t = tasks_[count_];
  t.name = name;
  t.fn = fn;
  t.ctx = ctx;
  t.period_us = period_us;
  t.priority = priority;
  t.enabled = true;
  t.next_release_us = first_release_us;
  t.stats = TaskStats{};
  return static_cast<int>(count_++);
}

void Scheduler::set_enabled(int id, bool enabled) {
  if (id < 0 || static_cast<size_t>(id) >= count_) return;
  Task& t = tasks_[id];
  if (enabled && !t.enabled) {
    // Re-anchor so a re-enabled task does not count its idle time as overruns.
    const uint64_t now = clock_(clock_ctx_);
    t.next_release_us = now;
  }
  t.enabled = enabled;
}

bool Scheduler::run_once() {
  const uint64_t now = clock_(clock_ctx_);
  int best = -1;
  for (size_t i = 0; i < count_; ++i) {
    const Task& t = tasks_[i];
    if (!t.enabled || t.next_release_us > now) continue;
    if (best < 0 || t.priority < tasks_[best].priority) best = static_cast<int>(i);
  }
  if (best < 0) return false;

  Task& t = tasks_[best];
  const uint64_t release = t.next_release_us;
  const uint32_t jitter = static_cast<uint32_t>(now - release);
  t.fn(t.ctx, now);
  const uint64_t end = clock_(clock_ctx_);

  TaskStats& s = t.stats;
  ++s.runs;
  s.sum_jitter_us += jitter;
  if (jitter > s.max_jitter_us) s.max_jitter_us = jitter;
  const uint32_t exec = static_cast<uint32_t>(end - now);
  if (exec > s.max_exec_us) s.max_exec_us = exec;
  if (best == probe_) hist_.record(jitter);

  t.next_release_us = release + t.period_us;
  // If the release after next is already due, the next one is late by a full
  // period: skip releases until at most one is pending.
  if (end >= t.next_release_us + t.period_us) {
    const uint64_t behind = (end - t.next_release_us) / t.period_us;
    s.overruns += static_cast<uint32_t>(behind);
    t.next_release_us += behind * t.period_us;
  }
  return true;
}

uint64_t Scheduler::next_release() const {
  uint64_t best = UINT64_MAX;
  for (size_t i = 0; i < count_; ++i) {
    if (tasks_[i].enabled && tasks_[i].next_release_us < best) best = tasks_[i].next_release_us;
  }
  return best;
}

}  // namespace shf
