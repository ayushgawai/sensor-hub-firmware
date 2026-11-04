// Fixed-priority cooperative scheduler for periodic tasks.
//
// Each call to run_once() runs at most one task: the highest-priority task
// whose release time has passed. Tasks run to completion, so a long
// low-priority task delays a higher-priority one. That delay shows up as
// release jitter, which the scheduler measures:
//
//   jitter = actual start time - scheduled release time
//
// Releases are anchored to the original schedule (no drift). If a task
// falls more than a full period behind, the missed releases are skipped and
// counted as overruns.
#pragma once

#include <cstddef>
#include <cstdint>

namespace shf {

using TaskFn = void (*)(void* ctx, uint64_t now_us);
using ClockFn = uint64_t (*)(void* ctx);

struct TaskStats {
  uint32_t runs;
  uint32_t overruns;
  uint32_t max_jitter_us;
  uint32_t max_exec_us;
  uint64_t sum_jitter_us;
};

// 1 us buckets with an overflow bucket. Percentiles in the overflow range
// report the exact maximum as an upper bound.
class JitterHistogram {
 public:
  static constexpr uint32_t kBuckets = 512;
  void record(uint32_t us);
  // per_mille: 500 = p50, 990 = p99.
  uint32_t percentile(uint32_t per_mille) const;
  uint32_t max() const { return max_; }
  uint64_t count() const { return count_; }
  void clear();

 private:
  uint32_t counts_[kBuckets] = {};
  uint32_t overflow_ = 0;
  uint32_t max_ = 0;
  uint64_t count_ = 0;
};

class Scheduler {
 public:
  static constexpr size_t kMaxTasks = 8;

  Scheduler(ClockFn clock, void* clock_ctx) : clock_(clock), clock_ctx_(clock_ctx) {}

  // Lower number = higher priority. Returns task id or -1 when full.
  int add_task(const char* name, TaskFn fn, void* ctx, uint32_t period_us, uint8_t priority,
               uint64_t first_release_us);
  void set_enabled(int id, bool enabled);

  // Runs at most one ready task. Returns true if a task ran.
  bool run_once();
  // Earliest pending release among enabled tasks (UINT64_MAX if none).
  uint64_t next_release() const;

  // Record the release jitter of this task in the histogram.
  void set_jitter_probe(int id) { probe_ = id; }
  const JitterHistogram& jitter() const { return hist_; }
  void clear_jitter() { hist_.clear(); }

  const TaskStats& stats(int id) const { return tasks_[id].stats; }
  const char* name(int id) const { return tasks_[id].name; }
  size_t task_count() const { return count_; }

 private:
  struct Task {
    const char* name;
    TaskFn fn;
    void* ctx;
    uint32_t period_us;
    uint8_t priority;
    bool enabled;
    uint64_t next_release_us;
    TaskStats stats;
  };

  ClockFn clock_;
  void* clock_ctx_;
  Task tasks_[kMaxTasks] = {};
  size_t count_ = 0;
  int probe_ = -1;
  JitterHistogram hist_;
};

}  // namespace shf
