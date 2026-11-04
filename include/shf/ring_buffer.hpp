// Lock-free single-producer single-consumer ring buffer.
//
// Intended use: the producer is an interrupt handler (UART RX), the consumer
// is the main loop. Head and tail are free-running 32-bit counters, so all N
// slots are usable and no slot is wasted to tell full from empty.
//
// Memory ordering: the producer writes the slot, then publishes head with
// release. The consumer reads head with acquire, then reads the slot. The
// same pairing in the other direction protects slot reuse. On a single-core
// Cortex-M these compile to plain loads/stores plus barriers.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#ifndef SHF_RING_ALIGN
#define SHF_RING_ALIGN 64  // avoids false sharing on the host; set to 4 on an MCU
#endif

namespace shf {

template <typename T, size_t N>
class SpscRing {
  static_assert(N >= 2 && (N & (N - 1)) == 0, "capacity must be a power of two");
  static_assert(N <= (1u << 31), "capacity too large for 32-bit counters");
  static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
  static_assert(std::atomic<uint32_t>::is_always_lock_free, "need lock-free 32-bit atomics");

 public:
  // Producer side only.
  bool push(const T& v) {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    const uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail == N) {
      // Single writer, so load+store is safe and avoids an RMW instruction
      // (Cortex-M0 has none).
      overflows_.store(overflows_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
      return false;
    }
    buf_[head & (N - 1)] = v;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  // Consumer side only.
  bool pop(T& out) {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    const uint32_t head = head_.load(std::memory_order_acquire);
    if (head == tail) return false;
    out = buf_[tail & (N - 1)];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  // Approximate when called concurrently; exact from either side when the
  // other side is idle.
  size_t size() const {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }
  bool empty() const { return size() == 0; }
  static constexpr size_t capacity() { return N; }
  uint32_t overflows() const { return overflows_.load(std::memory_order_relaxed); }

 private:
  alignas(SHF_RING_ALIGN) std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> overflows_{0};  // written by producer only
  alignas(SHF_RING_ALIGN) std::atomic<uint32_t> tail_{0};
  alignas(SHF_RING_ALIGN) T buf_[N];
};

}  // namespace shf
