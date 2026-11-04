// Hardware abstraction layer. The firmware core talks to hardware only
// through this interface. sim/sim_hal.* provides the host-side backend.
#pragma once

#include <cstddef>
#include <cstdint>

namespace shf {

// Raw IMU sample, sensor LSB units.
// Scales match a common 6-axis part: accel +/-4 g, gyro +/-500 dps.
struct ImuRaw {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

constexpr int32_t kAccelLsbPerG = 8192;
constexpr int32_t kGyroLsbPerDpsX10 = 655;  // 65.5 LSB per deg/s
constexpr uint32_t kImuRateHz = 1000;

enum class ResetCause : uint8_t { PowerOn = 0, Watchdog = 1, Software = 2 };

// RAM that survives warm resets (a .noinit section on a real MCU).
// Cleared on power-on.
struct RetainedState {
  uint32_t magic;
  uint32_t consecutive_wdt_resets;
  uint32_t total_resets;
};
constexpr uint32_t kRetainedMagic = 0x5245544Eu;

using IsrHandler = void (*)(void* ctx);
using UartRxHandler = void (*)(void* ctx, uint8_t byte);

class Hal {
 public:
  // Timer: free-running microsecond counter and a 1 kHz tick interrupt.
  virtual uint64_t micros() = 0;
  virtual void set_tick_handler(IsrHandler fn, void* ctx) = 0;

  // UART: the RX handler runs in interrupt context, one call per byte.
  virtual void set_uart_rx_handler(UartRxHandler fn, void* ctx) = 0;
  virtual void uart_write(const uint8_t* data, size_t n) = 0;

  // IMU: returns true when a new sample arrived since the last read.
  virtual bool imu_read(ImuRaw& out) = 0;
  virtual void imu_reset() = 0;

  // Flash (NOR semantics: erase sets bytes to 0xFF, writes can only clear
  // bits). All calls return false on failure, including power loss.
  virtual uint32_t flash_size() const = 0;
  virtual uint32_t flash_sector_size() const = 0;
  virtual bool flash_read(uint32_t addr, uint8_t* dst, size_t n) = 0;
  virtual bool flash_erase_sector(uint32_t addr) = 0;
  virtual bool flash_write(uint32_t addr, const uint8_t* src, size_t n) = 0;

  // Power, watchdog, reset.
  virtual uint16_t supply_mv() = 0;
  virtual void watchdog_start(uint32_t timeout_us) = 0;
  virtual void watchdog_kick() = 0;
  virtual ResetCause reset_cause() const = 0;
  virtual RetainedState& retained() = 0;
  virtual void system_reset() = 0;

 protected:
  ~Hal() = default;
};

}  // namespace shf
