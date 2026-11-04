// Device harness: bootloader + firmware on a SimHal, with reset handling.
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "shf/firmware.hpp"
#include "shf/ota.hpp"
#include "sim_hal.hpp"

namespace shf::sim {

struct Image {
  uint32_t version;
  std::vector<uint8_t> body;
  uint32_t crc;
};

// Deterministic pseudo-random image body for a version.
Image make_image(uint32_t version, size_t size);

// Factory programming: image into slot A, BCB pointing at it.
void provision(SimHal& hal, const Image& img);

class DeviceSim {
 public:
  explicit DeviceSim(SimHal& hal, const FirmwareConfig& cfg = FirmwareConfig{})
      : hal_(hal), cfg_(cfg) {}

  // Runs the bootloader and starts the firmware. False if nothing bootable.
  bool boot();
  // Advances virtual time, running the main loop and handling resets.
  void run_for(uint64_t us);

  // Simulates a hung main loop: time passes but the loop never runs.
  void set_hung(bool h) { hung_ = h; }
  // If set and returns true for the booted version, that image hangs right
  // after boot (a bad build).
  std::function<bool(uint32_t version)> image_hangs;
  // Called after every simulation step (host-side peers poll here).
  std::function<void(uint64_t now_us)> on_step;

  Firmware* fw() { return fw_ ? &*fw_ : nullptr; }
  const BootDecision& last_boot() const { return last_boot_; }
  uint32_t boots() const { return boots_; }
  uint64_t last_boot_us() const { return boot_time_us_; }
  uint32_t failed_boots() const { return failed_boots_; }

 private:
  void handle_reset();

  SimHal& hal_;
  FirmwareConfig cfg_;
  std::optional<Firmware> fw_;
  BootDecision last_boot_{};
  bool hung_ = false;
  uint32_t boots_ = 0;
  uint64_t boot_time_us_ = 0;
  uint32_t failed_boots_ = 0;
};

}  // namespace shf::sim
