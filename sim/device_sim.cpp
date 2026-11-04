#include "device_sim.hpp"

#include <algorithm>
#include <random>

#include "shf/crc.hpp"

namespace shf::sim {

Image make_image(uint32_t version, size_t size) {
  Image img;
  img.version = version;
  img.body.resize(size);
  std::mt19937 rng(version * 2654435761u + 17u);
  for (auto& b : img.body) b = static_cast<uint8_t>(rng());
  img.crc = crc32(img.body.data(), img.body.size());
  return img;
}

void provision(SimHal& hal, const Image& img) {
  const FlashLayout& l = kDefaultLayout;
  for (uint32_t a = 0; a < hal.flash_size(); a += l.sector_size) hal.flash_erase_sector(a);
  hal.flash_write(l.slot_addr[0] + kImageHeaderSize, img.body.data(), img.body.size());
  uint8_t hdr[kImageHeaderSize];
  encode_image_header(ImageHeader{kImageMagic, img.version, static_cast<uint32_t>(img.body.size()), img.crc},
                      hdr);
  hal.flash_write(l.slot_addr[0], hdr, 20);
  Bcb b{0, 0, kNoSlot, 0};
  bcb_write(hal, l, b);
}

bool DeviceSim::boot() {
  fw_.reset();
  ++boots_;
  boot_time_us_ = hal_.micros();
  last_boot_ = bootloader_run(hal_, kDefaultLayout);
  if (!last_boot_.ok) {
    ++failed_boots_;
    return false;
  }
  hung_ = image_hangs && image_hangs(last_boot_.version);
  fw_.emplace(hal_, BootInfo{last_boot_.slot, last_boot_.version, last_boot_.trial}, cfg_);
  return true;
}

void DeviceSim::handle_reset() {
  fw_.reset();
  hal_.apply_reset();
  boot();
}

void DeviceSim::run_for(uint64_t us) {
  const uint64_t end = hal_.micros() + us;
  while (hal_.micros() < end) {
    if (!fw_) {  // nothing bootable: time passes, nothing runs
      hal_.advance_to(end);
      break;
    }
    const uint64_t now = hal_.micros();
    const uint64_t next = hung_ ? std::min(end, now + 1000) : std::min(fw_->next_wakeup(), end);
    hal_.advance_to(std::max(next, now));
    if (hal_.reset_pending()) {
      handle_reset();
      continue;
    }
    if (!hung_) {
      while (fw_->poll()) {
        if (hal_.reset_pending()) break;
      }
      if (hal_.reset_pending()) {
        handle_reset();
        continue;
      }
    }
    if (on_step) on_step(hal_.micros());
  }
}

}  // namespace shf::sim
