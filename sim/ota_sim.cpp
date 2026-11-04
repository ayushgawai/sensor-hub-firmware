#include "ota_sim.hpp"

#include <algorithm>

#include "shf/firmware.hpp"
#include "shf/ota.hpp"

namespace shf::sim {

const char* cut_region_name(CutRegion r) {
  switch (r) {
    case CutRegion::None: return "none";
    case CutRegion::SlotErase: return "slot erase";
    case CutRegion::BodyWrite: return "image body write";
    case CutRegion::HeaderWrite: return "image header write";
    case CutRegion::BcbErase: return "boot-control erase";
    case CutRegion::BcbWrite: return "boot-control write";
    case CutRegion::Count: break;
  }
  return "?";
}

OtaRig::OtaRig(uint64_t seed, uint32_t initial_version, size_t image_size)
    : hal_(SimHal::ClockMode::Virtual, seed), rng_(seed) {
  provision(hal_, make_image(initial_version, image_size));
  boot_ = bootloader_run(hal_, kDefaultLayout);
  running_version_ = boot_.version;
}

CutRegion OtaRig::classify() {
  SimFlash& f = hal_.flash();
  const uint32_t a = f.last_cut_addr();
  const bool erase = f.last_cut_kind() == SimFlash::OpKind::Erase;
  if (a < kDefaultLayout.slot_addr[0]) return erase ? CutRegion::BcbErase : CutRegion::BcbWrite;
  if (erase) return CutRegion::SlotErase;
  const bool header = a == kDefaultLayout.slot_addr[0] || a == kDefaultLayout.slot_addr[1];
  return header ? CutRegion::HeaderWrite : CutRegion::BodyWrite;
}

void OtaRig::reboot(OtaExperiment& exp, uint32_t old_version, uint32_t new_version, bool count_only) {
  SimFlash& f = hal_.flash();
  for (;;) {
    f.restore_power();
    ++exp.boots;
    boot_ = bootloader_run(hal_, kDefaultLayout);
    if (!f.power_lost()) break;
    // Power failed again while the bootloader was updating the BCB.
    if (!count_only) ++exp.cuts;
  }
  const bool valid = boot_.ok && verify_slot(hal_, kDefaultLayout, boot_.slot) &&
                     (boot_.version == old_version || boot_.version == new_version);
  if (!valid) ++exp.invalid_boots;
}

void OtaRig::drive(OtaExperiment& exp, const Image& img, bool count_only) {
  const uint32_t old_version = running_version_;
  SimFlash& f = hal_.flash();
  bool downloaded = false;
  bool first_cut_seen = false;

  for (int step = 0; step < 400; ++step) {
    if (f.power_lost()) {
      if (!count_only) {
        ++exp.cuts;
        if (!first_cut_seen) exp.first_cut = classify();
      }
      reboot(exp, old_version, img.version, count_only);
      if (!first_cut_seen) {
        first_cut_seen = true;
        exp.version_after_first_cut = boot_.version;
      }
      if (exp.extra_cut_prob > 0 &&
          std::uniform_real_distribution<double>(0, 1)(rng_) < exp.extra_cut_prob) {
        f.arm_power_cut(std::uniform_int_distribution<uint64_t>(0, exp.clean_ops)(rng_));
      }
      continue;
    }

    if (boot_.trial) {
      if (exp.image_fails_self_test) {
        reboot(exp, old_version, img.version, count_only);  // it crashes; watchdog resets it
        continue;
      }
      OtaManager ota(hal_, kDefaultLayout, boot_.slot, true);
      const OtaStatus st = ota.confirm();
      if (st == OtaStatus::Ok) {
        boot_.trial = false;
      } else if (!f.power_lost()) {
        ++exp.unexpected_errors;
        break;
      }
      continue;
    }

    if (boot_.version == img.version) {
      exp.finished_new = true;
      break;
    }
    if (downloaded && exp.image_fails_self_test) {
      exp.rolled_back = true;
      break;
    }

    OtaManager ota(hal_, kDefaultLayout, boot_.slot, false);
    OtaStatus st = ota.begin(static_cast<uint32_t>(img.body.size()), img.crc, img.version);
    for (size_t off = 0; st == OtaStatus::Ok && off < img.body.size(); off += kOtaChunkMax) {
      const size_t n = std::min(kOtaChunkMax, img.body.size() - off);
      st = ota.write(static_cast<uint32_t>(off), img.body.data() + off, n);
    }
    if (st == OtaStatus::Ok) st = ota.finish();
    if (st != OtaStatus::Ok) {
      if (!f.power_lost()) {
        ++exp.unexpected_errors;
        break;
      }
      continue;
    }
    downloaded = true;
    reboot(exp, old_version, img.version, count_only);  // device restarts into the new image
  }
  exp.final_version = boot_.version;
  running_version_ = boot_.version;
}

void OtaRig::run(OtaExperiment& exp) {
  const Image img = make_image(exp.new_version, exp.image_size);
  SimFlash& f = hal_.flash();

  // Dry run without cuts to count the flash operations of a clean update.
  const std::vector<uint8_t> snapshot = f.raw();
  const BootDecision boot_snapshot = boot_;
  const uint32_t version_snapshot = running_version_;
  {
    OtaExperiment dry = exp;
    const uint64_t ops0 = f.mutating_ops();
    drive(dry, img, true);
    exp.clean_ops = f.mutating_ops() - ops0;
  }
  f.raw() = snapshot;
  boot_ = boot_snapshot;
  running_version_ = version_snapshot;

  const uint64_t cut =
      exp.cut_at_op >= 0
          ? static_cast<uint64_t>(exp.cut_at_op)
          : std::uniform_int_distribution<uint64_t>(0, exp.clean_ops - 1)(rng_);
  f.arm_power_cut(cut);
  drive(exp, img, false);
  f.disarm();
}

}  // namespace shf::sim
