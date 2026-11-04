#include <random>

#include "device_sim.hpp"
#include "ota_sim.hpp"
#include "shf/firmware.hpp"
#include "shf/ota.hpp"
#include "test.hpp"

using namespace shf;
using namespace shf::sim;

namespace {

OtaStatus download(SimHal& hal, const BootDecision& b, const Image& img, bool corrupt = false) {
  OtaManager ota(hal, kDefaultLayout, b.slot, b.trial);
  OtaStatus st = ota.begin(static_cast<uint32_t>(img.body.size()), img.crc, img.version);
  for (size_t off = 0; st == OtaStatus::Ok && off < img.body.size(); off += kOtaChunkMax) {
    const size_t n = std::min(kOtaChunkMax, img.body.size() - off);
    std::vector<uint8_t> chunk(img.body.begin() + static_cast<long>(off),
                               img.body.begin() + static_cast<long>(off + n));
    if (corrupt && off == 0) chunk[3] ^= 0x10;
    st = ota.write(static_cast<uint32_t>(off), chunk.data(), n);
  }
  return st == OtaStatus::Ok ? ota.finish() : st;
}

}  // namespace

TEST(ota_clean_update_trial_and_confirm) {
  SimHal hal(SimHal::ClockMode::Virtual, 1);
  provision(hal, make_image(1, 30000));
  BootDecision b = bootloader_run(hal, kDefaultLayout);
  REQUIRE(b.ok && b.version == 1 && b.slot == 0 && !b.trial);
  CHECK(download(hal, b, make_image(2, 25000)) == OtaStatus::Ok);
  b = bootloader_run(hal, kDefaultLayout);
  CHECK(b.ok && b.version == 2 && b.slot == 1 && b.trial);
  OtaManager running(hal, kDefaultLayout, b.slot, b.trial);
  CHECK(running.begin(100, 0, 3) == OtaStatus::NotAllowed);  // must confirm first
  CHECK(running.confirm() == OtaStatus::Ok);
  b = bootloader_run(hal, kDefaultLayout);
  CHECK(b.ok && b.version == 2 && b.slot == 1 && !b.trial);
  // And back the other way.
  CHECK(download(hal, b, make_image(3, 1000)) == OtaStatus::Ok);
  b = bootloader_run(hal, kDefaultLayout);
  CHECK(b.version == 3 && b.slot == 0 && b.trial);
  CHECK(hal.flash().nor_violations() == 0);
}

TEST(ota_rejects_bad_crc_and_bad_args) {
  SimHal hal(SimHal::ClockMode::Virtual, 2);
  provision(hal, make_image(1, 5000));
  BootDecision b = bootloader_run(hal, kDefaultLayout);
  CHECK(download(hal, b, make_image(2, 5000), /*corrupt=*/true) == OtaStatus::CrcMismatch);
  b = bootloader_run(hal, kDefaultLayout);
  CHECK(b.ok && b.version == 1 && !b.trial);
  OtaManager ota(hal, kDefaultLayout, b.slot, false);
  CHECK(ota.begin(0, 0, 2) == OtaStatus::BadArgs);
  CHECK(ota.begin(kDefaultLayout.slot_size, 0, 2) == OtaStatus::BadArgs);
  CHECK(ota.begin(100, 0, 2) == OtaStatus::Ok);
  const uint8_t d[10] = {};
  CHECK(ota.write(10, d, 10) == OtaStatus::BadArgs);  // out of order
  CHECK(ota.write(0, d, 10) == OtaStatus::Ok);
  CHECK(ota.write(0, d, 10) == OtaStatus::Ok);  // repeat is ignored
  CHECK(ota.write(95, d, 10) == OtaStatus::BadArgs);  // past the end
  CHECK(ota.finish() == OtaStatus::BadState);  // incomplete
}

TEST(ota_bootloader_fallbacks) {
  SimHal hal(SimHal::ClockMode::Virtual, 3);
  provision(hal, make_image(1, 5000));
  BootDecision b = bootloader_run(hal, kDefaultLayout);
  REQUIRE(download(hal, b, make_image(2, 5000)) == OtaStatus::Ok);
  b = bootloader_run(hal, kDefaultLayout);
  OtaManager(hal, kDefaultLayout, b.slot, true).confirm();
  // Both BCB records destroyed: factory default (slot A) still boots.
  hal.flash_erase_sector(kDefaultLayout.bcb_addr[0]);
  hal.flash_erase_sector(kDefaultLayout.bcb_addr[1]);
  b = bootloader_run(hal, kDefaultLayout);
  CHECK(b.ok && b.slot == 0 && b.version == 1);
  // Active slot corrupted: falls back to the other slot and repairs the BCB.
  hal.flash().raw()[kDefaultLayout.slot_addr[0] + 500] ^= 0xFF;
  b = bootloader_run(hal, kDefaultLayout);
  CHECK(b.ok && b.slot == 1 && b.version == 2 && b.fell_back);
  b = bootloader_run(hal, kDefaultLayout);
  CHECK(b.ok && b.slot == 1 && !b.fell_back);
}

TEST(ota_power_cut_at_every_flash_op) {
  // Exhaustive: one cut at each flash operation of a small update, three
  // torn-write patterns each. The device must always boot a valid image,
  // and must finish the update on retry.
  uint64_t total = 0;
  uint32_t invalid = 0, incomplete = 0, errors = 0;
  for (uint64_t seed = 1; seed <= 3; ++seed) {
    OtaExperiment probe;
    probe.new_version = 2;
    probe.image_size = 3000;
    probe.cut_at_op = 1 << 20;  // never reached: measures clean_ops
    OtaRig dry(seed, 1, 3000);
    dry.run(probe);
    for (int64_t op = 0; op < static_cast<int64_t>(probe.clean_ops); ++op) {
      OtaRig rig(seed * 1000 + static_cast<uint64_t>(op), 1, 3000);
      OtaExperiment e;
      e.new_version = 2;
      e.image_size = 3000;
      e.cut_at_op = op;
      rig.run(e);
      ++total;
      invalid += e.invalid_boots;
      incomplete += !e.finished_new;
      errors += e.unexpected_errors;
      CHECK(rig.hal().flash().nor_violations() == 0);
    }
  }
  CHECK(invalid == 0);
  CHECK(incomplete == 0);
  CHECK(errors == 0);
  NOTE("%llu single-cut runs: %u invalid boots, %u unfinished updates",
       static_cast<unsigned long long>(total), invalid, incomplete);
}

TEST(ota_random_multi_cut_chained_updates) {
  OtaRig rig(42, 1, 20000);
  uint32_t invalid = 0, cuts = 0, unfinished = 0;
  std::mt19937 rng(4);
  for (uint32_t v = 2; v < 202; ++v) {
    OtaExperiment e;
    e.new_version = v;
    e.image_size = 1000 + rng() % 40000;
    e.extra_cut_prob = 0.5;
    rig.run(e);
    invalid += e.invalid_boots;
    cuts += e.cuts;
    unfinished += !e.finished_new;
    CHECK(e.unexpected_errors == 0);
  }
  CHECK(invalid == 0);
  CHECK(unfinished == 0);
  CHECK(rig.running_version() == 201);
  CHECK(rig.hal().flash().nor_violations() == 0);
  NOTE("200 chained updates, %u power cuts, %u invalid boots", cuts, invalid);
}

TEST(ota_failed_self_test_rolls_back) {
  OtaRig rig(7, 1, 8000);
  uint32_t rolled = 0, invalid = 0;
  for (int i = 0; i < 100; ++i) {
    OtaExperiment e;
    e.new_version = 1000 + static_cast<uint32_t>(i);
    e.image_size = 8000;
    e.image_fails_self_test = true;
    e.extra_cut_prob = 0.3;
    rig.run(e);
    rolled += e.rolled_back && e.final_version == 1;
    invalid += e.invalid_boots;
  }
  CHECK(rolled == 100);
  CHECK(invalid == 0);
  NOTE("100 bad images with power cuts: %u rolled back to v1, %u invalid boots", rolled, invalid);
}
