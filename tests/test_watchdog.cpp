// Watchdog recovery and fault state machine, on the full firmware in the
// virtual-time simulator.
#include "device_sim.hpp"
#include "host_peer.hpp"
#include "test.hpp"

using namespace shf;
using namespace shf::sim;

namespace {

struct Rig {
  SimHal hal{SimHal::ClockMode::Virtual, 11};
  DeviceSim dev{hal};
  Rig() {
    provision(hal, make_image(1, 4096));
    hal.power_cycle();
  }
};

}  // namespace

TEST(watchdog_normal_operation_no_resets) {
  Rig r;
  REQUIRE(r.dev.boot());
  r.dev.run_for(3000000);
  REQUIRE(r.dev.fw());
  CHECK(r.dev.fw()->state() == FwState::Normal);
  CHECK(r.hal.watchdog_resets() == 0);
  CHECK(r.dev.fw()->counters().imu_samples >= 2990);
  CHECK(r.dev.fw()->watchdog_misses() == 0);
}

TEST(watchdog_recovers_hung_main_loop) {
  Rig r;
  REQUIRE(r.dev.boot());
  r.dev.run_for(500000);
  const uint64_t t_hang = r.hal.micros();
  r.dev.set_hung(true);
  r.dev.run_for(300000);
  CHECK(r.hal.watchdog_resets() == 1);
  CHECK(r.dev.boots() == 2);
  CHECK(r.hal.reset_cause() == ResetCause::Watchdog);
  CHECK(r.hal.retained().consecutive_wdt_resets == 1);
  // The reset happens one watchdog timeout (100 ms) after the last kick,
  // and the last kick is at most one health period (10 ms) before the hang.
  const uint64_t t_reset = r.dev.last_boot_us();
  CHECK(t_reset > t_hang + 90000 && t_reset < t_hang + 130000);
  REQUIRE(r.dev.fw());
  CHECK(r.dev.fw()->state() == FwState::Normal);
  const uint32_t before = r.dev.fw()->counters().imu_samples;
  r.dev.run_for(200000);
  CHECK(r.dev.fw()->counters().imu_samples >= before + 190);
  NOTE("hang at %.3f s, watchdog reset and reboot by %.3f s",
       t_hang / 1e6, t_reset / 1e6);
}

TEST(watchdog_catches_stalled_task) {
  // Only the comms task stalls; the loop still runs, so a naive "kick from
  // the loop" watchdog would never fire. The check-in monitor withholds kicks.
  Rig r;
  REQUIRE(r.dev.boot());
  r.dev.run_for(200000);
  r.dev.fw()->debug_stall_comms(true);
  r.dev.run_for(300000);
  CHECK(r.hal.watchdog_resets() == 1);
  REQUIRE(r.dev.fw());
  CHECK(r.dev.fw()->state() == FwState::Normal);
  CHECK(r.hal.reset_cause() == ResetCause::Watchdog);
}

TEST(watchdog_reset_loop_enters_safe_mode_and_clears) {
  Rig r;
  HostPeer host(r.hal, 0.0, 5);
  r.dev.on_step = [&](uint64_t now) { host.step(now); };
  REQUIRE(r.dev.boot());
  for (int i = 0; i < 3; ++i) {
    r.dev.run_for(100000);
    r.dev.set_hung(true);
    r.dev.run_for(200000);
  }
  CHECK(r.hal.watchdog_resets() == 3);
  REQUIRE(r.dev.fw());
  CHECK(r.dev.fw()->state() == FwState::SafeMode);
  CHECK((r.dev.fw()->fault_flags() & kFaultWatchdogLoop) != 0);
  // SAFE_MODE: IMU task off, watchdog still serviced, link still up.
  const uint32_t samples = r.dev.fw()->counters().imu_samples;
  r.dev.run_for(1000000);
  CHECK(r.hal.watchdog_resets() == 3);
  CHECK(r.dev.fw()->counters().imu_samples == samples);
  host.queue(msg_simple(kMsgClearFault));
  r.dev.run_for(300000);
  CHECK(host.last_status(kMsgClearFault) == 0);
  CHECK(r.dev.fw()->state() == FwState::Normal);
  CHECK(r.hal.retained().consecutive_wdt_resets == 0);
  r.dev.run_for(200000);
  CHECK(r.dev.fw()->counters().imu_samples > samples + 150);
}

TEST(watchdog_counter_clears_after_stable_run) {
  Rig r;
  REQUIRE(r.dev.boot());
  r.dev.run_for(100000);
  r.dev.set_hung(true);
  r.dev.run_for(200000);
  CHECK(r.hal.retained().consecutive_wdt_resets == 1);
  r.dev.run_for(11000000);  // 10 s of NORMAL clears the count
  CHECK(r.hal.retained().consecutive_wdt_resets == 0);
}

TEST(fault_stuck_sensor_recovers_after_imu_reset) {
  Rig r;
  REQUIRE(r.dev.boot());
  r.dev.run_for(300000);
  r.hal.imu().set_fault(SimImu::Fault::Frozen, /*clears_on_reset=*/true);
  r.dev.run_for(60000);  // 50 identical samples + one health period
  CHECK(r.dev.fw()->state() == FwState::Degraded);
  CHECK((r.dev.fw()->fault_flags() & kFaultImuStuck) != 0);
  r.dev.run_for(200000);
  CHECK(r.hal.imu().resets() >= 1);
  CHECK(r.dev.fw()->state() == FwState::Normal);
  CHECK(r.hal.watchdog_resets() == 0);
}

TEST(fault_dead_sensor_goes_to_safe_mode) {
  Rig r;
  REQUIRE(r.dev.boot());
  r.dev.run_for(300000);
  r.hal.imu().set_fault(SimImu::Fault::NoData, /*clears_on_reset=*/false);
  r.dev.run_for(40000);
  CHECK(r.dev.fw()->state() == FwState::Degraded);
  CHECK((r.dev.fw()->fault_flags() & kFaultImuNoData) != 0);
  r.dev.run_for(700000);  // 5 resets 100 ms apart, then give up
  CHECK(r.dev.fw()->state() == FwState::SafeMode);
  CHECK((r.dev.fw()->fault_flags() & kFaultImuRecoveryFailed) != 0);
  CHECK(r.hal.imu().resets() == 5);
  r.dev.run_for(1000000);
  CHECK(r.hal.watchdog_resets() == 0);  // safe mode keeps the watchdog fed
}

TEST(fault_brownout_blocks_flash_and_recovers) {
  Rig r;
  HostPeer host(r.hal, 0.0, 6);
  r.dev.on_step = [&](uint64_t now) { host.step(now); };
  REQUIRE(r.dev.boot());
  r.dev.run_for(200000);
  r.hal.set_supply_mv(2900);
  r.dev.run_for(40000);
  CHECK(r.dev.fw()->state() == FwState::Brownout);
  const Image img = make_image(2, 1000);
  host.queue(msg_ota_begin(1000, img.crc, 2));
  r.dev.run_for(200000);
  CHECK(host.last_status(kMsgOtaBegin) == static_cast<int>(OtaStatus::NotAllowed));
  r.hal.set_supply_mv(3050);  // between thresholds: stays in brownout
  r.dev.run_for(300000);
  CHECK(r.dev.fw()->state() == FwState::Brownout);
  r.hal.set_supply_mv(3300);
  r.dev.run_for(60000);
  CHECK(r.dev.fw()->state() == FwState::Brownout);  // 100 ms hold not yet met
  r.dev.run_for(80000);
  CHECK(r.dev.fw()->state() == FwState::Normal);
  CHECK(r.hal.watchdog_resets() == 0);
}

TEST(fault_brownout_aborts_ota_in_progress) {
  Rig r;
  HostPeer host(r.hal, 0.0, 8);
  r.dev.on_step = [&](uint64_t now) { host.step(now); };
  REQUIRE(r.dev.boot());
  const Image img = make_image(2, 2000);
  host.queue(msg_ota_begin(2000, img.crc, 2));
  host.queue(msg_ota_chunk(0, img.body.data(), kOtaChunkMax));
  r.dev.run_for(300000);
  CHECK(host.last_status(kMsgOtaBegin) == 0);
  r.hal.set_supply_mv(2800);
  r.dev.run_for(50000);
  r.hal.set_supply_mv(3300);
  r.dev.run_for(200000);
  CHECK(r.dev.fw()->state() == FwState::Normal);
  host.queue(msg_ota_chunk(kOtaChunkMax, img.body.data() + kOtaChunkMax, kOtaChunkMax));
  host.queue(msg_simple(kMsgOtaEnd));
  r.dev.run_for(300000);
  CHECK(host.last_status(kMsgOtaChunk) == static_cast<int>(OtaStatus::BadState));
  CHECK(host.last_status(kMsgOtaEnd) == static_cast<int>(OtaStatus::BadState));
  CHECK(r.dev.boots() == 1);  // no reboot into a half-written image
}
