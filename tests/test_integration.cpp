// End to end: host sends a firmware image over the noisy UART link; the
// device writes it, reboots into the trial image and confirms it.
#include "device_sim.hpp"
#include "host_peer.hpp"
#include "test.hpp"

using namespace shf;
using namespace shf::sim;

namespace {

uint32_t telemetry_version(const std::vector<uint8_t>& t) {
  if (t.size() < 15) return 0;
  return static_cast<uint32_t>(t[11]) | (static_cast<uint32_t>(t[12]) << 8) |
         (static_cast<uint32_t>(t[13]) << 16) | (static_cast<uint32_t>(t[14]) << 24);
}

void send_image(HostPeer& host, const Image& img) {
  host.queue(msg_ota_begin(static_cast<uint32_t>(img.body.size()), img.crc, img.version));
  for (size_t off = 0; off < img.body.size(); off += kOtaChunkMax) {
    const size_t n = std::min(kOtaChunkMax, img.body.size() - off);
    host.queue(msg_ota_chunk(static_cast<uint32_t>(off), img.body.data() + off, n));
  }
  host.queue(msg_simple(kMsgOtaEnd));
}

}  // namespace

TEST(integration_ota_over_noisy_uart) {
  SimHal hal(SimHal::ClockMode::Virtual, 21);
  provision(hal, make_image(1, 6000));
  hal.power_cycle();
  DeviceSim dev(hal);
  HostPeer host(hal, 2e-4, 99);
  dev.on_step = [&](uint64_t now) { host.step(now); };
  REQUIRE(dev.boot());
  dev.run_for(500000);
  CHECK(telemetry_version(host.last_telemetry()) == 1);

  const Image v2 = make_image(2, 10000);
  send_image(host, v2);
  for (int i = 0; i < 120 && dev.boots() < 2; ++i) dev.run_for(500000);
  CHECK(host.last_status(kMsgOtaEnd) == 0);
  REQUIRE(dev.boots() == 2);
  REQUIRE(dev.fw());
  CHECK(dev.fw()->boot_info().version == 2);
  CHECK(dev.fw()->trial());
  dev.run_for(3000000);  // healthy for 2 s => confirm
  CHECK(!dev.fw()->trial());
  CHECK(telemetry_version(host.last_telemetry()) == 2);
  CHECK(dev.fw()->state() == FwState::Normal);
  NOTE("10000-byte image over UART at BER 2e-4: host sent %u frames, %u retransmitted, "
       "%u frames rejected by the host",
       host.link().stats().data_first_tx, host.link().stats().data_retx,
       host.link().rx_stats().crc_errors + host.link().rx_stats().cobs_errors +
           host.link().rx_stats().format_errors);

  // A v3 build that hangs on boot: watchdog resets exhaust the trial boots
  // and the bootloader rolls back to v2.
  dev.image_hangs = [](uint32_t v) { return v == 3; };
  send_image(host, make_image(3, 4000));
  for (int i = 0; i < 120 && !(dev.last_boot().rolled_back); ++i) dev.run_for(500000);
  dev.run_for(1000000);
  REQUIRE(dev.fw());
  CHECK(dev.fw()->boot_info().version == 2);
  CHECK(!dev.fw()->trial());
  CHECK(hal.watchdog_resets() == kTrialBoots);
  CHECK(dev.failed_boots() == 0);
}
