#include "shf/firmware.hpp"

#include <cstring>

namespace shf {
namespace {

void put_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}
uint32_t get_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

Firmware::Firmware(Hal& hal, const BootInfo& boot, const FirmwareConfig& cfg)
    : hal_(hal),
      boot_(boot),
      cfg_(cfg),
      sched_(&Firmware::clock_fn, &hal),
      // Vary the initial sequence number across resets so a peer that missed
      // the reset is unlikely to treat our first frame as a duplicate.
      link_(LinkEndpoint::Config{cfg.link_timeout_us, cfg.link_max_retries,
                                 static_cast<uint8_t>(hal.retained().total_resets * 97u + 1u)},
            &Firmware::link_tx, this, &Firmware::link_deliver, this),
      faults_(cfg.faults),
      ota_(hal, kDefaultLayout, boot.slot, boot.trial) {
  const uint64_t now = hal_.micros();
  faults_.on_boot(hal_.retained(), hal_.reset_cause(), now);
  last_state_ = faults_.state();

  t_imu_ = sched_.add_task("imu", &Firmware::imu_task, this, 1000, 0, now);
  sched_.add_task("comms", &Firmware::comms_task, this, 1000, 1, now);
  sched_.add_task("telemetry", &Firmware::telemetry_task, this, cfg_.telemetry_period_us, 2, now);
  sched_.add_task("health", &Firmware::health_task, this, 10000, 3, now);
  sched_.set_jitter_probe(t_imu_);

  const bool imu_on = faults_.imu_active();
  sched_.set_enabled(t_imu_, imu_on);
  wdt_.set_required(kCheckinComms | (imu_on ? kCheckinImu : 0u));

  hal_.set_tick_handler(&Firmware::tick_isr, this);
  hal_.set_uart_rx_handler(&Firmware::uart_rx_isr, this);
  hal_.watchdog_start(cfg_.watchdog_timeout_us);
}

uint64_t Firmware::clock_fn(void* ctx) { return static_cast<Hal*>(ctx)->micros(); }

void Firmware::tick_isr(void* ctx) {
  auto* fw = static_cast<Firmware*>(ctx);
  fw->uptime_ms_.store(fw->uptime_ms_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
}

void Firmware::uart_rx_isr(void* ctx, uint8_t b) { static_cast<Firmware*>(ctx)->uart_rx_.push(b); }

void Firmware::link_tx(void* ctx, const uint8_t* bytes, size_t n) {
  static_cast<Firmware*>(ctx)->hal_.uart_write(bytes, n);
}

void Firmware::link_deliver(void* ctx, const uint8_t* payload, size_t n) {
  static_cast<Firmware*>(ctx)->handle_message(payload, n);
}

void Firmware::imu_task(void* ctx, uint64_t now) {
  auto* fw = static_cast<Firmware*>(ctx);
  fw->wdt_.checkin(kCheckinImu);
  ImuRaw s;
  if (!fw->hal_.imu_read(s)) return;
  ++fw->counters_.imu_samples;
  fw->faults_.on_imu_sample(s, now);
  if (fw->faults_.fusion_enabled()) fw->fusion_.update(s);
}

void Firmware::comms_task(void* ctx, uint64_t now) {
  auto* fw = static_cast<Firmware*>(ctx);
  if (fw->stall_comms_) return;
  fw->wdt_.checkin(kCheckinComms);
  fw->now_us_ = now;
  uint8_t b;
  // Bound the work per run so a byte flood cannot starve other tasks.
  for (int i = 0; i < 128 && fw->uart_rx_.pop(b); ++i) fw->link_.on_rx_byte(b, now);
  fw->link_.poll(now);
  if (!fw->link_.busy()) {
    Msg m;
    if (fw->txq_.pop(m)) fw->link_.send(m.data, m.len, now);
  }
}

void Firmware::telemetry_task(void* ctx, uint64_t) {
  auto* fw = static_cast<Firmware*>(ctx);
  // Telemetry is best effort: skip it while responses or older telemetry wait.
  if (!fw->txq_.empty() || fw->link_.busy()) {
    ++fw->counters_.telemetry_skipped;
    return;
  }
  uint8_t p[15];
  p[0] = kMsgTelemetry;
  put_u32(p + 1, fw->uptime_ms_.load(std::memory_order_relaxed));
  const q15_t r = fw->fusion_.roll_q15();
  const q15_t pt = fw->fusion_.pitch_q15();
  p[5] = static_cast<uint8_t>(r);
  p[6] = static_cast<uint8_t>(static_cast<uint16_t>(r) >> 8);
  p[7] = static_cast<uint8_t>(pt);
  p[8] = static_cast<uint8_t>(static_cast<uint16_t>(pt) >> 8);
  p[9] = static_cast<uint8_t>(fw->faults_.state());
  p[10] = static_cast<uint8_t>(fw->faults_.flags());
  put_u32(p + 11, fw->boot_.version);
  if (fw->enqueue(p, sizeof(p))) ++fw->counters_.telemetry_queued;
}

void Firmware::health_task(void* ctx, uint64_t now) {
  auto* fw = static_cast<Firmware*>(ctx);
  const FaultManager::Actions a = fw->faults_.step(now, fw->hal_.supply_mv());
  if (a.reset_imu) fw->hal_.imu_reset();
  if (a.abort_flash_ops && fw->ota_.state() == OtaManager::State::Receiving) fw->ota_.abort();

  const FwState st = fw->faults_.state();
  if (st != fw->last_state_) {
    const bool imu_on = fw->faults_.imu_active();
    fw->sched_.set_enabled(fw->t_imu_, imu_on);
    fw->wdt_.set_required(kCheckinComms | (imu_on ? kCheckinImu : 0u));
    if (st == FwState::Degraded || st == FwState::SafeMode) fw->fusion_.reset();
    fw->last_state_ = st;
  }
  fw->wdt_.service(fw->hal_);

  // A trial image confirms itself after a stretch of healthy operation.
  if (fw->ota_.trial()) {
    if (st != FwState::Normal) {
      fw->healthy_since_us_ = 0;
    } else if (fw->healthy_since_us_ == 0) {
      fw->healthy_since_us_ = now;
    } else if (now - fw->healthy_since_us_ >= fw->cfg_.confirm_after_us &&
               fw->faults_.flash_writes_allowed()) {
      fw->ota_.confirm();
    }
  }
  if (fw->reset_at_us_ != 0 && now >= fw->reset_at_us_) {
    fw->reset_at_us_ = 0;
    fw->hal_.system_reset();
  }
}

bool Firmware::enqueue(const uint8_t* p, size_t n) {
  Msg m;
  m.len = static_cast<uint8_t>(n);
  std::memcpy(m.data, p, n);
  return txq_.push(m);
}

void Firmware::reply_status(uint8_t req, uint8_t status) {
  const uint8_t p[3] = {kMsgStatus, req, status};
  enqueue(p, sizeof(p));
}

void Firmware::handle_message(const uint8_t* p, size_t n) {
  if (n == 0) return;
  ++counters_.messages_rx;
  const uint8_t id = p[0];
  const bool flash_ok = faults_.flash_writes_allowed();
  OtaStatus st = OtaStatus::Ok;
  switch (id) {
    case kMsgOtaBegin:
      if (n != 13) {
        st = OtaStatus::BadArgs;
      } else if (!flash_ok) {
        st = OtaStatus::NotAllowed;
      } else {
        st = ota_.begin(get_u32(p + 1), get_u32(p + 5), get_u32(p + 9));
      }
      reply_status(id, static_cast<uint8_t>(st));
      break;
    case kMsgOtaChunk:
      if (n < 6) {
        st = OtaStatus::BadArgs;
      } else if (!flash_ok) {
        ota_.abort();
        st = OtaStatus::NotAllowed;
      } else {
        st = ota_.write(get_u32(p + 1), p + 5, n - 5);
      }
      if (st != OtaStatus::Ok) reply_status(id, static_cast<uint8_t>(st));  // errors only
      break;
    case kMsgOtaEnd:
      st = flash_ok ? ota_.finish() : OtaStatus::NotAllowed;
      reply_status(id, static_cast<uint8_t>(st));
      // Reboot into the new image once the reply has had time to go out.
      if (st == OtaStatus::Ok) reset_at_us_ = now_us_ + 200000;
      break;
    case kMsgClearFault:
      reply_status(id, faults_.clear_safe_mode(now_us_) ? 0 : 1);
      break;
    case kMsgReboot:
      reply_status(id, 0);
      reset_at_us_ = now_us_ + 50000;
      break;
    default:
      reply_status(id, static_cast<uint8_t>(OtaStatus::BadArgs));
      break;
  }
  if (st != OtaStatus::Ok && id >= kMsgOtaBegin && id <= kMsgOtaEnd) ++counters_.ota_errors;
}

}  // namespace shf
