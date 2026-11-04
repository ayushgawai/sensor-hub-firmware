#include "shf/ota.hpp"

#include <cstring>

#include "shf/crc.hpp"

namespace shf {
namespace {

constexpr uint32_t kBcbMagic = 0x31424342u;  // "BCB1"
constexpr size_t kBcbSize = 16;

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

bool decode_bcb(const uint8_t* r, Bcb& out) {
  if (get_u32(r) != kBcbMagic || get_u32(r + 12) != crc32(r, 12)) return false;
  out.generation = get_u32(r + 4);
  out.active = r[8];
  out.pending = r[9];
  out.trial_left = r[10];
  return out.active < 2 && (out.pending < 2 || out.pending == kNoSlot);
}

// Index (0/1) of the newest valid record, or -1.
int newest_bcb(Hal& hal, const FlashLayout& l, Bcb& out) {
  int best = -1;
  for (int i = 0; i < 2; ++i) {
    uint8_t r[kBcbSize];
    Bcb b{};
    if (!hal.flash_read(l.bcb_addr[i], r, sizeof(r)) || !decode_bcb(r, b)) continue;
    if (best < 0 || b.generation > out.generation) {
      out = b;
      best = i;
    }
  }
  return best;
}

}  // namespace

const char* ota_status_name(OtaStatus s) {
  switch (s) {
    case OtaStatus::Ok: return "ok";
    case OtaStatus::BadState: return "bad-state";
    case OtaStatus::BadArgs: return "bad-args";
    case OtaStatus::FlashError: return "flash-error";
    case OtaStatus::CrcMismatch: return "crc-mismatch";
    case OtaStatus::NotAllowed: return "not-allowed";
  }
  return "?";
}

void encode_image_header(const ImageHeader& h, uint8_t out[kImageHeaderSize]) {
  std::memset(out, 0xFF, kImageHeaderSize);
  put_u32(out + 0, h.magic);
  put_u32(out + 4, h.version);
  put_u32(out + 8, h.size);
  put_u32(out + 12, h.body_crc);
  put_u32(out + 16, crc32(out, 16));
}

bool read_image_header(Hal& hal, const FlashLayout& l, uint8_t slot, ImageHeader& out) {
  if (slot > 1) return false;
  uint8_t r[20];
  if (!hal.flash_read(l.slot_addr[slot], r, sizeof(r))) return false;
  if (get_u32(r) != kImageMagic || get_u32(r + 16) != crc32(r, 16)) return false;
  out.magic = get_u32(r);
  out.version = get_u32(r + 4);
  out.size = get_u32(r + 8);
  out.body_crc = get_u32(r + 12);
  return out.size > 0 && out.size <= l.slot_size - kImageHeaderSize;
}

bool verify_slot(Hal& hal, const FlashLayout& l, uint8_t slot, ImageHeader* out) {
  ImageHeader h{};
  if (!read_image_header(hal, l, slot, h)) return false;
  uint32_t crc = 0;
  uint8_t buf[64];
  const uint32_t base = l.slot_addr[slot] + kImageHeaderSize;
  for (uint32_t off = 0; off < h.size; off += sizeof(buf)) {
    const uint32_t n = (h.size - off < sizeof(buf)) ? h.size - off : sizeof(buf);
    if (!hal.flash_read(base + off, buf, n)) return false;
    crc = crc32(buf, n, crc);
  }
  if (crc != h.body_crc) return false;
  if (out) *out = h;
  return true;
}

bool bcb_read(Hal& hal, const FlashLayout& l, Bcb& out) { return newest_bcb(hal, l, out) >= 0; }

bool bcb_write(Hal& hal, const FlashLayout& l, Bcb& b) {
  Bcb cur{};
  const int idx = newest_bcb(hal, l, cur);
  const int target = idx < 0 ? 0 : 1 - idx;
  b.generation = idx < 0 ? 1 : cur.generation + 1;
  uint8_t r[kBcbSize];
  put_u32(r, kBcbMagic);
  put_u32(r + 4, b.generation);
  r[8] = b.active;
  r[9] = b.pending;
  r[10] = b.trial_left;
  r[11] = 0xFF;
  put_u32(r + 12, crc32(r, 12));
  if (!hal.flash_erase_sector(l.bcb_addr[target])) return false;
  if (!hal.flash_write(l.bcb_addr[target], r, sizeof(r))) return false;
  uint8_t check[kBcbSize];
  return hal.flash_read(l.bcb_addr[target], check, sizeof(check)) &&
         std::memcmp(check, r, sizeof(r)) == 0;
}

BootDecision bootloader_run(Hal& hal, const FlashLayout& l) {
  BootDecision d{};
  Bcb b{};
  if (!bcb_read(hal, l, b)) b = Bcb{0, 0, kNoSlot, 0};

  if (b.pending != kNoSlot) {
    ImageHeader h{};
    if (b.trial_left > 0 && verify_slot(hal, l, b.pending, &h)) {
      Bcb next = b;
      next.trial_left = static_cast<uint8_t>(b.trial_left - 1);
      // Only boot the trial image if the decrement is on flash; otherwise a
      // crashing image could loop in trial forever.
      if (bcb_write(hal, l, next)) {
        d.ok = true;
        d.slot = b.pending;
        d.version = h.version;
        d.trial = true;
        return d;
      }
    } else {
      d.rolled_back = true;
      b.pending = kNoSlot;
      b.trial_left = 0;
      bcb_write(hal, l, b);  // if this is lost, the next boot rolls back again
    }
  }

  ImageHeader h{};
  if (verify_slot(hal, l, b.active, &h)) {
    d.ok = true;
    d.slot = b.active;
    d.version = h.version;
    return d;
  }
  const uint8_t other = static_cast<uint8_t>(1 - b.active);
  if (verify_slot(hal, l, other, &h)) {
    d.ok = true;
    d.slot = other;
    d.version = h.version;
    d.fell_back = true;
    b.active = other;
    b.pending = kNoSlot;
    b.trial_left = 0;
    bcb_write(hal, l, b);
    return d;
  }
  d.ok = false;
  return d;
}

OtaStatus OtaManager::begin(uint32_t size, uint32_t body_crc, uint32_t version) {
  if (trial_) return OtaStatus::NotAllowed;  // confirm the running image first
  if (size == 0 || size > l_.slot_size - kImageHeaderSize) return OtaStatus::BadArgs;
  state_ = State::Idle;
  target_ = static_cast<uint8_t>(1 - running_);

  Bcb b{};
  if (bcb_read(hal_, l_, b) && b.pending != kNoSlot) {
    if (b.pending == running_) return OtaStatus::BadState;
    // A previous download finished but never booted. Drop it before erasing.
    b.pending = kNoSlot;
    b.trial_left = 0;
    if (!bcb_write(hal_, l_, b)) return OtaStatus::FlashError;
  }

  // Erase from the header sector upward so the old image in the target slot
  // is invalid from the first erase on.
  const uint32_t base = l_.slot_addr[target_];
  for (uint32_t off = 0; off < kImageHeaderSize + size; off += l_.sector_size) {
    if (!hal_.flash_erase_sector(base + off)) return OtaStatus::FlashError;
  }
  hdr_ = ImageHeader{kImageMagic, version, size, body_crc};
  written_ = 0;
  state_ = State::Receiving;
  return OtaStatus::Ok;
}

OtaStatus OtaManager::write(uint32_t offset, const uint8_t* data, size_t n) {
  if (state_ != State::Receiving) return OtaStatus::BadState;
  if (offset < written_ && offset + n <= written_) return OtaStatus::Ok;  // repeat
  if (offset != written_ || n == 0 || offset + n > hdr_.size) return OtaStatus::BadArgs;
  const uint32_t addr = l_.slot_addr[target_] + kImageHeaderSize + offset;
  if (!hal_.flash_write(addr, data, n)) {
    state_ = State::Idle;
    return OtaStatus::FlashError;
  }
  written_ += static_cast<uint32_t>(n);
  return OtaStatus::Ok;
}

OtaStatus OtaManager::finish() {
  if (state_ != State::Receiving || written_ != hdr_.size) return OtaStatus::BadState;
  state_ = State::Idle;

  uint32_t crc = 0;
  uint8_t buf[64];
  const uint32_t base = l_.slot_addr[target_] + kImageHeaderSize;
  for (uint32_t off = 0; off < hdr_.size; off += sizeof(buf)) {
    const uint32_t n = (hdr_.size - off < sizeof(buf)) ? hdr_.size - off : sizeof(buf);
    if (!hal_.flash_read(base + off, buf, n)) return OtaStatus::FlashError;
    crc = crc32(buf, n, crc);
  }
  if (crc != hdr_.body_crc) return OtaStatus::CrcMismatch;

  uint8_t hb[kImageHeaderSize];
  encode_image_header(hdr_, hb);
  if (!hal_.flash_write(l_.slot_addr[target_], hb, 20)) return OtaStatus::FlashError;
  ImageHeader check{};
  if (!read_image_header(hal_, l_, target_, check) || check.version != hdr_.version) {
    return OtaStatus::FlashError;
  }

  Bcb b{};
  if (!bcb_read(hal_, l_, b)) b = Bcb{0, running_, kNoSlot, 0};
  // The running image is known good (begin() refuses trial images), so it is
  // the right rollback target even if we got here via a fallback boot.
  b.active = running_;
  b.pending = target_;
  b.trial_left = kTrialBoots;
  if (!bcb_write(hal_, l_, b)) return OtaStatus::FlashError;
  state_ = State::Ready;
  return OtaStatus::Ok;
}

OtaStatus OtaManager::confirm() {
  if (!trial_) return OtaStatus::BadState;
  Bcb b{};
  if (!bcb_read(hal_, l_, b) || b.pending != running_) return OtaStatus::BadState;
  b.active = running_;
  b.pending = kNoSlot;
  b.trial_left = 0;
  if (!bcb_write(hal_, l_, b)) return OtaStatus::FlashError;
  trial_ = false;
  return OtaStatus::Ok;
}

}  // namespace shf
