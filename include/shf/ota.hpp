// A/B over-the-air update and bootloader.
//
// Flash layout (4 KiB sectors):
//
//   0x00000  BCB record 0   (boot control block, ping-pong)
//   0x01000  BCB record 1
//   0x02000  slot A  [32-byte image header | image body ...]
//   0x12000  slot B  [32-byte image header | image body ...]
//
// Power-loss rules:
//  * The running slot is never erased or written.
//  * The image body is written first and the header last, so a partly
//    written slot never has a valid header.
//  * The body CRC-32 is checked after download and again by the bootloader
//    on every boot.
//  * BCB updates go to the older of the two records, with a generation
//    counter and CRC-32. A torn BCB write leaves the previous record as the
//    newest valid one.
//  * A new image boots in trial mode a limited number of times. If it never
//    confirms itself (crash, hang, power loss), the bootloader rolls back.
#pragma once

#include <cstddef>
#include <cstdint>

#include "shf/hal.hpp"

namespace shf {

struct FlashLayout {
  uint32_t sector_size;
  uint32_t bcb_addr[2];
  uint32_t slot_addr[2];
  uint32_t slot_size;
};

constexpr FlashLayout kDefaultLayout = {4096, {0x0000, 0x1000}, {0x2000, 0x12000}, 0x10000};
constexpr uint32_t kImageHeaderSize = 32;
constexpr uint32_t kImageMagic = 0x57464853u;  // "SHFW"
constexpr uint8_t kNoSlot = 0xFF;
constexpr uint8_t kTrialBoots = 2;

struct ImageHeader {
  uint32_t magic;
  uint32_t version;
  uint32_t size;      // body bytes
  uint32_t body_crc;  // CRC-32 of body
};

struct Bcb {
  uint32_t generation;
  uint8_t active;      // confirmed slot
  uint8_t pending;     // slot under trial, or kNoSlot
  uint8_t trial_left;  // trial boots remaining for pending
};

// Serialized header bytes (with header CRC), for writing an image.
void encode_image_header(const ImageHeader& h, uint8_t out[kImageHeaderSize]);

bool read_image_header(Hal& hal, const FlashLayout& l, uint8_t slot, ImageHeader& out);
// Header valid and body CRC matches.
bool verify_slot(Hal& hal, const FlashLayout& l, uint8_t slot, ImageHeader* out = nullptr);

// Returns false if neither record is valid (factory default is then used).
bool bcb_read(Hal& hal, const FlashLayout& l, Bcb& out);
// Writes b as the next generation. Updates b.generation.
bool bcb_write(Hal& hal, const FlashLayout& l, Bcb& b);

struct BootDecision {
  bool ok;
  uint8_t slot;
  uint32_t version;
  bool trial;        // running a pending image that has not confirmed yet
  bool rolled_back;  // a pending image ran out of trial boots or failed CRC
  bool fell_back;    // the confirmed slot was invalid, used the other one
};

// Bootloader entry: pick a slot, verify it, update trial bookkeeping.
BootDecision bootloader_run(Hal& hal, const FlashLayout& l);

enum class OtaStatus : uint8_t {
  Ok = 0,
  BadState = 1,
  BadArgs = 2,
  FlashError = 3,
  CrcMismatch = 4,
  NotAllowed = 5,
};
const char* ota_status_name(OtaStatus s);

class OtaManager {
 public:
  enum class State : uint8_t { Idle, Receiving, Ready };

  OtaManager(Hal& hal, const FlashLayout& l, uint8_t running_slot, bool trial)
      : hal_(hal), l_(l), running_(running_slot), trial_(trial) {}

  OtaStatus begin(uint32_t size, uint32_t body_crc, uint32_t version);
  // Chunks must arrive in order. A repeated chunk that was already written is
  // accepted and ignored.
  OtaStatus write(uint32_t offset, const uint8_t* data, size_t n);
  // Verifies the body, writes the header, marks the slot pending.
  OtaStatus finish();
  // Called by a trial image once it is healthy.
  OtaStatus confirm();
  void abort() { state_ = State::Idle; }

  State state() const { return state_; }
  bool trial() const { return trial_; }
  uint32_t received() const { return written_; }
  uint8_t target_slot() const { return target_; }

 private:
  Hal& hal_;
  FlashLayout l_;
  uint8_t running_;
  bool trial_;
  State state_ = State::Idle;
  uint8_t target_ = kNoSlot;
  ImageHeader hdr_{};
  uint32_t written_ = 0;
};

}  // namespace shf
