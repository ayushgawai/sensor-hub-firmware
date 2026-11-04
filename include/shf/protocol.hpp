// Framed serial protocol.
//
// Frame before COBS encoding:
//
//   +-----+-----+-----+-------------+----------+-----------+
//   | hdr | seq | len | payload[len]| crc16a   | crc16b    |
//   +-----+-----+-----+-------------+----------+-----------+
//     1     1     1     0..64          2 (BE)     2 (BE)
//
//   hdr: bits 0..1 type (1 DATA, 2 ACK, 3 NAK), bit 7 SYN, others zero.
//   crc16a: CRC-16/CCITT-FALSE over hdr..payload.
//   crc16b: CRC-16/T10-DIF over hdr..payload.
//
// On the wire each encoded frame is wrapped in 0x00 delimiters: 00 <cobs> 00.
//
// Why two CRC-16s: at a 1% bit-error rate most frames carry 3+ bit errors,
// and a single CRC-16 lets roughly 1 in 65536 of those through. Over the
// tens of millions of frames in the bench that is hundreds of accepted
// corrupted frames. The two polynomials share no factor, so together they
// act like one 32-bit check. The link simulator (sim/link_sim.hpp) counts how
// many damaged frames a CRC-16/CCITT-only design would have accepted.
//
// Link layer: stop-and-wait ARQ per direction. 8-bit sequence numbers, ACK
// on accept or duplicate, NAK(expected) on a damaged frame (at most one per
// gap, to avoid NAK storms), retransmit on NAK or timeout. The first frame
// after init (or after giving up) carries SYN, which makes the receiver adopt
// the sender's sequence number. A receiver that sees an unknown sequence
// (for example after it rebooted) answers NAK+SYN, asking for a SYN resend.
#pragma once

#include <cstddef>
#include <cstdint>

#include "shf/cobs.hpp"

namespace shf {

constexpr size_t kMaxPayload = 64;
constexpr size_t kFrameOverhead = 3 + 4;
constexpr size_t kMaxRawFrame = kMaxPayload + kFrameOverhead;
constexpr size_t kMaxEncodedFrame = cobs_max_encoded(kMaxRawFrame) + 2;  // + delimiters
constexpr size_t kCtrlEncodedLen = cobs_max_encoded(kFrameOverhead);       // ACK/NAK, no delimiter

enum class FrameType : uint8_t { Data = 1, Ack = 2, Nak = 3 };
constexpr uint8_t kFlagSyn = 0x80;

struct Frame {
  FrameType type;
  uint8_t flags;
  uint8_t seq;
  uint8_t len;
  uint8_t payload[kMaxPayload];
};

// Encodes a frame including both delimiters. Returns bytes written, 0 on error.
size_t encode_frame(const Frame& f, uint8_t* out, size_t cap);

struct DeframerStats {
  uint32_t frames_ok;
  uint32_t cobs_errors;
  uint32_t format_errors;  // bad header, length mismatch, too short
  uint32_t crc_errors;
  uint32_t overruns;                   // frame longer than any valid frame
};

class Deframer {
 public:
  enum class Result : uint8_t { None, Ok, Bad };
  Result push(uint8_t byte, Frame& out);
  const DeframerStats& stats() const { return stats_; }
  // Encoded length of the most recent complete (good or bad) frame.
  size_t last_frame_len() const { return last_len_; }

 private:
  Result finish(Frame& out);
  uint8_t buf_[kMaxEncodedFrame];
  size_t len_ = 0;
  size_t last_len_ = 0;
  bool overrun_ = false;
  DeframerStats stats_{};
};

struct LinkStats {
  uint32_t data_first_tx;
  uint32_t data_retx;
  uint32_t retx_on_nak;
  uint32_t retx_on_timeout;
  uint32_t acks_tx;
  uint32_t naks_tx;
  uint32_t delivered;
  uint32_t duplicates;
  uint32_t out_of_order;
  uint32_t abandoned;
  uint64_t wire_bytes_tx;
};

class LinkEndpoint {
 public:
  using TxFn = void (*)(void* ctx, const uint8_t* bytes, size_t n);
  using DeliverFn = void (*)(void* ctx, const uint8_t* payload, size_t n);

  struct Config {
    uint32_t retx_timeout_us;  // counted from the end of the frame on the wire
    uint16_t max_retries;      // timeouts before giving up on a frame
    uint8_t initial_seq;
    uint32_t tx_byte_us = 87;  // UART byte time (115200 baud 8N1)
  };

  LinkEndpoint(const Config& cfg, TxFn tx, void* tx_ctx, DeliverFn deliver, void* deliver_ctx);

  // Queues one payload for reliable delivery. Returns false if a frame is
  // still outstanding or the payload is too large.
  bool send(const uint8_t* payload, size_t n, uint64_t now_us);
  bool busy() const { return outstanding_; }
  // Time of the next retransmit check, UINT64_MAX when idle.
  uint64_t retx_deadline() const { return outstanding_ ? deadline_us_ : UINT64_MAX; }

  void on_rx_byte(uint8_t byte, uint64_t now_us);
  void poll(uint64_t now_us);  // retransmit timer

  const LinkStats& stats() const { return stats_; }
  const DeframerStats& rx_stats() const { return deframer_.stats(); }

 private:
  void transmit_data(uint64_t now_us);
  void send_ctrl(FrameType t, uint8_t seq, uint8_t flags = 0);
  void handle_data(const Frame& f);

  Config cfg_;
  TxFn tx_;
  void* tx_ctx_;
  DeliverFn deliver_;
  void* deliver_ctx_;

  // Sender half.
  Frame out_{};
  bool outstanding_ = false;
  bool syn_pending_ = true;
  uint8_t tx_seq_;
  uint16_t retries_ = 0;
  uint64_t deadline_us_ = 0;
  bool nak_honored_ = false;

  // Receiver half.
  bool rx_synced_ = false;
  uint8_t rx_expected_ = 0;
  bool nak_armed_ = true;
  uint64_t last_nak_us_ = 0;
  Deframer deframer_;

  uint8_t txbuf_[kMaxEncodedFrame];
  LinkStats stats_{};
};

}  // namespace shf
