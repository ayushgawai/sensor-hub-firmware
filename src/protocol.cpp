#include "shf/protocol.hpp"

#include <cstring>

#include "shf/crc.hpp"

namespace shf {

size_t encode_frame(const Frame& f, uint8_t* out, size_t cap) {
  if (f.len > kMaxPayload) return 0;
  uint8_t raw[kMaxRawFrame];
  raw[0] = static_cast<uint8_t>(static_cast<uint8_t>(f.type) | (f.flags & kFlagSyn));
  raw[1] = f.seq;
  raw[2] = f.len;
  std::memcpy(raw + 3, f.payload, f.len);
  const size_t body = 3u + f.len;
  const uint16_t a = crc16_ccitt(raw, body);
  const uint16_t b = crc16_t10dif(raw, body);
  raw[body + 0] = static_cast<uint8_t>(a >> 8);
  raw[body + 1] = static_cast<uint8_t>(a);
  raw[body + 2] = static_cast<uint8_t>(b >> 8);
  raw[body + 3] = static_cast<uint8_t>(b);
  // Delimiters on both sides: a flipped delimiter then merges the frame with
  // an empty one instead of destroying the neighbouring frame too.
  if (cap < 2) return 0;
  out[0] = 0x00;
  const size_t enc = cobs_encode(raw, body + 4, out + 1, cap - 2);
  if (enc == 0) return 0;
  out[enc + 1] = 0x00;
  return enc + 2;
}

Deframer::Result Deframer::push(uint8_t byte, Frame& out) {
  if (byte != 0x00) {
    if (len_ < sizeof(buf_)) {
      buf_[len_++] = byte;
    } else {
      overrun_ = true;
    }
    return Result::None;
  }
  if (len_ == 0 && !overrun_) return Result::None;  // idle delimiter
  last_len_ = overrun_ ? sizeof(buf_) + 1 : len_;
  Result r;
  if (overrun_) {
    ++stats_.overruns;
    r = Result::Bad;
  } else {
    r = finish(out);
  }
  len_ = 0;
  overrun_ = false;
  return r;
}

Deframer::Result Deframer::finish(Frame& out) {
  uint8_t raw[kMaxRawFrame];
  size_t n = 0;
  if (!cobs_decode(buf_, len_, raw, sizeof(raw), n)) {
    ++stats_.cobs_errors;
    return Result::Bad;
  }
  if (n < kFrameOverhead) {
    ++stats_.format_errors;
    return Result::Bad;
  }
  const uint8_t hdr = raw[0];
  const uint8_t type = hdr & 0x03u;
  const size_t body = n - 4;
  if ((hdr & 0x7Cu) != 0 || type == 0 || raw[2] != body - 3) {
    ++stats_.format_errors;
    return Result::Bad;
  }
  const uint16_t a = static_cast<uint16_t>((raw[body] << 8) | raw[body + 1]);
  const uint16_t b = static_cast<uint16_t>((raw[body + 2] << 8) | raw[body + 3]);
  const bool a_ok = crc16_ccitt(raw, body) == a;
  const bool b_ok = crc16_t10dif(raw, body) == b;
  if (!a_ok || !b_ok) {
    ++stats_.crc_errors;
    return Result::Bad;
  }
  out.type = static_cast<FrameType>(type);
  out.flags = hdr & kFlagSyn;
  out.seq = raw[1];
  out.len = raw[2];
  std::memcpy(out.payload, raw + 3, out.len);
  ++stats_.frames_ok;
  return Result::Ok;
}

LinkEndpoint::LinkEndpoint(const Config& cfg, TxFn tx, void* tx_ctx, DeliverFn deliver,
                           void* deliver_ctx)
    : cfg_(cfg),
      tx_(tx),
      tx_ctx_(tx_ctx),
      deliver_(deliver),
      deliver_ctx_(deliver_ctx),
      tx_seq_(cfg.initial_seq) {}

bool LinkEndpoint::send(const uint8_t* payload, size_t n, uint64_t now_us) {
  if (outstanding_ || n > kMaxPayload) return false;
  out_.type = FrameType::Data;
  out_.seq = tx_seq_;
  out_.len = static_cast<uint8_t>(n);
  std::memcpy(out_.payload, payload, n);
  outstanding_ = true;
  retries_ = 0;
  ++stats_.data_first_tx;
  transmit_data(now_us);
  return true;
}

void LinkEndpoint::transmit_data(uint64_t now_us) {
  out_.flags = syn_pending_ ? kFlagSyn : 0;
  const size_t n = encode_frame(out_, txbuf_, sizeof(txbuf_));
  stats_.wire_bytes_tx += n;
  tx_(tx_ctx_, txbuf_, n);
  // The timer starts when the last byte has left the UART, so long frames
  // on a slow link do not time out while still being sent.
  deadline_us_ = now_us + n * cfg_.tx_byte_us + cfg_.retx_timeout_us;
  nak_honored_ = false;
}

void LinkEndpoint::send_ctrl(FrameType t, uint8_t seq, uint8_t flags) {
  Frame f;
  f.type = t;
  f.flags = flags;
  f.seq = seq;
  f.len = 0;
  const size_t n = encode_frame(f, txbuf_, sizeof(txbuf_));
  stats_.wire_bytes_tx += n;
  if (t == FrameType::Ack) {
    ++stats_.acks_tx;
  } else {
    ++stats_.naks_tx;
  }
  tx_(tx_ctx_, txbuf_, n);
}

void LinkEndpoint::handle_data(const Frame& f) {
  const uint8_t prev = static_cast<uint8_t>(rx_expected_ - 1);
  const bool syn = (f.flags & kFlagSyn) != 0;
  bool accept;
  if (rx_synced_ && f.seq == prev) {
    ++stats_.duplicates;  // our ACK was lost or the frame was duplicated
    accept = false;
  } else if (syn || (rx_synced_ && f.seq == rx_expected_)) {
    accept = true;
  } else {
    // We do not know this sequence (we restarted, or the peer did without
    // SYN). Ask the peer to resend this frame with SYN.
    ++stats_.out_of_order;
    send_ctrl(FrameType::Nak, f.seq, kFlagSyn);
    return;
  }
  if (accept) {
    rx_synced_ = true;
    rx_expected_ = static_cast<uint8_t>(f.seq + 1);
    ++stats_.delivered;
    deliver_(deliver_ctx_, f.payload, f.len);
  }
  send_ctrl(FrameType::Ack, f.seq);
}

void LinkEndpoint::on_rx_byte(uint8_t byte, uint64_t now_us) {
  Frame f;
  const Deframer::Result r = deframer_.push(byte, f);
  if (r == Deframer::Result::None) return;
  if (r == Deframer::Result::Bad) {
    // NAK only damage that could have been a data frame (control frames are
    // shorter), and rate-limit NAKs to one per gap between valid frames or
    // one per half timeout. Otherwise damaged ACKs/NAKs trigger NAKs in
    // return and at high error rates the link drowns in its own NAKs.
    const bool could_be_data = deframer_.last_frame_len() > kCtrlEncodedLen;
    if (rx_synced_ && could_be_data &&
        (nak_armed_ || now_us - last_nak_us_ >= cfg_.retx_timeout_us / 2)) {
      nak_armed_ = false;
      last_nak_us_ = now_us;
      send_ctrl(FrameType::Nak, rx_expected_);
    }
    return;
  }
  nak_armed_ = true;  // any valid frame re-enables one NAK
  switch (f.type) {
    case FrameType::Data:
      handle_data(f);
      break;
    case FrameType::Ack:
      if (outstanding_ && f.seq == tx_seq_) {
        outstanding_ = false;
        syn_pending_ = false;
        tx_seq_ = static_cast<uint8_t>(tx_seq_ + 1);
      }
      break;
    case FrameType::Nak:
      // Honor one NAK per transmission, so stale or duplicated NAKs cannot
      // multiply retransmissions.
      if (outstanding_ && f.seq == tx_seq_ && !nak_honored_) {
        if (f.flags & kFlagSyn) syn_pending_ = true;  // peer lost sync
        nak_honored_ = true;
        ++stats_.data_retx;
        ++stats_.retx_on_nak;
        transmit_data(now_us);
      }
      break;
  }
}

void LinkEndpoint::poll(uint64_t now_us) {
  if (!outstanding_ || now_us < deadline_us_) return;
  if (retries_ >= cfg_.max_retries) {
    // Give up. Move on and resynchronize the receiver with SYN, because we
    // cannot know whether the peer received the abandoned frame.
    ++stats_.abandoned;
    outstanding_ = false;
    syn_pending_ = true;
    tx_seq_ = static_cast<uint8_t>(tx_seq_ + 1);
    return;
  }
  ++retries_;
  ++stats_.data_retx;
  ++stats_.retx_on_timeout;
  transmit_data(now_us);
}

}  // namespace shf
