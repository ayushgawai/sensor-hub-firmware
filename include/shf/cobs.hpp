// Consistent Overhead Byte Stuffing. Encoded output contains no 0x00 bytes,
// so 0x00 can delimit frames on the wire.
#pragma once

#include <cstddef>
#include <cstdint>

namespace shf {

constexpr size_t cobs_max_encoded(size_t n) { return n + n / 254 + 1; }

// Returns the encoded length (without delimiter), or 0 if dst is too small.
size_t cobs_encode(const uint8_t* src, size_t n, uint8_t* dst, size_t dst_cap);

// Decodes one frame (without delimiter). Returns false on malformed input or
// if dst is too small.
bool cobs_decode(const uint8_t* src, size_t n, uint8_t* dst, size_t dst_cap, size_t& out_len);

}  // namespace shf
