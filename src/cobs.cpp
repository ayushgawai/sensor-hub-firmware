#include "shf/cobs.hpp"

namespace shf {

size_t cobs_encode(const uint8_t* src, size_t n, uint8_t* dst, size_t dst_cap) {
  if (dst_cap < cobs_max_encoded(n)) return 0;
  size_t code_idx = 0;
  size_t out = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < n; ++i) {
    if (src[i] == 0) {
      dst[code_idx] = code;
      code_idx = out++;
      code = 1;
    } else {
      dst[out++] = src[i];
      if (++code == 0xFF) {
        dst[code_idx] = code;
        code_idx = out++;
        code = 1;
      }
    }
  }
  dst[code_idx] = code;
  return out;
}

bool cobs_decode(const uint8_t* src, size_t n, uint8_t* dst, size_t dst_cap, size_t& out_len) {
  size_t i = 0;
  size_t out = 0;
  while (i < n) {
    const uint8_t code = src[i++];
    if (code == 0) return false;
    for (uint8_t j = 1; j < code; ++j) {
      if (i >= n || src[i] == 0 || out >= dst_cap) return false;
      dst[out++] = src[i++];
    }
    if (code != 0xFF && i < n) {
      if (out >= dst_cap) return false;
      dst[out++] = 0;
    }
  }
  out_len = out;
  return true;
}

}  // namespace shf
