#include "shf/crc.hpp"

#include <array>

namespace shf {
namespace {

constexpr std::array<uint16_t, 256> make_crc16_table(uint16_t poly) {
  std::array<uint16_t, 256> t{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint16_t c = static_cast<uint16_t>(i << 8);
    for (int b = 0; b < 8; ++b) {
      c = (c & 0x8000u) ? static_cast<uint16_t>((c << 1) ^ poly) : static_cast<uint16_t>(c << 1);
    }
    t[i] = c;
  }
  return t;
}

constexpr std::array<uint32_t, 256> make_crc32_table() {
  std::array<uint32_t, 256> t{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int b = 0; b < 8; ++b) c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
    t[i] = c;
  }
  return t;
}

constexpr auto kCcittTable = make_crc16_table(0x1021);
constexpr auto kT10DifTable = make_crc16_table(0x8BB7);
constexpr auto kCrc32Table = make_crc32_table();

inline uint16_t crc16_msb(const std::array<uint16_t, 256>& t, const uint8_t* d, size_t n,
                          uint16_t crc) {
  for (size_t i = 0; i < n; ++i) {
    crc = static_cast<uint16_t>((crc << 8) ^ t[((crc >> 8) ^ d[i]) & 0xFFu]);
  }
  return crc;
}

}  // namespace

uint16_t crc16_ccitt(const uint8_t* data, size_t n, uint16_t crc) {
  return crc16_msb(kCcittTable, data, n, crc);
}

uint16_t crc16_t10dif(const uint8_t* data, size_t n, uint16_t crc) {
  return crc16_msb(kT10DifTable, data, n, crc);
}

uint32_t crc32(const uint8_t* data, size_t n, uint32_t prev) {
  uint32_t c = ~prev;
  for (size_t i = 0; i < n; ++i) c = kCrc32Table[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
  return ~c;
}

}  // namespace shf
