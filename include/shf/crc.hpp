#pragma once

#include <cstddef>
#include <cstdint>

namespace shf {

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
// Check value for "123456789" is 0x29B1.
uint16_t crc16_ccitt(const uint8_t* data, size_t n, uint16_t crc = 0xFFFF);

// CRC-16/T10-DIF: poly 0x8BB7, init 0x0000. Check value 0xD0DB.
// Used as the second, independent frame check (see protocol.hpp).
uint16_t crc16_t10dif(const uint8_t* data, size_t n, uint16_t crc = 0x0000);

// CRC-32/ISO-HDLC (zlib). Incremental: pass the previous return value.
// Check value 0xCBF43926.
uint32_t crc32(const uint8_t* data, size_t n, uint32_t prev = 0);

}  // namespace shf
