#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace crc16 {

constexpr uint16_t table_entry(uint16_t value) {
  value <<= 8;
  for (int bit = 0; bit < 8; ++bit) {
    value = (value & 0x8000U) != 0U ? static_cast<uint16_t>((value << 1) ^ 0x1021U) : static_cast<uint16_t>(value << 1);
  }
  return value;
}

constexpr std::array<uint16_t, 256> make_table() {
  std::array<uint16_t, 256> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = table_entry(static_cast<uint16_t>(i));
  }
  return result;
}

inline constexpr auto table = make_table();

constexpr uint16_t ccitt_update(uint16_t crc, const uint8_t* data, std::size_t size) {
  for (std::size_t i = 0; i < size; ++i) {
    const uint8_t index = static_cast<uint8_t>((crc >> 8) ^ data[i]);
    crc = static_cast<uint16_t>((crc << 8) ^ table[index]);
  }
  return crc;
}

constexpr uint16_t ccitt(const uint8_t* data, std::size_t size) { return ccitt_update(0, data, size); }

static_assert(table[0x00] == 0x0000);
static_assert(table[0x01] == 0x1021);
static_assert(table[0xFF] == 0x1EF0);

namespace detail {
inline constexpr uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
static_assert(ccitt(check, sizeof(check)) == 0x31C3);
}  // namespace detail

}  // namespace crc16
