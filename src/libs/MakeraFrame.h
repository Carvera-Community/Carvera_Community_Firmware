#pragma once

#include <cstddef>
#include <cstdint>

#include "CRC16.h"

namespace makera {

// Multi-byte fields are big-endian. Length counts the type, data and CRC bytes;
// the CRC-16/CCITT covers the length, type and data. A frame with type 0x71 and
// no data is therefore:
//
//   header | length | type | CRC   | footer
//   86 68  | 00 03  | 71   | 3b e5 | 55 aa
constexpr uint16_t header = 0x8668;
constexpr uint16_t footer = 0x55AA;
constexpr std::size_t max_frame_size = 544;
constexpr std::size_t frame_overhead = 9;
constexpr std::size_t max_data_size = max_frame_size - frame_overhead;
constexpr uint32_t frame_timeout_ms = 1000;

constexpr uint16_t read_be16(const uint8_t* data) {
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

constexpr uint32_t read_be32(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | data[3];
}

template <typename Byte>
void write_be16(Byte* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value >> 8);
  data[1] = static_cast<uint8_t>(value);
}

template <typename Byte>
void write_be32(Byte* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value >> 24);
  data[1] = static_cast<uint8_t>(value >> 16);
  data[2] = static_cast<uint8_t>(value >> 8);
  data[3] = static_cast<uint8_t>(value);
}

struct Packet {
  uint16_t length;
  uint8_t type;
  uint16_t data_length;
  uint8_t data[max_data_size];
  uint16_t crc;
};

enum class DecodeResult : uint8_t { incomplete, complete, invalid_length, invalid_crc, invalid_footer };
class FrameDecoder {
 public:
  explicit FrameDecoder(Packet& packet) : packet_(packet) {}

  DecodeResult decode_byte(uint8_t byte, uint32_t now_ms);
  void reset();

  bool in_progress() const { return received_ != 0 || header_prefix_; }
  bool has_header() const { return received_ >= 2; }
  std::size_t bytes_wanted() const;
  const Packet& packet() const { return packet_; }

 private:
  void restart_with(uint8_t byte, uint32_t now_ms);
  void keep_consumed_header(uint32_t now_ms);
  void restart_from_trailer(uint32_t now_ms);

  std::size_t received_ = 0;
  std::size_t expected_ = 0;
  uint8_t length_high_ = 0;
  uint8_t footer_high_ = 0;
  uint32_t trailer_ = 0;
  uint16_t calculated_crc_ = 0;
  uint32_t last_byte_ms_ = 0;
  bool header_prefix_ = false;
  bool have_last_byte_ = false;
  Packet& packet_;
};

}  // namespace makera
