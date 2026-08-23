#include "MakeraFrame.h"

namespace makera {

void FrameDecoder::reset() {
  received_ = 0;
  expected_ = 0;
  length_high_ = 0;
  footer_high_ = 0;
  trailer_ = 0;
  calculated_crc_ = 0;
  last_byte_ms_ = 0;
  header_prefix_ = false;
  have_last_byte_ = false;
}

void FrameDecoder::restart_with(uint8_t byte, uint32_t now_ms) {
  reset();
  header_prefix_ = byte == static_cast<uint8_t>(header >> 8);
  last_byte_ms_ = now_ms;
  have_last_byte_ = true;
}

void FrameDecoder::keep_consumed_header(uint32_t now_ms) {
  reset();
  received_ = 2;
  last_byte_ms_ = now_ms;
  have_last_byte_ = true;
}

void FrameDecoder::restart_from_trailer(uint32_t now_ms) {
  const uint32_t trailer = trailer_;
  if ((trailer & 0xFFFF) == header) {
    keep_consumed_header(now_ms);
    return;
  }

  if (((trailer >> 8) & 0xFFFF) == header) {
    keep_consumed_header(now_ms);
    decode_byte(static_cast<uint8_t>(trailer), now_ms);
    return;
  }

  if ((trailer >> 16) == header) {
    keep_consumed_header(now_ms);
    decode_byte(static_cast<uint8_t>(trailer >> 8), now_ms);
    decode_byte(static_cast<uint8_t>(trailer), now_ms);
    return;
  }

  restart_with(static_cast<uint8_t>(trailer), now_ms);
}

std::size_t FrameDecoder::bytes_wanted() const {
  if (received_ == 0) return header_prefix_ ? 1 : 2;
  if (received_ < 4) return 4 - received_;
  return expected_ - received_;
}

DecodeResult FrameDecoder::decode_byte(uint8_t byte, uint32_t now_ms) {
  if (in_progress() && have_last_byte_ && now_ms - last_byte_ms_ >= frame_timeout_ms) reset();
  last_byte_ms_ = now_ms;
  have_last_byte_ = true;

  if (received_ == 0) {
    if (header_prefix_ && byte == static_cast<uint8_t>(header)) {
      received_ = 2;
      header_prefix_ = false;
    } else {
      header_prefix_ = byte == static_cast<uint8_t>(header >> 8);
    }
    return DecodeResult::incomplete;
  }

  const std::size_t position = received_++;
  if (position == 2) {
    length_high_ = byte;
    calculated_crc_ = crc16::ccitt_update(calculated_crc_, &byte, 1);
    return DecodeResult::incomplete;
  }
  if (position == 3) {
    const uint16_t length = (static_cast<uint16_t>(length_high_) << 8) | byte;
    if (length < 3 || length > max_data_size + 3) {
      if (length == header) {
        keep_consumed_header(now_ms);
      } else {
        restart_with(byte, now_ms);
      }
      return DecodeResult::invalid_length;
    }
    packet_.length = length;
    packet_.data_length = length - 3;
    calculated_crc_ = crc16::ccitt_update(calculated_crc_, &byte, 1);
    expected_ = static_cast<std::size_t>(length) + 6;
    return DecodeResult::incomplete;
  }

  if (position == 4) {
    packet_.type = byte;
    calculated_crc_ = crc16::ccitt_update(calculated_crc_, &byte, 1);
  } else if (position < static_cast<std::size_t>(packet_.length) + 2) {
    packet_.data[position - 5] = byte;
    calculated_crc_ = crc16::ccitt_update(calculated_crc_, &byte, 1);
  } else if (position == static_cast<std::size_t>(packet_.length) + 2) {
    packet_.crc = static_cast<uint16_t>(byte) << 8;
  } else if (position == static_cast<std::size_t>(packet_.length) + 3) {
    packet_.crc |= byte;
  } else if (position == expected_ - 2) {
    footer_high_ = byte;
  }

  if (position >= static_cast<std::size_t>(packet_.length) + 2) trailer_ = (trailer_ << 8) | byte;

  if (received_ < expected_) return DecodeResult::incomplete;

  const uint16_t received_footer = (static_cast<uint16_t>(footer_high_) << 8) | byte;
  if (position != expected_ - 1 || received_footer != footer) {
    restart_from_trailer(now_ms);
    return DecodeResult::invalid_footer;
  }

  const bool crc_valid = packet_.crc == calculated_crc_;
  reset();
  return crc_valid ? DecodeResult::complete : DecodeResult::invalid_crc;
}

}  // namespace makera
