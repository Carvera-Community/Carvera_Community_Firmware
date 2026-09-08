#pragma once

#include <cstdint>

namespace canopen {

constexpr uint32_t nmt_cob_id = 0;
constexpr uint32_t sdo_response_base = 0x580;
constexpr uint32_t sdo_request_base = 0x600;
constexpr uint32_t heartbeat_base = 0x700;
constexpr uint32_t standard_id_max = 0x7ff;
constexpr uint8_t max_data_length = 8;
constexpr uint8_t sdo_upload_request_command = 0x40;

struct FrameView {
  uint32_t id;
  uint8_t len;
  bool standard;
  bool data_frame;
  const uint8_t* data;
};

enum class SdoResponseKind : uint8_t { unrelated, upload, download, abort, malformed };

struct SdoResponse {
  SdoResponseKind kind;
  uint8_t size;
  uint32_t value;
  uint32_t abort_code;
};

constexpr bool valid_node_id(uint32_t node) { return node >= 1 && node <= 127; }

constexpr uint32_t heartbeat_cob_id(uint8_t node) { return heartbeat_base + node; }

constexpr uint32_t sdo_request_cob_id(uint8_t node) { return sdo_request_base + node; }

constexpr uint8_t sdo_download_command(uint8_t size) {
  return size >= 1 && size <= 4 ? static_cast<uint8_t>(0x23U | ((4U - size) << 2U)) : 0;
}

constexpr uint32_t read_le32(const uint8_t* bytes) {
  return static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
}

constexpr SdoResponse decode_sdo_response(FrameView frame, uint8_t node, uint16_t index, uint8_t subindex,
                                          bool expect_upload) {
  constexpr SdoResponse unrelated{SdoResponseKind::unrelated, 0, 0, 0};
  constexpr SdoResponse malformed{SdoResponseKind::malformed, 0, 0, 0};

  if (frame.id != sdo_response_base + node) return unrelated;
  if (frame.len != 8 || !frame.standard || !frame.data_frame || frame.data == nullptr) {
    return malformed;
  }
  if (frame.data[1] != (index & 0xff) || frame.data[2] != (index >> 8) || frame.data[3] != subindex) {
    return unrelated;
  }

  const uint8_t command = frame.data[0];
  if (command == 0x80) {
    return SdoResponse{SdoResponseKind::abort, 0, 0, read_le32(frame.data + 4)};
  }

  if (!expect_upload) {
    return command == 0x60 ? SdoResponse{SdoResponseKind::download, 0, 0, 0} : malformed;
  }

  if ((command & 0xe0) != 0x40 || (command & 0x10) != 0 || (command & 0x02) == 0) return malformed;
  const uint8_t unused = (command >> 2) & 0x03;
  if ((command & 0x01) == 0 && unused != 0) return malformed;
  const uint8_t size = (command & 0x01) != 0 ? 4 - unused : 4;
  uint32_t value = 0;
  for (uint8_t i = 0; i < size; ++i) {
    value |= static_cast<uint32_t>(frame.data[4 + i]) << (i * 8);
  }
  return SdoResponse{SdoResponseKind::upload, size, value, 0};
}

}  // namespace canopen
