#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "libs/CRC16.h"
#include "libs/MakeraControl.h"
#include "libs/MakeraFrame.h"

namespace {

int checks;
int failures;
const char* current_test;

#define CHECK(condition)                                                      \
  do {                                                                        \
    ++checks;                                                                 \
    if (!(condition)) {                                                       \
      ++failures;                                                             \
      std::printf("  FAIL (%s:%d) %s\n", current_test, __LINE__, #condition); \
    }                                                                         \
  } while (false)

#define TEST(name)     \
  current_test = name; \
  std::printf("%s\n", name)

std::vector<uint8_t> make_frame(uint8_t type, const std::vector<uint8_t>& data) {
  const uint16_t length = static_cast<uint16_t>(data.size() + 3);
  std::vector<uint8_t> frame{static_cast<uint8_t>(makera::header >> 8), static_cast<uint8_t>(makera::header),
                             static_cast<uint8_t>(length >> 8), static_cast<uint8_t>(length), type};
  frame.insert(frame.end(), data.begin(), data.end());
  const uint16_t crc = crc16::ccitt(frame.data() + 2, length);
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  frame.push_back(static_cast<uint8_t>(crc));
  frame.push_back(static_cast<uint8_t>(makera::footer >> 8));
  frame.push_back(static_cast<uint8_t>(makera::footer));
  return frame;
}

std::vector<uint8_t> make_frame(uint8_t type, const char* data) {
  const auto* first = reinterpret_cast<const uint8_t*>(data);
  return make_frame(type, std::vector<uint8_t>(first, first + std::strlen(data)));
}

makera::DecodeResult feed(makera::FrameDecoder& decoder, const std::vector<uint8_t>& bytes, uint32_t start_ms = 10,
                          uint32_t step_ms = 0) {
  makera::DecodeResult result = makera::DecodeResult::incomplete;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    const makera::DecodeResult next = decoder.decode_byte(bytes[i], start_ms + i * step_ms);
    if (next != makera::DecodeResult::incomplete) result = next;
  }
  return result;
}

std::string packet_data(const makera::Packet& packet) {
  return std::string(reinterpret_cast<const char*>(packet.data), packet.data_length);
}

struct Decoder {
  makera::Packet packet{};
  makera::FrameDecoder frame{packet};

  operator makera::FrameDecoder&() { return frame; }
  makera::FrameDecoder* operator->() { return &frame; }
};

}  // namespace

int main() {
  {
    TEST("a valid frame exposes its type and data");
    Decoder decoder;
    CHECK(feed(decoder, make_frame(0xD3, "payload")) == makera::DecodeResult::complete);
    CHECK(decoder->packet().type == 0xD3);
    CHECK(packet_data(decoder->packet()) == "payload");
  }

  {
    TEST("an empty payload is valid");
    Decoder decoder;
    CHECK(feed(decoder, make_frame(0x71, "")) == makera::DecodeResult::complete);
    CHECK(decoder->packet().data_length == 0);
  }

  {
    TEST("the decoder does not interpret packet types");
    Decoder decoder;
    CHECK(feed(decoder, make_frame(0xFE, "unknown")) == makera::DecodeResult::complete);
    CHECK(decoder->packet().type == 0xFE);
    CHECK(packet_data(decoder->packet()) == "unknown");
  }

  {
    TEST("a bad CRC is rejected");
    Decoder decoder;
    std::vector<uint8_t> frame = make_frame(0xA2, "G28");
    frame[frame.size() - 3] ^= 1;
    CHECK(feed(decoder, frame) == makera::DecodeResult::invalid_crc);
  }

  {
    TEST("a bad footer is rejected");
    Decoder decoder;
    std::vector<uint8_t> frame = make_frame(0xA2, "G28");
    frame.back() ^= 1;
    CHECK(feed(decoder, frame) == makera::DecodeResult::invalid_footer);
  }

  {
    TEST("invalid lengths are rejected immediately");
    Decoder decoder;
    const uint8_t prefix[] = {0x86, 0x68, 0x00, 0x02};
    CHECK(decoder->decode_byte(prefix[0], 1) == makera::DecodeResult::incomplete);
    CHECK(decoder->decode_byte(prefix[1], 1) == makera::DecodeResult::incomplete);
    CHECK(decoder->decode_byte(prefix[2], 1) == makera::DecodeResult::incomplete);
    CHECK(decoder->decode_byte(prefix[3], 1) == makera::DecodeResult::invalid_length);

    const uint8_t oversized[] = {0x86, 0x68, 0xFF, 0xFF};
    CHECK(decoder->decode_byte(oversized[0], 2) == makera::DecodeResult::incomplete);
    CHECK(decoder->decode_byte(oversized[1], 2) == makera::DecodeResult::incomplete);
    CHECK(decoder->decode_byte(oversized[2], 2) == makera::DecodeResult::incomplete);
    CHECK(decoder->decode_byte(oversized[3], 2) == makera::DecodeResult::invalid_length);
  }

  {
    TEST("garbage before a frame is ignored");
    Decoder decoder;
    const uint8_t garbage[] = {0x00, 0xFF, 0x86, 0x12, 0x55};
    for (uint8_t byte : garbage) CHECK(decoder->decode_byte(byte, 1) == makera::DecodeResult::incomplete);
    CHECK(feed(decoder, make_frame(0xA2, "G28")) == makera::DecodeResult::complete);
  }

  {
    TEST("overlapping header prefixes are recognized");
    Decoder decoder;
    CHECK(decoder->decode_byte(0x86, 1) == makera::DecodeResult::incomplete);
    CHECK(decoder->decode_byte(0x86, 1) == makera::DecodeResult::incomplete);
    CHECK(decoder->decode_byte(0x68, 1) == makera::DecodeResult::incomplete);
    CHECK(decoder->in_progress());
  }

  {
    TEST("a header prefix is not a complete header");
    Decoder decoder;
    CHECK(!decoder->has_header());
    decoder->decode_byte(0x86, 1);
    CHECK(!decoder->has_header());
    decoder->decode_byte(0x68, 1);
    CHECK(decoder->has_header());
  }

  {
    TEST("a valid header in an invalid length is preserved");
    Decoder decoder;
    std::vector<uint8_t> bytes{0x86, 0x68};
    const std::vector<uint8_t> frame = make_frame(0xA2, "next");
    bytes.insert(bytes.end(), frame.begin(), frame.end());
    CHECK(feed(decoder, bytes) == makera::DecodeResult::complete);
    CHECK(packet_data(decoder->packet()) == "next");
  }

  {
    TEST("the next frame is recovered when it replaces a missing footer");
    Decoder decoder;
    std::vector<uint8_t> first = make_frame(0xA2, "lost-footer");
    first.resize(first.size() - 2);
    const std::vector<uint8_t> second = make_frame(0xA2, "next");
    first.insert(first.end(), second.begin(), second.end());
    CHECK(feed(decoder, first) == makera::DecodeResult::complete);
    CHECK(packet_data(decoder->packet()) == "next");
  }

  {
    TEST("the next frame is recovered after a frame truncated in its trailer");
    for (std::size_t missing = 3; missing <= 4; ++missing) {
      Decoder decoder;
      std::vector<uint8_t> first = make_frame(0xA2, "truncated");
      first.resize(first.size() - missing);
      const std::vector<uint8_t> second = make_frame(0xA2, "next");
      first.insert(first.end(), second.begin(), second.end());
      CHECK(feed(decoder, first) == makera::DecodeResult::complete);
      CHECK(packet_data(decoder->packet()) == "next");
    }
  }

  {
    TEST("a stalled partial frame is abandoned");
    Decoder decoder;
    std::vector<uint8_t> partial = make_frame(0xA2, "partial");
    partial.resize(6);
    feed(decoder, partial, 10);
    CHECK(decoder->in_progress());
    CHECK(feed(decoder, make_frame(0xA2, "next"), 10 + makera::frame_timeout_ms) == makera::DecodeResult::complete);
    CHECK(packet_data(decoder->packet()) == "next");
  }

  {
    TEST("a slow frame survives while every inter-byte gap is bounded");
    Decoder decoder;
    CHECK(feed(decoder, make_frame(0xA2, "slow"), 10, makera::frame_timeout_ms - 1) == makera::DecodeResult::complete);
  }

  {
    TEST("timeout arithmetic survives the ticker wrapping");
    Decoder decoder;
    CHECK(feed(decoder, make_frame(0xA2, "wrap"), UINT32_MAX - 8, 1) == makera::DecodeResult::complete);
  }

  {
    TEST("bytes_wanted follows the frame state");
    Decoder decoder;
    const std::vector<uint8_t> frame = make_frame(0xA2, "G28");
    CHECK(decoder->bytes_wanted() == 2);
    decoder->decode_byte(frame[0], 1);
    CHECK(decoder->bytes_wanted() == 1);
    decoder->decode_byte(frame[1], 1);
    CHECK(decoder->bytes_wanted() == 2);
    decoder->decode_byte(frame[2], 1);
    CHECK(decoder->bytes_wanted() == 1);
    decoder->decode_byte(frame[3], 1);
    CHECK(decoder->bytes_wanted() == frame.size() - 4);
  }

  {
    TEST("the largest supported payload is accepted");
    Decoder decoder;
    const std::vector<uint8_t> payload(makera::max_data_size, 0x5A);
    CHECK(feed(decoder, make_frame(0xD3, payload)) == makera::DecodeResult::complete);
    CHECK(decoder->packet().data_length == makera::max_data_size);
    CHECK(decoder->packet().data[makera::max_data_size - 1] == 0x5A);
  }

  {
    TEST("reset discards a partial frame");
    Decoder decoder;
    const std::vector<uint8_t> frame = make_frame(0xA2, "partial");
    for (std::size_t i = 0; i < 6; ++i) decoder->decode_byte(frame[i], 1);
    CHECK(decoder->in_progress());
    decoder->reset();
    CHECK(!decoder->in_progress());
    CHECK(feed(decoder, make_frame(0xA2, "next")) == makera::DecodeResult::complete);
  }

  {
    TEST("Makera control bytes have one shared interpretation");
    CHECK(makera::decode_control('?') == makera::ControlAction::query);
    CHECK(makera::decode_control('*') == makera::ControlAction::diagnose);
    CHECK(makera::decode_control('X' - 'A' + 1) == makera::ControlAction::halt);
    CHECK(makera::decode_control('Y' - 'A' + 1) == makera::ControlAction::stop);
    CHECK(makera::decode_control('Z' - 'A' + 1) == makera::ControlAction::keep_alive);
    CHECK(makera::decode_control('!') == makera::ControlAction::feed_hold);
    CHECK(makera::decode_control('~') == makera::ControlAction::feed_resume);
    CHECK(makera::decode_control('x') == makera::ControlAction::none);
  }

  {
    TEST("text diagnostic requests use the realtime diagnostic path");
    const char request[] = "diagnose\n";
    CHECK(makera::is_diagnostic_request(reinterpret_cast<const uint8_t *>(request), sizeof(request) - 1));

    const char other[] = "diagnose-now\n";
    CHECK(!makera::is_diagnostic_request(reinterpret_cast<const uint8_t *>(other), sizeof(other) - 1));
  }

  std::printf("\n%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
