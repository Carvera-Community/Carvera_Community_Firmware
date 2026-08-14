#include "CANopen.h"

#include <array>
#include <cstdlib>
#include <cstring>

#include "CANopenProtocol.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "checksumm.h"
#include "libs/Kernel.h"
#include "libs/PublicDataRequest.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "us_ticker_api.h"

#define canopen_checksum CHECKSUM("canopen")
#define can_enable_checksum CHECKSUM("can_enable")
#define can_bitrate_checksum CHECKSUM("can_bitrate")
#define can_node_id_checksum CHECKSUM("can_node_id")
#define slave_node_id_checksum CHECKSUM("slave_node_id")
#define can_heartbeat_ms_checksum CHECKSUM("can_heartbeat_ms")
#define status_checksum CHECKSUM("status")
#define send_frame_checksum CHECKSUM("send_frame")

namespace {
constexpr PinName can_rx_pin = P0_4;
constexpr PinName can_tx_pin = P0_5;
constexpr uint32_t sdo_timeout_us = 50000;
constexpr uint32_t tx_timeout_us = 5000;
constexpr uint16_t od_digital_inputs = 0x6000;
constexpr uint16_t od_digital_outputs = 0x6001;
constexpr uint16_t od_node_config = 0x6900;

constexpr std::array<uint32_t, 8> baud_rates{10000, 20000, 50000, 125000, 250000, 500000, 800000, 1000000};

bool supported_bitrate(uint32_t bitrate) {
  for (uint32_t supported : baud_rates) {
    if (bitrate == supported) return true;
  }
  return false;
}

bool valid_nmt_command(int command) {
  return command == 1 || command == 2 || command == 128 || command == 129 || command == 130;
}

uint32_t parsed_subcode(const Gcode* gcode) {
  const char* cursor = gcode->get_command();
  while (*cursor != '\0' && *cursor != 'M' && *cursor != 'm') ++cursor;
  if (*cursor == '\0') return 0;
  char* code_end = nullptr;
  std::strtoul(cursor + 1, &code_end, 10);
  if (code_end == cursor + 1 || *code_end != '.') return 0;
  char* end = nullptr;
  const unsigned long value = std::strtoul(code_end + 1, &end, 10);
  if (end == code_end + 1 || value > UINT32_MAX) return UINT32_MAX;
  return static_cast<uint32_t>(value);
}
}  // namespace

CANOpenManager::CANOpenManager()
    : bus(nullptr),
      enabled(false),
      bus_ready(false),
      bitrate(1000000),
      node_id(10),
      remote_node_id(1),
      pending_sdo_node(0),
      heartbeat_ms(1000),
      last_heartbeat_timestamp(0),
      last_remote_heartbeat_us(0),
      last_sdo_abort(0),
      remote_state(0),
      have_remote_heartbeat(false),
      has_last_rx_message(false) {}

void CANOpenManager::on_module_loaded() {
  register_for_event(ON_MAIN_LOOP);
  register_for_event(ON_GCODE_RECEIVED);
  register_for_event(ON_GET_PUBLIC_DATA);
  register_for_event(ON_SET_PUBLIC_DATA);
  register_for_event(ON_HALT);
  configure();
}

void CANOpenManager::configure() {
  enabled = THEKERNEL->config->value(canopen_checksum, can_enable_checksum)->as_bool(true);

  const int configured_bitrate = THEKERNEL->config->value(canopen_checksum, can_bitrate_checksum)->as_int(1000000);
  bitrate = configured_bitrate > 0 ? static_cast<uint32_t>(configured_bitrate) : 0;

  const int configured_node = THEKERNEL->config->value(canopen_checksum, can_node_id_checksum)->as_int(10);
  node_id = canopen::valid_node_id(configured_node) ? static_cast<uint8_t>(configured_node) : 0;

  const int configured_remote = THEKERNEL->config->value(canopen_checksum, slave_node_id_checksum)->as_int(1);
  remote_node_id = canopen::valid_node_id(configured_remote) ? static_cast<uint8_t>(configured_remote) : 0;

  const int configured_heartbeat = THEKERNEL->config->value(canopen_checksum, can_heartbeat_ms_checksum)->as_int(1000);
  heartbeat_ms = configured_heartbeat >= 0 && configured_heartbeat <= 60000
                     ? static_cast<uint32_t>(configured_heartbeat)
                     : UINT32_MAX;

  if (enabled) initialize_bus();
}

bool CANOpenManager::initialize_bus() {
  if (!supported_bitrate(bitrate) || !canopen::valid_node_id(node_id) || !canopen::valid_node_id(remote_node_id) ||
      heartbeat_ms == UINT32_MAX) {
    THEKERNEL->streams->printf("ERROR: Invalid CANopen configuration\n");
    bus_ready = false;
    return false;
  }

  if (bus == nullptr) {
    static CANBus can_storage(can_rx_pin, can_tx_pin, accept_message, this);
    bus = &can_storage;
  }
  bus_ready = bus->start(bitrate);
  if (!bus_ready) {
    THEKERNEL->streams->printf("ERROR: Failed to start CAN at %lu bps\n", bitrate);
    return false;
  }

  last_heartbeat_timestamp = us_ticker_read();
  last_remote_heartbeat_us = 0;
  have_remote_heartbeat = false;
  has_last_rx_message = false;
  return true;
}

bool CANOpenManager::accept_message(const mbed::CANMessage& message, void* context) {
  const auto* self = static_cast<const CANOpenManager*>(context);
  return message.id == canopen::heartbeat_cob_id(self->remote_node_id) ||
         (self->pending_sdo_node != 0 && message.id == canopen::sdo_response_base + self->pending_sdo_node);
}

void CANOpenManager::handle_rx(const mbed::CANMessage& message, uint32_t now) {
  if (!accept_message(message, this)) return;
  last_rx_message = message;
  has_last_rx_message = true;
  if (message.id == canopen::heartbeat_cob_id(remote_node_id) && message.len == 1) {
    remote_state = message.data[0];
    last_remote_heartbeat_us = now;
    have_remote_heartbeat = true;
  }
}

void CANOpenManager::drain_rx() {
  mbed::CANMessage message;
  while (bus->read(message)) handle_rx(message, us_ticker_read());
}

void CANOpenManager::send_heartbeat(uint32_t now) {
  if (heartbeat_ms == 0 || now - last_heartbeat_timestamp < heartbeat_ms * 1000U) return;
  last_heartbeat_timestamp = now;

  mbed::CANMessage message;
  message.id = canopen::heartbeat_cob_id(node_id);
  message.len = 1;
  message.data[0] = 5;
  if (!bus->send(message, tx_timeout_us)) bus_ready = bus->is_ready();
}

bool CANOpenManager::send_nmt(uint8_t command, uint8_t node) {
  mbed::CANMessage message;
  message.id = canopen::nmt_cob_id;
  message.len = 2;
  message.data[0] = command;
  message.data[1] = node;
  const bool sent = bus->send(message, tx_timeout_us);
  bus_ready = bus->is_ready();
  return sent;
}

void CANOpenManager::on_main_loop(void*) {
  if (!enabled || !bus_ready) return;
  drain_rx();
  send_heartbeat(us_ticker_read());
}

void CANOpenManager::on_halt(void* argument) {
  if (argument != nullptr) has_last_rx_message = false;
}

bool CANOpenManager::remote_online(uint32_t now) const {
  if (!have_remote_heartbeat || heartbeat_ms == 0) return false;
  return now - last_remote_heartbeat_us <= heartbeat_ms * 3000U;
}

bool CANOpenManager::wait_for_sdo_response(uint8_t node, uint16_t index, uint8_t subindex, bool upload,
                                           uint32_t timeout_us, uint32_t& value, uint8_t& size) {
  const uint32_t started = us_ticker_read();
  last_sdo_abort = 0;
  mbed::CANMessage message;

  while (us_ticker_read() - started < timeout_us) {
    if (!bus->read(message)) continue;
    handle_rx(message, us_ticker_read());
    const canopen::FrameView frame{message.id, message.len, message.format == CANStandard, message.type == CANData,
                                   message.data};
    const auto response = canopen::decode_sdo_response(frame, node, index, subindex, upload);
    switch (response.kind) {
      case canopen::SdoResponseKind::unrelated:
        continue;
      case canopen::SdoResponseKind::upload:
        value = response.value;
        size = response.size;
        return true;
      case canopen::SdoResponseKind::download:
        return true;
      case canopen::SdoResponseKind::abort:
        last_sdo_abort = response.abort_code;
        return false;
      case canopen::SdoResponseKind::malformed:
        return false;
    }
  }
  return false;
}

bool CANOpenManager::sdo_upload(uint8_t node, uint16_t index, uint8_t subindex, uint32_t& value, uint8_t& size) {
  last_sdo_abort = 0;
  if (!bus_ready || !canopen::valid_node_id(node)) return false;
  drain_rx();

  mbed::CANMessage request;
  request.id = canopen::sdo_request_cob_id(node);
  request.len = canopen::max_data_length;
  request.data[0] = canopen::sdo_upload_request_command;
  request.data[1] = index & 0xff;
  request.data[2] = index >> 8;
  request.data[3] = subindex;
  pending_sdo_node = node;
  if (!bus->send(request, tx_timeout_us)) {
    pending_sdo_node = 0;
    bus_ready = bus->is_ready();
    return false;
  }
  const bool received = wait_for_sdo_response(node, index, subindex, true, sdo_timeout_us, value, size);
  pending_sdo_node = 0;
  return received;
}

bool CANOpenManager::sdo_download(uint8_t node, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size) {
  last_sdo_abort = 0;
  if (!bus_ready || !canopen::valid_node_id(node) || size < 1 || size > 4) return false;
  drain_rx();

  mbed::CANMessage request;
  request.id = canopen::sdo_request_cob_id(node);
  request.len = canopen::max_data_length;
  request.data[0] = canopen::sdo_download_command(size);
  request.data[1] = index & 0xff;
  request.data[2] = index >> 8;
  request.data[3] = subindex;
  for (uint8_t i = 0; i < size; ++i) request.data[4 + i] = value >> (i * 8);
  pending_sdo_node = node;
  if (!bus->send(request, tx_timeout_us)) {
    pending_sdo_node = 0;
    bus_ready = bus->is_ready();
    return false;
  }
  uint32_t ignored_value = 0;
  uint8_t ignored_size = 0;
  const bool received =
      wait_for_sdo_response(node, index, subindex, false, sdo_timeout_us, ignored_value, ignored_size);
  pending_sdo_node = 0;
  return received;
}

bool CANOpenManager::addressed_node(Gcode* gcode, uint8_t& node) const {
  if (gcode != nullptr && gcode->has_letter('N')) {
    const int requested = gcode->get_int('N');
    if (!canopen::valid_node_id(requested)) return false;
    node = static_cast<uint8_t>(requested);
    return true;
  }
  node = remote_node_id;
  return true;
}

void CANOpenManager::report_sdo_failure(StreamOutput* stream, const char* operation) const {
  if (last_sdo_abort != 0) {
    stream->printf("ERROR: CANopen %s aborted (0x%08lX)\n", operation, last_sdo_abort);
  } else {
    stream->printf("ERROR: CANopen %s failed or timed out\n", operation);
  }
}

void CANOpenManager::print_status(StreamOutput* stream) {
  stream->printf("CANopen %s\n", enabled ? "enabled" : "disabled");
  if (!enabled) return;

  stream->printf("  Bitrate: %lu\n", bitrate);
  stream->printf("  Local node: %u\n", node_id);
  stream->printf("  Remote node: %u\n", remote_node_id);
  stream->printf("  Heartbeat: %lu ms\n", heartbeat_ms);
  stream->printf("  Bus ready: %s\n", bus_ready ? "yes" : "no");

  const CANBus::Statistics statistics = bus != nullptr ? bus->statistics() : CANBus::Statistics{};
  stream->printf("  TX: %lu RX: %lu RX overflow: %lu TX failed: %lu TX timeout: %lu\n", statistics.transmitted,
                 statistics.received, statistics.rx_overflow, statistics.tx_failed, statistics.tx_timeout);
  stream->printf("  Controller TX errors: %u RX errors: %u\n", statistics.tx_errors, statistics.rx_errors);

  const uint32_t now = us_ticker_read();
  if (have_remote_heartbeat) {
    stream->printf("  Remote heartbeat: state %u, age %lu ms, %s\n", remote_state,
                   (now - last_remote_heartbeat_us) / 1000U, remote_online(now) ? "online" : "stale");
  } else {
    stream->printf("  Remote heartbeat: not seen\n");
  }
  if (has_last_rx_message) {
    stream->printf("  Last RX ID: 0x%03X LEN:%u\n", last_rx_message.id, static_cast<unsigned int>(last_rx_message.len));
  }
}

void CANOpenManager::on_gcode_received(void* argument) {
  Gcode* gcode = static_cast<Gcode*>(argument);
  if (!gcode->has_m) return;

  if (gcode->m == 940) {
    print_status(gcode->stream);
    gcode->stream->printf("ok\n");
    return;
  }
  if (gcode->m != 941) return;
  if (!enabled || !bus_ready) {
    gcode->stream->printf("ERROR: CANopen is not ready\n");
    return;
  }

  const uint32_t full_subcode = parsed_subcode(gcode);
  if (full_subcode > 15) {
    gcode->stream->printf("ERROR: Unsupported M941.%lu\n", full_subcode);
    return;
  }
  const uint8_t subcode = static_cast<uint8_t>(full_subcode);
  if (subcode == 0) {
    if (!gcode->has_letter('X')) {
      gcode->stream->printf("ERROR: Missing CAN frame ID (X)\n");
      return;
    }
    const int id = gcode->get_int('X');
    const int length = gcode->has_letter('L') ? gcode->get_int('L') : 0;
    if (id < 0 || id > static_cast<int>(canopen::standard_id_max) || length < 0 || length > canopen::max_data_length) {
      gcode->stream->printf("ERROR: Invalid CAN frame ID or length\n");
      return;
    }

    mbed::CANMessage message;
    message.id = id;
    message.len = length;
    for (int i = 0; i < length; ++i) {
      const char letter = static_cast<char>('A' + i);
      const int byte = gcode->has_letter(letter) ? gcode->get_int(letter) : 0;
      if (byte < 0 || byte > 255) {
        gcode->stream->printf("ERROR: Invalid CAN data byte %c\n", letter);
        return;
      }
      message.data[i] = byte;
    }
    if (!bus->send(message, tx_timeout_us)) {
      bus_ready = bus->is_ready();
      gcode->stream->printf("ERROR: CAN frame send failed\n");
      return;
    }
    gcode->stream->printf("ok\n");
    return;
  }

  uint8_t node = 0;
  if (!addressed_node(gcode, node)) {
    gcode->stream->printf("ERROR: Invalid CANopen node ID (N)\n");
    return;
  }
  uint32_t value = 0;
  uint8_t size = 0;
  switch (subcode) {
    case 1:
      if (!sdo_upload(node, od_node_config, 1, value, size)) {
        report_sdo_failure(gcode->stream, "node-ID read");
        return;
      }
      gcode->stream->printf("node %u id: %lu\n", node, value & 0xffff);
      break;
    case 2: {
      if (!gcode->has_letter('X') || !canopen::valid_node_id(gcode->get_int('X'))) {
        gcode->stream->printf("ERROR: Invalid node ID (X)\n");
        return;
      }
      const uint32_t new_id = gcode->get_int('X');
      if (!sdo_download(node, od_node_config, 1, new_id, 2)) {
        report_sdo_failure(gcode->stream, "node-ID write");
        return;
      }
      gcode->stream->printf("node %u id set to %lu; power-cycle device to apply\n", node, new_id);
      break;
    }
    case 3:
      if (!sdo_upload(node, od_node_config, 2, value, size)) {
        report_sdo_failure(gcode->stream, "bitrate read");
        return;
      }
      if (value < baud_rates.size()) {
        gcode->stream->printf("node %u baud code: %lu (%lu bps)\n", node, value, baud_rates[value]);
      } else {
        gcode->stream->printf("node %u baud code: %lu\n", node, value);
      }
      break;
    case 4: {
      if (!gcode->has_letter('X')) {
        gcode->stream->printf("ERROR: Missing bitrate or code (X)\n");
        return;
      }
      const int requested = gcode->get_int('X');
      int code = requested;
      if (requested >= 8) {
        code = -1;
        for (unsigned int i = 0; i < baud_rates.size(); ++i) {
          if (baud_rates[i] == static_cast<uint32_t>(requested)) code = i;
        }
      }
      if (code < 0 || code >= static_cast<int>(baud_rates.size())) {
        gcode->stream->printf("ERROR: Unsupported CAN bitrate\n");
        return;
      }
      if (!sdo_download(node, od_node_config, 2, code, 2)) {
        report_sdo_failure(gcode->stream, "bitrate write");
        return;
      }
      gcode->stream->printf("node %u baud set to %lu bps (code %d)\n", node, baud_rates[code], code);
      break;
    }
    case 5:
      if (!sdo_upload(node, od_digital_outputs, 1, value, size)) {
        report_sdo_failure(gcode->stream, "digital-output read");
        return;
      }
      gcode->stream->printf("node %u digital outputs: 0x%08lX\n", node, value);
      break;
    case 6:
      if (!gcode->has_letter('X')) {
        gcode->stream->printf("ERROR: Missing output value (X)\n");
        return;
      }
      if (gcode->get_int('X') < 0) {
        gcode->stream->printf("ERROR: Invalid output value (X)\n");
        return;
      }
      value = static_cast<uint32_t>(gcode->get_int('X'));
      if (!sdo_download(node, od_digital_outputs, 1, value, 4)) {
        report_sdo_failure(gcode->stream, "digital-output write");
        return;
      }
      gcode->stream->printf("node %u digital outputs set to 0x%08lX\n", node, value);
      break;
    case 7:
      if (!sdo_upload(node, od_digital_inputs, 1, value, size)) {
        report_sdo_failure(gcode->stream, "digital-input read");
        return;
      }
      gcode->stream->printf("node %u digital inputs: 0x%08lX\n", node, value);
      break;
    case 8: {
      const int command = gcode->has_letter('X') ? gcode->get_int('X') : 1;
      if (!valid_nmt_command(command) || !send_nmt(command, node)) {
        gcode->stream->printf("ERROR: CANopen NMT command failed\n");
        return;
      }
      gcode->stream->printf("node %u NMT command %d sent\n", node, command);
      break;
    }
    default:
      gcode->stream->printf("ERROR: Unsupported M941.%u\n", subcode);
      return;
  }
  gcode->stream->printf("ok\n");
}

void CANOpenManager::on_get_public_data(void* argument) {
  PublicDataRequest* request = static_cast<PublicDataRequest*>(argument);
  if (!request->starts_with(canopen_checksum) || !request->second_element_is(status_checksum)) return;

  static canopen_status status;
  const uint32_t now = us_ticker_read();
  status.enabled = enabled;
  status.bus_ready = bus_ready;
  status.bitrate = bitrate;
  status.node_id = node_id;
  status.remote_node_id = remote_node_id;
  status.heartbeat_ms = heartbeat_ms;
  status.bus = bus != nullptr ? bus->statistics() : CANBus::Statistics{};
  status.have_last_rx = has_last_rx_message;
  if (has_last_rx_message) status.last_rx = last_rx_message;
  status.have_remote_heartbeat = have_remote_heartbeat;
  status.remote_online = remote_online(now);
  status.remote_state = remote_state;
  status.remote_heartbeat_age_ms = have_remote_heartbeat ? (now - last_remote_heartbeat_us) / 1000U : 0;
  request->set_data_ptr(&status);
  request->set_taken();
}

void CANOpenManager::on_set_public_data(void* argument) {
  PublicDataRequest* request = static_cast<PublicDataRequest*>(argument);
  if (!request->starts_with(canopen_checksum) || !request->second_element_is(send_frame_checksum)) return;

  const canopen_frame* frame = static_cast<const canopen_frame*>(request->get_data_ptr());
  if (frame == nullptr || !enabled || !bus_ready || frame->len > 8 || frame->type != CANData ||
      frame->format != CANStandard)
    return;

  mbed::CANMessage message;
  message.id = frame->id;
  message.len = frame->len;
  std::memcpy(message.data, frame->data, frame->len);
  if (bus->send(message, tx_timeout_us)) request->set_taken();
  bus_ready = bus->is_ready();
}
