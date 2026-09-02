#include "RemoteTransfer.h"

#include <cstddef>
#include <string_view>

#include "FactorySettings.h"
#include "MakeraFrame.h"
#include "PublicData.h"
#include "libs/ConfigCache.h"
#include "libs/ConfigSources/RemoteConfigSource.h"
#include "mbed.h"
#include "modules/communication/SerialConsole.h"

namespace remote {
namespace {

constexpr uint16_t factory_data_size = 132;
constexpr uint16_t config_data_size = 512;
constexpr uint32_t overall_timeout_ms = 2500;

static_assert(config_data_size + 4 <= makera::max_data_size, "configuration records must fit in a Makera frame");

struct PacketTypes {
  uint8_t start;
  uint8_t view;
  uint8_t data;
  uint8_t end;
  uint8_t cancel;
  uint16_t data_size;
};

constexpr PacketTypes config_packets{PTYPE_CONFIG_START, PTYPE_CONFIG_VIEW, PTYPE_CONFIG_DATA,
                                     PTYPE_CONFIG_END,   PTYPE_CONFIG_CAN,  config_data_size};
constexpr PacketTypes factory_packets{PTYPE_FACTORY_START, PTYPE_FACTORY_VIEW, PTYPE_FACTORY_DATA,
                                      PTYPE_FACTORY_END,   PTYPE_FACTORY_CAN,  factory_data_size};

bool belongs_to(const PacketTypes& types, uint8_t type) {
  return type == types.start || type == types.view || type == types.data || type == types.end || type == types.cancel;
}

void send(SerialConsole& serial, uint8_t type, const uint8_t* payload, std::size_t size) {
  serial.PacketMessage(static_cast<char>(type), reinterpret_cast<const char*>(payload), size);
}

template <typename StoreRecords, typename RecordsComplete>
Result receive_records(SerialConsole& serial, const PacketTypes& types, StoreRecords store_records,
                       RecordsComplete records_complete, bool end_on_success = true) {
  enum class Phase : uint8_t { start, view, data };

  const uint32_t started_ms = us_ticker_read() / 1000;
  Phase phase = Phase::start;
  uint32_t total_frames = 0;
  uint32_t next_frame = 1;
  uint8_t request_data[6]{};
  uint8_t request_type = types.start;
  std::size_t request_size = 0;
  uint8_t wait_count = 0;
  uint16_t data_size = 0;

  auto send_request = [&] { send(serial, request_type, request_size == 0 ? nullptr : request_data, request_size); };
  auto fail = [&](Result result) {
    send(serial, types.cancel, nullptr, 0);
    return result;
  };

  send_request();
  for (;;) {
    const uint32_t now_ms = us_ticker_read() / 1000;
    if (now_ms - started_ms >= overall_timeout_ms) return fail(Result::timeout);

    makera::Packet packet{};
    const int receive_result = serial.receive_packet(packet, 100);
    if (receive_result != 0) {
      if (receive_result != -1) return fail(Result::transport_error);
      ++wait_count;
      if (wait_count > 25) return fail(Result::timeout);
      if (wait_count % 5 == 0) send_request();
      continue;
    }
    if (!belongs_to(types, packet.type)) continue;
    wait_count = 0;

    if (packet.type == types.cancel) {
      send(serial, types.end, nullptr, 0);
      return Result::cancelled;
    }

    if (phase == Phase::start) {
      if (packet.type != types.start || packet.data_length != 0) return fail(Result::protocol_error);
      request_data[0] = request_data[1] = request_data[2] = request_data[3] = 0;
      request_data[4] = static_cast<uint8_t>(types.data_size >> 8);
      request_data[5] = static_cast<uint8_t>(types.data_size);
      request_type = types.view;
      request_size = sizeof(request_data);
      phase = Phase::view;
      send_request();
      continue;
    }

    if (phase == Phase::view) {
      const uint16_t data_limit =
          packet.data_length == 6 ? static_cast<uint16_t>((static_cast<uint16_t>(packet.data[4]) << 8) | packet.data[5])
                                  : 0;
      if (packet.type != types.view || packet.data_length != 6 || data_limit == 0 || data_limit > types.data_size) {
        return fail(Result::protocol_error);
      }
      data_size = data_limit;
      total_frames = makera::read_be32(packet.data);
      if (total_frames == 0) {
        if (!records_complete()) return fail(Result::invalid_data);
        if (end_on_success) send(serial, types.end, nullptr, 0);
        return Result::success;
      }
      makera::write_be32(request_data, next_frame);
      request_type = types.data;
      request_size = 4;
      phase = Phase::data;
      send_request();
      continue;
    }

    if (packet.type == types.end) return fail(Result::protocol_error);
    if (packet.type != types.data || packet.data_length <= 4 || packet.data_length > 4 + data_size) {
      return fail(Result::protocol_error);
    }

    const uint32_t frame_num = makera::read_be32(packet.data);
    if (frame_num != next_frame) return fail(Result::protocol_error);

    const uint32_t stored = store_records(packet.data + 4, packet.data_length - 4);
    if (stored == 0 || stored > total_frames - (next_frame - 1)) {
      return fail(stored == 0 ? Result::invalid_data : Result::protocol_error);
    }
    const uint32_t last_frame = (next_frame - 1) + stored;
    if (last_frame == total_frames) {
      if (!records_complete()) return fail(Result::invalid_data);
      if (end_on_success) send(serial, types.end, nullptr, 0);
      return Result::success;
    }

    next_frame = last_frame + 1;
    makera::write_be32(request_data, next_frame);
    send_request();
  }
}

uint32_t store_factory_record(FACTORY_SET& settings, const uint8_t* data, std::size_t size) {
  if (data == nullptr || size == 0 || size > factory_data_size) return 0;
  while (size != 0 && (data[size - 1] == '\r' || data[size - 1] == '\n' || data[size - 1] == '\0')) --size;
  if (size == 0) return 1;

  FactorySettings parser(settings);
  const auto result = parser.apply_line(std::string_view(reinterpret_cast<const char*>(data), size));
  return result == FactorySettings::LineResult::missing_pair || result == FactorySettings::LineResult::missing_value
             ? 0
             : 1;
}

}  // namespace

Result receive_config(SerialConsole& serial, RemoteConfigSource& source, ConfigCache& cache, uint32_t& stored_records) {
  stored_records = 0;
  return receive_records(
      serial, config_packets,
      [&](const uint8_t* data, std::size_t size) {
        const uint32_t stored = source.store_config_records(cache, data, size);
        stored_records += stored;
        return stored;
      },
      [] { return true; });
}

Result receive_factory_settings(SerialConsole& serial, const FACTORY_SET& current, FACTORY_SET& received) {
  FACTORY_SET candidate = current;
  const auto valid_machine = [&] {
#if defined(MACHINE_FAMILY_Z1)
    return candidate.MachineModel == Z1 || candidate.MachineModel == Z1PRO;
#else
    return candidate.MachineModel == CARVERA || candidate.MachineModel == CARVERA_AIR;
#endif
  };
  const Result result = receive_records(
      serial, factory_packets,
      [&](const uint8_t* data, std::size_t size) { return store_factory_record(candidate, data, size); }, valid_machine,
      false);
  if (result == Result::success) received = candidate;
  return result;
}

void finish_factory_settings(SerialConsole& serial, bool stored) {
  send(serial, stored ? factory_packets.end : factory_packets.cancel, nullptr, 0);
}

}  // namespace remote
