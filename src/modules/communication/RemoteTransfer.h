#pragma once

#include <cstdint>

class ConfigCache;
class RemoteConfigSource;
class SerialConsole;
struct FACTORY_SET;

namespace remote {

enum class Result : uint8_t { success, cancelled, timeout, transport_error, protocol_error, invalid_data };

Result receive_config(SerialConsole& serial, RemoteConfigSource& source, ConfigCache& cache, uint32_t& stored_records);
Result receive_factory_settings(SerialConsole& serial, const FACTORY_SET& current, FACTORY_SET& received);
void finish_factory_settings(SerialConsole& serial, bool stored);

}  // namespace remote
