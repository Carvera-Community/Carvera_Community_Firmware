#pragma once

#include "libs/ConfigSource.h"
class ConfigCache;
class SerialConsole;

class RemoteConfigSource : public ConfigSource {
 public:
  explicit RemoteConfigSource(SerialConsole& serial);

  void transfer_values_to_cache(ConfigCache* cache) override;
  uint32_t store_config_records(ConfigCache& cache, const uint8_t* data, std::size_t size);
  bool is_named(uint16_t check_sum) override;
  bool write(std::string setting, std::string value) override;
  std::string read(uint16_t check_sums[3]) override;

 private:
  SerialConsole& serial_;
};
