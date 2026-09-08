#include "RemoteConfigSource.h"

#include "FirmConfigSource.h"
#include "libs/ConfigCache.h"
#include "libs/Kernel.h"
#include "libs/StreamOutputPool.h"
#include "modules/communication/RemoteTransfer.h"
#include "modules/communication/SerialConsole.h"
#include "utils.h"

RemoteConfigSource::RemoteConfigSource(SerialConsole& serial) : serial_(serial) {
  name_checksum = get_checksum("remote");
}

void RemoteConfigSource::transfer_values_to_cache(ConfigCache* cache) {
  uint32_t stored_records = 0;
  remote::Result result = remote::receive_config(serial_, *this, *cache, stored_records);

  // TODO(f355): Remove timeout completion after Makera fixes the ESP32 firmware
  // to advertise only the configuration records it actually sends.
  if (result == remote::Result::timeout && stored_records != 0) {
    result = remote::Result::success;
  }

  if (result != remote::Result::success) {
    cache->clear();
    FirmConfigSource embedded("firm");
    embedded.transfer_values_to_cache(cache);
  }
  if (result != remote::Result::success) {
    THEKERNEL->streams->printf("ERROR: remote configuration transfer failed (%u); using embedded defaults\n",
                               static_cast<unsigned>(result));
    THEKERNEL->set_config_load_error(true);
  }
}

uint32_t RemoteConfigSource::store_config_records(ConfigCache& cache, const uint8_t* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return 0;
  }

  uint32_t accepted = 0;
  std::size_t begin = 0;
  for (std::size_t i = 0; i <= size; ++i) {
    const bool delimiter = i == size || data[i] == '\n' || data[i] == '\0';
    if (!delimiter) continue;
    if (i != begin) {
      const std::string line(reinterpret_cast<const char*>(data + begin), i - begin);
      const std::size_t content = line.find_first_not_of(" \t\r");
      if (content != std::string::npos && line[content] != '#' &&
          process_line_from_ascii_config(line, &cache) == nullptr) {
        return 0;
      }
      ++accepted;
    }
    begin = i + 1;
  }
  return accepted;
}

bool RemoteConfigSource::is_named(uint16_t check_sum) { return check_sum == name_checksum; }

bool RemoteConfigSource::write(std::string, std::string) { return false; }

std::string RemoteConfigSource::read(uint16_t[3]) { return {}; }
