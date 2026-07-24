/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

using namespace std;
#include <vector>
#include <string>

#include "libs/Kernel.h"
#include "Config.h"
#include "ConfigValue.h"
#include "ConfigSource.h"
#include "ConfigCache.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/ConfigSources/FileConfigSource.h"

extern "C" caddr_t _sbrk(int);
extern "C" void setHeapCeiling(unsigned int);
extern "C" void clearHeapCeiling(void);
extern "C" unsigned int getHeapHighWaterMark(void);
#include "libs/ConfigSources/FirmConfigSource.h"
#include "StreamOutputPool.h"

// Add various config sources. Config can be fetched from several places.
// All values are read into a cache, that is then used by modules to read their configuration
Config::Config()
{
    this->config_cache = NULL;

    // Config source for firm config found in src/config.default
    this->config_sources.push_back( new FirmConfigSource("firm") );

    // Config source for */config files
    FileConfigSource *fcs = NULL;
    if( file_exists("/local/config") )
        fcs = new FileConfigSource("/local/config", "local");
    else if( file_exists("/local/config.txt") )
        fcs = new FileConfigSource("/local/config.txt", "local");
    if( fcs != NULL ) {
        this->config_sources.push_back( fcs );
        fcs = NULL;
    }
    if( file_exists("/sd/config") )
        fcs = new FileConfigSource("/sd/config", "sd");
    else if( file_exists("/sd/config.txt") )
        fcs = new FileConfigSource("/sd/config.txt", "sd");
    if( fcs != NULL )
        this->config_sources.push_back( fcs );
}

Config::Config(ConfigSource *cs)
{
    this->config_cache = NULL;
    this->config_sources.push_back( cs );
}

Config::~Config()
{
    config_cache_clear();
    for(auto i : this->config_sources) {
        delete i;
    }
}

// Get a list of modules, used by module "pools" that look for the "enable" keyboard to find things like "moduletype.modulename.enable" as the marker of a new instance of a module
void Config::get_module_list(vector<uint16_t> *list, uint16_t family)
{
    this->config_cache->collect(family, CHECKSUM("enable"), list);
}

// Command to load config cache into buffer for multiple reads during init
void Config::config_cache_load(bool parse)
{
    // First clear the cache
    this->config_cache_clear();

    this->config_cache= new ConfigCache;

    // Verify the malloc heap hasn't already grown into the config cache region.
    // _sbrk(0) returns the current top of the newlib heap, which is what every
    // allocation on this platform routes through.
    const auto heap_top = reinterpret_cast<uintptr_t>(_sbrk(0));
    const auto cache_start = this->config_cache->start_address();
    if(heap_top > cache_start) {
        THEKERNEL->streams->printf("ERROR: not enough memory to load config cache "
            "(heap=0x%x, cache=0x%x)\n", heap_top, cache_start);
        THEKERNEL->set_config_load_error(true);
        delete this->config_cache;
        this->config_cache = NULL;
        return;
    }

    // Cap the malloc heap at the cache base while the cache is live, so an
    // over-allocating boot fails with ENOMEM instead of _sbrk handing out
    // memory inside the cache (which the stack-based guard alone permits).
    setHeapCeiling(cache_start);

    if(parse) {
        // For each ConfigSource in our stack
        for( ConfigSource *source : this->config_sources ) {
            source->transfer_values_to_cache(this->config_cache);
        }
    }
}

// Command to clear the config cache after init
void Config::config_cache_clear()
{
    if(this->config_cache != NULL) {
        // Verify the heap didn't grow into the config cache region
        const auto heap_top = reinterpret_cast<uintptr_t>(_sbrk(0));
        const auto cache_start = this->config_cache->start_address();
        if(heap_top > cache_start) {
            THEKERNEL->streams->printf("FATAL: heap collided with config cache "
                "(heap=0x%x, cache=0x%x)\n", heap_top, cache_start);
            system_reset(false);
        }

        this->config_cache->clear();
        delete this->config_cache;  // frees the small ConfigCache object itself
        this->config_cache = NULL;

        // The cache region is free again; let the heap use it.
        clearHeapCeiling();

        // Report the boot heap high-water mark against the cache base. This is
        // the margin that, when exhausted, used to corrupt the cache silently —
        // now it is measured on every boot instead of guessed at.
        const auto high_water = getHeapHighWaterMark();
        THEKERNEL->streams->printf("Config cache released, heap high water 0x%08lX, boot margin %ld bytes\n",
            (unsigned long)high_water, (long)cache_start - (long)high_water);
    }
}

// Three ways to read a value from the config, depending on adress length
ConfigValue *Config::value(uint16_t check_sum_a, uint16_t check_sum_b, uint16_t check_sum_c )
{
    uint16_t check_sums[3];
    check_sums[0] = check_sum_a;
    check_sums[1] = check_sum_b;
    check_sums[2] = check_sum_c;
    return this->value(check_sums);
}

// Get a value from the configuration as a string
// Because we don't like to waste space in Flash with lengthy config parameter names, we take a checksum instead so that the name does not have to be stored
// See get_checksum
ConfigValue *Config::value(uint16_t check_sums[])
{
    if( !is_config_cache_loaded() ) {
        // Cache is unavailable (either failed to load due to heap collision,
        // or value() was called after config_cache_clear()). Surface the
        // condition and return the dummy so callers fall back to defaults
        // instead of dereferencing NULL.
        THEKERNEL->streams->printf("ERROR: config cache is not loaded\n");
        THEKERNEL->set_config_load_error(true);
        ConfigValue::dummy.clear();
        return &ConfigValue::dummy;
    }

    ConfigValue *result = this->config_cache->lookup(check_sums);

    if(result == NULL) {
        ConfigValue::dummy.clear();
        result = &ConfigValue::dummy;
    }

    return result;
}



