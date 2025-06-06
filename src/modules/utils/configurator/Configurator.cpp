/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "libs/Kernel.h"
#include "Configurator.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "checksumm.h"
#include "Config.h"
#include "Gcode.h"
#include "ConfigSource.h"
#include "FileConfigSource.h"
#include "ConfigValue.h"
#include "ConfigCache.h"

#define CONF_NONE       0
#define CONF_ROM        1
#define CONF_SD         2
#define CONF_EEPROM     3


// Output a ConfigValue from the specified ConfigSource to the stream
void Configurator::config_get_command( string parameters, StreamOutput *stream )
{
    string source = shift_parameter(parameters);
    string setting = shift_parameter(parameters);
    if (setting == "") { // output settings from the config-cache
        setting = source;
        source = "";
        uint16_t setting_checksums[3];
        get_checksums(setting_checksums, setting );
        THEKERNEL->config->config_cache_load(); // need to load config cache first as it is unloaded after booting
        ConfigValue *cv = THEKERNEL->config->value(setting_checksums);
        if(cv != NULL && cv->found) {
            string value = cv->as_string();
            stream->printf( "cached: %s is set to %s\r\n", setting.c_str(), value.c_str() );
        } else {
            stream->printf( "cached: %s is not in config\r\n", setting.c_str());
        }
        THEKERNEL->config->config_cache_clear();

    } else { // output setting from specified source by parsing the config file
        uint16_t source_checksum = get_checksum( source );
        uint16_t setting_checksums[3];
        get_checksums(setting_checksums, setting );
        for(unsigned int i = 0; i < THEKERNEL->config->config_sources.size(); i++) {
            if( THEKERNEL->config->config_sources[i]->is_named(source_checksum) ) {
                string value = THEKERNEL->config->config_sources[i]->read(setting_checksums);
                if(value.empty()) {
                    stream->printf( "%s: %s is not in config\r\n", source.c_str(), setting.c_str() );
                } else {
                    stream->printf( "%s: %s is set to %s\r\n", source.c_str(), setting.c_str(), value.c_str() );
                }
                break;
            }
        }
    }
}

// Write the specified setting to the specified ConfigSource
void Configurator::config_set_command( string parameters, StreamOutput *stream )
{
    string source = shift_parameter(parameters);
    string setting = shift_parameter(parameters);
    string value = shift_parameter(parameters);
    if(source.empty() || setting.empty() || value.empty()) {
        stream->printf( "Usage: config-set source setting value # where source is sd, setting is the key and value is the new value\r\n" );
        return;
    }

    uint16_t source_checksum = get_checksum(source);
    for(unsigned int i = 0; i < THEKERNEL->config->config_sources.size(); i++) {
        if( THEKERNEL->config->config_sources[i]->is_named(source_checksum) ) {
            if(THEKERNEL->config->config_sources[i]->write(setting, value)) {
                if (setting == "zprobe.probe_tip_diameter"){
                    float fvalue = strtof(value.c_str(),nullptr);
                    if (fvalue){ THEKERNEL->probe_tip_diameter = fvalue;} //will fail if the tip diameter is set to 0, but it should not be anyway
                }
                stream->printf( "%s: %s has been set to %s\r\n", source.c_str(), setting.c_str(), value.c_str() );
            } else {
                stream->printf( "%s: %s not enough space to overwrite existing key/value\r\n", source.c_str(), setting.c_str() );
            }
            return;
        }
    }
    stream->printf( "%s source does not exist\r\n", source.c_str());


    /* Live setting not really supported anymore as the cache is never left loaded
        if (value == "") {
            if(!THEKERNEL->config->config_cache_loaded) {
                stream->printf( "live: setting not allowed as config cache is not loaded\r\n" );
                return;
            }
            value = setting;
            setting = source;
            source = "";
            THEKERNEL->config->set_string(setting, value);
            stream->printf( "live: %s has been set to %s\r\n", setting.c_str(), value.c_str() );
    */
}

// Reload config values from the specified ConfigSource, NOTE used for debugging by dumping the config-cache
void Configurator::config_load_command( string parameters, StreamOutput *stream )
{
    string source = shift_parameter(parameters);
    if(source == "load") {
        THEKERNEL->config->config_cache_load();
        stream->printf( "config cache loaded\r\n");

    } else if(source == "unload") {
        THEKERNEL->config->config_cache_clear();
        stream->printf( "config cache unloaded\r\n" );

    } else if(source == "dump") {
        THEKERNEL->config->config_cache_load();
        THEKERNEL->config->config_cache->dump(stream);
        THEKERNEL->config->config_cache_clear();

    } else if(source == "checksum") {
        string key = shift_parameter(parameters);
        uint16_t cs[3];
        get_checksums(cs, key);
        stream->printf( "checksum of %s = %02X %02X %02X\n", key.c_str(), cs[0], cs[1], cs[2]);

    } else {
        stream->printf( "unsupported option: must be one of load|unload|dump|checksum\n" );
    }
}
