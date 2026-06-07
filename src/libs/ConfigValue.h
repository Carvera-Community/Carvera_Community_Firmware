/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIGVALUE_H
#define CONFIGVALUE_H

#include <string>
#include <cstdint>
#include <cstring>
using std::string;

#define CONFIGVALUE_MAX_LEN 20

class ConfigValue{
    public:
        ConfigValue();
        ConfigValue(uint16_t *check_sums);
        void clear();
        ConfigValue* required();
        float as_number();
        float as_number(float dflt);
        int as_int();
        int as_int(int dflt);
        bool as_bool();
        bool as_bool(bool dflt);
        string as_string();
        string as_string(const string &dflt);

        bool is_inverted();

        bool found() const { return this != &dummy; }

        static ConfigValue dummy; // returned by Config::value() when key is not in config

        // Set value from a string, truncating to fit
        void set_value(const char *s, size_t len);
        void set_value(const string &s) { set_value(s.c_str(), s.size()); }

        friend class ConfigCache;
        friend class Config;
        friend class ConfigSource;
        friend class Configurator;
        friend class FileConfigSource;

    private:
        bool has_characters( const char* mask );
        char value[CONFIGVALUE_MAX_LEN]; // config values are short; avoids per-entry heap alloc
        uint16_t check_sums[3];
};








#endif
