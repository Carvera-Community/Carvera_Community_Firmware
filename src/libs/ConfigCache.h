/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIGCACHE_H
#define CONFIGCACHE_H

using namespace std;
#include <vector>
#include <stdint.h>

#include "ConfigValue.h"

class StreamOutput;

class ConfigCache {
    public:
        ConfigCache();
        ~ConfigCache();
        void clear();

        void pop();

        // lookup and return the entry that matches the check sums, return NULL if not found
        ConfigValue *lookup(const uint16_t *check_sums);

        // collect enabled checksums of the given family
        void collect(uint16_t family, uint16_t cs, vector<uint16_t> *list);

        // If we find an existing value, replace it, otherwise copy it to the back
        ConfigValue *replace_or_push_back(const ConfigValue &new_value);

        // used for debugging, dumps the cache to a stream
        void dump(StreamOutput *stream);

    private:
        static const uint8_t VALUES_PER_CHUNK = 16;

        struct Chunk {
            Chunk() : next(NULL), used(0) {}
            ConfigValue values[VALUES_PER_CHUNK];
            Chunk *next;
            uint8_t used;
        };

        Chunk *first;
        Chunk *last;
        uint16_t count;
};



#endif
