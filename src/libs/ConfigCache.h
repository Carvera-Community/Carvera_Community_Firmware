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

// Config cache lives in a fixed region of RAM between the heap and stack,
// avoiding any heap allocation. The address is computed as:
//   __StackLimit - (capacity * sizeof(ConfigValue))
// where __StackLimit is the linker-defined bottom of the stack area.
// This memory is unused during boot (heap hasn't grown that high, stack
// hasn't grown that low). After config_cache_clear(), it returns to being
// ordinary free gap between heap and stack.
#define CONFIG_CACHE_CAPACITY 350

extern unsigned int __StackLimit;

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

        uintptr_t start_address() const { return reinterpret_cast<uintptr_t>(store); }

    private:
        ConfigValue *store;   // points into fixed RAM region, not heap
        uint16_t count;
};



#endif
