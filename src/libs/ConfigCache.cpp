#include "ConfigCache.h"

#include "libs/StreamOutput.h"

ConfigCache::ConfigCache()
{
    // Place the store in the gap between heap and stack — no heap allocation.
    store = (ConfigValue *)((uintptr_t)&__StackLimit
                            - CONFIG_CACHE_CAPACITY * sizeof(ConfigValue));
    count = 0;
}

ConfigCache::~ConfigCache()
{
    clear();
}

void ConfigCache::clear()
{
    count = 0;
}

void ConfigCache::pop()
{
    if(count > 0) count--;
}

// If we find an existing value, replace it, otherwise copy it to the back.
// Returns a pointer to the entry in the store.
ConfigValue *ConfigCache::replace_or_push_back(const ConfigValue &new_value)
{
    for(uint16_t i = 0; i < count; i++) {
        if(memcmp(new_value.check_sums, store[i].check_sums, sizeof(store[i].check_sums)) == 0) {
            store[i] = new_value;
            return &store[i];
        }
    }

    if(count < CONFIG_CACHE_CAPACITY) {
        store[count] = new_value;
        return &store[count++];
    }

    // Overflow — should not happen with correctly sized capacity
    return NULL;
}

ConfigValue *ConfigCache::lookup(const uint16_t *check_sums)
{
    for(uint16_t i = 0; i < count; i++) {
        if(memcmp(check_sums, store[i].check_sums, sizeof(store[i].check_sums)) == 0)
            return &store[i];
    }

    return NULL;
}

void ConfigCache::collect(uint16_t family, uint16_t cs, vector<uint16_t> *list)
{
    for(uint16_t i = 0; i < count; i++) {
        if( store[i].check_sums[2] == cs && store[i].check_sums[0] == family ) {
            list->push_back(store[i].check_sums[1]);
        }
    }
}

void ConfigCache::dump(StreamOutput *stream)
{
    for(uint16_t i = 0; i < count; i++) {
        stream->printf("%3d - %04X %04X %04X : '%s'\n",
                       i + 1, store[i].check_sums[0], store[i].check_sums[1],
                       store[i].check_sums[2], store[i].value);
    }
}
