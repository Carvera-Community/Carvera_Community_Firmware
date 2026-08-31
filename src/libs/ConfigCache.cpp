#include "ConfigCache.h"

#include "libs/Kernel.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include <new>

ConfigCache::ConfigCache()
{
    first = NULL;
    last = NULL;
    count = 0;
}

ConfigCache::~ConfigCache()
{
    clear();
}

void ConfigCache::clear()
{
    while(first != NULL) {
        Chunk *next = first->next;
        delete first;
        first = next;
    }
    last = NULL;
    count = 0;
}

void ConfigCache::pop()
{
    if(last == NULL) return;

    last->used--;
    count--;
    if(last->used != 0) return;

    if(first == last) {
        delete last;
        first = last = NULL;
        return;
    }

    Chunk *previous = first;
    while(previous->next != last) previous = previous->next;
    delete last;
    last = previous;
    last->next = NULL;
}

// If we find an existing value, replace it, otherwise copy it to the back.
// Returns a pointer to the entry in the store.
ConfigValue *ConfigCache::replace_or_push_back(const ConfigValue &new_value)
{
    for(Chunk *chunk = first; chunk != NULL; chunk = chunk->next) {
        for(uint8_t i = 0; i < chunk->used; i++) {
            if(memcmp(new_value.check_sums, chunk->values[i].check_sums,
                      sizeof(chunk->values[i].check_sums)) == 0) {
                chunk->values[i] = new_value;
                return &chunk->values[i];
            }
        }
    }

    if(last == NULL || last->used == VALUES_PER_CHUNK) {
        Chunk *chunk = new(std::nothrow) Chunk;
        if(chunk == NULL) {
            THEKERNEL->streams->printf("ERROR: out of memory growing config cache\n");
            THEKERNEL->set_config_load_error(true);
            return NULL;
        }
        if(last == NULL) first = chunk;
        else last->next = chunk;
        last = chunk;
    }

    ConfigValue *value = &last->values[last->used++];
    *value = new_value;
    count++;
    return value;
}

ConfigValue *ConfigCache::lookup(const uint16_t *check_sums)
{
    for(Chunk *chunk = first; chunk != NULL; chunk = chunk->next) {
        for(uint8_t i = 0; i < chunk->used; i++) {
            if(memcmp(check_sums, chunk->values[i].check_sums,
                      sizeof(chunk->values[i].check_sums)) == 0)
                return &chunk->values[i];
        }
    }

    return NULL;
}

void ConfigCache::collect(uint16_t family, uint16_t cs, vector<uint16_t> *list)
{
    for(Chunk *chunk = first; chunk != NULL; chunk = chunk->next) {
        for(uint8_t i = 0; i < chunk->used; i++) {
            ConfigValue &value = chunk->values[i];
            if(value.check_sums[2] == cs && value.check_sums[0] == family)
                list->push_back(value.check_sums[1]);
        }
    }
}

void ConfigCache::dump(StreamOutput *stream)
{
    unsigned index = 0;
    for(Chunk *chunk = first; chunk != NULL; chunk = chunk->next) {
        for(uint8_t i = 0; i < chunk->used; i++) {
            ConfigValue &value = chunk->values[i];
            stream->printf("%3u - %04X %04X %04X : '%s'\n", ++index,
                           value.check_sums[0], value.check_sums[1],
                           value.check_sums[2], value.value);
        }
    }
}
