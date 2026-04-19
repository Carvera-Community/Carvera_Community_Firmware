#include "utils.h"
#include "ConfigSource.h"
#include "ConfigValue.h"
#include "ConfigCache.h"

#include "stdio.h"

// Parse a config line into a ConfigValue on the stack.
// Returns true if the line is a valid key-value pair.
bool ConfigSource::process_line(const string &buffer, ConfigValue &result)
{
    if( buffer[0] == '#' ) {
        return false;
    }
    if( buffer.length() < 3 ) {
        return false;
    }

    size_t begin_key = buffer.find_first_not_of(" \t");
    if(begin_key == string::npos || buffer[begin_key] == '#') return false;

    size_t end_key = buffer.find_first_of(" \t", begin_key);
    if(end_key == string::npos) {
        printf("ERROR: config file line %s is invalid, no key value pair found\r\n", buffer.c_str());
        return false;
    }

    size_t begin_value = buffer.find_first_not_of(" \t", end_key);
    if(begin_value == string::npos || buffer[begin_value] == '#') {
        printf("ERROR: config file line %s has no value\r\n", buffer.c_str());
        return false;
    }

    string key = buffer.substr(begin_key, end_key - begin_key);
    get_checksums(result.check_sums, key);

    size_t end_value = buffer.find_first_of("\r\n# \t", begin_value + 1);
    size_t vsize = end_value == string::npos ? buffer.size() - begin_value : end_value - begin_value;
    result.set_value(buffer.c_str() + begin_value, vsize);

    return true;
}

ConfigValue* ConfigSource::process_line_from_ascii_config(const string &buffer, ConfigCache *cache)
{
    ConfigValue cv;
    if(process_line(buffer, cv)) {
        return cache->replace_or_push_back(cv);
    }
    return NULL;
}

string ConfigSource::process_line_from_ascii_config(const string &buffer, uint16_t line_checksums[3])
{
    ConfigValue cv;
    if(process_line(buffer, cv)) {
        if(cv.check_sums[0] == line_checksums[0] && cv.check_sums[1] == line_checksums[1] && cv.check_sums[2] == line_checksums[2]) {
            return cv.value;
        }
    }
    return "";
}
