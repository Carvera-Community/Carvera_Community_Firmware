#include "utils.h"
#include "ConfigSource.h"
#include "ConfigValue.h"
#include "ConfigCache.h"
#include "libs/Kernel.h"
#include "libs/StreamOutputPool.h"

#include "stdio.h"

ConfigLine::ConfigLine(std::string_view line) : result_(Result::ignored)
{
    if(line.size() < 3) return;

    const size_t begin_key = line.find_first_not_of(" \t");
    if(begin_key == std::string_view::npos || line[begin_key] == '#') return;

    const size_t end_key = line.find_first_of(" \t", begin_key);
    if(end_key == std::string_view::npos) {
        result_ = Result::missing_pair;
        return;
    }

    const size_t begin_value = line.find_first_not_of(" \t", end_key);
    if(begin_value == std::string_view::npos || line[begin_value] == '#') {
        result_ = Result::missing_value;
        return;
    }

    const size_t end_value = line.find_first_of("\r\n# \t", begin_value + 1);
    const size_t value_size = end_value == std::string_view::npos
        ? line.size() - begin_value
        : end_value - begin_value;
    key_ = line.substr(begin_key, end_key - begin_key);
    value_ = line.substr(begin_value, value_size);
    result_ = Result::parsed;
}

// Parse a config line into a ConfigValue on the stack.
// Returns true if the line is a valid key-value pair.
bool ConfigSource::process_line(const string &buffer, ConfigValue &result)
{
    const ConfigLine line(buffer);
    if(line.result() == ConfigLine::Result::ignored) return false;
    if(line.result() == ConfigLine::Result::missing_pair) {
        THEKERNEL->streams->printf("ERROR: config file line %s is invalid, no key value pair found\r\n", buffer.c_str());
        THEKERNEL->set_config_load_error(true);
        return false;
    }
    if(line.result() == ConfigLine::Result::missing_value) {
        THEKERNEL->streams->printf("ERROR: config file line %s has no value\r\n", buffer.c_str());
        THEKERNEL->set_config_load_error(true);
        return false;
    }

    get_checksums(result.check_sums, line.key());
    result.set_value(line.value().data(), line.value().size());

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
