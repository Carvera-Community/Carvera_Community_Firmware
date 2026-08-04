#include "ConfigValue.h"
#include "StreamOutputPool.h"

#include "libs/Kernel.h"
#include "libs/utils.h"  // remove_non_number
#include "libs/Pin.h"
#include "Pwm.h"


#define reportConfigError(...) do { \
    THEKERNEL->streams->printf(__VA_ARGS__); \
    THEKERNEL->set_config_load_error(true); \
} while (0)

#include <vector>
#include <stdio.h>

ConfigValue ConfigValue::dummy;

ConfigValue::ConfigValue()
{
    clear();
}

void ConfigValue::clear()
{
    this->check_sums[0] = 0x0000;
    this->check_sums[1] = 0x0000;
    this->check_sums[2] = 0x0000;
    this->value[0] = '\0';
}

ConfigValue::ConfigValue(uint16_t *cs) {
    memcpy(this->check_sums, cs, sizeof(this->check_sums));
    this->value[0] = '\0';
}

void ConfigValue::set_value(const char *s, size_t len) {
    if (len >= sizeof(this->value)) {
        reportConfigError("ERROR: config value '%.*s' exceeds %d char limit, truncating\r\n",
                          (int)len, s, (int)(sizeof(this->value) - 1));
        len = sizeof(this->value) - 1;
    }
    memcpy(this->value, s, len);
    this->value[len] = '\0';
}

ConfigValue *ConfigValue::required()
{
    if( !this->found() ) {
        reportConfigError("could not find config setting, please see http://smoothieware.org/configuring-smoothie\r\n");
    }
    return this;
}

float ConfigValue::as_number()
{
    if( !this->found() ) return 0.0f;
    char *endptr = NULL;
    string str = remove_non_number(this->value);
    const char *cp = str.c_str();
    float result = strtof(cp, &endptr);
    if( endptr <= cp ) {
        reportConfigError("config setting with value '%s' and checksums[%04X,%04X,%04X] is not a valid number, please see http://smoothieware.org/configuring-smoothie\r\n", this->value, this->check_sums[0], this->check_sums[1], this->check_sums[2] );
    }
    return result;
}

int ConfigValue::as_int()
{
    if( !this->found() ) return 0;
    char *endptr = NULL;
    string str = remove_non_number(this->value);
    const char *cp = str.c_str();
    int result = strtol(cp, &endptr, 10);
    if( endptr <= cp ) {
        reportConfigError("config setting with value '%s' and checksums[%04X,%04X,%04X] is not a valid int, please see http://smoothieware.org/configuring-smoothie\r\n", this->value, this->check_sums[0], this->check_sums[1], this->check_sums[2] );
    }
    return result;
}

std::string ConfigValue::as_string()
{
    return string(this->value);
}

bool ConfigValue::as_bool()
{
    if( !this->found() ) return false;
    return strchr(this->value, 't') || strchr(this->value, 'y') || strchr(this->value, '1');
}

float ConfigValue::as_number(float dflt)
{
    return this->found() ? this->as_number() : dflt;
}

int ConfigValue::as_int(int dflt)
{
    return this->found() ? this->as_int() : dflt;
}

bool ConfigValue::as_bool(bool dflt)
{
    return this->found() ? this->as_bool() : dflt;
}

string ConfigValue::as_string(const string &dflt)
{
    return this->found() ? this->as_string() : dflt;
}

bool ConfigValue::has_characters( const char *mask )
{
    return strpbrk(this->value, mask) != nullptr;
}

bool ConfigValue::is_inverted()
{
    return this->has_characters("!");
}

