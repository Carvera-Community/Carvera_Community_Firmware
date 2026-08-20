#include "FactorySettings.h"

#include "ConfigSource.h"
#include "checksumm.h"
#include "utils.h"

namespace {

#define machine_model_checksum CHECKSUM("Machine_Model")
#define a_axis_home_checksum CHECKSUM("A_Axis_home_enable")
#define c_axis_home_checksum CHECKSUM("C_Axis_home_enable")
#define atc_checksum CHECKSUM("Atc_enable")
#define ce1_expand_checksum CHECKSUM("CE1_Expand")

unsigned char parse_value(std::string_view text) {
  std::size_t cursor = 0;
  bool negative = false;
  if (cursor < text.size() && (text[cursor] == '+' || text[cursor] == '-')) {
    negative = text[cursor] == '-';
    ++cursor;
  }

  unsigned long value = 0;
  while (cursor < text.size() && text[cursor] >= '0' && text[cursor] <= '9') {
    value = value * 10 + static_cast<unsigned>(text[cursor] - '0');
    ++cursor;
  }
  const long signed_value = negative ? -static_cast<long>(value) : static_cast<long>(value);
  return static_cast<unsigned char>(signed_value);
}

}  // namespace

void FactorySettings::set_bit(uint8_t bit, bool enabled) {
  uint8_t bits = static_cast<uint8_t>(values_.FuncSetting);
  bits = enabled ? static_cast<uint8_t>(bits | (1U << bit)) : static_cast<uint8_t>(bits & ~(1U << bit));
  values_.FuncSetting = static_cast<char>(bits);
}

FactorySettings::LineResult FactorySettings::apply_line(std::string_view line) {
  // factory.ini uses the normal configuration-file syntax. For example:
  //
  //   Machine_Model 3
  //   A_Axis_home_enable 0  # No rotary axis fitted
  const ConfigLine setting(line);
  if (setting.result() == ConfigLine::Result::ignored) return LineResult::ignored;
  if (setting.result() == ConfigLine::Result::missing_pair) return LineResult::missing_pair;
  if (setting.result() == ConfigLine::Result::missing_value) return LineResult::missing_value;

  const unsigned char value = parse_value(setting.value());
  switch (get_checksum(setting.key())) {
    case machine_model_checksum:
      values_.MachineModel = value;
      break;
    case a_axis_home_checksum:
      set_bit(0, value == 1);
      break;
    case c_axis_home_checksum:
      set_bit(1, value == 1);
      break;
    case atc_checksum:
      set_bit(2, value == 1);
      break;
    case ce1_expand_checksum:
      set_bit(3, value == 1);
      break;
    default:
      return LineResult::ignored;
  }
  return LineResult::applied;
}
