#include "SpindleAccessories.h"

#include <string>

#include "BedCleaning.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "Kernel.h"
#include "PublicData.h"
#include "SpindlePublicAccess.h"
#include "StreamOutput.h"
#include "SwitchPublicAccess.h"
#include "checksumm.h"
#include "utils.h"

namespace {
constexpr uint16_t accessory_checksum = CHECKSUM("accessory");
constexpr uint16_t chip_clear_switch_checksum = CHECKSUM("chip_clear_switch");
constexpr uint16_t chip_clear_power_checksum = CHECKSUM("chip_clear_power");
constexpr uint16_t auto_blowing_switch_checksum = CHECKSUM("auto_blowing_switch");
constexpr uint16_t auto_blowing_power_checksum = CHECKSUM("auto_blowing_power");
constexpr uint16_t extendout_switch_checksum = CHECKSUM("extendout");
}  // namespace

void SpindleAccessories::on_module_loaded() {
  chip_clear_switch_ = read_switch_name(chip_clear_switch_checksum, "vacuum");
  auto_blowing_switch_ = read_switch_name(auto_blowing_switch_checksum, "nc");
  chip_clear_power_ = THEKERNEL->config->value(accessory_checksum, chip_clear_power_checksum)->as_number(-1.0F);
  auto_blowing_power_ = THEKERNEL->config->value(accessory_checksum, auto_blowing_power_checksum)->as_number(30.0F);

  register_for_event(ON_GCODE_RECEIVED);
}

uint16_t SpindleAccessories::read_switch_name(uint16_t setting, const char* fallback) {
  const std::string name = THEKERNEL->config->value(accessory_checksum, setting)->as_string(fallback);
  return name.empty() || name == "nc" ? 0 : get_checksum(name);
}

bool SpindleAccessories::spindle_is_running() {
  spindle_status status{};
  return PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &status) && status.state;
}

bool SpindleAccessories::set_state(uint16_t name, bool state) const {
  return name != 0 && PublicData::set_value(switch_checksum, name, state_checksum, &state);
}

bool SpindleAccessories::set_power(uint16_t name, float power) const {
  if (name == 0) return false;
  pad_switch value{};
  value.state = true;
  value.value = power;
  return PublicData::set_value(switch_checksum, name, state_value_checksum, &value);
}

bool SpindleAccessories::cleaning_owns(uint16_t name) const {
  return THEKERNEL->bed_cleaning != nullptr && THEKERNEL->bed_cleaning->owns_switch(name);
}

void SpindleAccessories::apply_chip_clear_power() const {
  if (chip_clear_power_ >= 0) set_power(chip_clear_switch_, chip_clear_power_);
}

void SpindleAccessories::spindle_started() {
  if (chip_clear_enabled_ && !cleaning_owns(chip_clear_switch_)) {
    set_state(chip_clear_switch_, true);
    apply_chip_clear_power();
  }
  if (auto_blowing_enabled_) {
    if (!cleaning_owns(auto_blowing_switch_)) set_power(auto_blowing_switch_, auto_blowing_power_);
    auto_blowing_active_ = true;
  }
  if (extendout_enabled_ && !cleaning_owns(extendout_switch_checksum)) set_state(extendout_switch_checksum, true);
}

void SpindleAccessories::spindle_stopped() {
  if (chip_clear_enabled_ && !cleaning_owns(chip_clear_switch_)) set_state(chip_clear_switch_, false);
  if (auto_blowing_active_) {
    if (!cleaning_owns(auto_blowing_switch_)) set_power(auto_blowing_switch_, 0);
    auto_blowing_active_ = false;
  }
  if (extendout_enabled_ && !cleaning_owns(extendout_switch_checksum)) set_state(extendout_switch_checksum, false);
}

bool SpindleAccessories::requested_fan_power(uint16_t switch_name, float& power) const {
  if (!auto_blowing_active_ || switch_name != auto_blowing_switch_) return false;
  power = auto_blowing_power_;
  return true;
}

bool SpindleAccessories::should_enable_switch(uint16_t switch_name) const {
  if (!spindle_is_running()) return false;
  return (chip_clear_enabled_ && switch_name == chip_clear_switch_) ||
         (extendout_enabled_ && switch_name == extendout_switch_checksum);
}

void SpindleAccessories::handle_mode_command(Gcode& gcode, bool enabled) {
  switch (gcode.subcode) {
    case 0:
      chip_clear_enabled_ = enabled;
      if (spindle_is_running()) {
        set_state(chip_clear_switch_, enabled);
        if (enabled) apply_chip_clear_power();
      }
      gcode.stream->printf("turning chip clearing mode %s\r\n", enabled ? "on" : "off");
      break;
    case 1:
      if (enabled) {
        float power = auto_blowing_power_;
        if (gcode.has_letter('S')) power = gcode.get_value('S');
        if (power < 0 || power > 100) {
          gcode.stream->printf("ERROR: auto blowing power must be between 0 and 100\r\n");
          return;
        }
        auto_blowing_power_ = power;
      }
      auto_blowing_enabled_ = enabled;
      gcode.stream->printf("turning auto blowing mode %s\r\n", enabled ? "on" : "off");
      break;
    case 3:
      extendout_enabled_ = enabled;
      if (spindle_is_running()) set_state(extendout_switch_checksum, enabled);
      gcode.stream->printf("turning extend out mode %s\r\n", enabled ? "on" : "off");
      break;
    default:
      break;
  }
}

void SpindleAccessories::on_gcode_received(void* argument) {
  Gcode& gcode = *static_cast<Gcode*>(argument);
  if (gcode.has_m && (gcode.m == 331 || gcode.m == 332)) handle_mode_command(gcode, gcode.m == 331);
}
