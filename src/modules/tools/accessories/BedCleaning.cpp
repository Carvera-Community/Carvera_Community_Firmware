#include "BedCleaning.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "Config.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "Kernel.h"
#include "PlayerPublicAccess.h"
#include "PublicData.h"
#include "Robot.h"
#include "SerialMessage.h"
#include "SpindleAccessories.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"
#include "SwitchPublicAccess.h"
#include "checksumm.h"
#include "utils.h"

namespace {
constexpr uint16_t accessory_checksum = CHECKSUM("accessory");
constexpr uint16_t bed_cleaning_checksum = CHECKSUM("bed_cleaning");
constexpr uint16_t bed_cleaning_switch_checksum = CHECKSUM("bed_cleaning_switch");
constexpr uint16_t bed_cleaning_fan_switch_checksum = CHECKSUM("bed_cleaning_fan_switch");
constexpr uint16_t bed_cleaning_fan_power_checksum = CHECKSUM("bed_cleaning_fan_power");
constexpr uint16_t enable_checksum = CHECKSUM("enable");
constexpr uint16_t cycles_checksum = CHECKSUM("cycles");
constexpr uint16_t spacing_checksum = CHECKSUM("spacing");
constexpr uint16_t x_min_checksum = CHECKSUM("x_min");
constexpr uint16_t x_max_checksum = CHECKSUM("x_max");
constexpr uint16_t y_min_checksum = CHECKSUM("y_min");
constexpr uint16_t y_max_checksum = CHECKSUM("y_max");
constexpr uint16_t park_y_checksum = CHECKSUM("park_y");
constexpr uint16_t front_x_checksum = CHECKSUM("front_x");
constexpr uint16_t coordinate_checksum = CHECKSUM("coordinate");
constexpr uint16_t anchor1_x_checksum = CHECKSUM("anchor1_x");
constexpr uint16_t anchor1_y_checksum = CHECKSUM("anchor1_y");
constexpr uint16_t clearance_x_checksum = CHECKSUM("clearance_x");
constexpr uint16_t clearance_y_checksum = CHECKSUM("clearance_y");
}  // namespace

void BedCleaning::on_module_loaded() {
  brush_switch_ = read_switch_name(bed_cleaning_switch_checksum, "nc");
  fan_switch_ = read_switch_name(bed_cleaning_fan_switch_checksum, "nc");
  fan_power_ = THEKERNEL->config->value(accessory_checksum, bed_cleaning_fan_power_checksum)->as_number(100.0F);
  supported_ = THEKERNEL->config->value(bed_cleaning_checksum, enable_checksum)->as_bool(false);

  const float anchor1_x = THEKERNEL->config->value(coordinate_checksum, anchor1_x_checksum)->as_number(-359.0F);
  const float anchor1_y = THEKERNEL->config->value(coordinate_checksum, anchor1_y_checksum)->as_number(-234.0F);
  const float clearance_x = THEKERNEL->config->value(coordinate_checksum, clearance_x_checksum)->as_number(-5.0F);
  const float clearance_y = THEKERNEL->config->value(coordinate_checksum, clearance_y_checksum)->as_number(-21.0F);
  cycles_ = THEKERNEL->config->value(bed_cleaning_checksum, cycles_checksum)->as_number(1.0F);
  spacing_ = THEKERNEL->config->value(bed_cleaning_checksum, spacing_checksum)->as_number(40.0F);
  x_min_ = THEKERNEL->config->value(bed_cleaning_checksum, x_min_checksum)->as_number(anchor1_x);
  x_max_ = THEKERNEL->config->value(bed_cleaning_checksum, x_max_checksum)->as_number(clearance_x);
  y_min_ = THEKERNEL->config->value(bed_cleaning_checksum, y_min_checksum)->as_number(anchor1_y);
  y_max_ = THEKERNEL->config->value(bed_cleaning_checksum, y_max_checksum)->as_number(-1.0F);
  park_y_ = THEKERNEL->config->value(bed_cleaning_checksum, park_y_checksum)->as_number(clearance_y);
  front_x_ = THEKERNEL->config->value(bed_cleaning_checksum, front_x_checksum)->as_number(-2.0F);

  register_for_event(ON_GCODE_RECEIVED);
  register_for_event(ON_ABORT);
  register_for_event(ON_HALT);
}

uint16_t BedCleaning::read_switch_name(uint16_t setting, const char* fallback) {
  const std::string name = THEKERNEL->config->value(accessory_checksum, setting)->as_string(fallback);
  return name.empty() || name == "nc" ? 0 : get_checksum(name);
}

bool BedCleaning::set_state(uint16_t name, bool state) const {
  return name != 0 && PublicData::set_value(switch_checksum, name, state_checksum, &state);
}

bool BedCleaning::set_power(uint16_t name, float power) const {
  if (name == 0) return false;
  pad_switch value{};
  value.state = true;
  value.value = power;
  return PublicData::set_value(switch_checksum, name, state_value_checksum, &value);
}

bool BedCleaning::owns_switch(uint16_t name) const {
  return active_ && name != 0 && (name == brush_switch_ || name == fan_switch_);
}

bool BedCleaning::requested_fan_power(uint16_t switch_name, float& power) const {
  if (!active_ || switch_name != fan_switch_) return false;
  power = fan_power_;
  return true;
}

bool BedCleaning::player_is_busy() const {
  void* value = nullptr;
  return PublicData::get_value(player_checksum, inner_playing_checksum, &value) && *static_cast<bool*>(value);
}

void BedCleaning::set_player_busy(bool busy) const {
  PublicData::set_value(player_checksum, inner_playing_checksum, &busy);
}

void BedCleaning::set_outputs(bool enabled) {
  if (enabled) {
    set_state(brush_switch_, true);
    set_power(fan_switch_, fan_power_);
    return;
  }

  const bool brush_needed =
      THEKERNEL->spindle_accessories != nullptr && THEKERNEL->spindle_accessories->should_enable_switch(brush_switch_);
  set_state(brush_switch_, brush_needed);

  float power = 0;
  if (THEKERNEL->spindle_accessories != nullptr &&
      THEKERNEL->spindle_accessories->requested_fan_power(fan_switch_, power)) {
    set_power(fan_switch_, power);
  } else {
    set_power(fan_switch_, 0);
  }
}

void BedCleaning::start(Gcode& gcode) {
  if (active_ || player_is_busy()) {
    gcode.stream->printf("ERROR: another operation is already active\n");
    return;
  }
  if (!supported_) {
    gcode.stream->printf("ERROR: bed cleaning is not configured\n");
    return;
  }
  if (!THEROBOT->is_homed_all_axes()) {
    gcode.stream->printf("ERROR: home the machine before bed cleaning\n");
    return;
  }
  if (THEKERNEL->get_laser_mode()) {
    gcode.stream->printf("ERROR: bed cleaning is unavailable in laser mode\n");
    return;
  }

  const float requested_cycles = gcode.has_letter('N') ? gcode.get_value('N') : cycles_;
  const float requested_spacing = gcode.has_letter('T') ? gcode.get_value('T') : spacing_;
  const bool valid_cycles = std::isfinite(requested_cycles) && requested_cycles >= 1.0F && requested_cycles <= 10.0F &&
                            std::floor(requested_cycles) == requested_cycles;
  const bool valid_area = std::isfinite(x_min_) && std::isfinite(x_max_) && std::isfinite(y_min_) &&
                          std::isfinite(y_max_) && std::isfinite(park_y_) && std::isfinite(front_x_) &&
                          x_min_ < x_max_ && y_min_ < y_max_ && front_x_ >= x_max_;
  const float sweeps = valid_area && std::isfinite(requested_spacing) && requested_spacing > 0
                           ? std::ceil((y_max_ - y_min_) / requested_spacing)
                           : 1000.0F;
  const float front_steps = valid_area ? std::ceil((front_x_ - x_max_) / 0.1F) : 1000.0F;
  const float path_moves = 4.0F + front_steps + requested_cycles * (2.0F + 2.0F * sweeps);
  if (!valid_cycles || !valid_area || !std::isfinite(requested_spacing) || requested_spacing <= 0 ||
      path_moves > 256.0F) {
    gcode.stream->printf("ERROR: invalid or excessively dense bed cleaning path\n");
    return;
  }

  THEROBOT->push_state();
  active_ = true;
  cancelled_ = false;
  set_player_busy(true);
  set_state(brush_switch_, true);
  const bool completed = clean(static_cast<unsigned int>(requested_cycles), requested_spacing);
  finish();
  if (completed) THEKERNEL->streams->printf("Done bed cleaning\r\n");
}

bool BedCleaning::move(float x, float y) {
  if (cancelled_ || THEKERNEL->is_halted()) return false;

  char command[64];
  int length = std::snprintf(command, sizeof(command), "G53 G0");
  if (!std::isnan(x))
    length += std::snprintf(command + length, sizeof(command) - length, " X%1.3f", THEROBOT->from_millimeters(x));
  if (!std::isnan(y))
    std::snprintf(command + length, sizeof(command) - length, " Y%1.3f", THEROBOT->from_millimeters(y));

  SerialMessage message{&StreamOutput::NullStream, command, 0};
  THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
  THECONVEYOR->wait_for_idle();
  return !cancelled_ && !THEKERNEL->is_halted();
}

bool BedCleaning::clean(unsigned int cycles, float spacing) {
  // Move from the parking point to the first row, creep X out to the front
  // edge and return, then sweep between the X limits while advancing Y by
  // the configured spacing. Each cycle finishes back at the parking point.
  if (!move(x_max_, park_y_) || !move(NAN, y_min_)) return false;
  set_outputs(true);
  for (float x = x_max_ + 0.1F; x <= front_x_ + 0.001F; x += 0.1F) {
    if (!move(std::min(x, front_x_), NAN)) return false;
  }
  if (!move(front_x_, NAN) || !move(x_max_, NAN)) return false;

  for (unsigned int cycle = 0; cycle < cycles; ++cycle) {
    float y = y_min_;
    if (!move(x_max_, y)) return false;
    while (y < y_max_) {
      if (!move(x_min_, NAN)) return false;
      y = std::min(y + spacing, y_max_);
      if (!move(NAN, y)) return false;
      if (y >= y_max_) break;
      if (!move(x_max_, NAN)) return false;
      y = std::min(y + spacing, y_max_);
      if (!move(NAN, y)) return false;
    }
    if (!move(x_max_, park_y_)) return false;
  }
  return true;
}

void BedCleaning::finish() {
  if (!active_) return;
  set_outputs(false);
  set_player_busy(false);
  THEROBOT->pop_state();
  active_ = false;
  cancelled_ = false;
}

void BedCleaning::on_gcode_received(void* argument) {
  Gcode& gcode = *static_cast<Gcode*>(argument);
  if (!gcode.has_m) return;
  if ((gcode.m == 331 || gcode.m == 332) && gcode.subcode == 2) {
    if (!supported_) {
      gcode.stream->printf("ERROR: bed cleaning is not configured\r\n");
      return;
    }
    automatic_cleaning_enabled_ = gcode.m == 331;
    gcode.stream->printf("turning auto bed cleaning mode %s\r\n", automatic_cleaning_enabled_ ? "on" : "off");
  } else if (gcode.m == 486 && gcode.subcode == 1) {
    start(gcode);
  }
}

void BedCleaning::on_abort(void*) { cancelled_ = true; }

void BedCleaning::on_halt(void* argument) {
  if (argument == nullptr) cancelled_ = true;
}

void BedCleaning::job_completed() {
  if (!automatic_cleaning_enabled_) return;
  SerialMessage message{THEKERNEL->streams, "M486.1", 0};
  THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
}
