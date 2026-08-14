#pragma once

#include <cstdint>

#include "Module.h"

class Gcode;

class SpindleAccessories : public Module {
 public:
  void on_module_loaded() override;
  void on_gcode_received(void* argument) override;

  void spindle_started();
  void spindle_stopped();

  bool chip_clear_enabled() const { return chip_clear_enabled_; }
  bool auto_blowing_enabled() const { return auto_blowing_enabled_; }
  bool extendout_enabled() const { return extendout_enabled_; }
  bool should_enable_switch(uint16_t switch_name) const;
  bool requested_fan_power(uint16_t switch_name, float& power) const;

 private:
  static uint16_t read_switch_name(uint16_t setting, const char* fallback);
  static bool spindle_is_running();

  bool set_state(uint16_t name, bool state) const;
  bool set_power(uint16_t name, float power) const;
  bool cleaning_owns(uint16_t name) const;
  void apply_chip_clear_power() const;
  void handle_mode_command(Gcode& gcode, bool enabled);

  uint16_t chip_clear_switch_ = 0;
  uint16_t auto_blowing_switch_ = 0;
  float chip_clear_power_ = -1;
  float auto_blowing_power_ = 30;
  bool chip_clear_enabled_ = false;
  bool auto_blowing_enabled_ = false;
  bool auto_blowing_active_ = false;
  bool extendout_enabled_ = false;
};
