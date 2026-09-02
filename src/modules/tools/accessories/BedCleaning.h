#pragma once

#include <cstdint>

#include "Module.h"

class Gcode;

class BedCleaning : public Module {
 public:
  void on_module_loaded() override;
  void on_gcode_received(void* argument) override;
  void on_abort(void* argument) override;
  void on_halt(void* argument) override;

  void job_completed();
  bool automatic_cleaning_enabled() const { return automatic_cleaning_enabled_; }
  bool owns_switch(uint16_t name) const;
  bool requested_fan_power(uint16_t switch_name, float& power) const;

 private:
  static uint16_t read_switch_name(uint16_t setting, const char* fallback);

  bool set_state(uint16_t name, bool state) const;
  bool set_power(uint16_t name, float power) const;
  bool player_is_busy() const;
  void set_player_busy(bool busy) const;
  void set_outputs(bool enabled);
  void start(Gcode& gcode);
  bool move(float x, float y);
  bool clean(unsigned int cycles, float spacing);
  void finish();

  uint16_t brush_switch_ = 0;
  uint16_t fan_switch_ = 0;
  float fan_power_ = 100;
  float x_min_ = 0;
  float x_max_ = 0;
  float y_min_ = 0;
  float y_max_ = 0;
  float park_y_ = 0;
  float front_x_ = 0;
  float spacing_ = 0;
  float cycles_ = 0;
  bool supported_ = false;
  bool automatic_cleaning_enabled_ = false;
  bool active_ = false;
  bool cancelled_ = false;
};
