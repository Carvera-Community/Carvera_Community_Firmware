/*
 * Tonghang spindle over Modbus RTU on a USB CDC adapter.
 *
 * Speed and direction. The drive jogs at parameter P-076.
 */

#ifndef TONGHANG_SPINDLE_CONTROL_H
#define TONGHANG_SPINDLE_CONTROL_H

#include <cstdint>

#include "SpindleControl.h"
#include "UsbModbusRTU.h"

class TonghangSpindleControl : public SpindleControl {
 public:
  TonghangSpindleControl() = default;

  void on_module_loaded() override;
  void on_idle(void *argument) override;
  void on_get_public_data(void *argument) override;
  void on_set_public_data(void *argument) override;

 private:
  void turn_on() override;
  void turn_off() override;
  void set_ccw(bool ccw_gcode) override;
  void reassert_run() override;
  bool supports_direction() const override { return true; }
  void set_speed(int rpm) override;
  void report_speed() override;
  void set_factor(float f) override;
  bool supports_modbus() const override { return true; }
  bool modbus_ready() const override { return ready; }
  bool modbus_exchange(bool writing, uint16_t reg, uint16_t value, uint16_t *out) override;
  bool modbus_link_defaults(uint16_t *reg, uint16_t *words) const override;
  bool modbus_read(uint16_t reg, uint16_t words) override;
  bool modbus_wait() override;
  void modbus_lock(bool locked) override;
  uint32_t modbus_baud() const override;

  bool probe();
  void service();
  bool dispatch();
  void on_response();
  bool wait_for_bus();
  // One blocking Modbus transaction. Does not touch the run state machine.
  bool transact(bool writing, uint16_t reg, uint16_t value, uint16_t words, uint8_t *data, uint8_t data_len);
  bool read_motor_counts(int64_t *counts);
  bool read_alarm(uint16_t *code);
  void note_alarm(uint16_t code);
  uint16_t commanded_magnitude() const;
  uint16_t run_code() const;
  float reported_rpm() const;
  void raise_alarm(const char *reason);

  enum class Step : uint8_t {
    Idle,
    SpeedThenRun,
    SpeedOnly,
    Run,
    Hold,
    WaitStop,
    ReadStop,
    Release,
    ReadRpm,
    ReadTorque,
    ReadLoad,
    ReadCurrent
  };

  UsbModbusRTU bus{};

  int32_t default_rpm = 1000;
  int32_t max_rpm = 6000;
  int delay_s = 1;
  float acc_ratio = 1.0f;
  int overload_threshold = 90;
  // Drive position is 32 bits. unwrapped_counts adds the short step from the last sample.
  int64_t unwrapped_counts = 0;
  uint32_t last_raw_counts = 0;
  bool have_unwrapped_counts = false;
  // True selects jog code 0x2222 (M4). False selects 0x1111 (M3).
  bool ccw = false;

  int32_t target_rpm = 0;
  int16_t current_rpm = 0;
  int16_t torque_pct = 0;
  int16_t load_pct = 0;
  int16_t current_raw = 0;  // 0.01 A per count
  uint16_t alarm_code = 0;
  float factor = 100.0f;

  Step step = Step::Idle;
  bool run_pending = false;
  bool stop_pending = false;
  bool speed_pending = false;
  uint32_t stop_wait_start_us = 0;
  uint32_t last_poll_us = 0;

  bool failed_init = false;
  // USB enumeration, then one Modbus baud probe per idle tick.
  enum class LinkSetup : uint8_t { Wait, Enumerate, Probe, Failed };
  LinkSetup link_setup = LinkSetup::Wait;
  uint8_t baud_index = 0;
  bool detect_baud = false;
  uint32_t link_baud = 19200;
  bool ready = false;
  bool comm_halted = false;
  uint8_t link_failures = 0;
  // While set, on_idle does not start a telemetry read.
  bool bus_locked = false;
};

#endif
