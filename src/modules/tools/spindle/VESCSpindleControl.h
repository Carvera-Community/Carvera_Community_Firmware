/*
 * VESC spindle control over USB CDC.
 *
 * Targets VESC FW 6.x packet protocol. All runtime USB transfers are
 * non-blocking so stepper timing is never affected.
 */

#ifndef VESC_SPINDLE_CONTROL_H
#define VESC_SPINDLE_CONTROL_H

#include <cstdint>

#include "SpindleControl.h"
#include "USBHostCDC.h"

// COMM_PACKET_ID values we use (VESC FW 6.x).
enum VESCCommPacketId {
  COMM_FW_VERSION = 0,
  COMM_GET_VALUES = 4,
  COMM_SET_RPM = 8,
  COMM_ALIVE = 30
};

enum VESCCommState {
  VESC_COMM_IDLE = 0,
  VESC_COMM_SEND_PENDING,
  VESC_COMM_SEND_WAIT,
  VESC_COMM_RECV_WAIT,
  VESC_COMM_PROCESS
};

enum VESCRxPacketState { VESC_RX_INCOMPLETE = 0, VESC_RX_COMPLETE, VESC_RX_INVALID };

class VESCSpindleControl : public SpindleControl {
 public:
  VESCSpindleControl() = default;

  void on_module_loaded() override;
  void on_idle(void *argument) override;
  void on_get_public_data(void *argument) override;
  void on_set_public_data(void *argument) override;

 private:
  void turn_on() override;
  void turn_off() override;
  void set_speed(int rpm) override;
  void report_speed() override;
  void set_p_term(float) override {}
  void set_i_term(float) override {}
  void set_d_term(float) override {}
  void report_settings() override {}
  void set_factor(float f) override;

  static uint16_t vesc_crc16(const uint8_t *buf, uint16_t len);
  static uint16_t vesc_build_packet(uint8_t *out, const uint8_t *payload, uint16_t plen);
  static bool extract_vesc_payload(const uint8_t *frame, uint16_t frame_len, const uint8_t **payload,
                                   uint16_t *payload_len);
  void reset_rx_packet();
  VESCRxPacketState append_rx_packet_bytes(const volatile uint8_t *buf, uint32_t len);
  void queue_set_rpm(int32_t erpm);
  void queue_get_values();
  void queue_alive();
  bool parse_get_values(const uint8_t *buf, uint16_t len);

  void comm_poll();
  void handle_usb_disconnected();
  void schedule_next_vesc_command(uint32_t now_us);
  void start_pending_send();
  void handle_send_wait(CDCXferState xfer_state);
  void handle_recv_wait(CDCXferState xfer_state);
  void process_pending_response();

  // CDC enumeration only proves that the USB serial endpoint is present. This
  // handshake proves the VESC packet protocol is answering before any spindle
  // command is accepted.
  bool verify_vesc_protocol();
  bool wait_for_usb_transfer(CDCXferState active_state);
  void handle_comm_error(const char *reason);
  void dwell_after_spindle_change();
  int32_t commanded_rpm() const;
  void reset_stall_monitor();
  bool vesc_timeout_status_is_stale() const;

  static void raise_spindle_alarm(const char *reason);

  USBHostCDC usb{};

  int32_t pole_pairs = 2;
  int32_t default_rpm = 10000;
  int32_t max_rpm = 15000;
  int32_t min_rpm = 3000;
  int delay_s = 3;
  uint32_t poll_interval_ms = 200;
  int32_t stall_rpm = 2000;
  uint32_t stall_us = 1000000;
  int32_t stall_current = 1500;  // same A * 100 units as VESC telemetry
  bool ignore_on_halt = false;

  // vesc_* fields hold the raw fixed-point values from the wire.
  int32_t current_rpm = 0;  // mechanical RPM
  int32_t target_rpm = 0;
  float factor = 100.0f;      // M223 override, percent
  int16_t vesc_voltage = 0;   // V * 10
  int32_t vesc_current = 0;   // A * 100
  int16_t vesc_temp_mos = 0;  // deg C * 10
  uint8_t vesc_fault_code = 0;
  uint8_t vesc_status = 0;

  uint32_t stall_timer = 0;
  uint32_t stall_monitor_start = 0;
  uint32_t stall_monitor_seq = 0;
  bool stall_speed_seen = false;

  VESCCommState comm_state = VESC_COMM_IDLE;
  uint8_t pkt_buf[64]{};
  uint16_t pkt_len = 0;
  uint8_t rx_pkt_buf[128]{};
  uint16_t rx_pkt_len = 0;
  uint32_t rx_start_time = 0;
  uint32_t last_poll_time = 0;
  uint32_t last_alive_time = 0;
  uint32_t last_timeout_reset_cmd_time = 0;
  uint8_t pending_cmd = 0;
  bool speed_update_pending = false;
  uint32_t telemetry_seq = 0;

  // Init can fail before the normal status path is ready, so the alarm is
  // deferred to on_idle. Communication alarms are latched only until the
  // operator clears the halt and tries to start the spindle again.
  bool failed_init = false;
  bool init_pending = false;
  bool comm_error_halted = false;
  bool protocol_ok = false;
};

#endif  // VESC_SPINDLE_CONTROL_H
