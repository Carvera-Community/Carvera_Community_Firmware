/*
 * VESC spindle control over USB CDC (FW 6.x packet protocol).
 *
 * The setup guide below assumes one USB-connected Flipsky Mini FSESC6.7 Pro
 * driving a 48 V / 500 W / 12k RPM, 4-pole BLDC spindle with hall sensors,
 * powered from a regulated 48 V DC PSU. This motor has 4 poles (2 pole pairs).
 * At 12000 spindle RPM the VESC sees 24000 eRPM.
 *
 * VESC Tool 6.x setup guide:
 * 1. Connect the motor to the VESC and connect the VESC to the computer
 *    over USB. Power it up from the 48 V power supply.
 * 2. Open VESC Tool and click AutoConnect.
 * 3. Press "Setup Motors FOC". Press "Yes" to load defaults. Select a generic
 *    motor and press "Next". Select "Medium Inrunner". Click
 *    "Override (Advanced)" and set "Motor Poles" to 4. Click "Next",
 *    then press "Yes". Click "Advanced", set "Battery Current Regen" to -2 A,
 *    "Battery Current Max" to 12 A. Click "Next". Click "Direct Drive", set
 *    the temperature sensor type to "Disabled".
 * 4. Make sure the motor is free to spin. Click "Run Detection", clear
 *    "Detect all motors" checkbox and press "OK". The motor will make strange
 *    noises, then spin, then turn slowly. This is normal. Press "OK".
 * 5. Press "FWD" and observe the direction of rotation. If it is wrong,
 *    enable "Inverted". Press "Finish".
 * 6. Set the following values:
 *
 *        Motor Settings -> General -> Current:
 *          Motor Current Max:         20 A
 *          Motor Current Max Brake:   -2 A
 *          Absolute Maximum Current:  25 A
 *          Battery Current Max:       12 A
 *          Battery Current Max Regen: -2 A
 *
 *        Motor Settings -> General -> RPM:
 *          Max ERPM: 26000
 *          Max ERPM Reverse: -26000
 *
 *        Motor Settings -> General -> Voltage:
 *          Battery Voltage Cutoff Start: 10 V
 *          Battery Voltage Cutoff End:   8 V
 *
 *        Motor Settings -> General -> Advanced:
 *          Minimum Input Voltage: 40 V
 *          Maximum Input Voltage: 57 V
 *
 *        Motor Settings -> Additional Info -> Setup:
 *          Motor Poles: 4
 *
 * 7. Check that the "Motor Settings -> FOC -> General -> Sensor Mode" is
 *    set to "Hall Sensors", and that the "Hall Table" on the "Hall Sensors"
 *    tab has six entries.
 *
 * 8. Click "Write Motor Configuration" (down arrow next to "M" on the right)
 *    to save these settings to the VESC.
 *
 * 9. Add the following to your config.txt (tweak as needed):
 *
 *   spindle.type                 vesc
 *   usb_msc.enable               false
 *   spindle.default_rpm          10000
 *   spindle.min_rpm              3000
 *   spindle.max_rpm              15000
 *   spindle.delay_s              3
 *   spindle.ignore_on_halt       false
 *   spindle.stall_s              1
 *   spindle.stall_alarm_rpm      2000
 *   spindle.vesc_pole_pairs      2
 *   spindle.vesc_poll_ms         200
 *   spindle.vesc_stall_current_a 15
 */

#include "VESCSpindleControl.h"
#include "libs/CRC16.h"

#include <cstdio>
#include <cstdlib>
#include <string>

#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "Pin.h"
#include "PublicDataRequest.h"
#include "SpindlePublicAccess.h"
#include "StreamOutputPool.h"
#include "USBHostCDC.h"
#include "checksumm.h"
#include "libs/Kernel.h"
#include "us_ticker_api.h"

constexpr uint16_t spindle_checksum = CHECKSUM("spindle");

namespace {
// The VESC packet payload is byte-packed and not guaranteed to be aligned, so
// copy through a local integer before swapping byte order.
uint16_t be16(const uint8_t *p) {
  uint16_t v;
  __builtin_memcpy(&v, p, sizeof v);
  return __builtin_bswap16(v);
}
uint32_t be32(const uint8_t *p) {
  uint32_t v;
  __builtin_memcpy(&v, p, sizeof v);
  return __builtin_bswap32(v);
}
void wbe16(uint8_t *p, const uint16_t v) {
  const uint16_t s = __builtin_bswap16(v);
  __builtin_memcpy(p, &s, sizeof s);
}
void wbe32(uint8_t *p, const uint32_t v) {
  const uint32_t s = __builtin_bswap32(v);
  __builtin_memcpy(p, &s, sizeof s);
}

bool elapsed_us(const uint32_t start_us, const uint32_t interval_us) {
  return static_cast<uint32_t>(us_ticker_read() - start_us) >= interval_us;
}

// The fields used by this driver are at the front of the VESC 6.x
// COMM_GET_VALUES response. Newer firmware can append more telemetry; that is
// fine as long as this prefix does not move.
struct __attribute__((packed)) VescGetValuesPayload {
  uint8_t cmd;                 // COMM_GET_VALUES
  uint8_t temp_mos[2];         // int16, scale 10  (deg C)
  uint8_t _temp_motor[2];      // int16, scale 10  (deg C)
  uint8_t current_motor[4];    // int32, scale 100 (A)
  uint8_t _current_input[4];   // int32, scale 100 (A)
  uint8_t _id[4];              // int32, scale 100 (A)
  uint8_t _iq[4];              // int32, scale 100 (A)
  uint8_t _duty[2];            // int16, scale 1000
  uint8_t rpm[4];              // int32, scale 1   (electrical RPM)
  uint8_t v_in[2];             // int16, scale 10  (V)
  uint8_t _ah[4];              // int32, scale 1e4
  uint8_t _ah_charged[4];      // int32, scale 1e4
  uint8_t _wh[4];              // int32, scale 1e4
  uint8_t _wh_charged[4];      // int32, scale 1e4
  uint8_t _tachometer[4];      // int32
  uint8_t _tachometer_abs[4];  // int32
  uint8_t fault_code;          // int8
};
static_assert(sizeof(VescGetValuesPayload) == 54, "VESC COMM_GET_VALUES payload layout drift");

// VESC FW 6.x appends this status byte after the fixed prefix above when
// COMM_GET_VALUES uses its default full mask. Bit 0 is the motor command
// timeout reported by timeout_has_timeout().
constexpr uint8_t VESC_STATUS_TIMEOUT = 0x01;
constexpr uint8_t VESC_GET_VALUES_STATUS_OFFSET = 73;
constexpr uint32_t VESC_HANDSHAKE_TIMEOUT_US = 2000000UL;
constexpr uint32_t VESC_RESPONSE_TIMEOUT_US = 400000UL;
constexpr uint32_t VESC_ALIVE_INTERVAL_MS = 200;
constexpr uint32_t VESC_IDLE_POLL_INTERVAL_MS = 1000;
constexpr uint32_t VESC_TIMEOUT_STATUS_GRACE_US = 100000UL;
}  // namespace

void VESCSpindleControl::raise_spindle_alarm(const char *reason) {
  THEKERNEL->streams->printf("ERROR: %s\n", reason);
  THEKERNEL->set_halt_reason(SPINDLE_ALARM);
  THEKERNEL->call_event(ON_HALT, nullptr);
}

void VESCSpindleControl::handle_comm_error(const char *reason) {
  if (!comm_error_halted || !THEKERNEL->is_halted()) {
    comm_error_halted = true;
    raise_spindle_alarm(reason);
  }
}

void VESCSpindleControl::dwell_after_spindle_change() {
  if (delay_s <= 0) return;

  char buf[80];
  size_t n = snprintf(buf, sizeof(buf), "G4P%d", delay_s);
  if (n > sizeof(buf)) n = sizeof(buf);
  std::string g(buf, n);
  Gcode gcode(g, &(StreamOutput::NullStream));
  THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);
}

int32_t VESCSpindleControl::commanded_rpm() const {
  return static_cast<int32_t>(static_cast<float>(target_rpm) * factor / 100.0f);
}

void VESCSpindleControl::reset_stall_monitor() {
  stall_timer = 0;
  stall_speed_seen = false;
  stall_monitor_start = us_ticker_read();
  stall_monitor_seq = telemetry_seq;
}

bool VESCSpindleControl::vesc_timeout_status_is_stale() const {
  if ((vesc_status & VESC_STATUS_TIMEOUT) == 0) return false;

  // VESC timeout_reset() updates the command timestamp immediately, but the
  // reported timeout flag is cleared by VESC's timeout thread on its next pass.
  return !elapsed_us(last_timeout_reset_cmd_time, VESC_TIMEOUT_STATUS_GRACE_US);
}

void VESCSpindleControl::on_module_loaded() {
  protocol_ok = false;
  default_rpm = THEKERNEL->config->value(spindle_checksum, CHECKSUM("default_rpm"))->as_int(10000);
  pole_pairs = THEKERNEL->config->value(spindle_checksum, CHECKSUM("vesc_pole_pairs"))->as_int(2);
  max_rpm = THEKERNEL->config->value(spindle_checksum, CHECKSUM("max_rpm"))->as_int(15000);
  min_rpm = THEKERNEL->config->value(spindle_checksum, CHECKSUM("min_rpm"))->as_int(3000);
  delay_s = THEKERNEL->config->value(spindle_checksum, CHECKSUM("delay_s"))->as_int(3);
  poll_interval_ms = static_cast<uint32_t>(THEKERNEL->config->value(spindle_checksum, CHECKSUM("vesc_poll_ms"))->as_int(200));
  const float configured_stall_s = THEKERNEL->config->value(spindle_checksum, CHECKSUM("stall_s"))->as_number(1.0f);
  stall_us = configured_stall_s > 0.0f ? static_cast<uint32_t>(configured_stall_s * 1000000.0f) : 0;
  stall_rpm = THEKERNEL->config->value(spindle_checksum, CHECKSUM("stall_alarm_rpm"))->as_int(2000);
  stall_current = THEKERNEL->config->value(spindle_checksum, CHECKSUM("vesc_stall_current_a"))->as_int(15) * 100;
  ignore_on_halt = THEKERNEL->config->value(spindle_checksum, CHECKSUM("ignore_on_halt"))->as_bool(false);

  spindle_on = false;
  target_rpm = default_rpm;

  // The mass-storage module and this CDC driver both assume they own the one
  // USB host controller. Do not try to arbitrate that at runtime; make the
  // configuration error explicit and leave the machine halted.
  if (THEKERNEL->config->value(CHECKSUM("usb_msc"), CHECKSUM("enable"))->as_bool(true)) {
    THEKERNEL->streams->printf("ERROR: VESC spindle requires usb_msc.enable false\n");
    failed_init = true;
    return;
  }

  // Power the external USB port before enumeration. On Carvera this enable pin
  // is active-low.
  Pin usb_en;
  usb_en.from_string(THEKERNEL->config->value(CHECKSUM("usb_en_pin"))->as_string("1.19"));
  usb_en.as_output();
  usb_en.set(false);
  init_pending = true;
}

bool VESCSpindleControl::wait_for_usb_transfer(const CDCXferState active_state) {
  while (usb.state() == active_state) {
    usb.poll();
  }
  return usb.state() == CDC_XFER_COMPLETE;
}

void VESCSpindleControl::reset_rx_packet() { rx_pkt_len = 0; }

VESCRxPacketState VESCSpindleControl::append_rx_packet_bytes(const volatile uint8_t *buf, const uint32_t len) {
  // CDC may split one VESC frame across reads, but this driver never expects
  // unrelated bytes in the stream. Malformed framing is a communication
  // failure, not something to resynchronise around silently.
  for (uint32_t i = 0; i < len; i++) {
    const uint8_t b = buf[i];

    if (rx_pkt_len == 0) {
      if (b != 0x02) return VESC_RX_INVALID;
      rx_pkt_buf[rx_pkt_len++] = b;
      continue;
    }

    if (rx_pkt_len >= sizeof(rx_pkt_buf)) return VESC_RX_INVALID;

    rx_pkt_buf[rx_pkt_len++] = b;

    if (rx_pkt_len >= 2) {
      const uint16_t expected_len = static_cast<uint16_t>(rx_pkt_buf[1]) + 5;
      if (rx_pkt_buf[1] < 1 || expected_len > sizeof(rx_pkt_buf)) return VESC_RX_INVALID;
      if (rx_pkt_len > expected_len) return VESC_RX_INVALID;
      if (rx_pkt_len == expected_len) {
        if (rx_pkt_buf[rx_pkt_len - 1] != 0x03) return VESC_RX_INVALID;
        return (i + 1 == len) ? VESC_RX_COMPLETE : VESC_RX_INVALID;
      }
    }
  }

  return VESC_RX_INCOMPLETE;
}

bool VESCSpindleControl::verify_vesc_protocol() {
  constexpr uint8_t payload[1] = {COMM_FW_VERSION};
  uint8_t tx[8];
  const uint16_t tx_len = vesc_build_packet(tx, payload, 1);
  const uint32_t start_us = us_ticker_read();
  reset_rx_packet();

  if (!usb.start_send(tx, tx_len)) return false;

  if (!wait_for_usb_transfer(CDC_XFER_SENDING)) {
    usb.finish();
    return false;
  }
  usb.finish();

  while (true) {
    if (static_cast<uint32_t>(us_ticker_read() - start_us) >= VESC_HANDSHAKE_TIMEOUT_US) return false;
    if (!usb.start_recv(128)) return false;
    if (!wait_for_usb_transfer(CDC_XFER_RECEIVING)) {
      usb.finish();
      return false;
    }

    const VESCRxPacketState rx_state = append_rx_packet_bytes(usb.rx_buf(), usb.rx_len());
    usb.finish();
    if (rx_state == VESC_RX_COMPLETE) break;
    if (rx_state == VESC_RX_INVALID) return false;
  }

  const uint8_t *payload_rx = nullptr;
  uint16_t payload_len = 0;
  if (!extract_vesc_payload(rx_pkt_buf, rx_pkt_len, &payload_rx, &payload_len)) return false;

  return payload_len >= 1 && payload_rx[0] == COMM_FW_VERSION;
}

void VESCSpindleControl::turn_on() {
  if (!protocol_ok) {
    raise_spindle_alarm("VESC spindle init failed");
    return;
  }

  spindle_on = true;
  THEKERNEL->spindleon = true;
  reset_stall_monitor();
  speed_update_pending = true;
  dwell_after_spindle_change();
}

void VESCSpindleControl::turn_off() {
  spindle_on = false;
  THEKERNEL->spindleon = false;
  reset_stall_monitor();
  speed_update_pending = true;
  dwell_after_spindle_change();
}

void VESCSpindleControl::set_speed(int rpm) {
  if (rpm > max_rpm) rpm = max_rpm;
  if (rpm > 0 && rpm < min_rpm) rpm = min_rpm;
  const bool changed = rpm != target_rpm;
  target_rpm = rpm;
  if (spindle_on && changed) reset_stall_monitor();
  speed_update_pending = true;
}

void VESCSpindleControl::report_speed() {
  THEKERNEL->streams->printf(
      "VESC  State: %s  Current RPM: %5ld  Target RPM: %ld  Vin: %d V/10  Imot: %ld A/100  Tmos: %d C/10\n",
      spindle_on ? "on" : "off", current_rpm, target_rpm, vesc_voltage, vesc_current, vesc_temp_mos);
}

void VESCSpindleControl::set_factor(const float f) {
  const bool changed = f != factor;
  factor = f;
  if (spindle_on && changed) reset_stall_monitor();
  // PWM applies M223 immediately; do the same instead of waiting for the next
  // explicit speed command.
  speed_update_pending = true;
}

void VESCSpindleControl::queue_set_rpm(const int32_t erpm) {
  uint8_t payload[5];
  payload[0] = COMM_SET_RPM;
  wbe32(&payload[1], static_cast<uint32_t>(erpm));
  pkt_len = vesc_build_packet(pkt_buf, payload, sizeof payload);
  pending_cmd = COMM_SET_RPM;
  last_alive_time = us_ticker_read();
  comm_state = VESC_COMM_SEND_PENDING;
}

void VESCSpindleControl::queue_get_values() {
  constexpr uint8_t payload[1] = {COMM_GET_VALUES};
  pkt_len = vesc_build_packet(pkt_buf, payload, 1);
  pending_cmd = COMM_GET_VALUES;
  reset_rx_packet();
  comm_state = VESC_COMM_SEND_PENDING;
}

void VESCSpindleControl::queue_alive() {
  constexpr uint8_t payload[1] = {COMM_ALIVE};
  pkt_len = vesc_build_packet(pkt_buf, payload, 1);
  pending_cmd = COMM_ALIVE;
  last_alive_time = us_ticker_read();
  comm_state = VESC_COMM_SEND_PENDING;
}

bool VESCSpindleControl::parse_get_values(const uint8_t *buf, const uint16_t len) {
  const uint8_t *payload_rx = nullptr;
  uint16_t payload_len = 0;
  if (!extract_vesc_payload(buf, len, &payload_rx, &payload_len)) return false;

  if (payload_len < sizeof(VescGetValuesPayload)) return false;

  const auto *gv = reinterpret_cast<const VescGetValuesPayload *>(payload_rx);
  if (gv->cmd != COMM_GET_VALUES) return false;

  vesc_temp_mos = static_cast<int16_t>(be16(gv->temp_mos));
  vesc_current = static_cast<int32_t>(be32(gv->current_motor));
  const auto erpm = static_cast<int32_t>(be32(gv->rpm));
  current_rpm = std::abs(erpm) / pole_pairs;
  vesc_voltage = static_cast<int16_t>(be16(gv->v_in));
  vesc_fault_code = gv->fault_code;
  vesc_status = payload_len > VESC_GET_VALUES_STATUS_OFFSET ? payload_rx[VESC_GET_VALUES_STATUS_OFFSET] : 0;
  telemetry_seq++;
  return true;
}

void VESCSpindleControl::handle_usb_disconnected() {
  // Once USB is gone, the VESC should stop itself on command timeout. The CNC
  // still needs an alarm so motion does not continue with an uncontrolled
  // spindle.
  if (comm_state != VESC_COMM_IDLE) {
    usb.finish();
    comm_state = VESC_COMM_IDLE;
  }
  if (spindle_on && (!comm_error_halted || !THEKERNEL->is_halted())) {
    comm_error_halted = true;
    raise_spindle_alarm("VESC USB disconnected");
  }
}

void VESCSpindleControl::schedule_next_vesc_command(const uint32_t now_us) {
  const bool halted = THEKERNEL->is_halted();
  const bool background_allowed = !halted || (spindle_on && ignore_on_halt);
  const uint32_t telemetry_interval_ms = spindle_on ? poll_interval_ms : VESC_IDLE_POLL_INTERVAL_MS;

  // A requested speed change must be delivered even while halted, because it
  // may be the RPM=0 command from turn_off(). Other traffic stops during halt
  // unless spindle.ignore_on_halt says to keep a running spindle alive.
  if (speed_update_pending) {
    speed_update_pending = false;
    int32_t erpm = commanded_rpm() * pole_pairs;
    if (!spindle_on) erpm = 0;
    queue_set_rpm(erpm);
  } else if (background_allowed) {
    if (spindle_on && (now_us - last_alive_time) / 1000 >= VESC_ALIVE_INTERVAL_MS) {
      queue_alive();
    } else if ((now_us - last_poll_time) / 1000 >= telemetry_interval_ms) {
      last_poll_time = now_us;
      queue_get_values();
    }
  }
}

void VESCSpindleControl::start_pending_send() {
  // If CDC is still busy, leave the command queued and try again on the next
  // idle tick instead of blocking here.
  if (usb.start_send(pkt_buf, pkt_len)) {
    comm_state = VESC_COMM_SEND_WAIT;
  }
}

void VESCSpindleControl::handle_send_wait(const CDCXferState xfer_state) {
  if (xfer_state == CDC_XFER_ERROR) {
    usb.finish();
    comm_state = VESC_COMM_IDLE;
    handle_comm_error("VESC USB transfer error");
    return;
  }

  if (xfer_state != CDC_XFER_COMPLETE) return;

  if (pending_cmd == COMM_SET_RPM || pending_cmd == COMM_ALIVE) {
    last_timeout_reset_cmd_time = us_ticker_read();
  }
  usb.finish();

  // Only COMM_GET_VALUES expects a response.
  if (pending_cmd != COMM_GET_VALUES) {
    comm_state = VESC_COMM_IDLE;
    return;
  }

  reset_rx_packet();
  rx_start_time = us_ticker_read();
  if (usb.start_recv(128)) {
    comm_state = VESC_COMM_RECV_WAIT;
  } else {
    comm_state = VESC_COMM_IDLE;
    handle_comm_error("VESC USB receive start failed");
  }
}

void VESCSpindleControl::handle_recv_wait(const CDCXferState xfer_state) {
  if (xfer_state == CDC_XFER_COMPLETE) {
    const VESCRxPacketState rx_state = append_rx_packet_bytes(usb.rx_buf(), usb.rx_len());
    usb.finish();
    if (rx_state == VESC_RX_COMPLETE) {
      comm_state = VESC_COMM_PROCESS;
    } else if (rx_state == VESC_RX_INVALID) {
      comm_state = VESC_COMM_IDLE;
      handle_comm_error("VESC invalid telemetry");
    } else if (!usb.start_recv(128)) {
      comm_state = VESC_COMM_IDLE;
      handle_comm_error("VESC USB receive start failed");
    }
  } else if (xfer_state == CDC_XFER_ERROR) {
    usb.finish();
    comm_state = VESC_COMM_IDLE;
    handle_comm_error("VESC USB receive error");
  } else if (static_cast<uint32_t>(us_ticker_read() - rx_start_time) >= VESC_RESPONSE_TIMEOUT_US) {
    usb.abort();
    usb.finish();
    comm_state = VESC_COMM_IDLE;
    handle_comm_error("VESC telemetry timeout");
  }
}

void VESCSpindleControl::process_pending_response() {
  if (pending_cmd == COMM_GET_VALUES && !parse_get_values(rx_pkt_buf, rx_pkt_len)) {
    comm_state = VESC_COMM_IDLE;
    handle_comm_error("VESC invalid telemetry");
    return;
  }

  comm_state = VESC_COMM_IDLE;
}

void VESCSpindleControl::comm_poll() {
  if (!usb.connected()) {
    handle_usb_disconnected();
    return;
  }

  const CDCXferState xfer_state = usb.poll();

  switch (comm_state) {
    case VESC_COMM_IDLE:
      schedule_next_vesc_command(us_ticker_read());
      break;
    case VESC_COMM_SEND_PENDING:
      start_pending_send();
      break;
    case VESC_COMM_SEND_WAIT:
      handle_send_wait(xfer_state);
      break;
    case VESC_COMM_RECV_WAIT:
      handle_recv_wait(xfer_state);
      break;
    case VESC_COMM_PROCESS:
      process_pending_response();
      break;
  }
}

void VESCSpindleControl::on_idle(void *) {
  // Init can fail before the normal status stream is ready. Raise the alarm
  // from idle so the controller can report the halt normally.
  if (failed_init) {
    failed_init = false;
    raise_spindle_alarm("VESC spindle init failed");
    return;
  }

  if (init_pending) {
    init_pending = false;

    if (!usb.init(5000)) {
      THEKERNEL->streams->printf("ERROR: VESC not detected on USB host\n");
      raise_spindle_alarm("VESC spindle init failed");
      return;
    }

    if (!verify_vesc_protocol()) {
      THEKERNEL->streams->printf("ERROR: VESC did not respond to COMM_FW_VERSION\n");
      raise_spindle_alarm("VESC spindle init failed");
      return;
    }

    protocol_ok = true;
    THEKERNEL->streams->printf("VESC spindle connected via USB\n");
  }

  // Keep the communication state machine alive during halt long enough to send
  // any queued stop command.
  comm_poll();

  if (THEKERNEL->is_halted()) return;

  if (vesc_fault_code != 0) {
    const uint8_t fault_code = vesc_fault_code;
    vesc_fault_code = 0;
    THEKERNEL->streams->printf("ERROR: VESC fault code %d - halt\n", fault_code);
    THEKERNEL->set_halt_reason(SPINDLE_ALARM);
    THEKERNEL->call_event(ON_HALT, nullptr);
    return;
  }

  // Without an active spindle command, VESC's command-timeout bit only means
  // the idle keepalive window expired. Do not carry that status into the next
  // start command.
  if (!spindle_on) {
    vesc_status &= ~VESC_STATUS_TIMEOUT;
  } else if (vesc_timeout_status_is_stale()) {
    vesc_status &= ~VESC_STATUS_TIMEOUT;
  }

  if (vesc_status & VESC_STATUS_TIMEOUT) {
    vesc_status = 0;
    THEKERNEL->streams->printf("ERROR: VESC motor timeout - halt\n");
    THEKERNEL->set_halt_reason(SPINDLE_ALARM);
    THEKERNEL->call_event(ON_HALT, nullptr);
    return;
  }

  const uint32_t now_us = us_ticker_read();
  const uint32_t grace_us = delay_s > 0 ? static_cast<uint32_t>(delay_s) * 1000000UL : 0;
  const bool monitor_has_sample = telemetry_seq != stall_monitor_seq;
  const bool monitor_armed = stall_speed_seen || (now_us - stall_monitor_start) >= grace_us;
  const bool stall_enabled = stall_us > 0 && stall_current > 0 && stall_rpm > 0;
  const bool stall_target = std::abs(commanded_rpm()) > stall_rpm;
  const int32_t stopped_rpm = stall_rpm < 500 ? stall_rpm : 500;

  if (monitor_has_sample && std::abs(current_rpm) >= stall_rpm) stall_speed_seen = true;

  // A loaded stall has high motor current and low speed. A command timeout or
  // controller release has the opposite shape: the spindle was running, then
  // VESC telemetry drops to near zero while the command is still active.
  const bool torque_stall = std::abs(vesc_current) >= stall_current && std::abs(current_rpm) < stall_rpm;
  const bool released_after_running =
      stall_speed_seen && std::abs(current_rpm) <= stopped_rpm && std::abs(vesc_current) < stall_current;
  if (spindle_on && stall_enabled && stall_target && monitor_has_sample && (torque_stall || released_after_running)) {
    if (stall_timer == 0) {
      stall_timer = now_us;
    } else if (monitor_armed && (now_us - stall_timer) > stall_us) {
      THEKERNEL->streams->printf("ERROR: %s - halt\n",
                                 torque_stall ? "Spindle stall detected" : "Spindle stopped unexpectedly");
      THEKERNEL->set_halt_reason(SPINDLE_STALL);
      THEKERNEL->call_event(ON_HALT, nullptr);
    }
  } else {
    stall_timer = 0;
  }
}

void VESCSpindleControl::on_get_public_data(void *argument) {
  auto *pdr = static_cast<PublicDataRequest *>(argument);
  if (!pdr->starts_with(pwm_spindle_control_checksum)) return;

  if (pdr->second_element_is(get_spindle_status_checksum)) {
    const auto t = static_cast<spindle_status *>(pdr->get_data_ptr());
    t->state = this->spindle_on;
    t->current_rpm = static_cast<float>(this->current_rpm);
    t->target_rpm = static_cast<float>(this->target_rpm);
    t->current_pwm_value = 0;
    t->factor = this->factor;
    pdr->set_taken();
  }
}

void VESCSpindleControl::on_set_public_data(void *argument) {
  auto *pdr = static_cast<PublicDataRequest *>(argument);
  if (!pdr->starts_with(pwm_spindle_control_checksum)) return;

  if (pdr->second_element_is(get_spindle_status_checksum)) {
    const auto *t = static_cast<spindle_status *>(pdr->get_data_ptr());
    this->set_factor(t->factor);
    pdr->set_taken();
    return;
  }
  if (pdr->second_element_is(turn_off_spindle_checksum)) {
    this->turn_off();
    pdr->set_taken();
  }
}

// VESC short packet (payload <= 256 B):  0x02  len(1)  payload  crc16(2)  0x03
uint16_t VESCSpindleControl::vesc_build_packet(uint8_t *out, const uint8_t *payload, const uint16_t plen) {
  uint16_t idx = 0;
  out[idx++] = 0x02;
  out[idx++] = static_cast<uint8_t>(plen);
  for (uint16_t i = 0; i < plen; i++) out[idx++] = payload[i];
  wbe16(&out[idx], crc16::ccitt(payload, plen));
  idx += 2;
  out[idx++] = 0x03;
  return idx;
}

bool VESCSpindleControl::extract_vesc_payload(const uint8_t *frame, const uint16_t frame_len, const uint8_t **payload,
                                              uint16_t *payload_len) {
  if (payload) *payload = nullptr;
  if (payload_len) *payload_len = 0;

  if (frame_len < 6 || frame[0] != 0x02) return false;

  const uint8_t plen = frame[1];
  const uint16_t expected_len = static_cast<uint16_t>(plen) + 5;
  if (plen < 1 || frame_len != expected_len || frame[expected_len - 1] != 0x03) return false;

  const uint8_t *payload_start = &frame[2];

  const uint8_t crc_bytes[2] = {frame[2 + plen], frame[2 + plen + 1]};
  if (be16(crc_bytes) != crc16::ccitt(payload_start, plen)) return false;

  if (payload) *payload = payload_start;
  if (payload_len) *payload_len = plen;
  return true;
}
