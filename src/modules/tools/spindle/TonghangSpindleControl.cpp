/*
 * Tonghang servo used as a spindle, through a USB CDC adapter speaking Modbus RTU.
 *
 * The adapter's serial settings are the reusable part. Another Modbus slave
 * on the same kind of adapter uses the same four link settings and its own
 * spindle type. This driver implements speed, direction, and orientation.
 *
 * The drive must already be in speed mode (P-004 = 1) and P-098 must be 0 so
 * Modbus can release the enable. Those are drive settings, not firmware ones.
 *
 *   spindle.type             tonghang
 *   spindle.modbus_baud            # empty detects; else 4800..115200
 *   spindle.modbus_parity    even
 *   spindle.modbus_stop_bits 1
 *   spindle.modbus_slave     1
 *   spindle.acc_ratio        1
 *   spindle.overload_threshold 90     # 0 disables the torque halt
 *   spindle.encoder_bits     17            # counts per turn is 2^bits
 *   spindle.orient_rpm       300           # fastest jog while orienting
 *   spindle.default_rpm      1000
 *   spindle.max_rpm          6000
 *   spindle.delay_s          1
 *
 * An empty modbus_baud tries the Tonghang rates in P-182: 19200, then 115200, then
 * 4800, 9600, 38400, and 57600. Framing defaults to 8E1 (P-183 = 1).
 * M3 is clockwise (jog code 0x1111, positive encoder counts). M4 is
 * counter-clockwise (0x2222). The drive's own CW/CCW labels are the opposite.
 * Speeds in gcode are spindle RPM. P-076 is commanded at spindle RPM / acc_ratio,
 * and the reported RPM is the motor reading times acc_ratio.
 */

#include "TonghangSpindleControl.h"

#include <cmath>
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
#include "checksumm.h"
#include "libs/Kernel.h"
#include "nuts_bolts.h"
#include "us_ticker_api.h"

namespace {
constexpr uint16_t SPEED_REG = 76;
constexpr uint16_t RUN_REG = 0x1010;
constexpr uint16_t RPM_REG = 0x1000;
// Drive jog codes. 0x1111 increases the encoder count, and that direction is M3.
constexpr uint16_t RUN_CCW = 0x1111;
constexpr uint16_t RUN_CW = 0x2222;
constexpr uint16_t HOLD_ZERO = 0x1234;
constexpr uint16_t RELEASE = 0x0000;
constexpr uint32_t POLL_INTERVAL_US = 200000;
constexpr uint32_t STOP_POLL_US = 50000;
constexpr uint32_t STOP_TIMEOUT_US = 3000000;
constexpr int16_t STOPPED_RPM = 5;
constexpr uint16_t TORQUE_REG = 0x1009;
constexpr uint16_t LOAD_REG = 0x1018;  // followed by output voltage, then alarm
constexpr uint16_t CURRENT_REG = 0x101E;
constexpr uint16_t POSITION_REG = 0x101F;
constexpr uint16_t ALARM_REG = 0x101A;
constexpr float ORIENT_TOLERANCE_DEG = 2.0f;
constexpr float ORIENT_SETTLE_DEG = 0.25f;
// M577 measures a position read at about 5 ms. The orient loop also services
// idle, so the catch window allows 10 ms per sample. Finish speed crosses half
// the tolerance in one sample. The last quarter degree is crept, because the
// drive stops short when the hold is commanded at the edge of the tolerance.
constexpr uint32_t ORIENT_SAMPLE_US = 5000;
constexpr uint32_t ORIENT_LOOP_US = 10000;
constexpr int ORIENT_CATCH_SAMPLES = 4;

const char *alarm_text(const uint16_t code) {
  switch (code) {
    case 1:
      return "overspeed";
    case 2:
      return "main circuit overvoltage";
    case 3:
      return "main circuit undervoltage";
    case 4:
      return "position following error";
    case 5:
      return "position command frequency too high";
    case 6:
      return "motor stall";
    case 7:
      return "drive inhibit";
    case 9:
      return "encoder AB signal fault";
    case 11:
      return "IPM module fault";
    case 12:
      return "overcurrent";
    case 13:
      return "overload";
    case 14:
      return "brake peak power overload";
    case 20:
      return "EEPROM error";
    case 21:
      return "logic circuit fault";
    case 23:
      return "AD reference voltage error";
    case 24:
      return "AD zero drift";
    case 25:
      return "motor code error";
    case 28:
      return "prolonged stall overload";
    case 29:
      return "user torque overload";
    case 30:
      return "encoder Z signal lost";
    case 31:
      return "encoder Z signal fault";
    case 32:
      return "encoder UVW signal illegal";
    case 33:
      return "wire-saving encoder signal error";
    case 35:
      return "absolute encoder multi-turn count error";
    case 38:
      return "absolute encoder zero error";
    case 39:
      return "absolute encoder communication lost";
    case 40:
      return "absolute encoder battery lost";
    case 41:
      return "absolute encoder count error";
    case 42:
      return "motor match error";
    case 43:
      return "absolute encoder position cleared";
    case 44:
      return "battery voltage low";
    case 45:
      return "absolute encoder overtemperature";
    default:
      return nullptr;
  }
}

int16_t signed_word(const uint8_t *adu, const int index) {
  const uint8_t *p = adu + 3 + index * 2;
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

uint32_t elapsed_us(const uint32_t start) {
  return static_cast<uint32_t>(us_ticker_read() - start);
}

float wrap_pm180(float degrees) {
  while (degrees > 180.0f) degrees -= 360.0f;
  while (degrees < -180.0f) degrees += 360.0f;
  return degrees;
}

// While this is alive, on_idle does not start a telemetry read.
class BusLock {
 public:
  explicit BusLock(bool &locked) : locked_(locked) { locked_ = true; }
  ~BusLock() { locked_ = false; }
  BusLock(const BusLock &) = delete;
  BusLock &operator=(const BusLock &) = delete;

 private:
  bool &locked_;
};

}  // namespace

void TonghangSpindleControl::raise_alarm(const char *reason) {
  if (comm_halted && THEKERNEL->is_halted()) return;
  comm_halted = true;
  THEKERNEL->streams->printf("ERROR: %s\n", reason);
  THEKERNEL->set_halt_reason(SPINDLE_ALARM);
  THEKERNEL->call_event(ON_HALT, nullptr);
}

void TonghangSpindleControl::note_alarm(const uint16_t code) {
  alarm_code = code;
  if (code == 0) return;
  char buf[80];
  if (const char *name = alarm_text(code)) {
    snprintf(buf, sizeof buf, "Tonghang spindle alarm %u: %s", code, name);
  } else {
    snprintf(buf, sizeof buf, "Tonghang spindle alarm %u", code);
  }
  raise_alarm(buf);
}

void TonghangSpindleControl::on_module_loaded() {
  const uint16_t spindle_cs = CHECKSUM("spindle");
  default_rpm = THEKERNEL->config->value(spindle_cs, CHECKSUM("default_rpm"))->as_int(1000);
  max_rpm = THEKERNEL->config->value(spindle_cs, CHECKSUM("max_rpm"))->as_int(6000);
  delay_s = THEKERNEL->config->value(spindle_cs, CHECKSUM("delay_s"))->as_int(1);
  acc_ratio = THEKERNEL->config->value(spindle_cs, CHECKSUM("acc_ratio"))->as_number(1.0f);
  overload_threshold = THEKERNEL->config->value(spindle_cs, CHECKSUM("overload_threshold"))->as_int(90);
  const int encoder_bits = THEKERNEL->config->value(spindle_cs, CHECKSUM("encoder_bits"))->as_int(17);
  orient_rpm = THEKERNEL->config->value(spindle_cs, CHECKSUM("orient_rpm"))->as_int(300);
  target_rpm = default_rpm;
  if (encoder_bits < 1 || encoder_bits > 30) {
    THEKERNEL->streams->printf("ERROR: spindle.encoder_bits must be 1 to 30\n");
    failed_init = true;
    return;
  }
  counts_per_rev = static_cast<int32_t>(1) << encoder_bits;
  if (orient_rpm < 1) orient_rpm = 1;
  if (orient_rpm > 32000) orient_rpm = 32000;
  if (acc_ratio <= 0.0f) {
    THEKERNEL->streams->printf("ERROR: spindle.acc_ratio must be greater than 0\n");
    failed_init = true;
    return;
  }

  const int slave = THEKERNEL->config->value(spindle_cs, CHECKSUM("modbus_slave"))->as_int(1);
  const int stop_bits = THEKERNEL->config->value(spindle_cs, CHECKSUM("modbus_stop_bits"))->as_int(1);
  const std::string parity = THEKERNEL->config->value(spindle_cs, CHECKSUM("modbus_parity"))->as_string("even");
  const std::string baud_text = THEKERNEL->config->value(spindle_cs, CHECKSUM("modbus_baud"))->as_string("");

  if (slave < 1 || slave > 32) {
    THEKERNEL->streams->printf("ERROR: spindle.modbus_slave must be 1 to 32\n");
    failed_init = true;
    return;
  }

  uint8_t cdc_stop = 0;
  if (stop_bits == 1) {
    cdc_stop = 0;
  } else if (stop_bits == 2) {
    cdc_stop = 2;
  } else {
    THEKERNEL->streams->printf("ERROR: spindle.modbus_stop_bits must be 1 or 2\n");
    failed_init = true;
    return;
  }

  uint8_t cdc_parity = 2;
  if (parity == "none") {
    cdc_parity = 0;
  } else if (parity == "odd") {
    cdc_parity = 1;
  } else if (parity == "even") {
    cdc_parity = 2;
  } else {
    THEKERNEL->streams->printf("ERROR: spindle.modbus_parity must be none, odd, or even\n");
    failed_init = true;
    return;
  }

  detect_baud = baud_text.empty();
  if (!detect_baud) {
    char *end = nullptr;
    const unsigned long parsed = strtoul(baud_text.c_str(), &end, 10);
    if (end == baud_text.c_str() || *end != '\0' || parsed == 0) {
      THEKERNEL->streams->printf("ERROR: spindle.modbus_baud must be empty or a baud rate\n");
      failed_init = true;
      return;
    }
    link_baud = static_cast<uint32_t>(parsed);
  }

  bus.set_serial(link_baud, cdc_stop, cdc_parity);
  bus.set_slave(static_cast<uint8_t>(slave));

  Pin usb_en;
  usb_en.from_string(THEKERNEL->config->value(CHECKSUM("usb_en_pin"))->as_string("1.19"));
  usb_en.as_output();
  usb_en.set(false);
  link_setup = LinkSetup::Enumerate;
}

bool TonghangSpindleControl::transact(const bool writing, const uint16_t reg, const uint16_t value, const uint16_t words,
                                 uint8_t *data, const uint8_t data_len) {
  bool started = false;
  const uint32_t start = us_ticker_read();
  while (elapsed_us(start) < 500000) {
    if (!started) started = writing ? bus.write_register(reg, value) : bus.read_holding(reg, words);
    bus.poll();
    if (bus.succeeded() || bus.failed()) break;
  }
  bool ok = false;
  if (bus.succeeded()) {
    const uint8_t *adu = bus.response();
    const uint8_t len = bus.response_length();
    if (len >= 2 && (adu[1] & 0x80) == 0) {
      if (writing) {
        ok = adu[1] == 0x06;
      } else if (data_len <= 16 && len >= 3 + data_len && adu[1] == 0x03 && adu[2] == data_len) {
        if (data != nullptr) {
          for (uint8_t i = 0; i < data_len; ++i) data[i] = adu[3 + i];
        }
        ok = true;
      }
    }
    bus.consume();
  } else if (bus.failed()) {
    bus.consume();
  } else {
    bus.abort();
  }
  return ok;
}

bool TonghangSpindleControl::probe() {
  uint8_t raw[2];
  return transact(false, 3, 0, 1, raw, sizeof raw);
}

uint16_t TonghangSpindleControl::commanded_magnitude() const {
  float rpm = static_cast<float>(target_rpm < 0 ? -target_rpm : target_rpm);
  rpm = rpm * factor / 100.0f;
  if (rpm > static_cast<float>(max_rpm)) rpm = static_cast<float>(max_rpm);
  rpm /= acc_ratio;
  if (rpm < 0.0f) rpm = 0.0f;
  if (rpm > 32000.0f) rpm = 32000.0f;
  return static_cast<uint16_t>(rpm + 0.5f);
}

float TonghangSpindleControl::reported_rpm() const {
  return std::fabs(static_cast<float>(current_rpm) * acc_ratio);
}

uint16_t TonghangSpindleControl::run_code() const {
  if (commanded_magnitude() == 0) return HOLD_ZERO;
  return ccw ? RUN_CW : RUN_CCW;
}

void TonghangSpindleControl::turn_on() {
  if (!ready) {
    raise_alarm("Tonghang spindle init failed");
    return;
  }
  spindle_on = true;
  THEKERNEL->spindleon = true;
  run_pending = true;
  stop_pending = false;
  comm_halted = false;

  if (delay_s <= 0) return;
  char buf[16];
  const int n = snprintf(buf, sizeof buf, "G4P%d", delay_s);
  if (n <= 0) return;
  std::string g(buf, static_cast<size_t>(n));
  Gcode gcode(g, &(StreamOutput::NullStream));
  THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);
}

void TonghangSpindleControl::turn_off() {
  spindle_on = false;
  THEKERNEL->spindleon = false;
  stop_pending = true;
  run_pending = false;
}

void TonghangSpindleControl::set_ccw(const bool ccw_gcode) {
  if (ccw_gcode == ccw) return;
  ccw = ccw_gcode;
  if (spindle_on) run_pending = true;
}

void TonghangSpindleControl::reassert_run() {
  run_pending = true;
  stop_pending = false;
}

void TonghangSpindleControl::set_speed(const int rpm) {
  int32_t limited = rpm;
  if (limited < 0) limited = -limited;
  if (limited > max_rpm) limited = max_rpm;
  target_rpm = limited;
  if (spindle_on) speed_pending = true;
}

void TonghangSpindleControl::set_factor(const float f) {
  factor = f;
  if (spindle_on) speed_pending = true;
}

bool TonghangSpindleControl::wait_for_bus() {
  const uint32_t start = us_ticker_read();
  while (elapsed_us(start) < 4000000) {
    if (!bus.busy() && step == Step::Idle && !stop_pending && !run_pending && !speed_pending) return true;
    service();
  }
  return false;
}

bool TonghangSpindleControl::modbus_exchange(const bool writing, const uint16_t reg, const uint16_t value, uint16_t *out) {
  if (!wait_for_bus()) return false;
  if (writing) return transact(true, reg, value, 1, nullptr, 0);
  uint8_t raw[2];
  if (!transact(false, reg, 0, 1, raw, sizeof raw)) return false;
  if (out != nullptr) *out = static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
  return true;
}

bool TonghangSpindleControl::modbus_link_defaults(uint16_t *reg, uint16_t *words) const {
  // Position is a 32-bit value, so the default read is two registers.
  *reg = POSITION_REG;
  *words = 2;
  return true;
}

bool TonghangSpindleControl::modbus_read(const uint16_t reg, const uint16_t words) {
  if (words < 1 || words > 8) return false;
  uint8_t raw[16];
  return transact(false, reg, 0, words, raw, static_cast<uint8_t>(words * 2));
}

bool TonghangSpindleControl::modbus_wait() { return wait_for_bus(); }

void TonghangSpindleControl::modbus_lock(const bool locked) { bus_locked = locked; }

uint32_t TonghangSpindleControl::modbus_baud() const { return link_baud; }

bool TonghangSpindleControl::read_motor_counts(int64_t *counts) {
  uint8_t raw[4];
  if (!transact(false, POSITION_REG, 0, 2, raw, sizeof raw)) return false;
  const uint16_t lo = static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
  const uint16_t hi = static_cast<uint16_t>((static_cast<uint16_t>(raw[2]) << 8) | raw[3]);
  const uint32_t sample = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
  if (!have_unwrapped_counts) {
    unwrapped_counts = static_cast<int64_t>(static_cast<int32_t>(sample));
    have_unwrapped_counts = true;
  } else {
    unwrapped_counts += static_cast<int32_t>(sample - last_raw_counts);
  }
  last_raw_counts = sample;
  *counts = unwrapped_counts;
  return true;
}

bool TonghangSpindleControl::write_reg(const uint16_t reg, const uint16_t value) {
  return transact(true, reg, value, 1, nullptr, 0);
}

bool TonghangSpindleControl::hold_stopped() {
  if (!write_reg(RUN_REG, HOLD_ZERO)) return false;
  const uint32_t start = us_ticker_read();
  while (elapsed_us(start) < 1000000) {
    uint8_t raw[2];
    // A dropped frame is not a failed stop. Keep reading until the shaft is still.
    if (!transact(false, RPM_REG, 0, 1, raw, sizeof raw)) continue;
    const int16_t rpm = static_cast<int16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
    if (std::abs(static_cast<int>(rpm)) <= STOPPED_RPM) return true;
  }
  return false;
}

bool TonghangSpindleControl::hold_position() {
  // Same end state as M3 S0: enabled, zero speed, and no pending jog.
  if (!write_reg(SPEED_REG, 0)) return false;
  if (!hold_stopped()) return false;
  target_rpm = 0;
  spindle_on = true;
  THEKERNEL->spindleon = true;
  run_pending = false;
  speed_pending = false;
  stop_pending = false;
  return true;
}

bool TonghangSpindleControl::release_drive() {
  run_pending = false;
  speed_pending = false;
  stop_pending = false;
  if (!hold_stopped()) return false;
  if (!write_reg(RUN_REG, RELEASE)) return false;
  spindle_on = false;
  THEKERNEL->spindleon = false;
  return true;
}

bool TonghangSpindleControl::read_alarm(uint16_t *code) {
  uint8_t raw[2];
  if (!transact(false, ALARM_REG, 0, 1, raw, sizeof raw)) return false;
  *code = static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
  alarm_code = *code;
  return true;
}

float TonghangSpindleControl::spindle_angle(const int64_t counts) const {
  const double revolutions =
      static_cast<double>(counts) * static_cast<double>(acc_ratio) / static_cast<double>(counts_per_rev);
  double fraction = revolutions - std::floor(revolutions);
  if (fraction < 0.0) fraction += 1.0;
  return static_cast<float>(fraction * 360.0);
}

bool TonghangSpindleControl::orient_to(const float target_deg, const int mode, const float timeout_s) {
  if (!wait_for_bus()) return false;
  const BusLock lock(bus_locked);
  int64_t counts = 0;
  if (!read_motor_counts(&counts)) return false;
  float delta = wrap_pm180(target_deg - spindle_angle(counts));
  if (mode == 1) {
    if (delta < 0.0f) delta += 360.0f;
  } else if (mode == 2 && delta > 0.0f) {
    delta -= 360.0f;
  }
  // A fast jog moves farther than this between position samples.
  if (fabsf(delta) <= ORIENT_SETTLE_DEG) {
    if (!hold_position()) return false;
    if (!read_motor_counts(&counts)) return false;
    delta = wrap_pm180(target_deg - spindle_angle(counts));
    if (mode == 1) {
      if (delta < 0.0f) delta += 360.0f;
    } else if (mode == 2 && delta > 0.0f) {
      delta -= 360.0f;
    }
    if (fabsf(delta) <= ORIENT_SETTLE_DEG) return true;
  }
  const float motor_delta = delta / acc_ratio;
  const int32_t delta_counts = static_cast<int32_t>(lroundf(motor_delta / 360.0f * static_cast<float>(counts_per_rev)));
  if (delta_counts == 0) return hold_position();
  const int64_t target = counts + delta_counts;
  const uint32_t deadline = us_ticker_read() + static_cast<uint32_t>(timeout_s * 1000000.0f);
  // Finish speed crosses half the tolerance in one position sample. The final
  // quarter degree is slower, so the hold is not commanded while a degree of
  // travel is still left. Approach speed minimises this move: a faster approach
  // needs a longer catch window, and that window is then crossed at the finish
  // speed. The run command latches P-076, so the speed for the distance already
  // in hand is written before enable. Past the target, the jog turns around at
  // the finish speed. Once the held angle is inside the settle band the drive
  // stays enabled at zero speed, the same as M3 S0, until M5.
  const float sample_s = static_cast<float>(ORIENT_SAMPLE_US) / 1000000.0f;
  const float half_tol_deg = ORIENT_TOLERANCE_DEG * 0.5f / acc_ratio;
  int finish_rpm_i = static_cast<int>(lroundf(half_tol_deg / 360.0f * 60.0f / sample_s));
  if (finish_rpm_i < 1) finish_rpm_i = 1;
  if (finish_rpm_i > 60) finish_rpm_i = 60;
  if (finish_rpm_i > orient_rpm) finish_rpm_i = orient_rpm;
  const uint16_t finish_rpm = static_cast<uint16_t>(finish_rpm_i);
  const float settle_motor_deg = ORIENT_SETTLE_DEG / acc_ratio;
  int settle_rpm_i = static_cast<int>(lroundf(settle_motor_deg / 360.0f * 60.0f / sample_s));
  if (settle_rpm_i < 1) settle_rpm_i = 1;
  if (settle_rpm_i > finish_rpm_i) settle_rpm_i = finish_rpm_i;
  const uint16_t settle_rpm = static_cast<uint16_t>(settle_rpm_i);

  int32_t tolerance_counts = static_cast<int32_t>(
      lroundf(ORIENT_TOLERANCE_DEG / acc_ratio / 360.0f * static_cast<float>(counts_per_rev)));
  if (tolerance_counts < 1) tolerance_counts = 1;
  int32_t settle_counts = static_cast<int32_t>(
      lroundf(ORIENT_SETTLE_DEG / acc_ratio / 360.0f * static_cast<float>(counts_per_rev)));
  if (settle_counts < 1) settle_counts = 1;

  const int32_t initial_distance = delta_counts < 0 ? -delta_counts : delta_counts;
  const float distance_rev = static_cast<float>(initial_distance) / static_cast<float>(counts_per_rev);
  const float finish_rps = static_cast<float>(finish_rpm) / 60.0f;
  const float catch_s = static_cast<float>(ORIENT_CATCH_SAMPLES) * static_cast<float>(ORIENT_LOOP_US) / 1000000.0f;
  int approach_rpm_i = finish_rpm_i;
  if (distance_rev > 0.0f && catch_s > 0.0f) {
    approach_rpm_i = static_cast<int>(lroundf(sqrtf(distance_rev * finish_rps / catch_s) * 60.0f));
  }
  if (approach_rpm_i < finish_rpm_i) approach_rpm_i = finish_rpm_i;
  if (approach_rpm_i > orient_rpm) approach_rpm_i = orient_rpm;
  const uint16_t approach_rpm = static_cast<uint16_t>(approach_rpm_i);

  int32_t slow_counts = static_cast<int32_t>(
      lroundf(static_cast<float>(approach_rpm) / 60.0f * catch_s * static_cast<float>(counts_per_rev)));
  if (slow_counts < tolerance_counts) slow_counts = tolerance_counts;

  bool finishing = initial_distance <= slow_counts;
  bool settling = false;
  bool jogging = false;
  int dir = delta_counts > 0 ? 1 : -1;
  uint16_t commanded_rpm = approach_rpm;
  if (initial_distance <= slow_counts) commanded_rpm = finish_rpm;
  if (initial_distance <= tolerance_counts) commanded_rpm = settle_rpm;
  if (!write_reg(SPEED_REG, commanded_rpm)) return false;
  if (!write_reg(RUN_REG, dir > 0 ? RUN_CCW : RUN_CW)) return false;
  jogging = true;
  spindle_on = true;
  THEKERNEL->spindleon = true;
  ccw = dir < 0;

  while (static_cast<int32_t>(us_ticker_read() - deadline) < 0) {
    THEKERNEL->call_event(ON_IDLE, this);
    if (THEKERNEL->is_halted()) {
      release_drive();
      return false;
    }
    int64_t pos = 0;
    if (!read_motor_counts(&pos)) {
      release_drive();
      return false;
    }
    int64_t error = target - pos;
    int64_t distance = error < 0 ? -error : error;
    int want_dir = error > 0 ? 1 : (error < 0 ? -1 : dir);

    if (settling) {
      uint8_t raw[2];
      if (!transact(false, RPM_REG, 0, 1, raw, sizeof raw)) {
        release_drive();
        return false;
      }
      const int16_t rpm = static_cast<int16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
      if (std::abs(static_cast<int>(rpm)) > 1) continue;
      if (!read_motor_counts(&pos)) {
        release_drive();
        return false;
      }
      error = target - pos;
      distance = error < 0 ? -error : error;
      if (distance <= settle_counts) return hold_position();
      settling = false;
      commanded_rpm = 0;
      finishing = true;
      want_dir = error > 0 ? 1 : (error < 0 ? -1 : dir);
    } else if (distance <= settle_counts && commanded_rpm == settle_rpm && want_dir == dir) {
      if (!hold_stopped()) {
        release_drive();
        return false;
      }
      settling = true;
      jogging = false;
      continue;
    }

    if (want_dir != dir) finishing = true;
    if (!finishing && distance <= slow_counts) finishing = true;

    uint16_t want_rpm = finishing ? finish_rpm : approach_rpm;
    if (distance <= tolerance_counts) want_rpm = settle_rpm;

    // P-076 updates a jog that is already running. The run register is written
    // when the jog starts or the direction changes.
    const bool write_speed = want_rpm != commanded_rpm;
    const bool write_run = want_dir != dir || !jogging;
    if (write_speed || write_run) {
      if (write_speed && !write_reg(SPEED_REG, want_rpm)) {
        release_drive();
        return false;
      }
      if (write_run && !write_reg(RUN_REG, want_dir > 0 ? RUN_CCW : RUN_CW)) {
        release_drive();
        return false;
      }
      commanded_rpm = want_rpm;
      dir = want_dir;
      ccw = dir < 0;
      jogging = true;
    }
  }
  release_drive();
  return false;
}

void TonghangSpindleControl::on_spindle_orient(Gcode *gcode) {
  if (!ready) {
    gcode->stream->printf("ERROR: Tonghang spindle is not connected\n");
    return;
  }
  if (THEKERNEL->is_halted()) return;
  if (!gcode->has_letter('R')) {
    gcode->stream->printf("ERROR: M19 requires R\n");
    return;
  }
  float angle = gcode->get_value('R');
  if (angle < 0.0f || angle > 360.0f) {
    gcode->stream->printf("ERROR: M19 R must be 0 to 360\n");
    return;
  }
  if (angle == 360.0f) angle = 0.0f;
  int mode = 0;
  if (gcode->has_letter('P')) mode = static_cast<int>(gcode->get_value('P'));
  if (mode < 0 || mode > 2) {
    gcode->stream->printf("ERROR: M19 P must be 0, 1, or 2\n");
    return;
  }
  float timeout_s = 30.0f;
  if (gcode->has_letter('Q')) timeout_s = gcode->get_value('Q');
  if (!(timeout_s > 0.0f) || timeout_s > 120.0f) {
    gcode->stream->printf("ERROR: M19 Q must be greater than 0 and at most 120 seconds\n");
    return;
  }
  if (!orient_to(angle, mode, timeout_s)) {
    gcode->stream->printf("ERROR: M19 orient failed\n");
    THEKERNEL->set_halt_reason(SPINDLE_ALARM);
    THEKERNEL->call_event(ON_HALT, nullptr);
    return;
  }
  int64_t counts = 0;
  if (!read_motor_counts(&counts)) return;
  gcode->stream->printf("oriented A %.3f\n", static_cast<double>(spindle_angle(counts)));
}

void TonghangSpindleControl::on_spindle_angle(Gcode *gcode) {
  if (!ready) {
    gcode->stream->printf("ERROR: Tonghang spindle is not connected\n");
    return;
  }
  if (!wait_for_bus()) {
    gcode->stream->printf("ERROR: spindle angle read failed\n");
    return;
  }
  int64_t counts = 0;
  if (!read_motor_counts(&counts)) {
    gcode->stream->printf("ERROR: spindle angle read failed\n");
    return;
  }
  gcode->stream->printf("A %.3f\n", static_cast<double>(spindle_angle(counts)));
}

void TonghangSpindleControl::report_speed() {
  char alarm_buf[48];
  if (alarm_code == 0) {
    snprintf(alarm_buf, sizeof alarm_buf, "none");
  } else if (const char *name = alarm_text(alarm_code)) {
    snprintf(alarm_buf, sizeof alarm_buf, "%u %s", alarm_code, name);
  } else {
    snprintf(alarm_buf, sizeof alarm_buf, "%u", alarm_code);
  }
  THEKERNEL->streams->printf(
      "TH  State: %s  RPM: %5.0f  Target RPM: %5.0f  Direction: %s  Torque: %d%%  Current: %.2f A  Load: %d%%  "
      "Alarm: %s\n",
      spindle_on ? "on" : "off", static_cast<double>(reported_rpm()), static_cast<double>(target_rpm),
      ccw ? "ccw" : "cw", torque_pct, static_cast<double>(current_raw) * 0.01, load_pct, alarm_buf);
}

bool TonghangSpindleControl::dispatch() {
  switch (step) {
    case Step::SpeedThenRun:
    case Step::SpeedOnly:
      return bus.write_register(SPEED_REG, commanded_magnitude());
    case Step::Run:
      return bus.write_register(RUN_REG, run_code());
    case Step::Hold:
      return bus.write_register(RUN_REG, HOLD_ZERO);
    case Step::Release:
      return bus.write_register(RUN_REG, RELEASE);
    case Step::ReadRpm:
    case Step::ReadStop:
      return bus.read_holding(RPM_REG, 1);
    case Step::ReadTorque:
      return bus.read_holding(TORQUE_REG, 1);
    case Step::ReadLoad:
      return bus.read_holding(LOAD_REG, 3);
    case Step::ReadCurrent:
      return bus.read_holding(CURRENT_REG, 1);
    default:
      return true;
  }
}

void TonghangSpindleControl::on_response() {
  const uint8_t *adu = bus.response();
  const uint8_t len = bus.response_length();
  if (len < 5 || (adu[1] & 0x80)) {
    raise_alarm("Tonghang spindle rejected a Modbus command");
    step = Step::Idle;
    return;
  }

  if (step == Step::ReadRpm || step == Step::ReadStop || step == Step::ReadTorque || step == Step::ReadCurrent) {
    if (len < 7 || adu[1] != 0x03 || adu[2] != 2) {
      raise_alarm("Tonghang spindle bad Modbus read");
      step = Step::Idle;
      return;
    }
    const int16_t value = signed_word(adu, 0);
    if (step == Step::ReadRpm || step == Step::ReadStop) current_rpm = value;
    else if (step == Step::ReadTorque) {
      torque_pct = value;
      if (overload_threshold > 0 && std::abs(static_cast<int>(torque_pct)) > overload_threshold) {
        char buf[64];
        snprintf(buf, sizeof buf, "Tonghang spindle torque %d%% is above %d%%", torque_pct, overload_threshold);
        raise_alarm(buf);
      }
    } else current_raw = value;
  } else if (step == Step::ReadLoad) {
    if (len < 11 || adu[1] != 0x03 || adu[2] != 6) {
      raise_alarm("Tonghang spindle bad Modbus read");
      step = Step::Idle;
      return;
    }
    load_pct = signed_word(adu, 0);
    note_alarm(static_cast<uint16_t>(signed_word(adu, 2)));
  }

  switch (step) {
    case Step::SpeedThenRun:
      step = Step::Run;
      break;
    case Step::Hold:
      step = Step::WaitStop;
      stop_wait_start_us = us_ticker_read();
      break;
    case Step::ReadStop:
      step = Step::WaitStop;
      break;
    case Step::ReadRpm:
      step = Step::ReadTorque;
      break;
    case Step::ReadTorque:
      step = Step::ReadLoad;
      break;
    case Step::ReadLoad:
      step = Step::ReadCurrent;
      break;
    default:
      step = Step::Idle;
      break;
  }
}

void TonghangSpindleControl::service() {
  bus.poll();
  if (bus.busy()) return;

  if (bus.failed()) {
    const Step failed = step;
    bus.consume();
    const bool poll = failed == Step::ReadRpm || failed == Step::ReadTorque || failed == Step::ReadLoad ||
                      failed == Step::ReadCurrent;
    const bool stop_read = failed == Step::ReadStop;
    if (bus.connected() && link_failures < 2 && (poll || stop_read)) {
      link_failures++;
      // A failed stop read has to be repeated. Dropping back to idle would skip the release.
      step = stop_read ? Step::ReadStop : Step::Idle;
      return;
    }
    link_failures = 0;
    step = Step::Idle;
    raise_alarm(bus.connected() ? "Tonghang spindle no Modbus response" : "Tonghang spindle USB adapter disconnected");
    return;
  }
  if (bus.succeeded()) {
    link_failures = 0;
    on_response();
    bus.consume();
  }

  // A stop requested while another command is queued replaces that command.
  // An in-flight transfer is left to finish; this runs once the bus is idle.
  if (stop_pending && step != Step::Hold && step != Step::WaitStop && step != Step::ReadStop && step != Step::Release) {
    stop_pending = false;
    step = Step::Hold;
  }

  if (step == Step::WaitStop) {
    const bool stopped = std::abs(static_cast<int>(current_rpm)) <= STOPPED_RPM;
    if (stopped || elapsed_us(stop_wait_start_us) >= STOP_TIMEOUT_US) {
      step = Step::Release;
    } else if (elapsed_us(last_poll_us) >= STOP_POLL_US) {
      step = Step::ReadStop;
    } else {
      return;
    }
  }

  if (step == Step::Idle) {
    if (stop_pending) {
      stop_pending = false;
      step = Step::Hold;
    } else if (run_pending) {
      run_pending = false;
      step = Step::SpeedThenRun;
    } else if (speed_pending && spindle_on) {
      speed_pending = false;
      step = Step::SpeedOnly;
    } else if (elapsed_us(last_poll_us) >= POLL_INTERVAL_US) {
      step = Step::ReadRpm;
    } else {
      return;
    }
  }

  if (step == Step::Idle || step == Step::WaitStop) return;
  if (!dispatch()) return;
  if (step == Step::ReadRpm || step == Step::ReadStop) last_poll_us = us_ticker_read();
}

void TonghangSpindleControl::on_idle(void *) {
  if (failed_init) {
    failed_init = false;
    raise_alarm("Tonghang spindle init failed");
    return;
  }

  if (link_setup == LinkSetup::Enumerate) {
    if (!bus.init(5000)) {
      THEKERNEL->streams->printf("ERROR: USB CDC adapter not detected\n");
      raise_alarm("Tonghang spindle init failed");
      link_setup = LinkSetup::Failed;
      return;
    }
    link_setup = LinkSetup::Probe;
    baud_index = 0;
    return;
  }

  if (link_setup == LinkSetup::Probe) {
    // Factory rate first, then the fastest, then the rest of P-182.
    static const uint32_t rates[] = {19200, 115200, 4800, 9600, 38400, 57600};
    const uint32_t rate = detect_baud ? rates[baud_index] : link_baud;
    // Enumeration already programmed a fixed baud, and 19200 is the rate it came up at.
    const bool opened = !detect_baud || rate == 19200 || bus.use_baud(rate);
    if (opened && probe()) {
      link_baud = rate;
      ready = true;
      link_setup = LinkSetup::Wait;
      THEKERNEL->streams->printf("Tonghang spindle connected via USB at %lu baud\n",
                                 static_cast<unsigned long>(link_baud));
      return;
    }
    if (!detect_baud || ++baud_index >= (sizeof rates / sizeof rates[0])) {
      if (detect_baud) {
        THEKERNEL->streams->printf("ERROR: Tonghang spindle did not answer at any Modbus baud\n");
      } else {
        THEKERNEL->streams->printf("ERROR: Tonghang spindle did not answer the Modbus probe at %lu baud\n",
                                   static_cast<unsigned long>(link_baud));
      }
      raise_alarm("Tonghang spindle init failed");
      link_setup = LinkSetup::Failed;
    }
    return;
  }

  if (!ready || bus_locked) return;
  service();
}

void TonghangSpindleControl::on_get_public_data(void *argument) {
  auto *pdr = static_cast<PublicDataRequest *>(argument);
  if (!pdr->starts_with(pwm_spindle_control_checksum)) return;
  if (pdr->second_element_is(get_spindle_angle_checksum)) {
    if (!ready || !wait_for_bus()) return;
    int64_t counts = 0;
    if (!read_motor_counts(&counts)) return;
    *static_cast<float *>(pdr->get_data_ptr()) = spindle_angle(counts);
    pdr->set_taken();
    return;
  }
  if (!pdr->second_element_is(get_spindle_status_checksum)) return;

  const auto status = static_cast<spindle_status *>(pdr->get_data_ptr());
  status->state = spindle_on;
  status->current_rpm = reported_rpm();
  status->target_rpm = static_cast<float>(target_rpm);
  status->current_pwm_value = 0;
  status->factor = factor;
  status->can_orient = true;
  status->has_additional_metrics = true;
  snprintf(status->additional_metrics, sizeof status->additional_metrics, "|TH:%.0f,%.2f,%.0f,%d",
           static_cast<double>(torque_pct), static_cast<double>(current_raw) * 0.01, static_cast<double>(load_pct),
           alarm_code);
  pdr->set_taken();
}

void TonghangSpindleControl::on_set_public_data(void *argument) {
  auto *pdr = static_cast<PublicDataRequest *>(argument);
  if (!pdr->starts_with(pwm_spindle_control_checksum)) return;

  if (pdr->second_element_is(get_spindle_status_checksum)) {
    const auto *status = static_cast<const spindle_status *>(pdr->get_data_ptr());
    set_factor(status->factor);
    pdr->set_taken();
    return;
  }
  if (pdr->second_element_is(turn_off_spindle_checksum)) {
    turn_off();
    pdr->set_taken();
  }
}
