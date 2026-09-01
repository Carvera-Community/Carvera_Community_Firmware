/*
 * StatusJson.cpp
 *
 * JSON status document for the read-only HTTP status API.
 * Mirrors the field sources of Kernel::get_query_string() (the `?` report),
 * but emits a stable, self-describing JSON schema for consumers like
 * Home Assistant's rest: integration.
 */

#include "StatusJson.h"

#include "libs/Kernel.h"
#include "libs/PublicData.h"
#include "libs/nuts_bolts.h"
#include "modules/robot/Robot.h"
#include "modules/robot/Conveyor.h"
#include "StepperMotor.h"
#include "checksumm.h"
#include "SpindlePublicAccess.h"
#include "TemperatureControlPublicAccess.h"
#include "ATCHandlerPublicAccess.h"
#include "LaserPublicAccess.h"
#include "PlayerPublicAccess.h"
#include "version.h"

#include <cstdio>
#include <cstring>
#include <cstdarg>

// append with a running cursor; always NUL-safe, returns the new cursor.
// Once the buffer is full further calls are no-ops, yielding truncated
// (possibly invalid) JSON rather than a buffer overrun.
static size_t jput(char *buf, size_t size, size_t pos, const char *fmt, ...)
{
	if (pos + 1 >= size) return size - 1;
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf + pos, size - pos, fmt, ap);
	va_end(ap);
	if (n < 0) return pos;
	pos += (size_t)n;
	return pos >= size ? size - 1 : pos;
}

// copy src into dst replacing characters that would need JSON escaping
// ('"', '\\', control and non-ASCII bytes) with '_' so the output size
// stays deterministic.
static void json_sanitize(char *dst, size_t dst_size, const char *src)
{
	size_t i;
	// dst_size also bounds how far src is read: callers pass fixed-size
	// firmware buffers that are not guaranteed to be terminated.
	for (i = 0; i + 1 < dst_size && src[i] != '\0'; i++) {
		char c = src[i];
		dst[i] = (c == '"' || c == '\\' || c < 0x20 || c > 0x7E) ? '_' : c;
	}
	dst[i] = '\0';
}

static const char *jbool(bool b)
{
	return b ? "true" : "false";
}

size_t build_status_json(char *buf, size_t buf_size,
                         const char *machine_name,
                         const char *sta_ip,
                         int console_clients)
{
	size_t pos = 0;

	uint8_t state = THEKERNEL->get_state();
	bool running = (state == RUN || state == HOME);

	const char *state_str;
	switch (state) {
		case SLEEP:   state_str = "Sleep"; break;
		case SUSPEND: state_str = "Pause"; break;
		case WAIT:    state_str = "Wait"; break;
		case TOOL:    state_str = "Tool"; break;
		case ALARM:   state_str = "Alarm"; break;
		case HOME:    state_str = "Home"; break;
		case HOLD:    state_str = "Hold"; break;
		case RUN:     state_str = "Run"; break;
		case IDLE:
		default:      state_str = "Idle"; break;
	}

	const char *model_str;
	switch (THEKERNEL->factory_set->MachineModel) {
		case CARVERA:     model_str = "C1"; break;
		case CARVERA_AIR: model_str = "CA1"; break;
		default:          model_str = "unknown"; break;
	}

	// Never format these straight into the document: they are fixed-size
	// firmware buffers that may not be terminated yet (the machine's IP is
	// only filled in once the wifi module reports an association), and a
	// stray %s would read past the end of one.
	char name_s[36], ip_s[16];
	json_sanitize(name_s, sizeof(name_s), machine_name);
	json_sanitize(ip_s, sizeof(ip_s), sta_ip);

	pos = jput(buf, buf_size, pos,
	           "{\"name\":\"%s\",\"model\":\"%s\",\"fw\":\"%s\",\"ip\":\"%s\",\"state\":\"%s\"",
	           name_s, model_str, Version().get_build(), ip_s, state_str);

	pos = jput(buf, buf_size, pos, ",\"alarm\":%s,\"halt_reason\":%d",
	           jbool(THEKERNEL->is_halted()),
	           THEKERNEL->is_halted() ? THEKERNEL->get_halt_reason() : 0);

	// machine and work positions, mirroring Kernel::get_query_string()
	float mpos[5] = {0, 0, 0, 0, 0};
	float wpos[5];
	if (running) {
		THEROBOT->get_current_machine_position(mpos);
		// current_position includes the compensation transform, get the inverse for the actual position
		if (THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false);
#if MAX_ROBOT_ACTUATORS > 3
		mpos[A_AXIS] = THEROBOT->actuators[A_AXIS]->get_current_position();
		mpos[B_AXIS] = THEROBOT->actuators[B_AXIS]->get_current_position();
#else
		mpos[A_AXIS] = 0;
		mpos[B_AXIS] = 0;
#endif
	} else {
		// last milestone when idle
		Robot::wcs_t mp = THEROBOT->get_axis_position();
		mpos[X_AXIS] = std::get<X_AXIS>(mp);
		mpos[Y_AXIS] = std::get<Y_AXIS>(mp);
		mpos[Z_AXIS] = std::get<Z_AXIS>(mp);
		mpos[A_AXIS] = std::get<A_AXIS>(mp);
		mpos[B_AXIS] = std::get<B_AXIS>(mp);
	}
	Robot::wcs_t wp = THEROBOT->mcs2wcs(mpos);
	wpos[X_AXIS] = std::get<X_AXIS>(wp);
	wpos[Y_AXIS] = std::get<Y_AXIS>(wp);
	wpos[Z_AXIS] = std::get<Z_AXIS>(wp);
	wpos[A_AXIS] = std::get<A_AXIS>(wp);
	wpos[B_AXIS] = std::get<B_AXIS>(wp);

	pos = jput(buf, buf_size, pos, ",\"mpos\":[%1.4f,%1.4f,%1.4f,%1.4f,%1.4f]",
	           THEROBOT->from_millimeters(mpos[X_AXIS]),
	           THEROBOT->from_millimeters(mpos[Y_AXIS]),
	           THEROBOT->from_millimeters(mpos[Z_AXIS]),
	           mpos[A_AXIS], mpos[B_AXIS]);
	pos = jput(buf, buf_size, pos, ",\"wpos\":[%1.4f,%1.4f,%1.4f,%1.4f,%1.4f]",
	           THEROBOT->from_millimeters(wpos[X_AXIS]),
	           THEROBOT->from_millimeters(wpos[Y_AXIS]),
	           THEROBOT->from_millimeters(wpos[Z_AXIS]),
	           wpos[A_AXIS], wpos[B_AXIS]);

	static const char *const wcs_names[] = {"G54", "G55", "G56", "G57", "G58", "G59", "G59.1", "G59.2", "G59.3"};
	uint8_t wcs = THEROBOT->get_current_wcs();
	pos = jput(buf, buf_size, pos, ",\"wcs\":\"%s\",\"wcs_rotation\":%1.4f,\"inch\":%s,\"absolute\":%s",
	           wcs < (sizeof(wcs_names) / sizeof(wcs_names[0])) ? wcs_names[wcs] : "?",
	           THEROBOT->r[wcs],
	           jbool(THEROBOT->inch_mode), jbool(THEROBOT->absolute_mode));

	pos = jput(buf, buf_size, pos, ",\"feed\":{\"current\":%1.1f,\"target\":%1.1f,\"override\":%1.1f}",
	           running ? THEROBOT->from_millimeters(THECONVEYOR->get_current_feedrate() * 60.0F) : 0.0F,
	           THEROBOT->from_millimeters(THEROBOT->get_feed_rate()),
	           6000.0F / THEROBOT->get_seconds_per_minute());

	struct spindle_status ss;
	if (PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &ss)) {
		pos = jput(buf, buf_size, pos,
		           ",\"spindle\":{\"on\":%s,\"rpm\":%1.1f,\"target_rpm\":%1.1f,\"override\":%1.1f,\"vacuum\":%s",
		           jbool(ss.state), ss.current_rpm, ss.target_rpm, ss.factor,
		           jbool(THEKERNEL->get_vacuum_mode()));
		struct pad_temperature temp;
		if (PublicData::get_value(temperature_control_checksum, current_temperature_checksum, spindle_temperature_checksum, &temp)) {
			pos = jput(buf, buf_size, pos, ",\"temp\":%1.1f", temp.current_temperature);
		}
		pos = jput(buf, buf_size, pos, "}");
	}

	// power supply temperature (Carvera Air only, self-gating)
	struct pad_temperature ptemp;
	if (PublicData::get_value(temperature_control_checksum, current_temperature_checksum, power_temperature_checksum, &ptemp)) {
		pos = jput(buf, buf_size, pos, ",\"power_temp\":%1.1f", ptemp.current_temperature);
	}

	struct tool_status tool;
	if (PublicData::get_value(atc_handler_checksum, get_tool_status_checksum, &tool)) {
		pos = jput(buf, buf_size, pos, ",\"tool\":{\"number\":%d,\"target\":%d,\"offset\":%1.3f}",
		           tool.active_tool, tool.target_tool, tool.tool_offset);
	}

	float wp_voltage;
	if (PublicData::get_value(atc_handler_checksum, get_wp_voltage_checksum, &wp_voltage)) {
		pos = jput(buf, buf_size, pos, ",\"probe_voltage\":%1.2f", wp_voltage);
	}

	struct laser_status ls;
	if (PublicData::get_value(laser_checksum, get_laser_status_checksum, &ls)) {
		pos = jput(buf, buf_size, pos, ",\"laser\":{\"mode\":%s,\"on\":%s,\"power\":%1.1f,\"scale\":%1.1f}",
		           jbool(ls.mode), jbool(ls.state), ls.power, ls.scale);
	}

	// job progress; Player returns a pointer to its own static pad_progress,
	// read it in place (single-threaded main-loop context) to avoid copying
	// the std::string filename
	void *returned_data;
	if (PublicData::get_value(player_checksum, get_progress_checksum, &returned_data)) {
		struct pad_progress *p = static_cast<struct pad_progress *>(returned_data);
		if (!p->is_playing && p->filename.empty()) {
			// nothing has played since boot, Player's last-progress fields are unset
			pos = jput(buf, buf_size, pos, ",\"job\":{\"playing\":false}");
		} else {
			char fname[100];
			json_sanitize(fname, sizeof(fname), p->filename.c_str());
			pos = jput(buf, buf_size, pos,
			           ",\"job\":{\"playing\":%s,\"file\":\"%s\",\"percent\":%u,\"played_lines\":%lu,\"parsed_lines\":%lu,\"elapsed_secs\":%lu}",
			           jbool(p->is_playing), fname, p->percent_complete,
			           p->played_lines, p->parsed_lines, p->elapsed_secs);
		}
	} else {
		pos = jput(buf, buf_size, pos, ",\"job\":{\"playing\":false}");
	}

	if (THEROBOT->compensationTransform != nullptr) {
		pos = jput(buf, buf_size, pos, ",\"leveling\":{\"active\":true,\"max_delta\":%1.3f}", THEROBOT->get_max_delta());
	} else {
		pos = jput(buf, buf_size, pos, ",\"leveling\":{\"active\":false}");
	}

	if (THEKERNEL->factory_set->FuncSetting & (1 << 2)) { // ATC present
		pos = jput(buf, buf_size, pos, ",\"atc_state\":%d", THEKERNEL->get_atc_state());
	}

	pos = jput(buf, buf_size, pos, ",\"clients\":%d,\"controller_connected\":%s}",
	           console_clients, jbool(console_clients > 0));

	return pos;
}
