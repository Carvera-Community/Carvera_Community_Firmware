#ifndef ATCHANDLERPUBLICACCESS_H
#define ATCHANDLERPUBLICACCESS_H

#include "checksumm.h"
#include <string>

#define atc_handler_checksum   		CHECKSUM("atc_handler")
#define get_tool_status_checksum    CHECKSUM("get_tool_status")
#define get_machine_offsets_checksum CHECKSUM("get_machine_offsets")
#define set_ref_tool_mz_checksum	CHECKSUM("set_ref_tool_mz")
#define get_atc_pin_status_checksum	CHECKSUM("get_atc_pin_status")
#define set_job_complete_checksum	CHECKSUM("set_job_complete")
#define abort_checksum				CHECKSUM("abort")

#define set_serial_rx_irq_checksum	CHECKSUM("set_serial_rx_irq")

#define set_wp_laser_checksum	CHECKSUM("set_wp_laser")
#define get_wp_voltage_checksum	CHECKSUM("get_wp_voltage")
#define show_wp_state_checksum  CHECKSUM("show_wp_state")
#define get_atc_clamped_status_checksum CHECKSUM("atc_clamp_status")

struct tool_status {
	int active_tool;
	int target_tool;
	float cur_tool_mz;
	float ref_tool_mz;
	float tool_offset;
};

struct machine_offsets {
	float anchor1_x;
    float anchor1_y;
    float anchor2_offset_x;
    float anchor2_offset_y;
    float anchor_width;

    float rotation_offset_x;
    float rotation_offset_y;
    float rotation_offset_z;
    float rotation_width;

	float clearance_z;
};

#endif
