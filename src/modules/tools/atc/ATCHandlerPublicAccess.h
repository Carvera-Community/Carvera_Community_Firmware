#ifndef ATCHANDLERPUBLICACCESS_H
#define ATCHANDLERPUBLICACCESS_H

#include "checksumm.h"
#include <string>

#define atc_handler_checksum   		CHECKSUM("atc_handler")
#define get_tool_status_checksum    CHECKSUM("get_tool_status")
#define set_ref_tool_mz_checksum	CHECKSUM("set_ref_tool_mz")
#define get_atc_pin_status_checksum	CHECKSUM("get_atc_pin_status")

#define set_serial_rx_irq_checksum	CHECKSUM("set_serial_rx_irq")

#define set_wp_laser_checksum	CHECKSUM("set_wp_laser")
#define get_wp_voltage_checksum	CHECKSUM("get_wp_voltage")
#define show_wp_state_checksum  CHECKSUM("show_wp_state")

struct tool_status {
	int active_tool;
	float cur_tool_mz;
	float ref_tool_mz;
	float tool_offset;
};

#endif
