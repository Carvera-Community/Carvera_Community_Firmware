#ifndef SPINDLEPUBLICACCESS_H
#define SPINDLEPUBLICACCESS_H

#include "checksumm.h"

#include <string>

#define pwm_spindle_control_checksum		CHECKSUM("pwm_spindle_control")
#define get_spindle_status_checksum    CHECKSUM("get_spindle_status")
#define turn_off_spindle_checksum    CHECKSUM("turn_off_spindle_status")

struct spindle_status {
	bool state;
    float current_rpm;
    float target_rpm;
    float current_pwm_value;
	float factor;
    // When set, additional_metrics is a complete status field, including its leading '|'.
    // The spindle chooses the name and the values. Empty when the spindle has nothing to add.
    bool has_additional_metrics;
    char additional_metrics[64];
};

#endif
