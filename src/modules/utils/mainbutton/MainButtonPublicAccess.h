#ifndef MAINBUTTONPUBLICACCESS_H
#define MAINBUTTONPUBLICACCESS_H

#include "checksumm.h"
#include <string>

#define main_button_checksum   		CHECKSUM("main_button")

#define switch_power_12_checksum	CHECKSUM("switch_power_12")
#define switch_power_24_checksum	CHECKSUM("switch_power_24")
#define get_e_stop_state_checksum	CHECKSUM("get_e_stop_state")
#define set_led_bar_checksum        CHECKSUM("set_led_bar")

struct led_rgb{
    int r;
    int g;
    int b;
};

#endif


