#ifndef MAINBUTTONPUBLICACCESS_H
#define MAINBUTTONPUBLICACCESS_H

#include "checksumm.h"
#include <stdint.h>
#include <string>

#define main_button_checksum   		CHECKSUM("main_button")

#define switch_power_12_checksum	CHECKSUM("switch_power_12")
#define switch_power_24_checksum	CHECKSUM("switch_power_24")
#define get_e_stop_state_checksum	CHECKSUM("get_e_stop_state")
#define set_led_bar_checksum        CHECKSUM("set_led_bar")
#define get_led_bar_checksum        CHECKSUM("get_led_bar")
#define restore_led_bar_checksum    CHECKSUM("restore_led_bar")

struct led_rgb{
    int r;
    int g;
    int b;
    int i;
};

struct led_bar_state{
    uint8_t r[5];
    uint8_t g[5];
    uint8_t b[5];
    uint8_t n;
};

#endif


