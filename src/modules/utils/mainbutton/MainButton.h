#ifndef MAINBUTTON_H
#define MAINBUTTON_H

#include "libs/Pin.h"

class MainButton : public Module {
    public:
		MainButton();
        void on_module_loaded();
        void on_idle(void *argument);
        uint32_t button_tick(uint32_t dummy);
        uint32_t led_tick(uint32_t dummy);
        void on_second_tick(void *);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);

    private:
        Pin main_button;
        Pin main_button_LED_R;
        Pin main_button_LED_G;
        Pin main_button_LED_B;
        enum BUTTON_STATE {
            NONE,
			BUTTON_LONG_PRESSED,
			BUTTON_SHORT_PRESSED,
			BUTTON_LED_UPDATE
        };

        Pin e_stop;
        Pin PS12;
        Pin PS24;

        uint16_t power_fan_delay_s;
        uint32_t power_fan_countdown_us;

        uint8_t hold_toggle;
        uint8_t led_light_cnt;
        uint8_t led_update_timer;
        uint32_t button_press_time;
        uint32_t long_press_time_ms;
        std::string long_press_enable;

        bool auto_sleep;
        uint8_t auto_sleep_min;
        uint32_t sleep_countdown_us;

        bool enable_light;
        uint8_t turn_off_light_min;
        uint32_t light_countdown_us;

        bool button_pressed;
        volatile BUTTON_STATE button_state;
        
        bool stop_on_cover_open;

        uint32_t poll_frequency;

        bool sd_ok;
        bool using_12v;

        void switch_power_12(int state);
        void switch_power_24(int state);
        uint8_t old_state;
        void set_led_color(unsigned char R1, unsigned char G1, unsigned char B1,unsigned char R2, unsigned char G2, unsigned char B2,unsigned char R3, unsigned char G3, unsigned char B3,unsigned char R4, unsigned char G4, unsigned char B4,unsigned char R5, unsigned char G5, unsigned char B5);
        void set_led_colors(unsigned char R, unsigned char G, unsigned char B);
        void set_led_num(unsigned char ColorFR, unsigned char ColorFG, unsigned char ColorFB, unsigned char ColorBR, unsigned char ColorBG, unsigned char ColorBB, unsigned char num, bool row = false);
        void set_progress(unsigned char R, unsigned char G, unsigned char B, unsigned char num);
        void set_progress_blink_led_off(unsigned char R, unsigned char G, unsigned char B, unsigned char num);
};

#endif
