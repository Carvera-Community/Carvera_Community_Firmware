#include "MakeraControl.h"

#include "mbed.h" // for us_ticker_read()
#include "libs/Kernel.h"

MakeraControlFlag makera_handle_control(uint8_t control)
{
    switch (control) {
        case '?':
            return MakeraControlFlag::Query;

        case '*':
            return MakeraControlFlag::Diagnose;

        case 'X' - 'A' + 1: // ^X
            return MakeraControlFlag::Halt;

        case 'Y' - 'A' + 1: // ^Y
            // Acknowledges an outstanding internal stop, otherwise it is a
            // fresh stop request from the host.
            if (THEKERNEL->get_internal_stop_request()) {
                THEKERNEL->set_internal_stop_request(false);
            } else {
                THEKERNEL->set_stop_request(true);
                THEKERNEL->set_stop_request_time(us_ticker_read() / 1000);
            }
            break;

        case 'Z' - 'A' + 1: // ^Z
            THEKERNEL->set_keep_alive_request(true);
            break;

        case '!': // safe pause
            if (THEKERNEL->is_feed_hold_enabled()) {
                THEKERNEL->set_feed_hold(true);
            }
            break;

        case '~': // safe resume
            if (THEKERNEL->is_feed_hold_enabled()) {
                THEKERNEL->set_feed_hold(false);
            }
            break;

        default:
            break;
    }

    return MakeraControlFlag::None;
}
