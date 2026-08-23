#include "MakeraControl.h"

#include "Kernel.h"
#include "mbed.h"

namespace makera {

ControlAction handle_control(uint8_t control) {
  const ControlAction action = decode_control(control);
  switch (action) {
    case ControlAction::stop:
      if (THEKERNEL->get_internal_stop_request()) {
        THEKERNEL->set_internal_stop_request(false);
      } else {
        THEKERNEL->set_stop_request(true);
        THEKERNEL->set_stop_request_time(us_ticker_read() / 1000);
      }
      break;
    case ControlAction::keep_alive:
      THEKERNEL->set_keep_alive_request(true);
      break;
    case ControlAction::feed_hold:
      if (THEKERNEL->is_feed_hold_enabled()) THEKERNEL->set_feed_hold(true);
      break;
    case ControlAction::feed_resume:
      if (THEKERNEL->is_feed_hold_enabled()) THEKERNEL->set_feed_hold(false);
      break;
    default:
      break;
  }
  return action;
}

}  // namespace makera
