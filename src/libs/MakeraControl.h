#pragma once

#include <cstdint>

namespace makera {

enum class ControlAction : uint8_t { none, query, diagnose, halt, stop, keep_alive, feed_hold, feed_resume };

constexpr ControlAction decode_control(uint8_t control) {
  switch (control) {
    case '?':
      return ControlAction::query;
    case '*':
      return ControlAction::diagnose;
    case 'X' - 'A' + 1:
      return ControlAction::halt;
    case 'Y' - 'A' + 1:
      return ControlAction::stop;
    case 'Z' - 'A' + 1:
      return ControlAction::keep_alive;
    case '!':
      return ControlAction::feed_hold;
    case '~':
      return ControlAction::feed_resume;
    default:
      return ControlAction::none;
  }
}

ControlAction handle_control(uint8_t control);

}  // namespace makera
