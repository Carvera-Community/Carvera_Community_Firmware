#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

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

inline bool is_diagnostic_request(const uint8_t* data, std::size_t length) {
  constexpr char command[] = "diagnose";
  constexpr std::size_t command_length = sizeof(command) - 1;
  if (data == nullptr || length < command_length || std::memcmp(data, command, command_length) != 0) return false;
  for (std::size_t i = command_length; i < length; ++i) {
    if (data[i] != ' ' && data[i] != '\t' && data[i] != '\r' && data[i] != '\n') return false;
  }
  return true;
}

ControlAction handle_control(uint8_t control);

}  // namespace makera
