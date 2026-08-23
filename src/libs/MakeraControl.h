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

inline bool is_jog_command(const char* command, std::size_t length) {
  if (command == nullptr || length < 2 || std::memcmp(command, "$J", 2) != 0) return false;
  if (length == 2) return true;
  const char next = command[2];
  return next == ' ' || next == '\t' || next == '\r' || next == '\n';
}

inline bool is_deferred_command(const char* command, std::size_t length) {
  if (command == nullptr) return false;
  const auto starts_with_token = [command, length](const char* token, std::size_t token_length, bool subcodes = false) {
    if (length < token_length || std::memcmp(command, token, token_length) != 0) return false;
    if (length == token_length) return true;
    const char next = command[token_length];
    return next == ' ' || next == '\t' || next == '\r' || next == '\n' || (subcodes && next == '.');
  };

  return starts_with_token("suspend", 7) || starts_with_token("abort", 5) || starts_with_token("resume", 6) ||
         starts_with_token("M600", 4, true) || starts_with_token("M601", 4, true) || starts_with_token("goto", 4) ||
         is_jog_command(command, length);
}

ControlAction handle_control(uint8_t control);

}  // namespace makera
