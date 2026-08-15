/*
 * Handling for the single-byte control codes carried by PTYPE_CTRL_SINGLE
 * frames, shared by every console that speaks the Makera protocol.
 *
 * Kernel-wide effects (stop request, keep-alive, feed hold) are applied here
 * because they are identical for every console. The query, diagnose and halt
 * responses are per-stream, so those are reported back to the caller to raise
 * its own flag -- they cannot be written through a pointer because each console
 * stores them as bitfields.
 */

#ifndef MAKERACONTROL_H
#define MAKERACONTROL_H

#include <stdint.h>

enum class MakeraControlFlag : uint8_t {
    None,
    Query,
    Diagnose,
    Halt,
};

MakeraControlFlag makera_handle_control(uint8_t control);

#endif /* MAKERACONTROL_H */
