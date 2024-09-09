#ifndef __ENDSTOPSPUBLICACCESS_H_
#define __ENDSTOPSPUBLICACCESS_H_

// addresses used for public data access
#define endstops_checksum    CHECKSUM("endstop")
#define trim_checksum        CHECKSUM("trim")
#define home_offset_checksum CHECKSUM("home_offset")
#define g28_position_checksum CHECKSUM("g28_position")
#define get_homing_status_checksum CHECKSUM("homing_status")
#define get_homed_status_checksum CHECKSUM("homed_status")
#define get_endstop_states_checksum CHECKSUM("endstop_states")
#define get_cover_endstop_state_checksum CHECKSUM("cover_endstop_state")

#endif
