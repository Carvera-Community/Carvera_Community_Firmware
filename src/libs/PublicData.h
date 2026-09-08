/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PUBLICDATA_H
#define PUBLICDATA_H

//new communication protocol
#define HEADER        0x8668
#define FOOTER        0x55AA
#define PTYPE_CTRL_SINGLE	0xA1
#define	PTYPE_CTRL_MULTI	0xA2
#define PTYPE_FILE_START	0xB0
#define PTYPE_FILE_MD5		0xB1
#define PTYPE_FILE_VIEW		0xB2
#define PTYPE_FILE_DATA		0xB3
#define PTYPE_FILE_END		0xB4
#define PTYPE_FILE_CAN		0xB5
#define PTYPE_FILE_RETRY	0xB6
#define PTYPE_CONFIG_START	0xD1
#define PTYPE_CONFIG_VIEW	0xD2
#define PTYPE_CONFIG_DATA	0xD3
#define PTYPE_CONFIG_END		0xD4
#define PTYPE_CONFIG_CAN		0xD5
#define PTYPE_FACTORY_START	0xE1
#define PTYPE_FACTORY_VIEW	0xE2
#define PTYPE_FACTORY_DATA	0xE3
#define PTYPE_FACTORY_END	0xE4
#define PTYPE_FACTORY_CAN	0xE5
#define PTYPE_PLAY_START	0xF1
#define PTYPE_PLAY_VIEW		0xF2
#define PTYPE_PLAY_DATA		0xF3
#define PTYPE_PLAY_END		0xF4
#define PTYPE_PLAY_CAN		0xF5
#define PTYPE_GOTO_START	0xF6
#define PTYPE_GOTO_LINES	0xF7
#define PTYPE_FIRM_VER		0x71
#define PTYPE_STATUS_RES	0x81
#define PTYPE_DIAG_RES		0x82
#define PTYPE_LOAD_INFO		0x83
#define PTYPE_LOAD_FINISH	0x84
#define PTYPE_LOAD_ERROR	0x85
#define PTYPE_NORMAL_INFO	0x90
#include <stdint.h>

class PublicData {
    public:
        // there are two ways to get data from a module
        // 1. pass in a pointer to a data storage area that the caller creates, the callee module will put the returned data in that pointer
        // 2. pass in a pointer to a pointer, the callee will set that pointer to some storage the callee has control over, with the requested data
        // the version used is dependent on the target (callee) module
        static bool get_value(uint16_t csa, void *data) { return get_value(csa, 0, 0, data); }
        static bool get_value(uint16_t csa, uint16_t csb, void *data) { return get_value(csa, csb, 0, data); }
        static bool get_value(uint16_t cs[3], void *data) { return get_value(cs[0], cs[1], cs[2], data); };
        static bool get_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data);

        static bool set_value(uint16_t csa, void *data) { return set_value(csa, 0, 0, data); }
        static bool set_value(uint16_t csa, uint16_t csb, void *data) { return set_value(csa, csb, 0, data); }
        static bool set_value(uint16_t cs[3], void *data) { return set_value(cs[0], cs[1], cs[2], data); }
        static bool set_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data);
};

#endif
