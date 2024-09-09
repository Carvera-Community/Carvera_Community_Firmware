/*
 * LaserPublicAccess.h
 *
 *  Created on: 2020年8月10日
 *      Author: josh
 */

#ifndef SRC_MODULES_TOOLS_LASER_LASERPUBLICACCESS_H_
#define SRC_MODULES_TOOLS_LASER_LASERPUBLICACCESS_H_

#include "checksumm.h"

#include <string>

#define laser_checksum		         CHECKSUM("laser")
#define get_laser_status_checksum    CHECKSUM("get_laser_status")

struct laser_status {
	bool mode;
	bool state;
	bool testing;
    float power;	// 0 - 100
    float scale; // 0 - 100
};

#endif /* SRC_MODULES_TOOLS_LASER_LASERPUBLICACCESS_H_ */
