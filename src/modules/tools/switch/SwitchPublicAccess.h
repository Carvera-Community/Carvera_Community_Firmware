#ifndef __SWITCHPUBLICACCESS_H
#define __SWITCHPUBLICACCESS_H

// addresses used for public data access
#define switch_checksum              CHECKSUM("switch")
#define state_checksum               CHECKSUM("state")
#define state_value_checksum         CHECKSUM("state_value")
#define spindlefan_checksum          CHECKSUM("spindlefan")
#define vacuum_checksum              CHECKSUM("vacuum")
#define light_checksum               CHECKSUM("light")
#define toolsensor_checksum          CHECKSUM("toolsensor")
#define probecharger_checksum        CHECKSUM("probecharger")
#define air_checksum               	 CHECKSUM("air")

struct pad_switch {
    int name;
    bool state;
    float value;
};

#endif // __SWITCHPUBLICACCESS_H
