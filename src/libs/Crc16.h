#ifndef CRC16_H
#define CRC16_H

#include <cstddef>
#include <cstdint>

namespace crc16 {

uint16_t ccitt_update(uint16_t crc, const uint8_t *data, size_t len);
uint16_t ccitt(const uint8_t *data, size_t len);

}

#endif
