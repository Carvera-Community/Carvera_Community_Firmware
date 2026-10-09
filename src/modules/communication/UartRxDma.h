#pragma once

#include <cstdint>

namespace uart2_rx_dma {

void initialize();
bool try_get(uint8_t& byte);
bool take_error();

}  // namespace uart2_rx_dma
