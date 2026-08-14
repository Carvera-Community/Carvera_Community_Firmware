#pragma once

#include <cstdint>

namespace uart_rx_dma {

void initialize();
bool try_get(uint8_t& byte);
bool take_error();

}  // namespace uart_rx_dma
