#include "UartRxDma.h"

#include <cstring>

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"

namespace {

constexpr uint8_t uart2_rx_dma_channel = 1;
constexpr uint32_t uart2_rx_dma_channel_mask = 1U << uart2_rx_dma_channel;
constexpr std::size_t rx_buffer_size = 544;
constexpr std::size_t dma_buffer_size = 384;

class DmaRxRing {
 public:
  void reset() {
    head_ = 0;
    tail_ = 0;
    count_ = 0;
    overflow_ = false;
  }

  std::size_t push(const uint8_t* source, std::size_t length) {
    const std::size_t free = rx_buffer_size - count_;
    if (length > free) {
      length = free;
      overflow_ = true;
    }
    if (length == 0) return 0;

    const std::size_t first = length < rx_buffer_size - head_ ? length : rx_buffer_size - head_;
    std::memcpy(data_ + head_, source, first);
    std::memcpy(data_, source + first, length - first);
    head_ = (head_ + length) % rx_buffer_size;
    count_ += length;
    return length;
  }

  std::size_t pop(uint8_t* destination, std::size_t length) {
    if (length > count_) length = count_;
    if (length == 0) return 0;

    const std::size_t first = length < rx_buffer_size - tail_ ? length : rx_buffer_size - tail_;
    std::memcpy(destination, data_ + tail_, first);
    std::memcpy(destination + first, data_, length - first);
    tail_ = (tail_ + length) % rx_buffer_size;
    count_ -= length;
    return length;
  }

  bool take_overflow() {
    const bool overflow = overflow_;
    overflow_ = false;
    return overflow;
  }

 private:
  uint8_t data_[rx_buffer_size]{};
  std::size_t head_ = 0;
  std::size_t tail_ = 0;
  std::size_t count_ = 0;
  bool overflow_ = false;
};

struct RxStorage {
  DmaRxRing ring;
  alignas(4) uint8_t dma_buffer[dma_buffer_size];
};

alignas(4) RxStorage storage;
volatile uint32_t last_processed_position;
volatile bool rx_error;

bool dma_irq_enabled() {
  const uint32_t irq = static_cast<uint32_t>(DMA_IRQn);
  return (NVIC->ISER[irq >> 5] & (1UL << (irq & 0x1f))) != 0;
}

class DmaIrqGuard {
 public:
  DmaIrqGuard() : restore_(dma_irq_enabled()) { NVIC_DisableIRQ(DMA_IRQn); }
  ~DmaIrqGuard() {
    if (restore_) NVIC_EnableIRQ(DMA_IRQn);
  }

  DmaIrqGuard(const DmaIrqGuard&) = delete;
  DmaIrqGuard& operator=(const DmaIrqGuard&) = delete;

 private:
  bool restore_;
};

void append_locked(const uint8_t* source, std::size_t length) {
  if (storage.ring.push(source, length) != length) {
    rx_error = true;
  }
}

void capture_partial_locked() {
  const uintptr_t base = reinterpret_cast<uintptr_t>(storage.dma_buffer);
  const uintptr_t destination = LPC_GPDMACH1->DMACCDestAddr;
  if (destination < base || destination > base + dma_buffer_size) {
    rx_error = true;
    return;
  }

  const uint32_t position = static_cast<uint32_t>(destination - base);
  if (position < last_processed_position) {
    rx_error = true;
    return;
  }
  if (position != last_processed_position) {
    append_locked(storage.dma_buffer + last_processed_position, position - last_processed_position);
    last_processed_position = position;
  }
}

void setup_rx() {
  LPC_GPDMACH1->DMACCConfig = 0;
  LPC_GPDMACH1->DMACCSrcAddr = reinterpret_cast<uint32_t>(&LPC_UART2->RBR);
  LPC_GPDMACH1->DMACCDestAddr = reinterpret_cast<uint32_t>(storage.dma_buffer);
  LPC_GPDMACH1->DMACCLLI = 0;
  LPC_GPDMACH1->DMACCControl =
      GPDMA_DMACCxControl_TransferSize(dma_buffer_size) | GPDMA_DMACCxControl_DI | GPDMA_DMACCxControl_I;

  // Request inputs 8-15 are multiplexed with timer matches. Clear only the
  // UART2 RX selector bit; leave every unrelated DMA route untouched.
  LPC_SC->DMAREQSEL &= ~(1UL << (GPDMA_CONN_UART2_Rx - 8));
  LPC_GPDMACH1->DMACCConfig = GPDMA_DMACCxConfig_SrcPeripheral(GPDMA_CONN_UART2_Rx) |
                              GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_P2M) | GPDMA_DMACCxConfig_IE |
                              GPDMA_DMACCxConfig_ITC | GPDMA_DMACCxConfig_E;
}

}  // namespace

namespace uart_rx_dma {

void initialize() {
  NVIC_DisableIRQ(DMA_IRQn);
  storage.ring.reset();
  std::memset(storage.dma_buffer, 0, sizeof(storage.dma_buffer));
  last_processed_position = 0;
  rx_error = false;

  CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPDMA, ENABLE);
  LPC_GPDMACH1->DMACCConfig = 0;
  LPC_GPDMA->DMACIntTCClear = uart2_rx_dma_channel_mask;
  LPC_GPDMA->DMACIntErrClr = uart2_rx_dma_channel_mask;
  LPC_GPDMA->DMACConfig = GPDMA_DMACConfig_E;
  while ((LPC_GPDMA->DMACConfig & GPDMA_DMACConfig_E) == 0) {
  }

  UART_FIFO_CFG_Type fifo{};
  UART_FIFOConfigStructInit(&fifo);
  fifo.FIFO_DMAMode = ENABLE;
  UART_FIFOConfig(LPC_UART2, &fifo);

  setup_rx();
  NVIC_SetPriority(DMA_IRQn, 5);
  NVIC_EnableIRQ(DMA_IRQn);
}

bool try_get(uint8_t& byte) {
  DmaIrqGuard guard;
  capture_partial_locked();
  return storage.ring.pop(&byte, 1) == 1;
}

bool take_error() {
  DmaIrqGuard guard;
  const bool overflow = storage.ring.take_overflow();
  const bool error = rx_error || overflow;
  rx_error = false;
  return error;
}

}  // namespace uart_rx_dma

extern "C" void DMA_IRQHandler() {
  const uint32_t terminal_count = LPC_GPDMA->DMACIntTCStat;
  const uint32_t errors = LPC_GPDMA->DMACIntErrStat;

  if ((errors & uart2_rx_dma_channel_mask) != 0) {
    capture_partial_locked();
    LPC_GPDMA->DMACIntErrClr = uart2_rx_dma_channel_mask;
    if ((terminal_count & uart2_rx_dma_channel_mask) != 0) {
      LPC_GPDMA->DMACIntTCClear = uart2_rx_dma_channel_mask;
    }
    rx_error = true;
    last_processed_position = 0;
    setup_rx();
    return;
  }

  if ((terminal_count & uart2_rx_dma_channel_mask) != 0) {
    LPC_GPDMA->DMACIntTCClear = uart2_rx_dma_channel_mask;
    append_locked(storage.dma_buffer + last_processed_position, dma_buffer_size - last_processed_position);
    last_processed_position = 0;
    setup_rx();
  }
}
