#include "CANBus.h"

#include "LPC17xx.h"
#include "us_ticker_api.h"

namespace {
constexpr uint32_t tx_buffers_free_mask = (1UL << 2) | (1UL << 10) | (1UL << 18);
constexpr uint32_t tx_buffer_1_free_mask = 1UL << 2;
constexpr uint32_t tx_buffer_1_complete_mask = 1UL << 3;
constexpr uint32_t abort_tx_buffer_1_command = (1UL << 1) | (1UL << 5);

bool valid_can2_pins(PinName rd, PinName td) { return rd == P0_4 && td == P0_5; }

}  // namespace

CANBus::CANBus(PinName rd, PinName td, RxFilter rx_filter, void* rx_filter_context)
    : can(rd, td),
      controller(valid_can2_pins(rd, td) ? LPC_CAN2 : nullptr),
      rx_filter(rx_filter),
      rx_filter_context(rx_filter_context),
      rx_head(0),
      rx_tail(0),
      rx_count(0),
      rx_overflow_count(0),
      tx_count(0),
      tx_failed_count(0),
      tx_timeout_count(0),
      ready(false) {}

CANBus::~CANBus() { stop(); }

bool CANBus::start(uint32_t bitrate) {
  stop();
  if (controller == nullptr || bitrate == 0 || bitrate > 1000000) return false;

  can.reset();
  if (!can.frequency(static_cast<int>(bitrate))) return false;

  rx_head = 0;
  rx_tail = 0;
  rx_count = 0;
  rx_overflow_count = 0;
  tx_count = 0;
  tx_failed_count = 0;
  tx_timeout_count = 0;
  ready = true;
  NVIC_SetPriority(CAN_IRQn, 5);
  can.attach(this, &CANBus::on_receive);
  return true;
}

void CANBus::stop() {
  can.attach(static_cast<void (*)(void)>(nullptr));
  ready = false;
}

bool CANBus::wait_for_tx_idle(uint32_t timeout_us) const {
  const uint32_t started = us_ticker_read();
  while ((controller->SR & tx_buffers_free_mask) != tx_buffers_free_mask) {
    if (us_ticker_read() - started >= timeout_us) return false;
  }
  return true;
}

bool CANBus::wait_for_tx_complete(uint32_t timeout_us, bool& completed) const {
  const uint32_t started = us_ticker_read();
  while ((controller->SR & tx_buffer_1_free_mask) == 0) {
    if (us_ticker_read() - started >= timeout_us) return false;
  }
  completed = (controller->SR & tx_buffer_1_complete_mask) != 0;
  return true;
}

bool CANBus::abort_tx(uint32_t timeout_us) {
  controller->CMR = abort_tx_buffer_1_command;
  if (wait_for_tx_idle(timeout_us)) return true;

  can.attach(static_cast<void (*)(void)>(nullptr));
  can.reset();
  ready = false;
  return false;
}

bool CANBus::send(const mbed::CANMessage& message, uint32_t timeout_us) {
  if (!ready || message.len > 8 || message.format != CANStandard || message.type != CANData || message.id > 0x7ff ||
      controller == nullptr)
    return false;
  if (!wait_for_tx_idle(timeout_us)) {
    ++tx_timeout_count;
    return false;
  }
  if (!can.write(message)) {
    ++tx_failed_count;
    return false;
  }
  bool completed = false;
  if (!wait_for_tx_complete(timeout_us, completed)) {
    ++tx_timeout_count;
    abort_tx(timeout_us);
    return false;
  }
  if (!completed) {
    ++tx_failed_count;
    return false;
  }
  ++tx_count;
  return true;
}

bool CANBus::read(mbed::CANMessage& message) {
  const uint8_t tail = rx_tail;
  if (tail == rx_head) return false;
  __DMB();
  message = rx_queue[tail];
  rx_tail = static_cast<uint8_t>((tail + 1U) % rx_storage_size);
  return true;
}

void CANBus::on_receive() {
  mbed::CANMessage message;
  while (can.read(message)) {
    if (message.format != CANStandard || message.type != CANData || message.id > 0x7ff) {
      continue;
    }
    if (rx_filter != nullptr && !rx_filter(message, rx_filter_context)) continue;

    const uint8_t head = rx_head;
    const uint8_t next = static_cast<uint8_t>((head + 1U) % rx_storage_size);
    if (next == rx_tail) {
      ++rx_overflow_count;
      continue;
    }

    rx_queue[head] = message;
    __DMB();
    rx_head = next;
    ++rx_count;
  }
}

CANBus::Statistics CANBus::statistics() {
  return Statistics{tx_count,         rx_count,      rx_overflow_count, tx_failed_count,
                    tx_timeout_count, can.tderror(), can.rderror()};
}
