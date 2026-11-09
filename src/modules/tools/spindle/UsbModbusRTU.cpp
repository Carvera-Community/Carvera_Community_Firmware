#include "UsbModbusRTU.h"

#include "us_ticker_api.h"

namespace {
constexpr uint32_t RESPONSE_TIMEOUT_US = 200000;

uint16_t modbus_crc(const uint8_t *data, const uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}
}  // namespace

void UsbModbusRTU::set_serial(const uint32_t baud, const uint8_t stop_bits, const uint8_t parity) {
  this->baud = baud;
  this->stop_bits = stop_bits;
  this->parity = parity;
  usb.set_serial(baud, stop_bits, parity, 8);
}

bool UsbModbusRTU::use_baud(const uint32_t baud) {
  abort();
  this->baud = baud;
  usb.set_serial(baud, stop_bits, parity, 8);
  return usb.apply_serial();
}

bool UsbModbusRTU::init(const uint32_t timeout_ms) { return usb.init(timeout_ms); }

bool UsbModbusRTU::read_holding(const uint16_t reg, const uint16_t count) {
  const uint8_t adu[6] = {
      slave_id, 0x03, static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg),
      static_cast<uint8_t>(count >> 8), static_cast<uint8_t>(count)};
  return start(adu, sizeof adu);
}

bool UsbModbusRTU::write_register(const uint16_t reg, const uint16_t value) {
  const uint8_t adu[6] = {
      slave_id, 0x06, static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg),
      static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  return start(adu, sizeof adu);
}

uint32_t UsbModbusRTU::frame_gap_us() const {
  // 3.5 character times at 11 bits/character. Above 19200 the spec fixes the gap at 1.75 ms.
  if (baud > 19200) return 1750;
  const uint32_t gap = (38500000u + baud - 1u) / baud;
  return gap < 1750u ? 1750u : gap;
}

bool UsbModbusRTU::start(const uint8_t *adu, const uint8_t adu_len) {
  if (state == Sending || state == Receiving) return false;
  if (static_cast<uint32_t>(us_ticker_read() - last_done_us) < frame_gap_us()) return false;
  if (static_cast<size_t>(adu_len) + 2 > sizeof tx) return false;

  for (uint8_t i = 0; i < adu_len; i++) tx[i] = adu[i];
  const uint16_t crc = modbus_crc(tx, adu_len);
  tx[adu_len] = static_cast<uint8_t>(crc);
  tx[adu_len + 1] = static_cast<uint8_t>(crc >> 8);
  tx_len = static_cast<uint8_t>(adu_len + 2);
  rx_len = 0;

  if (!usb.start_send(tx, tx_len)) return false;
  state = Sending;
  return true;
}

int UsbModbusRTU::expected_length() const {
  if (rx_len < 2) return 0;
  const uint8_t func = rx[1];
  if (func & 0x80) return 5;
  if (func == 0x06) return 8;
  if (func == 0x03 || func == 0x04) {
    if (rx_len < 3) return 0;
    const int total = 3 + rx[2] + 2;
    if (rx[2] > 16 || total > static_cast<int>(sizeof rx)) return -1;
    return total;
  }
  return -1;
}

void UsbModbusRTU::abort() {
  if (state == Sending || state == Receiving) {
    usb.abort();
    usb.finish();
  }
  state = Idle;
  last_done_us = us_ticker_read();
}

void UsbModbusRTU::poll() {
  if (state != Sending && state != Receiving) return;

  if (!usb.connected()) {
    if (state == Sending || state == Receiving) {
      usb.abort();
      usb.finish();
    }
    state = Failed;
    last_done_us = us_ticker_read();
    return;
  }

  const CDCXferState xfer = usb.poll();
  if (state == Sending) {
    if (xfer == CDC_XFER_COMPLETE) {
      usb.finish();
      rx_len = 0;
      rx_start_us = us_ticker_read();
      if (!usb.start_recv(sizeof rx)) {
        state = Failed;
        last_done_us = rx_start_us;
      } else {
        state = Receiving;
      }
    } else if (xfer == CDC_XFER_ERROR) {
      usb.finish();
      state = Failed;
      last_done_us = us_ticker_read();
    }
    return;
  }

  if (xfer == CDC_XFER_COMPLETE) {
    const uint32_t n = usb.rx_len();
    volatile uint8_t *buf = usb.rx_buf();
    for (uint32_t i = 0; i < n && rx_len < sizeof rx; i++) rx[rx_len++] = buf[i];
    usb.finish();

    const int need = expected_length();
    if (need < 0 || rx_len > sizeof rx) {
      state = Failed;
      last_done_us = us_ticker_read();
      return;
    }
    if (need == 0 || rx_len < need) {
      // A completed short packet is not a finished frame. Give up on the same
      // timer as a transfer that never completes.
      if (static_cast<uint32_t>(us_ticker_read() - rx_start_us) >= RESPONSE_TIMEOUT_US) {
        state = Failed;
        last_done_us = us_ticker_read();
        return;
      }
      if (!usb.start_recv(sizeof rx)) {
        state = Failed;
        last_done_us = us_ticker_read();
      }
      return;
    }
    if (rx_len != need || modbus_crc(rx, static_cast<uint8_t>(need - 2)) !=
                              (static_cast<uint16_t>(rx[need - 2]) | (static_cast<uint16_t>(rx[need - 1]) << 8))) {
      state = Failed;
      last_done_us = us_ticker_read();
      return;
    }
    state = Done;
    last_done_us = us_ticker_read();
  } else if (xfer == CDC_XFER_ERROR ||
             static_cast<uint32_t>(us_ticker_read() - rx_start_us) >= RESPONSE_TIMEOUT_US) {
    usb.abort();
    usb.finish();
    state = Failed;
    last_done_us = us_ticker_read();
  }
}
