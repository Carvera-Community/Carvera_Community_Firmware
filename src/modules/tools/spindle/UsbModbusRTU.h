/*
 * Modbus RTU master over the USB host CDC port.
 *
 * A single USB adapter can interface with one or more Modbus slave devices. The serial
 * settings are chosen by the caller. This class sends one frame at a time
 * and is polled from on_idle(); it does not block the motion loop.
 */

#ifndef USB_MODBUS_RTU_H
#define USB_MODBUS_RTU_H

#include <cstdint>

#include "USBHostCDC.h"

class UsbModbusRTU {
 public:
  void set_serial(uint32_t baud, uint8_t stop_bits, uint8_t parity);
  void set_slave(uint8_t slave) { slave_id = slave; }

  bool init(uint32_t timeout_ms);
  // Change baud after init() without enumerating the adapter again.
  bool use_baud(uint32_t baud);
  bool connected() const { return usb.connected(); }

  bool read_holding(uint16_t reg, uint16_t count);
  bool write_register(uint16_t reg, uint16_t value);

  void poll();
  void abort();
  bool busy() const { return state == Sending || state == Receiving; }
  bool succeeded() const { return state == Done; }
  bool failed() const { return state == Failed; }
  void consume() { state = Idle; }

  const uint8_t *response() const { return rx; }
  uint8_t response_length() const { return rx_len; }

 private:
  enum State { Idle, Sending, Receiving, Done, Failed };

  bool start(const uint8_t *adu, uint8_t adu_len);
  uint32_t frame_gap_us() const;
  int expected_length() const;

  USBHostCDC usb{};
  uint32_t baud = 19200;
  uint8_t stop_bits = 0;
  uint8_t parity = 2;
  uint8_t slave_id = 1;
  State state = Idle;
  uint8_t tx[16]{};
  uint8_t tx_len = 0;
  uint8_t rx[32]{};
  uint8_t rx_len = 0;
  uint32_t rx_start_us = 0;
  uint32_t last_done_us = 0;
};

#endif
