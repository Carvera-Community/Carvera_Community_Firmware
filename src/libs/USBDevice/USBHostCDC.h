/*
 * USB Host CDC-ACM driver.
 *
 * USBHostLite setup functions wait inside Host_ProcessTD, which is acceptable
 * during enumeration but not while the machine is running. This class uses
 * them for setup, then submits bulk transfers and lets callers poll for
 * completion without blocking the main loop.
 */

#ifndef USBHOST_CDC_H
#define USBHOST_CDC_H

#include <cstdint>

#include "usbhost_inc.h"

constexpr uint8_t CDC_COMMUNICATION_INTERFACE_CLASS = 0x02;
constexpr uint8_t CDC_DATA_INTERFACE_CLASS = 0x0A;

constexpr uint8_t CDC_SET_LINE_CODING = 0x20;
constexpr uint8_t CDC_SET_CONTROL_LINE_STATE = 0x22;

constexpr uint32_t CDC_XFER_TIMEOUT_MS = 500;

enum CDCXferState { CDC_XFER_IDLE = 0, CDC_XFER_SENDING, CDC_XFER_RECEIVING, CDC_XFER_COMPLETE, CDC_XFER_ERROR };

class USBHostCDC {
 public:
  USBHostCDC();

  /** Blocking setup path: initialise the host, enumerate the device, and
   *  configure the CDC interface. The board-specific VBUS-enable pin must
   *  already be asserted.
   *  @param timeout_ms  max time to wait for device connection (0 = forever)
   *  @return true on success */
  bool init(uint32_t timeout_ms = 3000);

  /** @return true if a CDC device was enumerated successfully */
  bool connected() const { return _connected; }

  /** Submit an asynchronous bulk-OUT transfer. Fails if a transfer is
   *  already in flight or the device is not connected.
   *  @return true if the transfer was submitted */
  bool start_send(const uint8_t *data, uint32_t len);

  /** Submit an asynchronous bulk-IN transfer.
   *  @param max_len  maximum bytes to receive (capped to 128)
   *  @return true if the transfer was submitted */
  bool start_recv(uint32_t max_len);

  /** Drive the transfer state machine; call every on_idle().
   *  @return current transfer state */
  CDCXferState poll();

  /** Pointer to the receive buffer. After poll() returns CDC_XFER_COMPLETE
   *  for a recv, copy the data out before starting the next transfer. */
  volatile uint8_t *rx_buf() { return TDBuffer; }

  /** Bytes received in the last completed IN transfer. */
  uint32_t rx_len() const { return _rx_actual; }

  /** Cancel the current transfer and make the bulk endpoints usable again. */
  void abort() const {
    if (_state == CDC_XFER_SENDING || _state == CDC_XFER_RECEIVING) abort_transfer();
  }

  /** Reset the state machine to IDLE; call after consuming a response. */
  void finish() { _state = CDC_XFER_IDLE; }

  CDCXferState state() const { return _state; }

 private:
  // Only the bytes actually fetched are safe to parse; some devices report a
  // larger wTotalLength than this firmware's shared USB buffer can hold.
  USB_INT32S parse_cdc_configuration(uint16_t fetched_len);

  USB_INT32S set_line_coding() const;
  USB_INT32S set_control_line_state(uint16_t state) const;

  static void submit_bulk_td(volatile HCED *ed, uint32_t token, const volatile uint8_t *buffer, uint32_t len);
  static void abort_transfer();

  bool _connected;
  CDCXferState _state;
  uint8_t _cdc_iface;
  uint8_t _data_iface;

  uint32_t _xfer_start;
  uint32_t _xfer_max_len;
  uint32_t _rx_actual;
};

#endif  // USBHOST_CDC_H
