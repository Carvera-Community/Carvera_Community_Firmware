/*
 * USB Host CDC-ACM driver.
 * See USBHostCDC.h for API documentation.
 */

#include "USBHostCDC.h"

#include "StreamOutputPool.h"
#include "libs/Kernel.h"
#include "libs/StreamOutput.h"
#include "us_ticker_api.h"
#include "usbhost_err.h"
#include "usbhost_lpc17xx.h"

// USBHostLite keeps the host controller state in file-scope globals. CDC
// transfers must use the same descriptors that enumeration configured.
extern volatile USB_INT32U HOST_WdhIntr;
extern volatile USB_INT08U HOST_TDControlStatus;
extern volatile int gUSBConnected;
extern volatile HCED *EDCtrl;

namespace {
constexpr uint32_t CDC_PORT_RESET_TIMEOUT_MS = 500;
constexpr uint16_t CDC_CONFIGURATION_DESCRIPTOR_MAX = 128;
constexpr uint32_t CDC_BULK_TRANSFER_MAX = 128;
constexpr uint32_t CDC_ABORT_SETTLE_MS = 2;
constexpr uint32_t CDC_BAUD_RATE = 115200;
constexpr uint8_t CDC_STOP_BITS_1 = 0;
constexpr uint8_t CDC_PARITY_NONE = 0;
constexpr uint8_t CDC_DATA_BITS_8 = 8;
constexpr uint16_t CDC_CONTROL_LINE_DTR_RTS = 0x0003;
constexpr uint8_t CDC_REQUEST_TYPE_HOST_TO_INTERFACE =
    USB_HOST_TO_DEVICE | USB_REQUEST_TYPE_CLASS | USB_RECIPIENT_INTERFACE;

}  // namespace

USBHostCDC::USBHostCDC()
    : _connected(false),
      _state(CDC_XFER_IDLE),
      _cdc_iface(0),
      _data_iface(0),
      _xfer_start(0),
      _xfer_max_len(0),
      _rx_actual(0) {}

bool USBHostCDC::init(const uint32_t timeout_ms) {
  Host_Init();

  USB_INT32S rc = Host_WaitForDevice(timeout_ms);
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: timeout waiting for device (%lu ms)\n",
                               static_cast<unsigned long>(timeout_ms));
    return false;
  }

  HostPortResetStatus port_reset = {};
  rc = Host_ResetRootPort(CDC_PORT_RESET_TIMEOUT_MS, &port_reset);
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: port reset timeout port=%08lx int=%08lx waited=%lu\n",
                               static_cast<unsigned long>(port_reset.port_status),
                               static_cast<unsigned long>(port_reset.interrupt_status),
                               static_cast<unsigned long>(port_reset.waited_ms));
    return false;
  }

  rc = Host_ReadDeviceDescriptor();
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: GET_DESCRIPTOR(device,8) failed rc=%d\n", static_cast<int>(rc));
    return false;
  }

  rc = Host_SetDeviceAddress(USB_DEVICE_ADDRESS);
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: SET_ADDRESS failed rc=%d\n", static_cast<int>(rc));
    return false;
  }

  // TDBuffer is shared with the low-level host stack and is only 176 bytes.
  // Fetch enough configuration data for a normal CDC descriptor tree without
  // trusting a device-reported length that might not fit.
  uint16_t cfg_len = 0;
  rc = Host_ReadConfigurationDescriptor(CDC_CONFIGURATION_DESCRIPTOR_MAX, &cfg_len);
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: GET_DESCRIPTOR(config) failed rc=%d\n", static_cast<int>(rc));
    return false;
  }
  rc = parse_cdc_configuration(cfg_len);
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: parse_cdc_configuration failed rc=%d\n", static_cast<int>(rc));
    return false;
  }

  rc = USBH_SET_CONFIGURATION(USB_CONFIGURATION_VALUE);
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: SET_CONFIGURATION failed rc=%d\n", static_cast<int>(rc));
    return false;
  }
  Host_DelayMS(USB_SET_CONFIGURATION_SETTLE_MS);

  rc = set_line_coding();
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: SET_LINE_CODING failed rc=%d\n", static_cast<int>(rc));
    return false;
  }
  rc = set_control_line_state(CDC_CONTROL_LINE_DTR_RTS);
  if (rc != OK) {
    THEKERNEL->streams->printf("USB CDC: SET_CONTROL_LINE_STATE failed rc=%d\n", static_cast<int>(rc));
    return false;
  }

  _connected = true;
  return true;
}

USB_INT32S USBHostCDC::parse_cdc_configuration(const uint16_t fetched_len) {
  volatile USB_INT08U *desc = TDBuffer;
  const volatile USB_INT08U *end = TDBuffer + fetched_len;
  bool data_iface_found = false;
  bool bulk_in_found = false, bulk_out_found = false;

  if (DESC_TYPE(desc) != USB_DESCRIPTOR_TYPE_CONFIGURATION) return ERR_BAD_CONFIGURATION;
  desc += DESC_LENGTH(desc);

  while (desc < end) {
    const uint8_t len = DESC_LENGTH(desc);
    const uint8_t type = DESC_TYPE(desc);

    if (len == 0) break;
    if (desc + len > end) break;

    switch (type) {
      case USB_DESCRIPTOR_TYPE_INTERFACE:
        // Endpoint descriptors do not repeat their interface number, so keep
        // accepting endpoints only while walking the CDC data interface.
        if (desc[USB_INTERFACE_CLASS_OFFSET] == CDC_DATA_INTERFACE_CLASS) {
          data_iface_found = true;
          _data_iface = desc[USB_INTERFACE_NUMBER_OFFSET];
        } else {
          data_iface_found = false;
          if (desc[USB_INTERFACE_CLASS_OFFSET] == CDC_COMMUNICATION_INTERFACE_CLASS) {
            _cdc_iface = desc[USB_INTERFACE_NUMBER_OFFSET];
          }
        }
        break;

      case USB_DESCRIPTOR_TYPE_ENDPOINT:
        if (data_iface_found &&
            (desc[USB_ENDPOINT_ATTRIBUTES_OFFSET] & USB_ENDPOINT_TRANSFER_TYPE_MASK) == USB_ENDPOINT_TRANSFER_TYPE_BULK) {
          const uint8_t endpoint_address = desc[USB_ENDPOINT_ADDRESS_OFFSET];
          const uint16_t max_pkt = ReadLE16U(&desc[USB_ENDPOINT_MAX_PACKET_SIZE_OFFSET]);
          if (endpoint_address & USB_ENDPOINT_DIRECTION_IN) {
            EDBulkIn->Control =
                OHCI_ED_BULK_ENDPOINT_CONTROL(USB_DEVICE_ADDRESS, endpoint_address, OHCI_ED_DIR_IN, max_pkt);
            bulk_in_found = true;
          } else {
            EDBulkOut->Control =
                OHCI_ED_BULK_ENDPOINT_CONTROL(USB_DEVICE_ADDRESS, endpoint_address, OHCI_ED_DIR_OUT, max_pkt);
            bulk_out_found = true;
          }
        }
        break;
      default:;
    }
    desc += len;
  }

  if (bulk_in_found && bulk_out_found) return OK;
  return ERR_NO_MS_INTERFACE;
}

USB_INT32S USBHostCDC::set_line_coding() const {
  uint8_t lc[7];
  lc[0] = static_cast<uint8_t>(CDC_BAUD_RATE);
  lc[1] = static_cast<uint8_t>(CDC_BAUD_RATE >> 8);
  lc[2] = static_cast<uint8_t>(CDC_BAUD_RATE >> 16);
  lc[3] = static_cast<uint8_t>(CDC_BAUD_RATE >> 24);
  lc[4] = CDC_STOP_BITS_1;
  lc[5] = CDC_PARITY_NONE;
  lc[6] = CDC_DATA_BITS_8;
  return Host_CtrlSend(CDC_REQUEST_TYPE_HOST_TO_INTERFACE, CDC_SET_LINE_CODING, 0, _cdc_iface, sizeof(lc), lc);
}

USB_INT32S USBHostCDC::set_control_line_state(const uint16_t state) const {
  // Many CDC devices wait for the usual terminal control lines before they
  // accept traffic.
  return Host_CtrlSend(CDC_REQUEST_TYPE_HOST_TO_INTERFACE, CDC_SET_CONTROL_LINE_STATE, state, _cdc_iface, 0, nullptr);
}

void USBHostCDC::submit_bulk_td(volatile HCED *ed, const uint32_t token, const volatile uint8_t *buffer,
                                const uint32_t len) {
  if (len == 0) return;

  NVIC_DisableIRQ(USB_IRQn);
  HOST_WdhIntr = 0;
  HOST_TDControlStatus = 0;
  NVIC_EnableIRQ(USB_IRQn);

  TDHead->Control = (TD_ROUNDING | token | TD_DELAY_INT(0) | TD_CC);
  TDTail->Control = 0;
  TDHead->CurrBufPtr = reinterpret_cast<USB_INT32U>(buffer);
  TDTail->CurrBufPtr = 0;
  TDHead->Next = reinterpret_cast<USB_INT32U>(TDTail);
  TDTail->Next = 0;
  TDHead->BufEnd = reinterpret_cast<USB_INT32U>(buffer + (len - 1));
  TDTail->BufEnd = 0;

  ed->HeadTd = reinterpret_cast<USB_INT32U>(TDHead) | ((ed->HeadTd) & OHCI_ED_HEAD_TOGGLE_CARRY);
  ed->TailTd = reinterpret_cast<USB_INT32U>(TDTail);
  ed->Next = 0;

  LPC_USB->HcBulkHeadED = reinterpret_cast<USB_INT32U>(ed);
  LPC_USB->HcCommandStatus = LPC_USB->HcCommandStatus | OR_CMD_STATUS_BLF;
  LPC_USB->HcControl = LPC_USB->HcControl | OR_CONTROL_BLE;
}

// Clearing BLE does not instantly stop the host controller. Mark both
// endpoints as skipped, let one full-speed frame pass so DMA is quiet, then
// rebuild the descriptor queues with interrupts masked while touching
// ISR-shared state.
void USBHostCDC::abort_transfer() {
  NVIC_DisableIRQ(USB_IRQn);

  EDBulkIn->Control |= OHCI_ED_SKIP;
  EDBulkOut->Control |= OHCI_ED_SKIP;
  LPC_USB->HcControl &= ~OR_CONTROL_BLE;

  NVIC_EnableIRQ(USB_IRQn);

  Host_DelayMS(CDC_ABORT_SETTLE_MS);

  NVIC_DisableIRQ(USB_IRQn);

  const USB_INT32U in_toggle = EDBulkIn->HeadTd & OHCI_ED_HEAD_TOGGLE_CARRY;
  const USB_INT32U out_toggle = EDBulkOut->HeadTd & OHCI_ED_HEAD_TOGGLE_CARRY;

  Host_TDInit(TDHead);
  Host_TDInit(TDTail);

  // Empty OHCI queues are represented by HeadTd and TailTd pointing at the
  // same transfer descriptor.
  EDBulkIn->HeadTd = reinterpret_cast<USB_INT32U>(TDTail) | in_toggle;
  EDBulkIn->TailTd = reinterpret_cast<USB_INT32U>(TDTail);
  EDBulkOut->HeadTd = reinterpret_cast<USB_INT32U>(TDTail) | out_toggle;
  EDBulkOut->TailTd = reinterpret_cast<USB_INT32U>(TDTail);

  HOST_WdhIntr = 0;
  HOST_TDControlStatus = 0;

  NVIC_EnableIRQ(USB_IRQn);

  EDBulkIn->Control &= ~OHCI_ED_SKIP;
  EDBulkOut->Control &= ~OHCI_ED_SKIP;
}

bool USBHostCDC::start_send(const uint8_t *data, uint32_t len) {
  if (_state != CDC_XFER_IDLE || !_connected) return false;
  if (len == 0) return false;
  if (len > CDC_BULK_TRANSFER_MAX) len = CDC_BULK_TRANSFER_MAX;
  // The OHCI engine can DMA from AHB SRAM only, so copy caller data out of
  // normal RAM before submitting the transfer.
  for (uint32_t i = 0; i < len; i++) TDBuffer[i] = data[i];

  _state = CDC_XFER_SENDING;
  _xfer_start = us_ticker_read();

  submit_bulk_td(EDBulkOut, TD_OUT, TDBuffer, len);
  return true;
}

bool USBHostCDC::start_recv(uint32_t max_len) {
  if (_state != CDC_XFER_IDLE || !_connected) return false;
  if (max_len == 0) return false;
  if (max_len > CDC_BULK_TRANSFER_MAX) max_len = CDC_BULK_TRANSFER_MAX;

  _state = CDC_XFER_RECEIVING;
  _xfer_max_len = max_len;
  _rx_actual = 0;
  _xfer_start = us_ticker_read();

  submit_bulk_td(EDBulkIn, TD_IN, TDBuffer, max_len);
  return true;
}

// A disconnect while idle is only noticed when the next transfer is submitted
// and times out. Callers that care about quick detection should keep a small
// amount of periodic traffic flowing.
CDCXferState USBHostCDC::poll() {
  if (_state == CDC_XFER_SENDING || _state == CDC_XFER_RECEIVING) {
    if (!gUSBConnected) {
      abort_transfer();
      _connected = false;
      _state = CDC_XFER_ERROR;
      return _state;
    }

    // Take a short, consistent snapshot of the completion flags that the USB
    // interrupt handler updates.
    NVIC_DisableIRQ(USB_IRQn);
    const USB_INT32U wdh = HOST_WdhIntr;
    const USB_INT08U td_sts = HOST_TDControlStatus;
    if (wdh) HOST_WdhIntr = 0;
    NVIC_EnableIRQ(USB_IRQn);

    if (wdh) {
      if (td_sts == 0) {
        if (_state == CDC_XFER_RECEIVING) {
          // OHCI leaves CurrBufPtr just past the last byte written, except
          // that a completely full buffer is reported as zero.
          if (TDHead->CurrBufPtr == 0) {
            _rx_actual = _xfer_max_len;
          } else {
            const uint32_t raw = TDHead->CurrBufPtr - reinterpret_cast<USB_INT32U>(TDBuffer);
            _rx_actual = (raw <= _xfer_max_len) ? raw : _xfer_max_len;
          }
        }
        _state = CDC_XFER_COMPLETE;
      } else {
        _state = CDC_XFER_ERROR;
      }
    }

    if (_state == CDC_XFER_SENDING || _state == CDC_XFER_RECEIVING) {
      const uint32_t elapsed = (us_ticker_read() - _xfer_start) / 1000;
      if (elapsed >= CDC_XFER_TIMEOUT_MS) {
        abort_transfer();
        _state = CDC_XFER_ERROR;
      }
    }
  }
  return _state;
}
