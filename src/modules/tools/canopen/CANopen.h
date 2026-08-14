#pragma once

#include <cstdint>

#include "CAN.h"
#include "CANBus.h"
#include "libs/Module.h"

class Gcode;
class StreamOutput;

struct canopen_frame {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
  uint8_t type;
  uint8_t format;
};

struct canopen_status {
  bool enabled;
  bool bus_ready;
  uint32_t bitrate;
  uint8_t node_id;
  uint8_t remote_node_id;
  uint32_t heartbeat_ms;
  CANBus::Statistics bus;
  bool have_last_rx;
  mbed::CANMessage last_rx;
  bool have_remote_heartbeat;
  bool remote_online;
  uint8_t remote_state;
  uint32_t remote_heartbeat_age_ms;
};

class CANOpenManager : public Module {
 public:
  CANOpenManager();

  void on_module_loaded() override;
  void on_main_loop(void*) override;
  void on_gcode_received(void*) override;
  void on_get_public_data(void*) override;
  void on_set_public_data(void*) override;
  void on_halt(void*) override;

 private:
  void configure();
  bool initialize_bus();
  static bool accept_message(const mbed::CANMessage& message, void* context);
  void print_status(StreamOutput* stream);
  void drain_rx();
  void handle_rx(const mbed::CANMessage& message, uint32_t now);
  void send_heartbeat(uint32_t now);
  bool send_nmt(uint8_t command, uint8_t node);
  bool addressed_node(Gcode* gcode, uint8_t& node) const;
  bool wait_for_sdo_response(uint8_t node, uint16_t index, uint8_t subindex, bool upload, uint32_t timeout_us,
                             uint32_t& value, uint8_t& size);
  bool sdo_upload(uint8_t node, uint16_t index, uint8_t subindex, uint32_t& value, uint8_t& size);
  bool sdo_download(uint8_t node, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size);
  void report_sdo_failure(StreamOutput* stream, const char* operation) const;
  bool remote_online(uint32_t now) const;

  CANBus* bus;
  bool enabled;
  bool bus_ready;
  uint32_t bitrate;
  uint8_t node_id;
  uint8_t remote_node_id;
  volatile uint8_t pending_sdo_node;
  uint32_t heartbeat_ms;
  uint32_t last_heartbeat_timestamp;
  uint32_t last_remote_heartbeat_us;
  uint32_t last_sdo_abort;
  uint8_t remote_state;
  bool have_remote_heartbeat;
  mbed::CANMessage last_rx_message;
  bool has_last_rx_message;
};
