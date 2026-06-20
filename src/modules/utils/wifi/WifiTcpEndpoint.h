#ifndef WIFI_TCP_ENDPOINT_H_
#define WIFI_TCP_ENDPOINT_H_

#include <cstdint>

#include "M8266WIFIDrv.h"
#include "WifiConstants.h"
#include "libs/RingBuffer.h"
#include "libs/StreamOutput.h"
#include "CommunicationProtocol.h"

class WifiProvider;

class WifiTcpEndpoint : public StreamOutput {
public:
    WifiTcpEndpoint(WifiProvider& provider, u8 link_no);

    void configure(int port, const comms::ProtocolHandler *protocol);
    bool enabled() const { return protocol_handler != nullptr && tcp_port > 0; }
    int port() const { return tcp_port; }
    u8 link_no() const { return link; }
    bool uses_protocol(comms::Protocol protocol) const;

    bool setup(int timeout_s);
    void delete_connection();
    bool has_clients() const;

    void receive_data(const uint8_t *data, uint16_t size);
    void dispatch_line();
    void service_realtime_flags();
    void clear_file_state();
    int consume_framed_file_data(const uint8_t *data, uint16_t size, char **buf);

    int gets(char** buf, int size = 0) override;
    int printf(const char*, ...) __attribute__ ((format(printf, 2, 3)));
    int puts(const char*, int size = 0) override;
    int _putc(int c) override;
    int _getc(void) override;
    void reset_file_input() override;
    bool ready() override;
    const comms::ProtocolHandler& protocol() const override;
    int type() override { return 1; }

private:
    void receive_plain_data(const uint8_t *data, uint16_t size);
    void receive_framed_command_data(const uint8_t *data, uint16_t size);
    void apply_framed_result(const comms::FramedInputResult& result);
    void queue_framed_pending_data(const uint8_t *data, uint16_t size, bool preserve_on_reset = false);
    void clear_framed_pending_data();
    int read_pending_framed_packet(char **buf);
    bool has_char(char letter);

    WifiProvider& provider;
    const comms::ProtocolHandler *protocol_handler;
    int tcp_port;
    u8 link;
    RingBuffer<char, 256> buffer;
    uint8_t framed_pending_data[WifiDataMaxSize];
    uint16_t framed_pending_size;
    uint16_t framed_pending_index;
    uint8_t framed_command_buffer[comms::ControlPacketSize];
    comms::FramedInput framed_input;

    struct {
        volatile bool query_flag:1;
        volatile bool halt_flag:1;
        volatile bool diagnose_flag:1;
        bool preserve_framed_pending_on_reset:1;
    };
};

#endif
