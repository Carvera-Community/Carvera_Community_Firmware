#include "WifiTcpEndpoint.h"

#include "WifiProvider.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutputPool.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

WifiTcpEndpoint::WifiTcpEndpoint(WifiProvider& provider, u8 link_no)
    : provider(provider),
      protocol_handler(nullptr),
      tcp_port(0),
      link(link_no),
      framed_pending_size(0),
      framed_pending_index(0)
{
    query_flag = false;
    halt_flag = false;
    diagnose_flag = false;
    preserve_framed_pending_on_reset = false;
}

void WifiTcpEndpoint::configure(int port, const comms::ProtocolHandler *protocol)
{
    tcp_port = port;
    protocol_handler = port > 0 ? protocol : nullptr;
    clear_framed_pending_data();

    if (enabled()) {
        framed_input.set_protocol(*protocol_handler, framed_command_buffer, sizeof(framed_command_buffer),
                comms::TransferBuffer::data(), comms::TransferBuffer::capacity());
    }
}

bool WifiTcpEndpoint::uses_protocol(comms::Protocol protocol) const
{
    return enabled() && protocol_handler->kind() == protocol;
}

bool WifiTcpEndpoint::setup(int timeout_s)
{
    if (!enabled()) return true;

    u16 status = 0;
    char address[20];
    snprintf(address, sizeof(address), "192.168.4.10");

    if (M8266WIFI_SPI_Setup_Connection(2, tcp_port, address, 0, link, 3, &status) == 0) {
        THEKERNEL->streams->printf("M8266WIFI_SPI_Setup_Connection TCP port %d ERROR, status:%d, high: %d, low: %d!\n",
                tcp_port, status, int(status >> 8), int(status & 0xff));
        return false;
    }

    if (M8266WIFI_SPI_Set_TcpServer_Auto_Discon_Timeout(link, timeout_s, &status) == 0) {
        THEKERNEL->streams->printf("M8266WIFI_SPI_Set_TcpServer_Auto_Discon_Timeout port %d ERROR, status:%d, high: %d, low: %d!\n",
                tcp_port, status, int(status >> 8), int(status & 0xff));
        return false;
    }

    return true;
}

void WifiTcpEndpoint::delete_connection()
{
    if (!enabled()) return;

    u16 status = 0;
    if (M8266WIFI_SPI_Delete_Connection(link, &status) == 0) {
        THEKERNEL->streams->printf("M8266WIFI_SPI_Delete_Connection port %d ERROR, status:%d, high: %d, low: %d!\n",
                tcp_port, status, int(status >> 8), int(status & 0xff));
    }
}

bool WifiTcpEndpoint::has_clients() const
{
    if (!enabled()) return false;

    u16 status = 0;
    u8 client_num = 0;
    ClientInfo remote_clients[15];
    M8266WIFI_SPI_List_Clients_On_A_TCP_Server(link, &client_num, remote_clients, &status);
    return client_num > 0;
}

void WifiTcpEndpoint::receive_data(const uint8_t *data, uint16_t size)
{
    if (!enabled()) return;

    if (protocol().uses_framing()) {
        receive_framed_command_data(data, size);
    } else {
        receive_plain_data(data, size);
    }
}

void WifiTcpEndpoint::receive_plain_data(const uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; ++i) {
        if (THEKERNEL->is_cachewait()) continue;

        if (i < size - 1 && data[i] == '?' && data[i + 1] == '1') {
            const auto result = THEKERNEL->handle_realtime_control('?', true, false);
            if (result.query) query_flag = true;
            ++i;
            continue;
        }

        bool at_line_start = (buffer.head == buffer.tail);
        if (!at_line_start) {
            const int last_idx = buffer.prev_block_index(buffer.head);
            at_line_start = (buffer.buffer[last_idx] == '\n' || buffer.buffer[last_idx] == '\r');
        }

        const auto result = THEKERNEL->handle_realtime_control(data[i], false, at_line_start);
        if (result.handled) {
            if (result.query) query_flag = true;
            if (result.halt) halt_flag = true;
            continue;
        }

        buffer.push_back(data[i] == '\r' ? '\n' : char(data[i]));
    }
}

void WifiTcpEndpoint::receive_framed_command_data(const uint8_t *data, uint16_t size)
{
    comms::InputMessage message;
    for (uint16_t i = 0; i < size; ++i) {
        if (framed_input.feed_command_byte(data[i], message)) {
            if (message.starts_transfer) {
                queue_framed_pending_data(data + i + 1, size - i - 1, true);
                apply_framed_result(framed_input.dispatch(this, message));
                return;
            }

            apply_framed_result(framed_input.dispatch(this, message));
        }
    }
}

void WifiTcpEndpoint::dispatch_line()
{
    if (!enabled() || protocol().uses_framing() || !has_char('\n')) return;

    std::string received;
    received.reserve(20);
    while (true) {
        char c;
        buffer.pop_front(c);
        if (c == '\n') {
            SerialMessage message;
            message.message = received;
            message.stream = this;
            message.line = 0;
            THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            return;
        }

        received += c;
    }
}

void WifiTcpEndpoint::service_realtime_flags()
{
    if (!enabled()) return;

    if (query_flag) {
        query_flag = false;
        protocol().send_message(this, comms::MessageType::Status, THEKERNEL->get_query_string().c_str(), 0);
    }

    if (diagnose_flag) {
        diagnose_flag = false;
        protocol().send_message(this, comms::MessageType::Diagnostics, THEKERNEL->get_diagnose_string().c_str(), 0);
    }

    if (halt_flag) {
        halt_flag = false;
        THEKERNEL->set_halt_reason(MANUAL);
        protocol().send_message(this, comms::MessageType::Text, "ERROR: Controller Abort during cycle\r\n", 0);
        THEKERNEL->call_event(ON_HALT, nullptr);
    }
}

void WifiTcpEndpoint::apply_framed_result(const comms::FramedInputResult& result)
{
    if (result.query) query_flag = true;
    if (result.halt) halt_flag = true;
}

void WifiTcpEndpoint::queue_framed_pending_data(const uint8_t *data, uint16_t size, bool preserve_on_reset)
{
    if (size == 0) {
        clear_framed_pending_data();
        return;
    }

    if (size > sizeof(framed_pending_data)) size = sizeof(framed_pending_data);
    memcpy(framed_pending_data, data, size);
    framed_pending_size = size;
    framed_pending_index = 0;
    preserve_framed_pending_on_reset = preserve_on_reset;
}

void WifiTcpEndpoint::clear_framed_pending_data()
{
    framed_pending_size = 0;
    framed_pending_index = 0;
    preserve_framed_pending_on_reset = false;
}

int WifiTcpEndpoint::read_pending_framed_packet(char **buf)
{
    if (THEKERNEL->is_cachewait()) {
        framed_input.reset_file_parser();
        clear_framed_pending_data();
        return 0;
    }

    while (framed_pending_index < framed_pending_size) {
        const auto result = framed_input.feed_file_byte(this, framed_pending_data[framed_pending_index++], buf);
        apply_framed_result(result);
        if (result.file_type > 0) {
            if (framed_pending_index >= framed_pending_size) clear_framed_pending_data();
            return result.file_type;
        }
    }

    clear_framed_pending_data();
    return 0;
}

int WifiTcpEndpoint::consume_framed_file_data(const uint8_t *data, uint16_t size, char **buf)
{
    for (uint16_t i = 0; i < size; ++i) {
        const auto result = framed_input.feed_file_byte(this, data[i], buf);
        apply_framed_result(result);
        if (result.file_type > 0) {
            queue_framed_pending_data(data + i + 1, size - i - 1);
            return result.file_type;
        }
    }

    return 0;
}

int WifiTcpEndpoint::gets(char** buf, int size)
{
    if (!enabled()) return 0;

    if (protocol().uses_framing()) {
        const int pending_type = read_pending_framed_packet(buf);
        if (pending_type > 0) return pending_type;
        return provider.read_framed_file_for(*this, buf, size);
    }

    return provider.read_raw_for(*this, buf, size);
}

int WifiTcpEndpoint::printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const int sent = protocol().vprintf_message(this, comms::MessageType::Text, format, args);
    va_end(args);
    return sent;
}

int WifiTcpEndpoint::puts(const char* s, int size)
{
    if (!enabled()) return 0;
    return provider.send_on_link(link, s, size);
}

int WifiTcpEndpoint::_putc(int c)
{
    if (!enabled()) return 0;
    return provider.send_byte_on_link(link, c);
}

int WifiTcpEndpoint::_getc()
{
    if (!enabled()) return -1;
    return provider.read_byte_for(*this);
}

void WifiTcpEndpoint::reset_file_input()
{
    framed_input.reset_file_parser();
    if (preserve_framed_pending_on_reset) {
        preserve_framed_pending_on_reset = false;
        return;
    }
    clear_framed_pending_data();
}

void WifiTcpEndpoint::clear_file_state()
{
    framed_input.reset_file_parser();
    clear_framed_pending_data();
}

bool WifiTcpEndpoint::ready()
{
    if (framed_pending_index < framed_pending_size) return true;
    return M8266WIFI_SPI_Has_DataReceived();
}

const comms::ProtocolHandler& WifiTcpEndpoint::protocol() const
{
    return protocol_handler != nullptr ? *protocol_handler : comms::default_protocol();
}

bool WifiTcpEndpoint::has_char(char letter)
{
    int index = buffer.tail;
    while (index != buffer.head) {
        if (buffer.buffer[index] == letter) return true;
        index = buffer.next_block_index(index);
    }
    return false;
}
