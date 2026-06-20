#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <string>

class StreamOutput;

namespace comms {

constexpr size_t MaxPacketSize = 8208;
constexpr size_t ControlPacketSize = 544;

enum class Protocol {
    Smoothie,
    Makera
};

enum class MessageType : uint8_t {
    Status,
    Diagnostics,
    LoadData,
    LoadComplete,
    LoadFailed,
    Text,
    TransferStart,
    TransferChecksum,
    TransferView,
    TransferData,
    TransferComplete,
    TransferCancel,
    TransferRetry
};

enum class InputMessageKind {
    Ignore,
    RealtimeControl,
    Command,
    FileData
};

bool is_protocol_compiled(Protocol protocol);
const char *protocol_name(Protocol protocol);
bool protocol_from_name(const std::string& name, Protocol& protocol);

class FileTransferHost {
public:
    virtual ~FileTransferHost() = default;

    virtual int read_byte(StreamOutput *stream, unsigned int timeout_ms) = 0;
    virtual int read_bytes(StreamOutput *stream, char **buf, int size, unsigned int timeout_ms) = 0;
    virtual void set_serial_rx_irq(bool enable) = 0;
    virtual int decompress(const std::string& source, const std::string& destination,
            uint32_t source_size, StreamOutput *stream) = 0;
    virtual const char *last_md5() const = 0;
    virtual uint8_t *scratch_buffer() = 0;
    virtual size_t scratch_size() const = 0;
};

class TransferBuffer {
public:
    static uint8_t *data();
    static constexpr size_t capacity() { return MaxPacketSize; }
};

struct InputMessage {
    InputMessageKind kind = InputMessageKind::Ignore;
    char realtime = 0;
    bool keep_alive = false;
    bool starts_transfer = false;
    std::string command;
    char *raw = nullptr;
    uint16_t raw_size = 0;
    uint8_t raw_type = 0;
};

class FormattedMessage {
public:
    FormattedMessage(const char *format, va_list args);
    ~FormattedMessage();

    FormattedMessage(const FormattedMessage&) = delete;
    FormattedMessage& operator=(const FormattedMessage&) = delete;

    bool valid() const { return written >= 0; }
    const char *data() const { return heap_buffer != nullptr ? heap_buffer : stack_buffer; }
    int size() const { return written; }

private:
    char stack_buffer[64];
    char *heap_buffer;
    int written;
};

class ProtocolHandler;

class ProtocolInput {
public:
    virtual ~ProtocolInput() = default;

    virtual void reset_command_parser() = 0;
    virtual void reset_file_parser() = 0;
    virtual bool feed_command_byte(uint8_t byte, InputMessage& message) = 0;
    virtual bool feed_file_byte(uint8_t byte, InputMessage& message) = 0;
};

struct FramedInputResult {
    bool query = false;
    bool halt = false;
    bool starts_transfer = false;
    int file_type = 0;
    char *file_data = nullptr;
};

class FramedInput {
public:
    FramedInput();
    ~FramedInput();

    FramedInput(const FramedInput&) = delete;
    FramedInput& operator=(const FramedInput&) = delete;

    void set_protocol(const ProtocolHandler& protocol, uint8_t *command_buffer, size_t command_capacity,
            uint8_t *file_buffer, size_t file_capacity);
    void reset_file_parser();
    bool feed_command_byte(uint8_t byte, InputMessage& message);
    FramedInputResult feed_file_byte(StreamOutput *stream, uint8_t byte, char **buf);
    FramedInputResult dispatch(StreamOutput *stream, const InputMessage& input) const;

private:
    ProtocolInput *parser;
};

class ProtocolHandler {
public:
    virtual ~ProtocolHandler() = default;

    virtual Protocol kind() const = 0;
    virtual const char *name() const = 0;
    virtual bool uses_framing() const { return false; }
    virtual ProtocolInput *create_input(uint8_t *command_buffer, size_t command_capacity,
            uint8_t *file_buffer, size_t file_capacity) const;
    virtual bool decode_packet(uint8_t type, char *payload, uint16_t payload_size,
            char *raw, uint16_t raw_size, InputMessage& message) const;
    virtual int send_message(StreamOutput *stream, MessageType type, const char *data, int size = 0) const = 0;
    virtual bool upload_file(std::string parameters, StreamOutput *stream, FileTransferHost& host) const = 0;
    virtual bool download_file(std::string parameters, StreamOutput *stream, FileTransferHost& host) const = 0;

    int send_output(StreamOutput *stream, const char *data, int size = 0) const;
    int send_load_data(StreamOutput *stream, const char *data, int size = 0) const;
    int send_load_finish(StreamOutput *stream, const char *data = "ok\r\n") const;
    int send_load_error(StreamOutput *stream, const char *data = "ok\r\n") const;
    int vprintf_message(StreamOutput *stream, MessageType type, const char *format, va_list args) const;
    int printf_message(StreamOutput *stream, MessageType type, const char *format, ...) const __attribute__((format(printf, 4, 5)));
};

const ProtocolHandler *protocol_handler(Protocol protocol);
const ProtocolHandler& default_protocol(Protocol preferred = Protocol::Smoothie);
const ProtocolHandler& configured_protocol(uint16_t group_checksum, uint16_t key_checksum,
        Protocol fallback_protocol);

}

#endif
