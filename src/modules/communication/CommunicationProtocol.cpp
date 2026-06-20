#include "CommunicationProtocol.h"

#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "libs/compiler.h"

#include "mbed.h"

#include "SmoothieProtocol.h"

#include <cstdio>
#include <string>

namespace comms {

namespace {

uint8_t transfer_buffer_storage[MaxPacketSize] LOCATED_IN_AHBSRAM;
bool invalid_protocol_warning_printed = false;
bool unavailable_protocol_warning_printed = false;

const ProtocolHandler *handler_for(Protocol protocol)
{
    switch (protocol) {
        case Protocol::Smoothie:
            return &smoothie_protocol();
    }

    return nullptr;
}

void warn_invalid_protocol(const std::string& configured, const ProtocolHandler& fallback)
{
    if (invalid_protocol_warning_printed) return;
    if (THEKERNEL == nullptr || THEKERNEL->streams == nullptr) return;

    invalid_protocol_warning_printed = true;
    THEKERNEL->streams->printf("WARNING: unknown communication protocol %s; using %s\r\n",
            configured.c_str(), fallback.name());
}

void warn_unavailable_protocol(Protocol configured, const ProtocolHandler& fallback)
{
    if (unavailable_protocol_warning_printed) return;
    if (THEKERNEL == nullptr || THEKERNEL->streams == nullptr) return;

    unavailable_protocol_warning_printed = true;
    THEKERNEL->streams->printf("WARNING: communication protocol %s is not compiled; using %s\r\n",
            protocol_name(configured), fallback.name());
}

const ProtocolHandler& fallback_protocol()
{
    return smoothie_protocol();
}

}

uint8_t *TransferBuffer::data()
{
    return transfer_buffer_storage;
}

bool is_protocol_compiled(Protocol protocol)
{
    return handler_for(protocol) != nullptr;
}

const char *protocol_name(Protocol protocol)
{
    switch (protocol) {
        case Protocol::Smoothie:
            return "smoothie";
    }

    return "unknown";
}

bool protocol_from_name(const std::string& name, Protocol& protocol)
{
    if (name == "smoothie") {
        protocol = Protocol::Smoothie;
        return true;
    }

    return false;
}

FramedInput::FramedInput()
    : parser(nullptr)
{
}

FramedInput::~FramedInput()
{
    delete parser;
}

void FramedInput::set_protocol(const ProtocolHandler& protocol, uint8_t *command_buffer, size_t command_capacity,
        uint8_t *file_buffer, size_t file_capacity)
{
    delete parser;
    parser = protocol.create_input(command_buffer, command_capacity, file_buffer, file_capacity);
}

void FramedInput::reset_file_parser()
{
    if (parser != nullptr) parser->reset_file_parser();
}

bool FramedInput::feed_command_byte(uint8_t byte, InputMessage& message)
{
    if (parser == nullptr) return false;

    if (THEKERNEL->is_cachewait()) {
        parser->reset_command_parser();
        return false;
    }

    return parser->feed_command_byte(byte, message);
}

FramedInputResult FramedInput::feed_file_byte(StreamOutput *stream, uint8_t byte, char **buf)
{
    FramedInputResult result;
    if (parser == nullptr) return result;

    if (THEKERNEL->is_cachewait()) {
        parser->reset_file_parser();
        return result;
    }

    InputMessage message;
    if (!parser->feed_file_byte(byte, message)) return result;

    if (message.kind == InputMessageKind::FileData) {
        result.file_type = message.raw_type;
        result.file_data = message.raw;
        if (buf != nullptr) *buf = message.raw;
        return result;
    }

    return dispatch(stream, message);
}

FramedInputResult FramedInput::dispatch(StreamOutput *stream, const InputMessage& input) const
{
    FramedInputResult result;
    result.starts_transfer = input.starts_transfer;

    if (input.kind == InputMessageKind::RealtimeControl) {
        const auto realtime = THEKERNEL->handle_realtime_control(input.realtime, input.keep_alive, true);
        result.query = realtime.query;
        result.halt = realtime.halt;
        return result;
    }

    if (input.kind == InputMessageKind::Command) {
        SerialMessage message;
        message.message = input.command;
        message.stream = stream;
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    }

    return result;
}

ProtocolInput *ProtocolHandler::create_input(uint8_t *, size_t, uint8_t *, size_t) const
{
    return nullptr;
}

bool ProtocolHandler::decode_packet(uint8_t, char *, uint16_t, char *, uint16_t, InputMessage& message) const
{
    message = InputMessage();
    return false;
}

int ProtocolHandler::send_output(StreamOutput *stream, const char *data, int size) const
{
    return send_message(stream, MessageType::Text, data, size);
}

int ProtocolHandler::send_load_data(StreamOutput *stream, const char *data, int size) const
{
    return send_message(stream, MessageType::LoadData, data, size);
}

int ProtocolHandler::send_load_finish(StreamOutput *stream, const char *data) const
{
    return send_message(stream, MessageType::LoadComplete, data, 0);
}

int ProtocolHandler::send_load_error(StreamOutput *stream, const char *data) const
{
    return send_message(stream, MessageType::LoadFailed, data, 0);
}

FormattedMessage::FormattedMessage(const char *format, va_list args)
    : heap_buffer(nullptr),
      written(-1)
{
    va_list measure_args;
    va_copy(measure_args, args);
    written = vsnprintf(stack_buffer, sizeof(stack_buffer), format, measure_args);
    va_end(measure_args);
    if (written < 0 || static_cast<size_t>(written) < sizeof(stack_buffer)) return;

    heap_buffer = new char[static_cast<size_t>(written) + 1];
    va_list render_args;
    va_copy(render_args, args);
    vsnprintf(heap_buffer, static_cast<size_t>(written) + 1, format, render_args);
    va_end(render_args);
}

FormattedMessage::~FormattedMessage()
{
    delete[] heap_buffer;
}

int ProtocolHandler::vprintf_message(StreamOutput *stream, MessageType type, const char *format, va_list args) const
{
    FormattedMessage message(format, args);
    if (!message.valid()) return message.size();
    send_message(stream, type, message.data(), message.size());
    return message.size();
}

int ProtocolHandler::printf_message(StreamOutput *stream, MessageType type, const char *format, ...) const
{
    va_list args;
    va_start(args, format);
    const int sent = vprintf_message(stream, type, format, args);
    va_end(args);
    return sent;
}

const ProtocolHandler *protocol_handler(Protocol protocol)
{
    return handler_for(protocol);
}

const ProtocolHandler& default_protocol(Protocol preferred)
{
    const ProtocolHandler *handler = handler_for(preferred);
    return handler != nullptr ? *handler : fallback_protocol();
}

const ProtocolHandler& configured_protocol(uint16_t group_checksum, uint16_t key_checksum,
        Protocol fallback_protocol)
{
    const ProtocolHandler& fallback = default_protocol(fallback_protocol);
    if (THEKERNEL == nullptr || THEKERNEL->config == nullptr) return fallback;

    const std::string configured = THEKERNEL->config->value(group_checksum, key_checksum)->as_string(fallback.name());
    Protocol protocol;
    if (!protocol_from_name(configured, protocol)) {
        warn_invalid_protocol(configured, fallback);
        return fallback;
    }

    const ProtocolHandler *handler = handler_for(protocol);
    if (handler == nullptr) {
        warn_unavailable_protocol(protocol, fallback);
        return fallback;
    }

    return *handler;
}

}
