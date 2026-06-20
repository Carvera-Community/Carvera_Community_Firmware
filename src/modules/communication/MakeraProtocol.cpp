#include "MakeraProtocol.h"

#include "libs/Crc16.h"
#include "libs/FirmwareFileSystem.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/StreamOutput.h"
#include "libs/utils.h"
#include "modules/robot/Conveyor.h"

#include "mbed.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace comms {

namespace {

class MakeraProtocol final : public ProtocolHandler {
public:
    Protocol kind() const override { return Protocol::Makera; }
    const char *name() const override { return "makera"; }
    bool uses_framing() const override { return true; }
    ProtocolInput *create_input(uint8_t *command_buffer, size_t command_capacity,
            uint8_t *file_buffer, size_t file_capacity) const override;
    bool decode_packet(uint8_t type, char *payload, uint16_t payload_size,
            char *raw, uint16_t raw_size, InputMessage& message) const override;
    int send_message(StreamOutput *stream, MessageType type, const char *data, int size = 0) const override;
    bool upload_file(std::string parameters, StreamOutput *stream, FileTransferHost& host) const override;
    bool download_file(std::string parameters, StreamOutput *stream, FileTransferHost& host) const override;
};

enum class UploadState {
    WaitMd5,
    WaitFileView,
    ReadFileData
};

constexpr int MaxRetrans = 50;
constexpr int RetryTicks = 50;
constexpr int TimeoutMs = 10;
constexpr int SerialPacketSize = 128;
constexpr int MaxFilePacketSize = 8192;
constexpr uint32_t TransferTimeoutUs = 29000000;
constexpr uint16_t FramedHeader = 0x8668;
constexpr uint16_t FramedFooter = 0x55aa;
constexpr uint16_t PacketTypeBytes = 3;

enum class WireMessage : uint8_t {
    StatusResponse = 0x81,
    DiagnosticResponse = 0x82,
    LoadInfo = 0x83,
    LoadFinish = 0x84,
    LoadError = 0x85,
    NormalInfo = 0x90,
    ControlSingle = 0xa1,
    ControlMulti = 0xa2,
    FileStart = 0xb0,
    FileMd5 = 0xb1,
    FileView = 0xb2,
    FileData = 0xb3,
    FileEnd = 0xb4,
    FileCancel = 0xb5,
    FileRetry = 0xb6
};

WireMessage wire_message(MessageType type)
{
    switch (type) {
        case MessageType::Status: return WireMessage::StatusResponse;
        case MessageType::Diagnostics: return WireMessage::DiagnosticResponse;
        case MessageType::LoadData: return WireMessage::LoadInfo;
        case MessageType::LoadComplete: return WireMessage::LoadFinish;
        case MessageType::LoadFailed: return WireMessage::LoadError;
        case MessageType::Text: return WireMessage::NormalInfo;
        case MessageType::TransferStart: return WireMessage::FileStart;
        case MessageType::TransferChecksum: return WireMessage::FileMd5;
        case MessageType::TransferView: return WireMessage::FileView;
        case MessageType::TransferData: return WireMessage::FileData;
        case MessageType::TransferComplete: return WireMessage::FileEnd;
        case MessageType::TransferCancel: return WireMessage::FileCancel;
        case MessageType::TransferRetry: return WireMessage::FileRetry;
    }

    return WireMessage::NormalInfo;
}

/*
Makera frame, big-endian:

| bytes | field |
| 2     | header 0x8668 |
| 2     | length: type + payload + CRC |
| 1     | message type |
| N     | payload |
| 2     | CRC16-CCITT over length, type, and payload |
| 2     | footer 0x55aa |
*/

uint16_t read_be16(const uint8_t *data)
{
    return static_cast<uint16_t>((data[0] << 8) | data[1]);
}

class MakeraProtocolInput final : public ProtocolInput {
public:
    MakeraProtocolInput(const MakeraProtocol& protocol, uint8_t *command_buffer, size_t command_capacity,
            uint8_t *file_buffer, size_t file_capacity)
        : protocol(protocol),
          command_parser(command_buffer, command_capacity),
          file_parser(file_buffer, file_capacity)
    {
        reset(command_parser);
        reset(file_parser);
    }

    void reset_command_parser() override { reset(command_parser); }
    void reset_file_parser() override { reset(file_parser); }
    bool feed_command_byte(uint8_t byte, InputMessage& message) override { return feed(command_parser, byte, message); }
    bool feed_file_byte(uint8_t byte, InputMessage& message) override { return feed(file_parser, byte, message); }

private:
    enum class State {
        WaitHeader,
        ReadLength,
        ReadData,
        CheckFooter
    };

    struct Parser {
        Parser(uint8_t *buffer, size_t capacity)
            : buffer(buffer), capacity(capacity)
        {
        }

        uint8_t *buffer;
        size_t capacity;
        State state;
        uint16_t header;
        uint16_t footer;
        uint16_t expected_length;
        uint16_t position;
        uint8_t footer_bytes;
    };

    void reset(Parser& parser)
    {
        parser.state = State::WaitHeader;
        parser.header = 0;
        parser.footer = 0;
        parser.expected_length = 0;
        parser.position = 0;
        parser.footer_bytes = 0;
    }

    bool feed(Parser& parser, uint8_t byte, InputMessage& message)
    {
        switch (parser.state) {
            case State::WaitHeader:
                parser.header = static_cast<uint16_t>((parser.header << 8) | byte);
                if (parser.header == FramedHeader) {
                    parser.state = State::ReadLength;
                    parser.position = 0;
                }
                return false;

            case State::ReadLength:
                parser.buffer[parser.position++] = byte;
                if (parser.position == 2) {
                    parser.expected_length = read_be16(parser.buffer);
                    // Length covers the message type, payload, and CRC bytes.
                    if (parser.expected_length < PacketTypeBytes
                            || static_cast<size_t>(parser.expected_length) + 2 > parser.capacity) {
                        reset(parser);
                        return false;
                    }
                    parser.state = State::ReadData;
                }
                return false;

            case State::ReadData:
                parser.buffer[parser.position++] = byte;
                if (parser.position == parser.expected_length + 2) {
                    parser.state = State::CheckFooter;
                    parser.footer = 0;
                    parser.footer_bytes = 0;
                }
                return false;

            case State::CheckFooter:
                parser.footer = static_cast<uint16_t>((parser.footer << 8) | byte);
                if (++parser.footer_bytes < 2) return false;

                if (parser.footer == FramedFooter) {
                    // CRC covers the length, message type, and payload bytes.
                    const uint16_t received_crc = read_be16(&parser.buffer[parser.position - 2]);
                    const uint16_t calculated_crc = crc16::ccitt(parser.buffer, parser.position - 2);
                    if (received_crc == calculated_crc) {
                        const uint8_t type = parser.buffer[2];
                        char *payload = reinterpret_cast<char *>(&parser.buffer[3]);
                        char *raw = reinterpret_cast<char *>(parser.buffer);
                        const uint16_t payload_size = parser.expected_length - PacketTypeBytes;
                        const uint16_t raw_size = parser.position;
                        reset(parser);
                        message = InputMessage();
                        return protocol.decode_packet(type, payload, payload_size, raw, raw_size, message);
                    }
                }

                reset(parser);
                return false;
        }

        reset(parser);
        return false;
    }

    const MakeraProtocol& protocol;
    Parser command_parser;
    Parser file_parser;
};

uint32_t read_be32(const uint8_t *data)
{
    return (static_cast<uint32_t>(data[0]) << 24)
        | (static_cast<uint32_t>(data[1]) << 16)
        | (static_cast<uint32_t>(data[2]) << 8)
        | static_cast<uint32_t>(data[3]);
}

void write_be32(uint8_t *data, uint32_t value)
{
    data[0] = (value >> 24) & 0xff;
    data[1] = (value >> 16) & 0xff;
    data[2] = (value >> 8) & 0xff;
    data[3] = value & 0xff;
}

void write_be16(uint8_t *data, uint16_t value)
{
    data[0] = (value >> 8) & 0xff;
    data[1] = value & 0xff;
}

class FileHandle {
public:
    ~FileHandle() { close(); }

    bool open(const std::string& path, const char *mode)
    {
        close();
        file = fwfs::fopen(path.c_str(), mode);
        return file != nullptr;
    }

    void close()
    {
        if (file != nullptr) {
            fwfs::fclose(file);
            file = nullptr;
        }
    }

    FILE *get() const { return file; }
    bool is_open() const { return file != nullptr; }

private:
    FILE *file = nullptr;
};

class PacketView {
public:
    explicit PacketView(char *packet)
        : raw(reinterpret_cast<uint8_t *>(packet))
    {
    }

    bool is(WireMessage expected, uint16_t min_payload = 0) const
    {
        return raw != nullptr && type() == expected && payload_size() >= min_payload;
    }

    WireMessage type() const { return static_cast<WireMessage>(raw[2]); }
    uint16_t length() const { return read_be16(raw); }
    uint16_t payload_size() const { return length() >= PacketTypeBytes ? length() - PacketTypeBytes : 0; }
    uint8_t *payload() const { return raw + PacketTypeBytes; }

private:
    uint8_t *raw;
};

struct FileViewPayload {
    uint32_t packet_count = 0;
    uint16_t packet_size = 0;
};

struct FileDataPayload {
    uint32_t sequence = 0;
    uint8_t *data = nullptr;
    uint16_t size = 0;
};

/*
Makera payloads used here:

| type | payload |
| FileMd5 | 32 MD5 text bytes |
| FileView | u32 packet_count, u16 packet_size |
| FileData | u32 sequence, then file bytes |
| ControlSingle | realtime byte, optional '1' keep-alive byte |
| ControlMulti/FileStart | command text |
*/

bool parse_file_view(PacketView packet, FileViewPayload& payload)
{
    if (!packet.is(WireMessage::FileView, 6)) return false;
    payload.packet_count = read_be32(packet.payload());
    payload.packet_size = read_be16(packet.payload() + 4);
    return true;
}

bool parse_file_data(PacketView packet, FileDataPayload& payload)
{
    if (!packet.is(WireMessage::FileData, 4)) return false;
    payload.sequence = read_be32(packet.payload());
    payload.data = packet.payload() + 4;
    payload.size = packet.payload_size() - 4;
    return true;
}

bool host_is_valid(FileTransferHost& host)
{
    return host.scratch_buffer() != nullptr && host.scratch_size() > 0;
}

void send_sequence_request(const ProtocolHandler& protocol, StreamOutput *stream, uint32_t sequence)
{
    uint8_t payload[4];
    write_be32(payload, sequence);
    protocol.send_message(stream, MessageType::TransferData, reinterpret_cast<const char *>(payload), sizeof(payload));
}

void send_file_view(const ProtocolHandler& protocol, StreamOutput *stream, uint32_t packet_count, uint16_t packet_size)
{
    uint8_t payload[6];
    write_be32(payload, packet_count);
    write_be16(payload + 4, packet_size);
    protocol.send_message(stream, MessageType::TransferView, reinterpret_cast<const char *>(payload), sizeof(payload));
}

int send_file_data(const ProtocolHandler& protocol, StreamOutput *stream, FILE *fd,
        uint32_t sequence, int packet_size, bool final_packet)
{
    uint8_t *buffer = TransferBuffer::data();
    write_be32(buffer, sequence);
    if (fwfs::fseek(fd, (sequence - 1) * packet_size, SEEK_SET) != 0) return -1;
    const int bytes_read = fwfs::fread(buffer + 4, sizeof(char), packet_size, fd);
    if (bytes_read <= 0 || (!final_packet && bytes_read != packet_size)) return -1;

    protocol.send_message(stream, MessageType::TransferData, reinterpret_cast<const char *>(buffer), bytes_read + 4);
    return bytes_read;
}

class MakeraTransfer {
public:
    MakeraTransfer(const MakeraProtocol& protocol, StreamOutput *stream, FileTransferHost& host)
        : protocol(protocol), stream(stream), host(host)
    {
        snprintf(error_msg, sizeof(error_msg), "Nothing!");
    }

protected:
    bool valid() const
    {
        return stream != nullptr && host_is_valid(host);
    }

    void begin()
    {
        serial_stream = stream->type() == 0;
        if (serial_stream) host.set_serial_rx_irq(false);
        stream->reset_file_input();
        THEKERNEL->set_uploading(true);
    }

    void restore()
    {
        enable_timers();
        stream->reset_file_input();
        if (serial_stream) host.set_serial_rx_irq(!stream->protocol().uses_framing());
        THEKERNEL->set_uploading(false);
    }

    void restore_after_error()
    {
        restore();
        THEKERNEL->set_cachewait(true);
        safe_delay_ms(1000);
        THEKERNEL->set_cachewait(false);
    }

    void disable_timers()
    {
        if (!timers_disabled) {
            NVIC_DisableIRQ(TIMER0_IRQn);
            NVIC_DisableIRQ(TIMER1_IRQn);
            timers_disabled = true;
        }
    }

    void enable_timers()
    {
        if (timers_disabled) {
            NVIC_EnableIRQ(TIMER0_IRQn);
            NVIC_EnableIRQ(TIMER1_IRQn);
            timers_disabled = false;
        }
    }

    void set_error(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        vsnprintf(error_msg, sizeof(error_msg), format, args);
        va_end(args);
    }

    void send(MessageType type, const char *data = Ok) const
    {
        protocol.send_message(stream, type, data, 0);
    }

    int read_packet(char **buffer) const
    {
        return host.read_bytes(stream, buffer, 0, TimeoutMs);
    }

    static const char Ok[];
    const MakeraProtocol& protocol;
    StreamOutput *stream;
    FileTransferHost& host;
    bool serial_stream = false;
    bool timers_disabled = false;
    char error_msg[96];
};

const char MakeraTransfer::Ok[] = "ok\r\n";

class MakeraUpload : public MakeraTransfer {
public:
    MakeraUpload(std::string parameters, const MakeraProtocol& protocol, StreamOutput *stream, FileTransferHost& host)
        : MakeraTransfer(protocol, stream, host), parameters(parameters)
    {
    }

    bool run()
    {
        if (!valid()) return true;

        prepare_paths();
        begin();

        if (!THECONVEYOR->is_idle()) {
            send(MessageType::TransferCancel);
            restore_after_error();
            return true;
        }

        if (!open_files()) {
            send(MessageType::TransferCancel);
            fail(true);
            return true;
        }

        disable_timers();
        starttime = us_ticker_read();
        if (!receive()) {
            fail(remove_partial_file);
            return true;
        }

        fd.close();
        fd_md5.close();
        remove_partial_file = false;
        enable_timers();

        if (!decompress_if_needed()) {
            fail(false);
            return true;
        }

        if (!file_end_sent) send(MessageType::TransferComplete);
        restore();
        stream->printf("Info: upload success: %s.\r\n", output_filename.c_str());
        return true;
    }

private:
    void prepare_paths()
    {
        filename = absolute_from_relative(shift_parameter(parameters));
        md5_filename = change_to_md5_path(filename);
        lz_filename = change_to_lz_path(filename);
        output_filename = filename;
        check_and_make_path(md5_filename);
        check_and_make_path(lz_filename);

        size_t start_pos = filename.find(".lz");
        if (start_pos != std::string::npos) {
            start_pos = lz_filename.rfind(".lz");
            write_filename = lz_filename.substr(0, start_pos);
        } else {
            write_filename = filename;
        }

        start_pos = md5_filename.find(".lz");
        if (start_pos != std::string::npos) md5_filename = md5_filename.substr(0, start_pos);
        needs_md5 = filename.find("firmware.bin") == std::string::npos;
    }

    bool open_files()
    {
        const bool opened_data = fd.open(write_filename, "wb");
        const bool opened_md5 = !needs_md5 || fd_md5.open(md5_filename, "wb");
        if (opened_data && opened_md5) return true;

        set_error("Error: failed to open file [%s]!\r\n",
                opened_data ? md5_filename.substr(0, 30).c_str() : write_filename.substr(0, 30).c_str());
        return false;
    }

    bool receive()
    {
        while (!success) {
            bool handled = false;
            char *raw = nullptr;
            const int command = read_packet(&raw);
            if (command > 0) {
                starttime = us_ticker_read();
                PacketView packet(raw);
                if (packet.type() == WireMessage::FileCancel) {
                    set_error("Info: Upload canceled by Controller!\r\n");
                    return false;
                }

                handled = handle_packet(packet);
                if (failed) return false;
            } else if (us_ticker_read() - starttime > TransferTimeoutUs) {
                set_error("Error: Machine received cmd timeout!\r\n");
                send(MessageType::TransferCancel);
                return false;
            }

            if (!success && !handled) retry_last_request();
            THEKERNEL->call_event(ON_IDLE);
            if (!success && total_retry > MaxRetrans) {
                set_error("Info: Machine receive file too many retry error!\r\n");
                send(MessageType::TransferCancel);
                return false;
            }
        }

        return true;
    }

    bool handle_packet(PacketView packet)
    {
        switch (state) {
            case UploadState::WaitMd5:
                return handle_md5(packet);
            case UploadState::WaitFileView:
                return handle_file_view(packet);
            case UploadState::ReadFileData:
                return handle_file_data(packet);
        }

        return false;
    }

    bool handle_md5(PacketView packet)
    {
        if (!packet.is(WireMessage::FileMd5, 32)) return false;
        if (needs_md5 && fwfs::fwrite(packet.payload(), sizeof(char), 32, fd_md5.get()) != 32) {
            set_error("Error: MD5 file write error!\r\n");
            send(MessageType::TransferCancel);
            failed = true;
            return false;
        }

        send(MessageType::TransferView);
        state = UploadState::WaitFileView;
        reset_retries();
        return true;
    }

    bool handle_file_view(PacketView packet)
    {
        FileViewPayload view;
        if (!parse_file_view(packet, view)) return false;

        total_packet = view.packet_count;
        packet_size = view.packet_size;
        declared_size = static_cast<uint64_t>(total_packet) * packet_size;
        if (total_packet == 0 || packet_size == 0 || packet_size > MaxFilePacketSize
                || declared_size > 0xffffffffUL) {
            set_error("Error: Wrong file view packet!\r\n");
            send(MessageType::TransferCancel);
            failed = true;
            return false;
        }

        sequence = 1;
        send_sequence_request(protocol, stream, sequence);
        state = UploadState::ReadFileData;
        reset_retries();
        return true;
    }

    bool handle_file_data(PacketView packet)
    {
        FileDataPayload data;
        if (!parse_file_data(packet, data) || data.sequence != sequence) return false;

        const bool final_packet = sequence == total_packet;
        const uint64_t next_file_size = static_cast<uint64_t>(file_size) + data.size;
        if (data.size == 0 || data.size > packet_size
                || (!final_packet && data.size != packet_size)
                || next_file_size > declared_size) {
            set_error("Error: Wrong data len:%d!\r\n", data.size);
            send(MessageType::TransferCancel);
            failed = true;
            return false;
        }

        setvbuf(fd.get(), reinterpret_cast<char *>(host.scratch_buffer()), _IOFBF, host.scratch_size());
        if (fwfs::fwrite(data.data, sizeof(char), data.size, fd.get()) != data.size) {
            set_error("Error: File write error!\r\n");
            send(MessageType::TransferCancel);
            failed = true;
            return false;
        }

        fflush(fd.get());
        file_size = static_cast<uint32_t>(next_file_size);
        if (!final_packet) {
            send_sequence_request(protocol, stream, ++sequence);
        } else {
            send(MessageType::TransferComplete);
            file_end_sent = true;
            success = true;
        }

        reset_retries();
        return true;
    }

    void retry_last_request()
    {
        if (++retry <= RetryTicks) return;

        switch (state) {
            case UploadState::WaitMd5:
                send(MessageType::TransferChecksum);
                break;
            case UploadState::WaitFileView:
                send(MessageType::TransferView);
                break;
            case UploadState::ReadFileData:
                send_sequence_request(protocol, stream, sequence);
                break;
        }

        retry = 0;
        ++total_retry;
    }

    void reset_retries()
    {
        retry = 0;
        total_retry = 0;
    }

    bool decompress_if_needed()
    {
        const size_t start_pos = filename.find(".lz");
        if (start_pos == std::string::npos) return true;

        output_filename = filename.substr(0, start_pos);
        if (host.decompress(write_filename, output_filename, file_size, stream)) return true;

        set_error("Error: error in decompressing file!\r\n");
        success = false;
        return false;
    }

    void fail(bool remove_files)
    {
        fd.close();
        fd_md5.close();
        if (remove_files) {
            if (!write_filename.empty()) fwfs::remove(write_filename.c_str());
            if (!md5_filename.empty()) fwfs::remove(md5_filename.c_str());
        }
        restore_after_error();
        stream->printf("%s", error_msg);
    }

    std::string parameters;
    std::string filename;
    std::string md5_filename;
    std::string lz_filename;
    std::string write_filename;
    std::string output_filename;
    FileHandle fd;
    FileHandle fd_md5;
    UploadState state = UploadState::WaitMd5;
    uint32_t file_size = 0;
    uint32_t total_packet = 0;
    uint64_t declared_size = 0;
    uint16_t packet_size = 0;
    uint32_t sequence = 0;
    uint32_t starttime = 0;
    int retry = 0;
    int total_retry = 0;
    bool needs_md5 = false;
    bool success = false;
    bool failed = false;
    bool remove_partial_file = true;
    bool file_end_sent = false;
};

class MakeraDownload : public MakeraTransfer {
public:
    MakeraDownload(std::string parameters, const MakeraProtocol& protocol, StreamOutput *stream, FileTransferHost& host)
        : MakeraTransfer(protocol, stream, host), parameters(parameters)
    {
    }

    bool run()
    {
        if (!valid()) return true;

        prepare_paths();
        begin();
        if (serial_stream) packet_size = SerialPacketSize;

        if (!THECONVEYOR->is_idle()) {
            send(MessageType::TransferCancel);
            stream->printf("error: Machine is busy.\r\n");
            restore_after_error();
            return true;
        }

        if (!open_file()) {
            send(MessageType::TransferCancel);
            fail();
            return true;
        }

        starttime = us_ticker_read();
        send(MessageType::TransferChecksum, md5);
        if (!transfer()) {
            fail();
            return true;
        }

        fd.close();
        restore();
        stream->printf("Info: download success: %s.\r\n", filename.c_str());
        return true;
    }

private:
    void prepare_paths()
    {
        filename = absolute_from_relative(shift_parameter(parameters));
        md5_filename = change_to_md5_path(filename);
        lz_filename = change_to_lz_path(filename);
        memset(md5, 0, sizeof(md5));
    }

    bool open_file()
    {
        FileHandle md5_file;
        if (md5_file.open(md5_filename, "rb")) {
            fwfs::fread(md5, sizeof(char), sizeof(md5), md5_file.get());
        } else {
            snprintf(md5, sizeof(md5), "%s", host.last_md5());
        }

        if (fd.open(lz_filename, "rb") || fd.open(filename, "rb")) return true;

        set_error("Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
        return false;
    }

    bool transfer()
    {
        while (!success) {
            char *raw = nullptr;
            retry_packet = false;
            int command = read_packet(&raw);
            if (command > 0) {
                starttime = us_ticker_read();
                if (!handle_command(command, PacketView(raw))) return false;
            } else if (us_ticker_read() - starttime > TransferTimeoutUs) {
                set_error("Error: Machine received cmd timeout!\r\n");
                send(MessageType::TransferCancel);
                return false;
            }

            if (!success && error_cmd > MaxRetrans) {
                set_error("Error: Machine received too many wrong command!\r\n");
                send(MessageType::TransferCancel);
                return false;
            }
        }

        return true;
    }

    bool handle_command(int raw_command, PacketView packet)
    {
        int command = raw_command;
        if (command == static_cast<int>(WireMessage::FileRetry)) {
            if (++retry_count > MaxRetrans) {
                set_error("Error: Machine requested too many retries!\r\n");
                send(MessageType::TransferCancel);
                return false;
            }
            command = static_cast<int>(last_message);
            retry_packet = true;
        } else {
            retry_count = 0;
        }

        switch (static_cast<WireMessage>(command)) {
            case WireMessage::FileMd5:
                send(MessageType::TransferChecksum, md5);
                last_message = WireMessage::FileMd5;
                error_cmd = 0;
                return true;

            case WireMessage::FileView:
                return send_file_view_response();

            case WireMessage::FileData:
                return send_requested_file_data(packet);

            case WireMessage::FileEnd:
                send(MessageType::TransferComplete);
                success = true;
                return true;

            case WireMessage::FileCancel:
                set_error("Info: Download canceled by Controller!\r\n");
                return false;

            default:
                ++error_cmd;
                THEKERNEL->call_event(ON_IDLE);
                return true;
        }
    }

    bool send_file_view_response()
    {
        if (fwfs::fseek(fd.get(), 0, SEEK_END) != 0) {
            set_error("Error: Machine read file error!\r\n");
            send(MessageType::TransferCancel);
            return false;
        }

        file_size = fwfs::ftell(fd.get());
        if (file_size < 0 || fwfs::fseek(fd.get(), 0, SEEK_SET) != 0) {
            set_error("Error: Machine read file error!\r\n");
            send(MessageType::TransferCancel);
            return false;
        }

        packet_count = file_size / packet_size + ((file_size % packet_size) > 0 ? 1 : 0);
        send_file_view(protocol, stream, packet_count, packet_size);
        last_message = WireMessage::FileView;
        error_cmd = 0;
        return true;
    }

    bool send_requested_file_data(PacketView packet)
    {
        if (!retry_packet) {
            FileDataPayload request;
            if (!parse_file_data(packet, request)) {
                ++error_cmd;
                return true;
            }
            file_send_sequence = request.sequence;
        }

        if (file_send_sequence == 0 || packet_count <= 0
                || file_send_sequence > static_cast<uint32_t>(packet_count)) {
            set_error("Error: Machine requested invalid file sequence!\r\n");
            send(MessageType::TransferCancel);
            return false;
        }

        if (send_file_data(protocol, stream, fd.get(), file_send_sequence, packet_size,
                    file_send_sequence == static_cast<uint32_t>(packet_count)) <= 0) {
            set_error("Error: Machine read file error!\r\n");
            send(MessageType::TransferCancel);
            return false;
        }

        last_message = WireMessage::FileData;
        error_cmd = 0;
        return true;
    }

    void fail()
    {
        fd.close();
        restore_after_error();
        stream->printf("%s", error_msg);
    }

    std::string parameters;
    std::string filename;
    std::string md5_filename;
    std::string lz_filename;
    FileHandle fd;
    long packet_count = 0;
    long file_size = 0;
    uint32_t file_send_sequence = 0;
    WireMessage last_message = WireMessage::FileMd5;
    int packet_size = MaxFilePacketSize;
    int retry_count = 0;
    int error_cmd = 0;
    uint32_t starttime = 0;
    bool retry_packet = false;
    bool success = false;
    char md5[64];
};

}

ProtocolInput *MakeraProtocol::create_input(uint8_t *command_buffer, size_t command_capacity,
        uint8_t *file_buffer, size_t file_capacity) const
{
    return new MakeraProtocolInput(*this, command_buffer, command_capacity, file_buffer, file_capacity);
}

bool MakeraProtocol::decode_packet(uint8_t type, char *payload, uint16_t payload_size,
        char *raw, uint16_t raw_size, InputMessage& message) const
{
    message = InputMessage();

    switch (static_cast<WireMessage>(type)) {
        case WireMessage::ControlSingle:
            if (payload_size == 0) return false;
            message.kind = InputMessageKind::RealtimeControl;
            message.realtime = payload[0];
            message.keep_alive = payload_size > 1 && payload[1] == '1';
            return true;

        case WireMessage::ControlMulti:
            message.kind = InputMessageKind::Command;
            message.command.assign(payload, payload_size);
            return true;

        case WireMessage::FileStart:
            message.kind = InputMessageKind::Command;
            message.starts_transfer = true;
            message.command.assign(payload, payload_size);
            return true;

        case WireMessage::FileMd5:
        case WireMessage::FileView:
        case WireMessage::FileData:
        case WireMessage::FileEnd:
        case WireMessage::FileCancel:
        case WireMessage::FileRetry:
            message.kind = InputMessageKind::FileData;
            message.raw = raw;
            message.raw_size = raw_size;
            message.raw_type = type;
            return true;

        default:
            return false;
    }
}

int MakeraProtocol::send_message(StreamOutput *stream, MessageType type, const char *data, int size) const
{
    if (stream == nullptr) return 0;

    const WireMessage wire_type = wire_message(type);
    const uint16_t payload_size = size == 0 && data != nullptr ? strlen(data) : size;
    const uint16_t packet_length = payload_size + PacketTypeBytes;
    const uint8_t header[] = {
        static_cast<uint8_t>((FramedHeader >> 8) & 0xff),
        static_cast<uint8_t>(FramedHeader & 0xff)
    };
    const uint8_t prefix[] = {
        static_cast<uint8_t>((packet_length >> 8) & 0xff),
        static_cast<uint8_t>(packet_length & 0xff),
        static_cast<uint8_t>(wire_type)
    };

    uint16_t crc = crc16::ccitt(prefix, sizeof(prefix));
    if (data != nullptr && payload_size > 0) {
        crc = crc16::ccitt_update(crc, reinterpret_cast<const uint8_t *>(data), payload_size);
    }

    const uint8_t suffix[] = {
        static_cast<uint8_t>((crc >> 8) & 0xff),
        static_cast<uint8_t>(crc & 0xff),
        static_cast<uint8_t>((FramedFooter >> 8) & 0xff),
        static_cast<uint8_t>(FramedFooter & 0xff)
    };

    int sent = 0;
    sent += stream->puts(reinterpret_cast<const char *>(header), sizeof(header));
    sent += stream->puts(reinterpret_cast<const char *>(prefix), sizeof(prefix));
    if (data != nullptr && payload_size > 0) sent += stream->puts(data, payload_size);
    sent += stream->puts(reinterpret_cast<const char *>(suffix), sizeof(suffix));
    return sent;
}

bool MakeraProtocol::upload_file(std::string parameters, StreamOutput *stream, FileTransferHost& host) const
{
    MakeraUpload upload(parameters, *this, stream, host);
    return upload.run();
}

bool MakeraProtocol::download_file(std::string parameters, StreamOutput *stream, FileTransferHost& host) const
{
    MakeraDownload download(parameters, *this, stream, host);
    return download.run();
}

const ProtocolHandler& makera_protocol()
{
    static MakeraProtocol protocol;
    return protocol;
}

}
