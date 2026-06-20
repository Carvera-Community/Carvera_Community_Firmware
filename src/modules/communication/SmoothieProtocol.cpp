#include "SmoothieProtocol.h"

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

constexpr uint8_t Soh = 0x01;
constexpr uint8_t Stx = 0x02;
constexpr uint8_t Eot = 0x04;
constexpr uint8_t Ack = 0x06;
constexpr uint8_t Nak = 0x15;
constexpr uint8_t Can = 0x16;
constexpr uint8_t CtrlZ = 0x1a;

constexpr int MaxRetrans = 10;
constexpr unsigned int TimeoutMs = 100;
constexpr int SmallBlockSize = 128;
constexpr int LargeBlockSize = 8192;

bool host_is_valid(FileTransferHost& host)
{
    return host.scratch_buffer() != nullptr && host.scratch_size() > 0;
}

bool check_packet_crc(bool use_crc, uint8_t *data, unsigned int len)
{
    if (use_crc) {
        const uint16_t calculated = crc16::ccitt(data, len);
        const uint16_t received = static_cast<uint16_t>((data[len] << 8) + data[len + 1]);
        return calculated == received;
    }

    uint8_t checksum = 0;
    for (unsigned int i = 0; i < len; ++i) {
        checksum += data[i];
    }
    return checksum == data[len];
}

class FileHandle {
public:
    ~FileHandle()
    {
        close();
    }

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

    FILE *get() const
    {
        return file;
    }

    bool is_open() const
    {
        return file != nullptr;
    }

private:
    FILE *file = nullptr;
};

class SmoothieTransfer {
public:
    SmoothieTransfer(StreamOutput *stream, FileTransferHost& host)
        : stream(stream), host(host)
    {
        error_msg[0] = '\0';
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
        THEKERNEL->set_uploading(true);
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

    int read_byte(unsigned int timeout_ms = TimeoutMs) const
    {
        return host.read_byte(stream, timeout_ms);
    }

    int read_bytes(char **buf, int size, unsigned int timeout_ms = TimeoutMs) const
    {
        return host.read_bytes(stream, buf, size, timeout_ms);
    }

    void flush_input() const
    {
        while (read_byte() >= 0) {}
    }

    void cancel_transfer() const
    {
        stream->_putc(Can);
        flush_input();
    }

    void restore(bool wait_after_error)
    {
        enable_timers();
        flush_input();
        if (serial_stream) host.set_serial_rx_irq(!stream->protocol().uses_framing());
        THEKERNEL->set_uploading(false);

        if (wait_after_error) {
            THEKERNEL->set_cachewait(true);
            safe_delay_ms(1000);
            THEKERNEL->set_cachewait(false);
        }
    }

    void set_error(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        vsnprintf(error_msg, sizeof(error_msg), format, args);
        va_end(args);
    }

    StreamOutput *stream;
    FileTransferHost& host;
    bool serial_stream = false;
    bool timers_disabled = false;
    char error_msg[96];
};

class SmoothieUpload : public SmoothieTransfer {
public:
    SmoothieUpload(std::string parameters, StreamOutput *stream, FileTransferHost& host)
        : SmoothieTransfer(stream, host),
          parameters(parameters)
    {
    }

    bool run()
    {
        if (!valid()) return true;

        prepare_paths();
        begin();

        if (!THECONVEYOR->is_idle()) {
            stream->_putc(Eot);
            restore(true);
            return true;
        }

        if (!open_files()) {
            stream->_putc(Eot);
            restore_failed(true);
            return true;
        }

        disable_timers();
        if (!receive_file()) {
            restore_failed(true);
            return true;
        }

        fd.close();
        fd_md5.close();
        flush_input();
        THEKERNEL->set_uploading(false);

        std::string output_filename = filename;
        const size_t start_pos = filename.find(".lz");
        if (start_pos != std::string::npos) {
            output_filename = filename.substr(0, start_pos);
            if (!host.decompress(write_filename, output_filename, file_size, stream)) {
                set_error("Error: error in decompressing file!\r\n");
                restore_failed(false);
                return true;
            }
        }

        restore(false);
        stream->printf("Info: upload success: %s.\r\n", output_filename.c_str());
        return true;
    }

private:
    void prepare_paths()
    {
        filename = absolute_from_relative(shift_parameter(parameters));
        md5_filename = change_to_md5_path(filename);
        lz_filename = change_to_lz_path(filename);
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
        if (start_pos != std::string::npos) {
            md5_filename = md5_filename.substr(0, start_pos);
        }
    }

    bool open_files()
    {
        const bool opened_data = fd.open(write_filename, "wb");
        const bool needs_md5 = filename.find("firmware.bin") == std::string::npos;
        const bool opened_md5 = !needs_md5 || fd_md5.open(md5_filename, "wb");
        if (opened_data && opened_md5) return true;

        set_error("Error: failed to open file [%s]!\r\n",
                opened_data ? md5_filename.substr(0, 30).c_str() : write_filename.substr(0, 30).c_str());
        return false;
    }

    bool receive_file()
    {
        uint8_t trychar = 'C';
        uint8_t packetno = 1;
        int retrans = MaxRetrans;
        bool md5_received = false;

        while (true) {
            int block_size = 0;
            bool extended_length = false;
            int start_byte = -1;

            if (!wait_for_packet_start(trychar, start_byte, block_size, extended_length)) {
                return receive_finished;
            }

            const bool use_crc = trychar == 'C';
            trychar = 0;

            if (!read_packet(start_byte, block_size, extended_length, use_crc)) {
                if (!reject_packet(retrans)) return false;
                continue;
            }

            if (accept_packet(packetno, block_size, extended_length, use_crc, md5_received)) {
                retrans = MaxRetrans + 1;
                continue;
            }

            if (fatal_error) return false;
            if (!reject_packet(retrans)) return false;
        }
    }

    bool wait_for_packet_start(uint8_t& trychar, int& start_byte, int& block_size, bool& extended_length)
    {
        int c = 0;
        for (int retry = 0; retry < MaxRetrans; ++retry) {
            if (trychar != 0) stream->_putc(trychar);

            c = read_byte();
            if (c >= 0) {
                retry = 0;
                switch (c) {
                    case Soh:
                        start_byte = c;
                        block_size = SmallBlockSize;
                        extended_length = false;
                        return true;

                    case Stx:
                        start_byte = c;
                        block_size = LargeBlockSize;
                        extended_length = true;
                        return true;

                    case Eot:
                        stream->_putc(Ack);
                        flush_input();
                        return finish_receive();

                    case Can:
                        if (read_byte() == Can) {
                            stream->_putc(Ack);
                            flush_input();
                            set_error("Info: Upload canceled by remote!\r\n");
                            return false;
                        }
                        set_error("Error: upload canceled!\r\n");
                        return false;

                    default:
                        break;
                }
            } else {
                safe_delay_ms(10);
            }
        }

        if (trychar == 'C') {
            trychar = Nak;
            return wait_for_packet_start(trychar, start_byte, block_size, extended_length);
        }

        cancel_transfer();
        set_error("Error: upload sync error! get char [%d]!\r\n", c);
        return false;
    }

    bool finish_receive()
    {
        receive_finished = true;
        return false;
    }

    bool read_packet(int start_byte, int block_size, bool extended_length, bool use_crc)
    {
        uint8_t *buffer = TransferBuffer::data();
        char *recv_buff = nullptr;
        int remaining = 1 + block_size + (use_crc ? 1 : 0) + 3 + (extended_length ? 1 : 0);
        int timeouts = MaxRetrans;
        uint8_t *write = buffer;

        *write++ = start_byte;
        while (remaining > 0) {
            const int bytes_read = read_bytes(&recv_buff, remaining);
            if (bytes_read <= 0) {
                safe_delay_ms(10);
                if (--timeouts < 0) return false;
                continue;
            }

            timeouts = MaxRetrans;
            for (int i = 0; i < bytes_read; ++i) {
                *write++ = recv_buff[i];
            }
            remaining -= bytes_read;
        }

        return true;
    }

    bool accept_packet(uint8_t& packetno, int block_size, bool extended_length, bool use_crc, bool& md5_received)
    {
        uint8_t *buffer = TransferBuffer::data();
        const int length_offset = 3;
        const int payload_offset = 4 + (extended_length ? 1 : 0);
        const int packet_length = extended_length ? ((buffer[3] << 8) | buffer[4]) : buffer[3];
        const bool sequence_ok = buffer[1] == static_cast<uint8_t>(~buffer[2]);
        const bool crc_ok = check_packet_crc(use_crc, &buffer[length_offset], block_size + 1 + (extended_length ? 1 : 0));
        if (packet_length > block_size) return false;
        const size_t payload_length = static_cast<size_t>(packet_length);

        if (!md5_received && buffer[1] == 0 && sequence_ok && crc_ok && packet_length == 32) {
            if (fd_md5.is_open()) {
                if (fwfs::fwrite(&buffer[payload_offset], sizeof(char), 32, fd_md5.get()) != 32) {
                    set_error("Error: MD5 file write error!\r\n");
                    fatal_error = true;
                    return false;
                }
            }
            THEKERNEL->call_event(ON_IDLE);
            stream->_putc(Ack);
            md5_received = true;
            return true;
        }

        if (sequence_ok && buffer[1] == packetno && crc_ok) {
            setvbuf(fd.get(), reinterpret_cast<char *>(host.scratch_buffer()), _IOFBF, host.scratch_size());
            if (fwfs::fwrite(&buffer[payload_offset], sizeof(char), payload_length, fd.get()) != payload_length) {
                set_error("Error: File write error!\r\n");
                fatal_error = true;
                return false;
            }
            file_size += packet_length;
            ++packetno;
            THEKERNEL->call_event(ON_IDLE);
            stream->_putc(Ack);
            return true;
        }

        return false;
    }

    bool reject_packet(int& retrans)
    {
        if (receive_finished) return true;

        stream->_putc(Nak);
        if (--retrans > 0) return true;

        cancel_transfer();
        set_error("Error: too many retry error!\r\n");
        return false;
    }

    void restore_failed(bool remove_partial_files)
    {
        fd.close();
        fd_md5.close();

        if (remove_partial_files) {
            if (!write_filename.empty()) fwfs::remove(write_filename.c_str());
            if (!md5_filename.empty()) fwfs::remove(md5_filename.c_str());
        }

        restore(true);
        stream->printf("%s", error_msg);
    }

    std::string parameters;
    std::string filename;
    std::string md5_filename;
    std::string lz_filename;
    std::string write_filename;
    FileHandle fd;
    FileHandle fd_md5;
    uint32_t file_size = 0;
    bool receive_finished = false;
    bool fatal_error = false;
};

class SmoothieDownload : public SmoothieTransfer {
public:
    SmoothieDownload(std::string parameters, StreamOutput *stream, FileTransferHost& host)
        : SmoothieTransfer(stream, host),
          parameters(parameters)
    {
    }

    bool run()
    {
        if (!valid()) return true;

        prepare_paths();
        begin();

        packet_size = stream->type() == 0 ? SmallBlockSize : LargeBlockSize;
        extended_length = packet_size == LargeBlockSize;

        if (!THECONVEYOR->is_idle()) {
            cancel_transfer();
            restore(true);
            return true;
        }

        if (!open_file()) {
            cancel_transfer();
            restore_failed();
            return true;
        }

        bool use_crc = false;
        if (!wait_for_receiver(use_crc) || !send_packets(use_crc)) {
            restore_failed();
            return true;
        }

        fd.close();
        restore(false);
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

    bool wait_for_receiver(bool& use_crc)
    {
        int c = 0;
        for (int retry = 0; retry < MaxRetrans; ++retry) {
            c = read_byte();
            if (c >= 0) {
                retry = 0;
                switch (c) {
                    case 'C':
                        use_crc = true;
                        return true;

                    case Nak:
                        use_crc = false;
                        return true;

                    case Can:
                        if (read_byte() == Can) {
                            stream->_putc(Ack);
                            flush_input();
                            set_error("Info: canceled by remote!\r\n");
                            return false;
                        }
                        break;

                    default:
                        break;
                }
            } else {
                safe_delay_ms(10);
            }
        }

        cancel_transfer();
        set_error("Error: download sync error! get char [%02X]!\r\n", c);
        return false;
    }

    bool send_packets(bool use_crc)
    {
        bool md5_sent = false;
        uint8_t packetno = 0;

        while (true) {
            bool file_finished = false;
            const int payload_size = fill_next_payload(md5_sent, file_finished);
            if (payload_size < 0) return false;
            if (file_finished) return send_eot();

            build_packet(packetno, payload_size, use_crc);
            if (!send_packet(use_crc)) return false;
            ++packetno;
        }
    }

    int fill_next_payload(bool& md5_sent, bool& file_finished)
    {
        uint8_t *buffer = TransferBuffer::data();
        const int payload_offset = 4 + (extended_length ? 1 : 0);

        if (!md5_sent) {
            const int payload_size = strlen(md5);
            memcpy(&buffer[payload_offset], md5, payload_size);
            md5_sent = true;
            return payload_size;
        }

        const int payload_size = fwfs::fread(&buffer[payload_offset], sizeof(char), packet_size, fd.get());
        file_finished = payload_size <= 0;
        return file_finished ? 0 : payload_size;
    }

    void build_packet(uint8_t packetno, int payload_size, bool use_crc)
    {
        uint8_t *buffer = TransferBuffer::data();
        const int payload_offset = 4 + (extended_length ? 1 : 0);
        buffer[0] = extended_length ? Stx : Soh;
        buffer[1] = packetno;
        buffer[2] = ~packetno;
        buffer[3] = extended_length ? payload_size >> 8 : payload_size;
        if (extended_length) buffer[4] = payload_size & 0xff;

        if (payload_size < packet_size) {
            memset(&buffer[payload_offset + payload_size], CtrlZ, packet_size - payload_size);
        }

        const int checksum_length = packet_size + 1 + (extended_length ? 1 : 0);
        if (use_crc) {
            const uint16_t packet_crc = crc16::ccitt(&buffer[3], checksum_length);
            buffer[packet_size + 4 + (extended_length ? 1 : 0)] = (packet_crc >> 8) & 0xff;
            buffer[packet_size + 5 + (extended_length ? 1 : 0)] = packet_crc & 0xff;
        } else {
            uint8_t checksum = 0;
            for (int i = 3; i < 3 + checksum_length; ++i) {
                checksum += buffer[i];
            }
            buffer[packet_size + 4 + (extended_length ? 1 : 0)] = checksum;
        }
    }

    bool send_packet(bool use_crc)
    {
        uint8_t *buffer = TransferBuffer::data();
        const int packet_bytes = packet_size + 5 + (extended_length ? 1 : 0) + (use_crc ? 1 : 0);
        bool resend = true;
        int c = 0;

        for (int retry = 0; retry < MaxRetrans; ++retry) {
            if (resend) {
                stream->puts(reinterpret_cast<char *>(buffer), packet_bytes);
                resend = false;
            }

            c = read_byte();
            if (c >= 0) {
                retry = 0;
                switch (c) {
                    case Ack:
                        return true;

                    case Can:
                        if (read_byte() == Can) {
                            stream->_putc(Ack);
                            flush_input();
                            set_error("Info: canceled by remote!\r\n");
                            return false;
                        }
                        break;

                    case Nak:
                        resend = true;
                        break;

                    default:
                        break;
                }
            } else {
                safe_delay_ms(500);
            }
        }

        cancel_transfer();
        set_error("Error: transmit error, char: [%d]!\r\n", c);
        return false;
    }

    bool send_eot()
    {
        int c = 0;
        for (int retry = 0; retry < MaxRetrans; ++retry) {
            stream->_putc(Eot);
            c = read_byte();
            if (c == Ack) {
                flush_input();
                return true;
            }
        }

        set_error("Error: get finish ACK error!\r\n");
        return false;
    }

    void restore_failed()
    {
        fd.close();
        restore(true);
        stream->printf("%s", error_msg);
    }

    std::string parameters;
    std::string filename;
    std::string md5_filename;
    std::string lz_filename;
    FileHandle fd;
    int packet_size = LargeBlockSize;
    bool extended_length = true;
    char md5[64];
};

class SmoothieProtocol final : public ProtocolHandler {
public:
    Protocol kind() const override
    {
        return Protocol::Smoothie;
    }

    const char *name() const override
    {
        return "smoothie";
    }

    int send_message(StreamOutput *stream, MessageType type, const char *data, int size = 0) const override
    {
        if (stream == nullptr) return 0;

        const int data_size = size == 0 && data != nullptr ? strlen(data) : size;
        switch (type) {
            case MessageType::TransferComplete:
            case MessageType::LoadComplete:
                if (data != nullptr && data_size > 0) stream->puts(data, data_size);
                return stream->_putc(Eot);

            case MessageType::TransferCancel:
            case MessageType::LoadFailed:
                if (data != nullptr && data_size > 0) stream->puts(data, data_size);
                return stream->_putc(Can);

            case MessageType::Status:
            case MessageType::Diagnostics:
            case MessageType::LoadData:
            case MessageType::Text:
            default:
                if (data == nullptr) return 0;
                return stream->puts(data, data_size);
        }
    }

    bool upload_file(std::string parameters, StreamOutput *stream, FileTransferHost& host) const override
    {
        SmoothieUpload upload(parameters, stream, host);
        return upload.run();
    }

    bool download_file(std::string parameters, StreamOutput *stream, FileTransferHost& host) const override
    {
        SmoothieDownload download(parameters, stream, host);
        return download.run();
    }
};

}

const ProtocolHandler& smoothie_protocol()
{
    static SmoothieProtocol protocol;
    return protocol;
}

}
