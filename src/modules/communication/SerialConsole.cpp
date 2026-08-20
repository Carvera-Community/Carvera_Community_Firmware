/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <string.h>
#include <stdarg.h>
using std::string;
#include "mbed.h" // for us_ticker_read()
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/CRC16.h"
#include "libs/MakeraControl.h"
#include "libs/MakeraFrame.h"
#include "libs/nuts_bolts.h"
#include "SerialConsole.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "ATCHandlerPublicAccess.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "libs/Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#if defined(SERIAL_RX_DMA)
#include "UartRxDma.h"
#endif
#if defined(MACHINE_FAMILY_Z1)
#include "version.h"
#endif

#define uart_checksum CHECKSUM("uart")
#define XBUFF_LENGTH 8208

extern unsigned char xbuff[XBUFF_LENGTH];

static makera::Packet makera_packet;
static RingBuffer<char, 1024> makera_rx_bytes;
// Let a back-to-back burst finish before command handlers reply on the same UART.
constexpr uint32_t makera_rx_quiet_ms = 2;
constexpr int uart_rx_error = -2;
#if defined(MACHINE_FAMILY_Z1)
static uint32_t last_version_us;
constexpr uint32_t version_interval_us = 5 * 1000 * 1000;
#endif

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole( PinName tx_pin, PinName rx_pin, int baud_rate )
    : makera_frame_decoder(makera_packet) {
    this->serial = new mbed::Serial( tx_pin, rx_pin );
    this->serial->baud(baud_rate);
    this->previous_char = 0;
    this->current_baud_rate = baud_rate;
    this->default_baud_rate = baud_rate;
    this->temp_baud_rate = 0;
    this->last_activity_ms = 0;
    this->makera_rx_overflow = false;
    this->command_waiting = false;
    this->makera_frame_decoder.reset();
    makera_rx_bytes.tail = makera_rx_bytes.head;
    this->reset_file_parser();
#if defined(SERIAL_RX_DMA)
    this->rx_dispatch_enabled = false;
    this->rx_lookahead = -1;
    this->serial->attach(nullptr, mbed::Serial::RxIrq);
    uart_rx_dma::initialize();
#endif
}

SerialConsole::~SerialConsole(){
    delete this->serial;
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    // We want to be called every time a new char is received
    query_flag = false;
    halt_flag = false;
    diagnose_flag = false;
    makera_file_cancel = false;

#if defined(MACHINE_FAMILY_CARVERA)
    default_baud_rate = THEKERNEL->config->value(uart_checksum, baud_rate_setting_checksum)->as_number(current_baud_rate);
    if (default_baud_rate != current_baud_rate) {
        this->serial->baud(default_baud_rate);
        this->current_baud_rate = default_baud_rate;
    }
#endif

    this->set_rx_enabled(true);

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams->append_stream(this);
}

void SerialConsole::set_baud_temporary(int new_baud) {
    this->temp_baud_rate = new_baud;
    this->current_baud_rate = new_baud;
    this->serial->baud(new_baud);
    this->last_activity_ms = us_ticker_read() / 1000;
}

void SerialConsole::set_rx_enabled(bool enabled) {
#if defined(SERIAL_RX_DMA)
    this->rx_dispatch_enabled = enabled;
#else
	if (enabled) {
	    this->serial->attach(this, &SerialConsole::on_serial_char_received, mbed::Serial::RxIrq);
	} else {
	    this->serial->attach(nullptr, mbed::Serial::RxIrq);
	}
#endif
}

void SerialConsole::on_set_public_data(void *argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_serial_rx_irq_checksum)) {
        bool enable_irq = *static_cast<bool *>(pdr->get_data_ptr());
        this->set_rx_enabled(enable_irq);
        pdr->set_taken();
    }
}


// Drain bytes supplied by the active IRQ- or DMA-backed transport.
void SerialConsole::on_serial_char_received() {
	int received_byte;
	while ((received_byte = this->read_byte()) >= 0) {
		char received = static_cast<char>(received_byte);
		last_activity_ms = us_ticker_read() / 1000;

		if(THEKERNEL->is_cachewait()) {
			continue;
		}

        if (communication_protocol == PROTOCOL_MAKERA) {
            const int next = makera_rx_bytes.next_block_index(makera_rx_bytes.head);
            if (next == makera_rx_bytes.tail) {
                makera_rx_overflow = true;
            } else {
                makera_rx_bytes.push_back(received);
            }
            continue;
        }
		
		if (received == '?') {
			query_flag = true;
			continue;
		} else if (this->previous_char == '?' && received == '1') {
			// Found ?1 pattern
			query_flag = true;
			THEKERNEL->set_keep_alive_request(true);
			continue;
		}
		
		//if (received == '*') {
		//	diagnose_flag = true;
		//	continue;
		//}
		if (received == 'X'-'A'+1) { // ^X
			halt_flag = true;
			continue;
		}
        if(received == 'Y' - 'A' + 1) { // ^Y
            if(THEKERNEL->get_internal_stop_request()) {
                THEKERNEL->set_internal_stop_request(false);
            } else {
                THEKERNEL->set_stop_request(true); // generic stop what you are doing request
                THEKERNEL->set_stop_request_time(us_ticker_read() / 1000);
            }
            continue;
        }
        if(received == 'Z' - 'A' + 1) { // ^Z
            THEKERNEL->set_keep_alive_request(true);
            continue;
        }
        if(THEKERNEL->is_feed_hold_enabled()) {
            bool at_line_start = (this->buffer.head == this->buffer.tail) || (this->previous_char == '\n') || (this->previous_char == '\r');
            if(at_line_start) {
                if(received == '!') { // safe pause
                    THEKERNEL->set_feed_hold(true);
                    continue;
                }
                if(received == '~') { // safe resume
                    THEKERNEL->set_feed_hold(false);
                    continue;
                }
            }
        }
		// convert CR to NL (for host OSs that don't send NL)
		if ( received == '\r' ) { received = '\n'; }
		this->buffer.push_back(received);

        // Reset previous_char for any other character
		this->previous_char = received;
    }
}

void SerialConsole::on_idle(void * argument)
{
#if defined(SERIAL_RX_DMA)
    if (!rx_dispatch_enabled) handle_rx_error();
#endif
	if (THEKERNEL->is_uploading()) return;

#if defined(SERIAL_RX_DMA)
    if (rx_dispatch_enabled) on_serial_char_received();
#endif

    const uint32_t now_ms = us_ticker_read() / 1000;
    if (communication_protocol == PROTOCOL_MAKERA && !command_waiting &&
        now_ms - last_activity_ms >= makera_rx_quiet_ms) {
        while (!command_waiting && makera_rx_bytes.tail != makera_rx_bytes.head) {
            char received;
            makera_rx_bytes.pop_front(received);
            process_makera_byte(static_cast<uint8_t>(received));
            if (THEKERNEL->is_uploading()) break;
        }
    }

#if defined(MACHINE_FAMILY_CARVERA)
    if (temp_baud_rate != 0) {
        if ((now_ms - last_activity_ms) >= 15000) {
            this->serial->baud(default_baud_rate);
            this->current_baud_rate = default_baud_rate;
            this->temp_baud_rate = 0;
        }
    }
#endif

    if (makera_rx_overflow) {
        makera_rx_overflow = false;
        PacketMessage(PTYPE_NORMAL_INFO, "ERROR: serial receive buffer full\r\n", 0);
    }

    if (makera_file_cancel) {
        makera_file_cancel = false;
        static const char cancel_payload[] = "ok\r\n";
        PacketMessage(PTYPE_FILE_CAN, cancel_payload, sizeof(cancel_payload));
    }

    if (query_flag ) {
        query_flag = false;
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
            puts(THEKERNEL->get_query_string().c_str(), 0);
        } else {
            PacketMessage(PTYPE_STATUS_RES, THEKERNEL->get_query_string().c_str(), 0);
        }
    }

    if (diagnose_flag) {
    	diagnose_flag = false;
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
			puts(THEKERNEL->get_diagnose_string().c_str(), 0);
        } else {
            PacketMessage(PTYPE_DIAG_RES, THEKERNEL->get_diagnose_string().c_str(), 0);
        }
    }

    if (halt_flag) {
        halt_flag= false;
        THEKERNEL->set_halt_reason(MANUAL);
        
        if (communication_protocol == PROTOCOL_MAKERA) {
            PacketMessage(PTYPE_NORMAL_INFO, "ERROR: Abort during cycle\r\n", 0);
        } else if(THEKERNEL->is_grbl_mode()) {
            puts("ERROR: Abort during cycle\r\n", 0);
        } else {
            puts("ERROR: Abort during cycle\r\nM999 or $X to exit HALT state\r\n", 0);
        }
        THEKERNEL->call_event(ON_HALT, nullptr);
    }

#if defined(MACHINE_FAMILY_Z1)
    const uint32_t now_us = us_ticker_read();
    if (now_us - last_version_us > version_interval_us) {
        Version version;
        PacketMessage(PTYPE_FIRM_VER, version.get_build(), 0);
        last_version_us = now_us;
    }
#endif
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument){
    if (communication_protocol == PROTOCOL_MAKERA) {
        if (command_waiting && !THEKERNEL->is_dispatching_console_line()) {
            const makera::Packet &packet = makera_frame_decoder.packet();
            struct SerialMessage message;
            message.message.assign(reinterpret_cast<const char *>(packet.data), packet.data_length);
            message.stream = this;
            message.line = 0;

            command_waiting = false;
            THEKERNEL->dispatch_console_line(message);
        }
        return;
    }

    if ( this->has_char('\n') ){
        string received;
        received.reserve(20);
        while(1){
           char c;
           this->buffer.pop_front(c);
           if( c == '\n' ){
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                message.line = 0;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                // this->puts(received.c_str());
                return;
            }else{
                received += c;
            }
        }
    }
}

int SerialConsole::puts(const char* s, int size)
{
    size_t n = size == 0 ? strlen(s) : size;
    for (size_t i = 0; i < n; ++i) {
        this->_putc(s[i]);
    }
    return n;
}

int SerialConsole::gets(char** buf, int size)
{
	if (communication_protocol == PROTOCOL_MAKERA) {
        while (makera_rx_bytes.tail != makera_rx_bytes.head || this->ready()) {
            uint8_t received;
            if (makera_rx_bytes.tail != makera_rx_bytes.head) {
                char buffered;
                makera_rx_bytes.pop_front(buffered);
                received = static_cast<uint8_t>(buffered);
            } else {
                const int received_byte = this->read_byte();
                if (received_byte < 0) break;
                received = static_cast<uint8_t>(received_byte);
            }
            uint16_t checksum;

            switch (file_parse_state) {
                case FILE_WAIT_HEADER:
                    file_header[0] = file_header[1];
                    file_header[1] = received;
                    if (((file_header[0] << 8) | file_header[1]) == HEADER) {
                        file_parse_state = FILE_READ_LENGTH;
                        file_bytes_needed = 2;
                        file_frame_index = 0;
                    }
                    break;

                case FILE_READ_LENGTH:
                    xbuff[file_frame_index++] = received;
                    if (--file_bytes_needed == 0) {
                        uint16_t expected_length = (xbuff[0] << 8) | xbuff[1];
                        if (expected_length >= 3 && expected_length <= XBUFF_LENGTH - 2) {
                            file_parse_state = FILE_READ_DATA;
                            file_bytes_needed = expected_length;
                        } else {
                            reset_file_parser();
                        }
                    }
                    break;

                case FILE_READ_DATA:
                    xbuff[file_frame_index++] = received;
                    if (--file_bytes_needed == 0) {
                        file_parse_state = FILE_CHECK_FOOTER;
                        file_bytes_needed = 2;
                    }
                    break;

                case FILE_CHECK_FOOTER:
                    file_footer[0] = file_footer[1];
                    file_footer[1] = received;
                    if (--file_bytes_needed == 0) {
                        checksum = (file_footer[0] << 8) | file_footer[1];
                        int command = checksum == FOOTER ? check_file_packet(buf) : 0;
                        reset_file_parser();
                        if (command != 0) return command;
                    }
                    break;
            }
        }
        return 0;
    }

	getc_result = this->_getc();
	*buf = &getc_result;
	return 1;
}

void SerialConsole::process_makera_byte(uint8_t received)
{
    const makera::DecodeResult result = makera_frame_decoder.decode_byte(received, last_activity_ms);
    if (result != makera::DecodeResult::complete) return;

    const makera::Packet &packet = makera_frame_decoder.packet();
    if (packet.type == PTYPE_CTRL_SINGLE && packet.data_length > 0) {
        switch (makera::handle_control(packet.data[0])) {
            case makera::ControlAction::query: query_flag = true; break;
            case makera::ControlAction::diagnose: diagnose_flag = true; break;
            case makera::ControlAction::halt: halt_flag = true; break;
            default: break;
        }
        return;
    }

    if (packet.type == PTYPE_CTRL_MULTI && makera::is_diagnostic_request(packet.data, packet.data_length)) {
        diagnose_flag = true;
        return;
    }

    if (packet.type == PTYPE_CTRL_MULTI || packet.type == PTYPE_FILE_START) {
        if (packet.data_length == 0) {
            if (packet.type == PTYPE_FILE_START) makera_file_cancel = true;
            return;
        }
        struct SerialMessage message;
        message.message.assign(reinterpret_cast<const char *>(packet.data), packet.data_length);
        message.stream = this;
        message.line = 0;
        if (!THEKERNEL->dispatch_console_line(message)) command_waiting = true;
#if defined(STREAMED_JOB_PLAYBACK)
    } else if (packet.type >= PTYPE_PLAY_VIEW && packet.type <= PTYPE_GOTO_LINES) {
        player_link_packet link { packet.type, packet.data, packet.data_length };
        PublicData::set_value(player_checksum, link_packet_checksum, &link);
#endif
    }
}

int SerialConsole::receive_packet(makera::Packet& packet, uint32_t timeout_ms)
{
    makera_frame_decoder.reset();
    const uint32_t start_us = us_ticker_read();
    const uint32_t timeout_us = timeout_ms * 1000;
    while (us_ticker_read() - start_us < timeout_us) {
        const int byte = read_byte();
        if (byte == uart_rx_error) {
            makera_frame_decoder.reset();
            return -4;
        }
        if (byte < 0) continue;

        const makera::DecodeResult result = makera_frame_decoder.decode_byte(
            static_cast<uint8_t>(byte), us_ticker_read() / 1000);
        if (result == makera::DecodeResult::invalid_crc) {
            makera_frame_decoder.reset();
            return -3;
        }
        if (result != makera::DecodeResult::complete) continue;
        packet = makera_frame_decoder.packet();
        makera_frame_decoder.reset();
        return 0;
    }

    makera_frame_decoder.reset();
    return -1;
}

void SerialConsole::reset_file_parser()
{
    file_parse_state = FILE_WAIT_HEADER;
    file_frame_index = 0;
    file_bytes_needed = 2;
    file_header[0] = file_header[1] = 0;
    file_footer[0] = file_footer[1] = 0;
}

int SerialConsole::check_file_packet(char **buf)
{
    if (file_frame_index < 5) return 0;

    uint16_t calculated_crc = crc16::ccitt(xbuff, file_frame_index - 2);
    uint16_t received_crc = (xbuff[file_frame_index - 2] << 8) | xbuff[file_frame_index - 1];
    if (calculated_crc != received_crc) return 0;

    uint8_t command = xbuff[2];
    switch (command) {
        case PTYPE_FILE_MD5:
        case PTYPE_FILE_CAN:
        case PTYPE_FILE_VIEW:
        case PTYPE_FILE_DATA:
        case PTYPE_FILE_END:
        case PTYPE_FILE_RETRY:
        case 0xA0:
        case PTYPE_CTRL_SINGLE:
        case PTYPE_CTRL_MULTI:
            *buf = reinterpret_cast<char *>(xbuff);
            return command;
        default:
            return 0;
    }
}

void SerialConsole::reset()
{
    reset_file_parser();
}

void SerialConsole::on_protocol_changed()
{
    buffer.tail = buffer.head;
    previous_char = 0;
    query_flag = false;
    halt_flag = false;
    diagnose_flag = false;
    makera_file_cancel = false;
    makera_rx_overflow = false;
    command_waiting = false;
    makera_rx_bytes.tail = makera_rx_bytes.head;
    makera_frame_decoder.reset();
    reset_file_parser();
}

int SerialConsole::_putc(int c)
{
    return this->serial->putc(c);
}

int SerialConsole::_getc()
{
#if defined(SERIAL_RX_DMA)
    if (rx_lookahead >= 0) {
        const int result = rx_lookahead;
        rx_lookahead = -1;
        return result;
    }
    uint8_t byte = 0;
    return uart_rx_dma::try_get(byte) ? byte : -1;
#else
    return this->serial->getc();
#endif
}

bool SerialConsole::ready()
{
    if (communication_protocol == PROTOCOL_MAKERA && makera_rx_bytes.tail != makera_rx_bytes.head) return true;
#if defined(SERIAL_RX_DMA)
    if (rx_lookahead >= 0) return true;
    uint8_t byte = 0;
    if (!uart_rx_dma::try_get(byte)) return false;
    rx_lookahead = byte;
    return true;
#else
    return this->serial->readable();
#endif
}

#if defined(SERIAL_RX_DMA)
bool SerialConsole::handle_rx_error()
{
    if (!uart_rx_dma::take_error()) return false;

    makera_frame_decoder.reset();
    reset_file_parser();
    THEKERNEL->streams->printf("ERROR: UART RX DMA error or overflow; input resynchronized\n");
    return true;
}
#endif

int SerialConsole::read_byte()
{
#if defined(SERIAL_RX_DMA)
    if (handle_rx_error()) return uart_rx_error;
    return _getc();
#else
    return this->serial->readable() ? this->serial->getc() : -1;
#endif
}

// Does the queue have a given char ?
bool SerialConsole::has_char(char letter){
    int index = this->buffer.tail;
    while( index != this->buffer.head ){
        if( this->buffer.buffer[index] == letter ){
            return true;
        }
        index = this->buffer.next_block_index(index);
    }
    return false;
}
