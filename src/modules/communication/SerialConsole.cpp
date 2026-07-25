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
#include "libs/nuts_bolts.h"
#include "SerialConsole.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "libs/Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define uart_checksum CHECKSUM("uart")
#define XBUFF_LENGTH 8208

extern unsigned char xbuff[XBUFF_LENGTH];
alignas(4) static unsigned char serial_protocol_buffer[544];

// Queue lives in main RAM (SerialConsole itself is AHB-allocated)
enum { MAKERA_CMD_QUEUE_DEPTH = 4, MAKERA_CMD_MAX_LEN = 256 };
static char makera_cmd_payloads[MAKERA_CMD_QUEUE_DEPTH][MAKERA_CMD_MAX_LEN];
static uint16_t makera_cmd_lengths[MAKERA_CMD_QUEUE_DEPTH];
static volatile uint8_t makera_cmd_head;
static volatile uint8_t makera_cmd_tail;

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole( PinName tx_pin, PinName rx_pin, int baud_rate ){
    this->serial = new mbed::Serial( tx_pin, rx_pin );
    this->serial->baud(baud_rate);
    this->previous_char = 0;
    this->current_baud_rate = baud_rate;
    this->default_baud_rate = baud_rate;
    this->temp_baud_rate = 0;
    this->last_activity_ms = 0;
    this->makera_cmd_queue_clear();
    this->reset_makera_command_parser();
    this->reset_file_parser();
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

    default_baud_rate = THEKERNEL->config->value(uart_checksum, baud_rate_setting_checksum)->as_number(current_baud_rate);
    if (default_baud_rate != current_baud_rate) {
        this->serial->baud(default_baud_rate);
        this->current_baud_rate = default_baud_rate;
    }

    this->attach_irq(true);

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

void SerialConsole::attach_irq(bool enable_irq) {
	if (enable_irq) {
	    this->serial->attach(this, &SerialConsole::on_serial_char_received, mbed::Serial::RxIrq);
	} else {
	    this->serial->attach(nullptr, mbed::Serial::RxIrq);
	}
}

void SerialConsole::on_set_public_data(void *argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_serial_rx_irq_checksum)) {
        bool enable_irq = *static_cast<bool *>(pdr->get_data_ptr());
        this->attach_irq(enable_irq);
        pdr->set_taken();
    }
}


// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received() {
	while (this->serial->readable()) {
		char received = this->serial->getc();
		last_activity_ms = us_ticker_read() / 1000;

		if(THEKERNEL->is_cachewait()) {
			continue;
		}

        if (communication_protocol == PROTOCOL_MAKERA) {
            // Keep RX IRQ enabled and queue completed commands. Disabling IRQ while a
            // command is pending drops back-to-back host traffic (e.g. buffer then play).
            process_makera_byte(static_cast<uint8_t>(received));
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
	if (THEKERNEL->is_uploading()) return;

    if (temp_baud_rate != 0) {
        uint32_t now_ms = us_ticker_read() / 1000;
        if ((now_ms - last_activity_ms) >= 15000) {
            this->serial->baud(default_baud_rate);
            this->current_baud_rate = default_baud_rate;
            this->temp_baud_rate = 0;
        }
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
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument){
    if (communication_protocol == PROTOCOL_MAKERA) {
        if (!makera_cmd_queue_empty()) {
            uint8_t idx = makera_cmd_head;
            uint16_t payload_length = makera_cmd_lengths[idx];
            struct SerialMessage message;
            message.message.assign(makera_cmd_payloads[idx], payload_length);
            message.stream = this;
            message.line = 0;

            makera_cmd_head = (idx + 1) % MAKERA_CMD_QUEUE_DEPTH;
            THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
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
        while (this->serial->readable()) {
            uint8_t received = static_cast<uint8_t>(this->serial->getc());
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

void SerialConsole::reset_makera_command_parser()
{
    makera_header = 0;
    makera_received = 0;
    makera_data_length = 0;
}

bool SerialConsole::makera_cmd_queue_empty() const
{
    return makera_cmd_head == makera_cmd_tail;
}

void SerialConsole::makera_cmd_queue_clear()
{
    makera_cmd_head = 0;
    makera_cmd_tail = 0;
}

bool SerialConsole::makera_cmd_queue_push(const char *data, uint16_t len)
{
    if (data == nullptr || len == 0 || len > MAKERA_CMD_MAX_LEN) {
        return false;
    }

    uint8_t next = (makera_cmd_tail + 1) % MAKERA_CMD_QUEUE_DEPTH;
    if (next == makera_cmd_head) {
        return false; // queue full — drop
    }

    memcpy(makera_cmd_payloads[makera_cmd_tail], data, len);
    makera_cmd_lengths[makera_cmd_tail] = len;
    makera_cmd_tail = next;
    return true;
}

void SerialConsole::process_makera_byte(uint8_t received)
{
    if (makera_received < 2) {
        makera_header = (makera_header << 8) | received;
        if (makera_header == HEADER) {
            serial_protocol_buffer[0] = (HEADER >> 8) & 0xff;
            serial_protocol_buffer[1] = HEADER & 0xff;
            makera_received = 2;
        }
        return;
    }

    serial_protocol_buffer[makera_received++] = received;
    if (makera_received == 4) {
        makera_data_length = (serial_protocol_buffer[2] << 8) | serial_protocol_buffer[3];
        if (makera_data_length < 3 || makera_data_length + 6 > sizeof(serial_protocol_buffer)) {
            reset_makera_command_parser();
        }
        return;
    }

    if (makera_received < makera_data_length + 6) return;

    uint16_t footer = (serial_protocol_buffer[makera_received - 2] << 8)
                    | serial_protocol_buffer[makera_received - 1];
    uint16_t received_crc = (serial_protocol_buffer[makera_received - 4] << 8)
                          | serial_protocol_buffer[makera_received - 3];
    uint16_t calculated_crc = crc16_ccitt(&serial_protocol_buffer[2], makera_data_length);
    if (footer != FOOTER || received_crc != calculated_crc) {
        reset_makera_command_parser();
        return;
    }

    uint8_t command = serial_protocol_buffer[4];
    if (command == PTYPE_CTRL_SINGLE) {
        if (makera_data_length >= 4) {
            uint8_t control = serial_protocol_buffer[5];
            if (control == '?') {
                query_flag = true;
            } else if (control == '*') {
                diagnose_flag = true;
            } else if (control == 'X' - 'A' + 1) {
                halt_flag = true;
            } else if (control == 'Y' - 'A' + 1) {
                if (THEKERNEL->get_internal_stop_request()) {
                    THEKERNEL->set_internal_stop_request(false);
                } else {
                    THEKERNEL->set_stop_request(true);
                    THEKERNEL->set_stop_request_time(us_ticker_read() / 1000);
                }
            } else if (control == 'Z' - 'A' + 1) {
                THEKERNEL->set_keep_alive_request(true);
            } else if (THEKERNEL->is_feed_hold_enabled() && control == '!') {
                THEKERNEL->set_feed_hold(true);
            } else if (THEKERNEL->is_feed_hold_enabled() && control == '~') {
                THEKERNEL->set_feed_hold(false);
            }
        }
    } else if (command == PTYPE_CTRL_MULTI || command == PTYPE_FILE_START) {
        // Copy payload now so the parser can accept the next frame immediately
        uint16_t payload_len = makera_data_length - 3;
        makera_cmd_queue_push(reinterpret_cast<char *>(&serial_protocol_buffer[5]), payload_len);
        reset_makera_command_parser();
        return;
    }

    reset_makera_command_parser();
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

    uint16_t calculated_crc = crc16_ccitt(xbuff, file_frame_index - 2);
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
    makera_cmd_queue_clear();
    reset_makera_command_parser();
    reset_file_parser();
}

int SerialConsole::_putc(int c)
{
    return this->serial->putc(c);
}

int SerialConsole::_getc()
{
    return this->serial->getc();
}

bool SerialConsole::ready()
{
    return this->serial->readable();
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
