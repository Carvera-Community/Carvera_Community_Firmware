/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
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
#include "CommunicationProtocol.h"
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "libs/Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define uart_checksum CHECKSUM("uart")
#define protocol_checksum CHECKSUM("protocol")

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole( PinName tx_pin, PinName rx_pin, int baud_rate )
    : protocol_handler(&comms::default_protocol())
{
    this->serial = new mbed::Serial( tx_pin, rx_pin );
    this->serial->baud(baud_rate);
    this->previous_char = 0;
    this->current_baud_rate = baud_rate;
    this->default_baud_rate = baud_rate;
    this->temp_baud_rate = 0;
    this->last_activity_ms = 0;
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

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams->append_stream(this);

    protocol_handler = &comms::configured_protocol(uart_checksum, protocol_checksum, comms::Protocol::Smoothie);
    framed_input.set_protocol(*protocol_handler, framed_command_buffer, sizeof(framed_command_buffer),
            comms::TransferBuffer::data(), comms::TransferBuffer::capacity());

    default_baud_rate = THEKERNEL->config->value(uart_checksum, baud_rate_setting_checksum)->as_number(current_baud_rate);
    if (default_baud_rate != current_baud_rate) {
        this->serial->baud(default_baud_rate);
        this->current_baud_rate = default_baud_rate;
    }

    this->attach_irq(!protocol().uses_framing());

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_SET_PUBLIC_DATA);

}

void SerialConsole::set_baud_temporary(int new_baud) {
    this->temp_baud_rate = new_baud;
    this->current_baud_rate = new_baud;
    this->serial->baud(new_baud);
    this->last_activity_ms = us_ticker_read() / 1000;
}

void SerialConsole::attach_irq(bool enable_irq) {
    if (protocol().uses_framing()) enable_irq = false;
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
    if (protocol().uses_framing()) {
        receive_framed_data();
        return;
    }

    while (this->serial->readable()) {
        char received = this->_getc();

        if(THEKERNEL->is_cachewait()) {
            continue;
        }

        if (received == '?') {
            const auto result = THEKERNEL->handle_realtime_control(received, false, false);
            if (result.query) query_flag = true;
            continue;
        } else if (this->previous_char == '?' && received == '1') {
            const auto result = THEKERNEL->handle_realtime_control('?', true, false);
            if (result.query) query_flag = true;
            continue;
        }

        //if (received == '*') {
        //	diagnose_flag = true;
        //	continue;
        //}
        const bool at_line_start = (this->buffer.head == this->buffer.tail) || (this->previous_char == '\n') || (this->previous_char == '\r');
        const auto result = THEKERNEL->handle_realtime_control(received, false, at_line_start);
        if (result.handled) {
            if (result.query) query_flag = true;
            if (result.halt) halt_flag = true;
            continue;
        }
        // convert CR to NL (for host OSs that don't send NL)
        if ( received == '\r' ) { received = '\n'; }
        this->buffer.push_back(received);

        // Reset previous_char for any other character
        this->previous_char = received;
    }
}

void SerialConsole::receive_framed_data()
{
    comms::InputMessage message;
    while (this->serial->readable()) {
        const uint8_t received = this->_getc();
        if (framed_input.feed_command_byte(received, message)) {
            apply_framed_result(framed_input.dispatch(this, message));
        }
    }
}

void SerialConsole::apply_framed_result(const comms::FramedInputResult& result)
{
    if (result.query) query_flag = true;
    if (result.halt) halt_flag = true;
}

void SerialConsole::on_idle(void * argument)
{
    if (THEKERNEL->is_uploading()) {
        service_realtime_flags();
        return;
    }

    if (protocol().uses_framing()) {
        receive_framed_data();
    }

    if (temp_baud_rate != 0) {
        uint32_t now_ms = us_ticker_read() / 1000;
        if ((now_ms - last_activity_ms) >= 15000) {
            this->serial->baud(default_baud_rate);
            this->current_baud_rate = default_baud_rate;
            this->temp_baud_rate = 0;
        }
    }

    service_realtime_flags();
}

void SerialConsole::service_realtime_flags()
{
    if (query_flag ) {
        query_flag = false;
        protocol().send_message(this, comms::MessageType::Status, THEKERNEL->get_query_string().c_str(), 0);
    }

    if (diagnose_flag) {
        diagnose_flag = false;
        protocol().send_message(this, comms::MessageType::Diagnostics, THEKERNEL->get_diagnose_string().c_str(), 0);
    }

    if (halt_flag) {
        halt_flag= false;
        THEKERNEL->set_halt_reason(MANUAL);
        
        if(THEKERNEL->is_grbl_mode()) {
            protocol().send_message(this, comms::MessageType::Text, "ERROR: Abort during cycle\r\n", 0);
        } else {
            protocol().send_message(this, comms::MessageType::Text, "ERROR: Abort during cycle\r\nM999 or $X to exit HALT state\r\n", 0);
        }
        THEKERNEL->call_event(ON_HALT, nullptr);
    }
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument){
    if (protocol().uses_framing()) {
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
    if (protocol().uses_framing()) {
        while (this->serial->readable()) {
            const uint8_t received = this->_getc();
            const auto result = framed_input.feed_file_byte(this, received, buf);
            apply_framed_result(result);
            if (result.file_type > 0) return result.file_type;
        }
        return 0;
    }

	getc_result = this->_getc();
	*buf = &getc_result;
	return 1;
}

void SerialConsole::reset_file_input()
{
    framed_input.reset_file_parser();
}

int SerialConsole::_putc(int c)
{
    return this->serial->putc(c);
}

int SerialConsole::_getc()
{
    mark_activity();
    return this->serial->getc();
}

bool SerialConsole::ready()
{
    return this->serial->readable();
}

const comms::ProtocolHandler& SerialConsole::protocol() const
{
    return *protocol_handler;
}

void SerialConsole::mark_activity()
{
    last_activity_ms = us_ticker_read() / 1000;
}

int SerialConsole::printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const int sent = protocol().vprintf_message(this, comms::MessageType::Text, format, args);
    va_end(args);
    return sent;
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
