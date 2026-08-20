/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SERIALCONSOLE_H
#define SERIALCONSOLE_H

#include "libs/Module.h"
#include "Serial.h" // mbed.h lib
#include "libs/Kernel.h"
#include <vector>
#include <string>
using std::string;
#include "libs/RingBuffer.h"
#include "libs/StreamOutput.h"
#include "libs/MakeraFrame.h"


#define baud_rate_setting_checksum CHECKSUM("baud_rate")

class SerialConsole : public Module, public StreamOutput {
    public:
        SerialConsole( PinName tx_pin, PinName rx_pin, int baud_rate );
        ~SerialConsole();

        void on_module_loaded();
        void on_serial_char_received();
        void on_main_loop(void * argument);
        void on_idle(void * argument);
        void on_set_public_data(void *argument);
        bool has_char(char letter);
        void set_rx_enabled(bool enabled);

        int _putc(int c);
        int _getc(void);
        int puts(const char*, int size = 0);
        int gets(char** buf, int size = 0);
        bool ready();
        bool frames_protocol_output() const { return true; }
        void on_protocol_changed();
        void reset();
        char getc_result;

        int get_baud() const { return current_baud_rate; }
        void set_baud_temporary(int new_baud);
        int receive_packet(makera::Packet& packet, uint32_t timeout_ms = 100);

        //string receive_buffer;                 // Received chars are stored here until a newline character is received
        //vector<std::string> received_lines;    // Received lines are stored here until they are requested
        RingBuffer<char,256> buffer;             // Receive buffer
        mbed::Serial* serial;
        char previous_char;                       // Track previous character for ?1 detection
        int current_baud_rate;
        int default_baud_rate;
        int temp_baud_rate;                       // non-zero = temporary baud active
        uint32_t last_activity_ms;                // for 15s timeout revert
        makera::FrameDecoder makera_frame_decoder;
#if defined(SERIAL_RX_DMA)
        bool rx_dispatch_enabled;
        int rx_lookahead;
        bool handle_rx_error();
#endif
        int read_byte();

        enum FileParseState {
            FILE_WAIT_HEADER,
            FILE_READ_LENGTH,
            FILE_READ_DATA,
            FILE_CHECK_FOOTER
        } file_parse_state;
        uint16_t file_frame_index;
        uint16_t file_bytes_needed;
        uint8_t file_header[2];
        uint8_t file_footer[2];

        void process_makera_byte(uint8_t received);
        void reset_file_parser();
        int check_file_packet(char **buf);
        struct {
          volatile bool query_flag:1;
          volatile bool halt_flag:1;
          volatile bool diagnose_flag:1;
          bool command_waiting:1;
        };
        volatile bool makera_file_cancel;
        volatile bool makera_rx_overflow;
};

#endif
