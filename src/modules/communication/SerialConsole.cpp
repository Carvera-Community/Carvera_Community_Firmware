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
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"

#define XBUFF_LENGTH	8208
extern unsigned char xbuff[XBUFF_LENGTH];
extern unsigned char fbuff[4096];

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole( PinName rx_pin, PinName tx_pin, int baud_rate ){
    this->serial = new mbed::Serial( rx_pin, tx_pin );
    this->serial->baud(baud_rate);
    this->previous_char = 0;
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    // We want to be called every time a new char is received
    query_flag = false;
    halt_flag = false;
    diagnose_flag = false;
	this->attach_irq(true);

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams->append_stream(this);
}

void SerialConsole::attach_irq(bool enable_irq) {
	if (enable_irq) {
	    this->serial->attach(nullptr, mbed::Serial::RxIrq);
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
	static uint8_t headerBuffer[2];
    static uint8_t footerBuffer[2];
    static uint8_t lengthBuffer[2];
    static uint16_t bytesNeeded = 2;
    uint16_t expectedLength = 0;
    uint16_t checksum;
    
	while (this->serial->readable()) 
	{
		char byte = this->serial->getc();
		switch(this->currentState) 
		{
            case WAIT_HEADER:{
                headerBuffer[0] = headerBuffer[1];
                headerBuffer[1] = byte;
                checksum = (headerBuffer[0] << 8) | headerBuffer[1];
                if(checksum == HEADER) {
                    this->currentState = READ_LENGTH;
                    bytesNeeded = 2;
                }
                break;
            }
            case READ_LENGTH:{
                lengthBuffer[0] = lengthBuffer[1];
                lengthBuffer[1] = byte;
            	this->buffer.push_back(byte);
                if(--bytesNeeded == 0) {
                    expectedLength = (lengthBuffer[0] << 8) | lengthBuffer[1];
                    if(expectedLength >=0 && expectedLength<=256)
                    {
	                    this->currentState = READ_DATA;
	                    bytesNeeded = expectedLength;
	                }
	                else
	                {
	                	this->currentState = WAIT_HEADER;
	                }
                }
            	break;
            }
            
            case READ_DATA:{
            	this->buffer.push_back(byte);
                if(--bytesNeeded == 0) {
                    this->currentState = CHECK_FOOTER;
                    bytesNeeded = 2;
                }
                break;
            }
                
            case CHECK_FOOTER:{
                footerBuffer[0] = footerBuffer[1];
                footerBuffer[1] = byte;
                if(--bytesNeeded == 0) {
                	checksum = (footerBuffer[0] << 8) | footerBuffer[1];
                    this->currentState = WAIT_HEADER;
                    if(checksum == FOOTER) {
                    	processPacket();
                    	return;
                    }
                }
                break;
            }
            
            default:
            	break;
        }
	}
}

void SerialConsole::processPacket() {
	char c;
	
	uint8_t cmdType = 0;
    uint16_t calcCRC = 0;
    uint16_t receivedCRC = 0;
    int len = this->buffer.size();
    for(int i=0; i< len && i < 256; i++)
    {
    	this->buffer.pop_front(c);
    	packetData[i] = c;
    }
    calcCRC = crc16_ccitt((unsigned char*)packetData, len-2);
    receivedCRC = (packetData[len-2] << 8) | packetData[len-1];
    if(calcCRC == receivedCRC) {
    	cmdType = packetData[2];
        switch(cmdType) {
            case PTYPE_CTRL_SINGLE: { 
                if(packetData[3] == '?') {
	            	query_flag = true;
                }
	        	else if(packetData[3] == 'X' - 'A' + 1) {
	            	halt_flag = true;
	        	}
                else if(packetData[3] == 'Y' - 'A' + 1) { // ^Y
                    THEKERNEL->set_stop_request(true);
                    THEKERNEL->streams->printf("^Y\n");
                    return;
                }
	        	else if(THEKERNEL->is_feed_hold_enabled()) {
		            if(packetData[3] == '!') { // safe pause
		                THEKERNEL->set_feed_hold(true);
		            }
		            else if(packetData[3] == '~') { // safe resume
		                THEKERNEL->set_feed_hold(false);
		            }
		        }
                break;
            }
            case PTYPE_CTRL_MULTI: {
                
                struct SerialMessage message;
                message.message.assign(packetData+3,len- 5);
                if (message.message == "?!") {
                    query_flag = true;
                    THEKERNEL->set_keep_alive_request(true);
                    return;
                }
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                break;
            }
            case PTYPE_FILE_START: {
                struct SerialMessage message;
                message.message.assign(packetData+3,len- 5);
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                break;
            }
            default:
            	break;
        }
    }
}


unsigned int SerialConsole::crc16_ccitt(unsigned char *data, unsigned int len)
{
	static const unsigned short crc_table[] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
		0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
		0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
		0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
		0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
		0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
		0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
		0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
		0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
		0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
		0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
		0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
		0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
		0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
		0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
		0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
		0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
		0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
		0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
		0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
	};

	unsigned char tmp;
	unsigned short crc = 0;

	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]) & 0xffff;
	}

	return crc & 0xffff;
}

void SerialConsole::PacketMessage(char cmd, const char* s, int size)
{
	int crc = 0;
    unsigned int len = 0;
	size_t total_length = size == 0 ? strlen(s) : size;
	
	fbuff[0] = (HEADER>>8)&0xFF;
	fbuff[1] = HEADER&0xFF;
	fbuff[4] = cmd;
	
	memcpy(&fbuff[5], s, total_length);
	len = total_length + 3;
	fbuff[2] = (len>>8)&0xFF;
	fbuff[3] = len&0xFF;
	crc = crc16_ccitt(&fbuff[2], len);
	fbuff[total_length+5] = (crc>>8)&0xFF;
	fbuff[total_length+6] = crc&0xFF;
	fbuff[total_length+7] = (FOOTER>>8)&0xFF;
	fbuff[total_length+8] = FOOTER&0xFF;
	
	puts((char *)fbuff, len+6);
}


int SerialConsole::printfcmd(const char cmd, const char *format, ...)
{
	char b[64];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);

    int size = vsnprintf(b, 64, format, args) + 1; // we add one to take into account space for the terminating \0

    if (size < 64) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }
    va_end(args);

//    puts(buffer, strlen(buffer));
	PacketMessage(PTYPE_DIAG_RES, buffer, strlen(buffer));

    if (buffer != b)
        delete[] buffer;

    return size - 1;
}

int SerialConsole::printf(const char *format, ...)
{
	char b[64];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);

    int size = vsnprintf(b, 64, format, args) + 1; // we add one to take into account space for the terminating \0

    if (size < 64) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }
    va_end(args);

	PacketMessage(PTYPE_NORMAL_INFO, buffer, strlen(buffer));

    if (buffer != b)
        delete[] buffer;

    return size - 1;
}


void SerialConsole::on_idle(void * argument)
{	
	if (THEKERNEL->is_uploading()) return;
	
	on_serial_char_received();

    if (query_flag ) {
        query_flag = false;
        PacketMessage(PTYPE_STATUS_RES,THEKERNEL->get_query_string().c_str(),0);
    }

    if (diagnose_flag) {
    	diagnose_flag = false;
    	PacketMessage(PTYPE_DIAG_RES,THEKERNEL->get_diagnose_string().c_str(),0);
    }

    if (halt_flag) {
        halt_flag= false;
        THEKERNEL->set_halt_reason(MANUAL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        if(THEKERNEL->is_grbl_mode()) {
            PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort during cycle\r\n", 0);
        } else {
            PacketMessage(PTYPE_NORMAL_INFO, "HALTED, M999 or $X to exit HALT state\r\n", 0);
        }
    }
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument){
    

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
	static uint8_t headerBuffer[2];
    static uint8_t footerBuffer[2];
    static uint16_t bytesNeeded = 2;
    uint16_t expectedLength = 0;
    uint16_t checksum;
    
	while (this->serial->readable()) {
		char byte = this->serial->getc();
		switch(this->currentState) {
            case WAIT_HEADER:
                headerBuffer[0] = headerBuffer[1];
                headerBuffer[1] = byte;
                checksum = (headerBuffer[0] << 8) | headerBuffer[1];
                if(checksum == HEADER) {
                    this->currentState = READ_LENGTH;
                    bytesNeeded = 2;
                    memset(xbuff, 0, sizeof(xbuff));
                }
                break;
            case READ_LENGTH:
            xbuff[this->ptr_xbuff] = byte;
            if(++this->ptr_xbuff >= XBUFF_LENGTH) 
            {
            	this->ptr_xbuff = 0;
            	this->currentState = WAIT_HEADER;
            	return 0;
            }
            
            if(--bytesNeeded == 0) {
                expectedLength = (xbuff[0] << 8) | xbuff[1];
                this->currentState = READ_DATA;
                bytesNeeded = expectedLength;
            }
            	break;
            
            case READ_DATA:
	            xbuff[this->ptr_xbuff] = byte;
	            
	            if(++this->ptr_xbuff >= XBUFF_LENGTH) 
	            {
	            	this->ptr_xbuff = 0;
	            	this->currentState = WAIT_HEADER;
	            	return 0;
	            }
	            
                if(--bytesNeeded == 0) {
                    this->currentState = CHECK_FOOTER;
                    bytesNeeded = 2;
                }
                break;
                
            case CHECK_FOOTER:
                footerBuffer[0] = footerBuffer[1];
                footerBuffer[1] = byte;
                if(--bytesNeeded == 0) {
                    this->currentState = WAIT_HEADER;
                	checksum = (footerBuffer[0] << 8) | footerBuffer[1];
                    if(checksum == FOOTER) {
                        return CheckFilePacket(buf);
                    }
                }
                break;
        }
	}
	
	return 0;
	
}

int SerialConsole::CheckFilePacket(char** buf) {
	uint8_t cmdType = 0;
    uint16_t calcCRC = 0;
    uint16_t receivedCRC = 0;
    calcCRC = crc16_ccitt(xbuff, this->ptr_xbuff-2);
    receivedCRC = (xbuff[this->ptr_xbuff-2] << 8) | xbuff[this->ptr_xbuff-1];
    this->ptr_xbuff = 0;
    
    if(calcCRC == receivedCRC) {
    	cmdType = xbuff[2];
        switch(cmdType) {
            case PTYPE_FILE_MD5:
            case PTYPE_FILE_CAN:
            case PTYPE_FILE_VIEW:
            case PTYPE_FILE_DATA:
            case PTYPE_FILE_END:
            case PTYPE_FILE_RETRY:
            case 0xA0:
            case 0xA1:
            case 0xA2:
            	*buf = (char*) &xbuff[0];
            	break;
            default:
            	cmdType = 0;
            	break;
            	
        }
    }
    return cmdType;
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
