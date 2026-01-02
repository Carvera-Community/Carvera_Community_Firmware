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
#include "Ticker.h"
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
__attribute__((section("AHBSRAM1"), aligned(4))) char Serialbuff[544];
extern const unsigned short crc_table[256];

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
	uint8_t headerBuffer[2];
    uint32_t received = 0;
    uint32_t timeout_ms = 100000;	//100 ms
    uint32_t starttime = 0;
    
	if (this->serial->readable()) 
	{
	    // wait head
	    starttime = us_ticker_read();
	    while ((received < 2) && ((us_ticker_read() - starttime) < timeout_ms) ) {
	    	if (this->serial->readable()) {
	    		headerBuffer[0] = headerBuffer[1];
	    		char byte = this->serial->getc();
				received++;
	            headerBuffer[1] = byte;
	            if (received >= 2 && (headerBuffer[0] != ((HEADER >> 8) & 0xFF) || 
	                                 headerBuffer[1] != (HEADER & 0xFF))) {
	                received = 1;
	            }
	        } 
	    }
	    
	    if (received < 2){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive header\r\n", 0);
	    	return;
	    }
	    
	    // receive length	    
	    starttime = us_ticker_read();
	    while ((received < 4) && ((us_ticker_read() - starttime) < timeout_ms) ) {
	    	if (this->serial->readable()) {
	        	char byte = this->serial->getc();
	        	Serialbuff[received] = byte;
	            received ++;
	        } 
	    }
	    
	    if (received < 4){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive length\r\n", 0);
	    	return;
	    }
	    
	    uint16_t data_len = (Serialbuff[2]<<8) | Serialbuff[3];
	    uint16_t total_len = 4 + data_len + 2; // header + data + crc + tail
	    
	    if (data_len > 513 || total_len > sizeof(Serialbuff)){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive datalen error\r\n", 0);
	    	 return; 
	    }
	    
	    starttime = us_ticker_read();
	    while ((received < total_len) && ((us_ticker_read() - starttime) < timeout_ms) ) {	    	
	    	if (this->serial->readable()) {
	        	char byte = this->serial->getc();
	        	Serialbuff[received] = byte;
	            received ++;
	        }
	    }
	    
	    if (received < total_len) {
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive data body\r\n", 0);
	    	return;
	    }
	    
	    // check tail
	    uint16_t tail = (Serialbuff[total_len-2]<<8) | Serialbuff[total_len-1];
	    if (tail != FOOTER) {
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive footer\r\n", 0);
	    	return;
	    }
/*	    
	    // check CRC
	    uint16_t received_crc = (Serialbuff[total_len-4] << 8) | Serialbuff[total_len-3];
		uint16_t calculated_crc = crc16_ccitt((unsigned char *)&Serialbuff[2], data_len);
	    if (received_crc != calculated_crc) {
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive wrong crc\r\n", 0);
	        return;
	    }
*/	    
		uint8_t cmdType = Serialbuff[4];
        switch(cmdType) {
            case PTYPE_CTRL_SINGLE: { 
                if(Serialbuff[5] == '?') {
	            	query_flag = true;
                }
	        	else if(Serialbuff[5] == 'X' - 'A' + 1) {
	            	halt_flag = true;
	        	}
                else if(Serialbuff[5] == 'Y' - 'A' + 1) { // ^Y
                    THEKERNEL->set_stop_request(true);
                    THEKERNEL->streams->printf("^Y\n");
                    return;
                }
	        	else if(THEKERNEL->is_feed_hold_enabled()) {
		            if(Serialbuff[5] == '!') { // safe pause
		                THEKERNEL->set_feed_hold(true);
		            }
		            else if(Serialbuff[5] == '~') { // safe resume
		                THEKERNEL->set_feed_hold(false);
		            }
		        }
                this->previous_char = received;
                break;
            }
            case PTYPE_CTRL_MULTI: {
                
                struct SerialMessage message;
                message.message.assign(Serialbuff+5,data_len- 3);
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
                message.message.assign(Serialbuff+5,data_len-3);
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
                if(expectedLength >=0 && expectedLength<=XBUFF_LENGTH)
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
