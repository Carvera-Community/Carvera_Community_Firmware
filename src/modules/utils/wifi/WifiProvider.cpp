/*
 * WifiProvider.cpp
 *
 *  Created on: 2020年6月10日
 *      Author: josh
 */

#include "WifiProvider.h"

#include "brd_cfg.h"
#include "M8266HostIf.h"

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "SlowTicker.h"
#include "Tool.h"
#include "PublicDataRequest.h"
#include "Config.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "PublicData.h"
#include "Gcode.h"
#include "modules/robot/Conveyor.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "SwitchPublicAccess.h"
#include "WifiPublicAccess.h"
#include "libs/utils.h"

#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"

#include "platform_memory.h" // Needed for AHB allocator

#include "port_api.h"
#include "InterruptIn.h"

#include "gpio.h"

#include <math.h>

#define wifi_checksum                     CHECKSUM("wifi")
#define wifi_enable                       CHECKSUM("enable")
#define wifi_interrupt_pin_checksum       CHECKSUM("interrupt_pin")
#define machine_name_checksum             CHECKSUM("machine_name")
#define tcp_port_checksum		          CHECKSUM("tcp_port")
#define udp_send_port_checksum		      CHECKSUM("udp_send_port")
#define udp_recv_port_checksum		      CHECKSUM("udp_recv_port")
#define tcp_timeout_s_checksum			  CHECKSUM("tcp_timeout_s")

#define XBUFF_LENGTH	8208
extern unsigned char xbuff[XBUFF_LENGTH];
extern unsigned char fbuff[4096];

WifiProvider::WifiProvider()
{
	tcp_link_no = 0;
	udp_link_no = 1;
	wifi_init_ok = false;
	has_data_flag = false;
	connection_fail_count = 0;
}

void WifiProvider::on_module_loaded()
{
    if( !THEKERNEL->config->value( wifi_checksum,  wifi_enable)->by_default(true)->as_bool() ) {
        // as not needed free up resource
        delete this;
        return;
    }

	this->tcp_port = THEKERNEL->config->value(wifi_checksum, tcp_port_checksum)->by_default(2222)->as_int();
	this->udp_send_port = THEKERNEL->config->value(wifi_checksum, udp_send_port_checksum)->by_default(3333)->as_int();
	this->udp_recv_port = THEKERNEL->config->value(wifi_checksum, udp_recv_port_checksum)->by_default(4444)->as_int();
	this->tcp_timeout_s = THEKERNEL->config->value(wifi_checksum, tcp_timeout_s_checksum)->by_default(10)->as_int();
    std::string config_name = THEKERNEL->config->value(wifi_checksum, machine_name_checksum)->by_default("CARVERA")->as_string();
    strncpy(this->machine_name, config_name.c_str(), sizeof(this->machine_name) - 1);
    this->machine_name[sizeof(this->machine_name) - 1] = '\0'; // Ensure null termination

    // Init Wifi Module
    this->init_wifi_module(false);

    // Add interrupt for WIFI data receving
    Pin *smoothie_pin = new(AHB) Pin();
    smoothie_pin->from_string(THEKERNEL->config->value(wifi_checksum, wifi_interrupt_pin_checksum)->by_default("2.11")->as_string());
    smoothie_pin->as_input();
    if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
        PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
        wifi_interrupt_pin = new(AHB) mbed::InterruptIn(pinname);
        wifi_interrupt_pin->rise(this, &WifiProvider::on_pin_rise);
        NVIC_SetPriority(EINT3_IRQn, 16);
    } else {
        THEKERNEL->streams->printf("Error: Wifi interrupt pin has to be on P0 or P2.\n");
        delete this;
        return;
    }
    delete smoothie_pin;

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams->append_stream(this);

    query_flag = false;
    diagnose_flag = false;
    halt_flag = false;

	this->register_for_event(ON_IDLE);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
}


void WifiProvider::on_pin_rise()
{
	has_data_flag = true;
}

void WifiProvider::receive_wifi_data() {
	u8 link_no;
	u16 received = 0;
	u16 status;
	static uint8_t headerBuffer[2];
    static uint8_t footerBuffer[2];
    static uint16_t bytesNeeded = 2;
    static uint8_t errorcnt=0;
    uint16_t expectedLength = 0;
    uint16_t checksum;


	while (true)
	{
		received = M8266WIFI_SPI_RecvData(WifiData, WIFI_DATA_MAX_SIZE, WIFI_DATA_TIMEOUT_MS, &link_no, &status);
		if (link_no == udp_link_no) {
			return;
		}
		if (int(status & 0xff) == 0x20 || int(status & 0xff) == 0x22 || int(status & 0xff) == 0x2f) {
			THEKERNEL->streams->printf("gets, received: %d, status:%d, high: %d, low: %d!\n", received, status, int(status >> 8), int(status & 0xff));
			return ;
		}
		for (int i = 0; i < received; i ++) {
			uint8_t byte;
			byte = WifiData[i];
			
			switch(this->currentState) {
	            case WAIT_HEADER:{
	                headerBuffer[0] = headerBuffer[1];
	                headerBuffer[1] = byte;
	                checksum = (headerBuffer[0] << 8) | headerBuffer[1];
	                if(checksum == HEADER) {
	                    this->currentState = READ_LENGTH;
	                    bytesNeeded = 2;
	                    packetData.clear();
	                    errorcnt = 0;
	                }
	                else if( ++errorcnt > 20)
	                {
	                	errorcnt = 0;
	                	THEKERNEL->streams->puts("Please use Controller version V0.9.12 or later to connect.\r\n", 124); 
	                }
	                break;
	            }
	            case READ_LENGTH:{
                packetData.push_back(byte);
	                if(--bytesNeeded == 0) {
	                    expectedLength = (packetData[0] << 8) | packetData[1];
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
	                packetData.push_back(byte);
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
	                    }
	                }
	                break;
	            }
	        }
		}
		if (received < WIFI_DATA_MAX_SIZE) {
			return;
		}
	}
}

void WifiProvider::processPacket() {
	uint8_t cmdType = 0;
	// CRC校验
    uint16_t calcCRC = 0;
    uint16_t receivedCRC = 0;
    calcCRC = crc16_ccitt(packetData.data(), packetData.size()-2); // 最后两个字节是CRC
    receivedCRC = (packetData[packetData.size()-2] << 8) | packetData[packetData.size()-1];
    if(calcCRC == receivedCRC) {
    	cmdType = packetData[2];
        switch(cmdType) {
            case PTYPE_CTRL_SINGLE: {
                // 处理单字节控制指令
                if(packetData[3] == '?') {
	            	query_flag = true;
	        	}
	        	else if(packetData[3] == 'X' - 'A' + 1) {
	            	halt_flag = true;
	        	}
				else if(packetData[3] == 'Y' - 'A' + 1) {
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
                // 处理多字节控制指令
                std::string received;
                received.assign(reinterpret_cast<const char*>(&packetData[3]), packetData.size()- 5);
				if (received == "?!") {
					query_flag = true;
					THEKERNEL->set_keep_alive_request(true);
					return;
				}
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                break;
            }
            case PTYPE_FILE_START: {
                std::string received;
                received.assign(reinterpret_cast<const char*>(&packetData[3]), packetData.size()- 5);
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                break;
            }
        }
    }
}


unsigned int WifiProvider::crc16_ccitt(unsigned char *data, unsigned int len)
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

bool WifiProvider::ready() {
	return M8266WIFI_SPI_Has_DataReceived();
}

void WifiProvider::get_broadcast_from_ip_and_netmask(char *broadcast_addr, size_t broadcast_buffer_size, char *ip_addr, char *netmask)
{
	uint32_t i_ip = ip_to_int(ip_addr);
	uint32_t i_mask = ip_to_int(netmask);
	uint32_t i_broadcast = i_ip | (i_mask ^ 0xffffffff);
	int_to_ip(i_broadcast, broadcast_addr, broadcast_buffer_size);
}

void WifiProvider::int_to_ip(uint32_t i_ip, char *ip_addr, size_t buffer_size) {
    unsigned char bytes[4];
    bytes[0] = i_ip & 0xFF;
    bytes[1] = (i_ip >> 8) & 0xFF;
    bytes[2] = (i_ip >> 16) & 0xFF;
    bytes[3] = (i_ip >> 24) & 0xFF;
	snprintf(ip_addr, buffer_size, "%d.%d.%d.%d", bytes[3], bytes[2], bytes[1], bytes[0]);
}

uint32_t WifiProvider::ip_to_int(char* ip_addr) {
  unsigned char bytes[4];
  sscanf(ip_addr, "%u.%u.%u.%u", &bytes[0], &bytes[1], &bytes[2], &bytes[3]);
  return bytes[0] * 16777216 + bytes[1] * 65536 + bytes[2] * 256 + bytes[3];
}

void WifiProvider::on_second_tick(void *)
{
	u16 status = 0;
	char address[20];
	char udp_buff[100];
	u8 param_len = 0;
	u8 connection_status = 0;
	u8 client_num = 0;
	ClientInfo RemoteClients[15];

	if (!wifi_init_ok || THEKERNEL->is_uploading()) return;

	if (M8266WIFI_SPI_List_Clients_On_A_TCP_Server(tcp_link_no, &client_num, RemoteClients, &status)) {

		if (M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status)) {
			if (connection_status == 5) {
				// get ip and netmask
				M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_IP_ADDR, (u8 *)this->sta_address, &param_len, &status);
				M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_NETMASK_ADDR, (u8 *)this->sta_netmask, &param_len, &status);
				// send data to sta broadcast address
				get_broadcast_from_ip_and_netmask(address, this->sta_address, this->sta_netmask);
				snprintf(udp_buff, sizeof(udp_buff), "%s,%s,%d,%d", this->machine_name.c_str(), this->sta_address, this->tcp_port, client_num > 0 ? 1 : 0);
				if (M8266WIFI_SPI_Send_Udp_Data((u8 *)udp_buff, strlen(udp_buff), udp_link_no, address, this->udp_send_port, &status) < strlen(udp_buff)) {
					// THEKERNEL->streams->printf("Send UDP through STA ERROR, status: %d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
				} else {
					// THEKERNEL->streams->printf("Send UDP through STA Success!\n");
				}
				connection_fail_count = 0;
			} else if (connection_status == 2 || connection_status == 3 || connection_status == 4) {
				// wrong password or can not find STA or fail to connect
				connection_fail_count ++;
				if (connection_fail_count > 30) {
					// disconnect Wifi
					if (M8266WIFI_SPI_STA_DisConnect_Ap(&status)) {
						THEKERNEL->streams->printf("STA connection timeout, disconnected!\n");
					}
					connection_fail_count = 0;
				}
			} else {
				connection_fail_count = 0;
			}
		
			// send ap info through UDP
			memset(udp_buff, 0, sizeof(udp_buff));
			get_broadcast_from_ip_and_netmask(address, this->ap_address, this->ap_netmask);
			snprintf(udp_buff, sizeof(udp_buff), "%s,%s,%d,%d", this->machine_name.c_str(), this->ap_address, this->tcp_port, client_num > 0 ? 1 : 0);
			if (M8266WIFI_SPI_Send_Udp_Data((u8 *)udp_buff, strlen(udp_buff), udp_link_no, address, this->udp_send_port, &status) < strlen(udp_buff)) {
				// THEKERNEL->streams->printf("Send UDP through AP ERROR, status: %d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
			}
		}
	}

	// check AP and disconnect every 5 seconds
	/*
	M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_SSID, (u8 *)ssid, &ssid_len, &status);

	M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);

	if (M8266WIFI_SPI_STA_DisConnect_Ap(&status) == 0) {
		s->has_error = true;
		snprintf(s->error_info, sizeof(s->error_info), "Disconnect error!");
	}*/

}

void WifiProvider::on_idle(void *argument)
 {
	if (THEKERNEL->is_uploading()) return;

	if (has_data_flag || M8266WIFI_SPI_Has_DataReceived()) {
		has_data_flag = false;
		receive_wifi_data();
	}

    if (query_flag) {
        query_flag = false;
		PacketMessage(PTYPE_STATUS_RES,THEKERNEL->get_query_string().c_str(),0);
    }

    if (diagnose_flag) {
    	diagnose_flag = false;
    	PacketMessage(PTYPE_DIAG_RES,THEKERNEL->get_diagnose_string().c_str(),0);
    }

    if (halt_flag) {
        halt_flag = false;
        THEKERNEL->set_halt_reason(MANUAL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        if(THEKERNEL->is_grbl_mode()) {
            PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort during cycle\r\n", 0);
        } else {
            PacketMessage(PTYPE_NORMAL_INFO, "HALTED, M999 or $X to exit HALT state\r\n", 0);
        }
    }
}

void WifiProvider::on_main_loop(void *argument)
{
    
}

void WifiProvider::PacketMessage(char cmd, const char* s, int size)
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

int WifiProvider::printfcmd(const char cmd, const char *format, ...)
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

int WifiProvider::printf(const char *format, ...)
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

int WifiProvider::puts(const char* s, int size)
{
	size_t total_length = size == 0 ? strlen(s) : size;
    size_t sent_index = 0;
	u16 status = 0;
	u32 sent = 0;
	u32 to_send = 0;
    while (sent_index < total_length) {
    	to_send = total_length - sent_index > WIFI_DATA_MAX_SIZE ? WIFI_DATA_MAX_SIZE : total_length - sent_index;
    	memcpy(WifiData, s + sent_index, to_send);
		// errcode:
		// 	0x13: Wrong link_no used
		// 	0x14: connection by link_no not present
		// 	0x15: connection by link_no closed
		// 	0x18: No clients connecting to this TCP server
		// 	0x1E: too many errors ecountered during sending can not fixed
		// 	0x1F: Other errors
    	sent = M8266WIFI_SPI_Send_BlockData(WifiData, to_send, 5000, tcp_link_no, NULL, 0, &status);
    	sent_index += sent;
		if (sent == to_send) {
			continue;
		} else {
    		break;
		}
    }
    return sent_index;
}

int WifiProvider::_putc(int c)
{
	u16 status = 0;
	u8 to_send = c;
	if (M8266WIFI_SPI_Send_Data(&to_send, 1, tcp_link_no, &status) == 0) {
		return 0;
	} else {
		return 1;
	}
}

int WifiProvider::_getc()
{
	u16 status;
	u8 to_recv = 0, link_no;
	M8266WIFI_SPI_RecvData(&to_recv, 1, WIFI_DATA_TIMEOUT_MS, &link_no, &status);
	return to_recv;
}

int WifiProvider::gets(char** buf, int size)
{
	u8 link_no;
	static u16 received = 0;
	u16 status;
	static uint8_t headerBuffer[2];
    static uint8_t footerBuffer[2];
    static uint16_t bytesNeeded = 2;
    uint16_t expectedLength = 0;
    uint16_t checksum;
    
	if(this->ptrData == 0)
	{
		received = M8266WIFI_SPI_RecvData(WifiData,
				(size == 0 || size > WIFI_DATA_MAX_SIZE) ? WIFI_DATA_MAX_SIZE : size, WIFI_DATA_TIMEOUT_MS, &link_no, &status);
		if (link_no == udp_link_no) {
			// THEKERNEL->streams->printf("gets, data from udp");
			return 0;
		}
		if (int(status & 0xff) == 0x20 || int(status & 0xff) == 0x22 || int(status & 0xff) == 0x2f) {
			THEKERNEL->streams->printf("gets, received: %d, status:%d, high: %d, low: %d!\n", received, status, int(status >> 8), int(status & 0xff));
			return 0;
		}
	}
	
	for (int i = this->ptrData; i < received; i ++) {
		uint8_t byte;
		byte = WifiData[i];
		if(i == received -1)
		{
			this->ptrData = 0;
		}
		else
			this->ptrData = i;
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
            if(++this->ptr_xbuff >= XBUFF_LENGTH) this->ptr_xbuff = XBUFF_LENGTH -1;
            
            if(--bytesNeeded == 0) {
                expectedLength = (xbuff[0] << 8) | xbuff[1];
                this->currentState = READ_DATA;
                bytesNeeded = expectedLength;
            }
            	break;
            
            case READ_DATA:
	            xbuff[this->ptr_xbuff] = byte;
	            if(++this->ptr_xbuff >= XBUFF_LENGTH) this->ptr_xbuff = XBUFF_LENGTH -1;
	            
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

int WifiProvider::CheckFilePacket(char** buf) {
	uint8_t cmdType = 0;
	// CRC校验
    uint16_t calcCRC = 0;
    uint16_t receivedCRC = 0;
    calcCRC = crc16_ccitt(xbuff, this->ptr_xbuff-2); // 最后两个字节是CRC
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


void WifiProvider::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);
    if (gcode->has_m) {
    	if (gcode->m == 481)  {
    		// basic wifi operations
			if (gcode->subcode == 1) {
		    	// reset wifi module
				wifi_init_ok = false;
				init_wifi_module(true);
			} else if (gcode->subcode == 2) {
				// set op mode to STA+AP
				set_wifi_op_mode(3);
			} else if (gcode->subcode == 3) {
				// connect to AP
				u8 connection_state;
				THEKERNEL->streams->printf("M8266WIFI_SPI_Query_Connection...\n");
				//u8 M8266WIFI_SPI_Query_Connection(u8 link_no, u8* connection_type, u8* connection_state,
				//												u16* local_port, u8* remote_ip, u16* remote_port, u16* status);

				if (M8266WIFI_SPI_Query_Connection(tcp_link_no, NULL, &connection_state, NULL, NULL, NULL, NULL) == 0) {
					THEKERNEL->streams->printf("M8266WIFI_SPI_Query_Connection ERROR!\n");
				} else {
					THEKERNEL->streams->printf("connection_state : %d\n", connection_state);
				}
			} else if (gcode->subcode == 4) {
				// test
				gcode->stream->printf("M8266WIFI_SPI_Has_DataReceived...\n");
				if (M8266WIFI_SPI_Has_DataReceived()) {
					gcode->stream->printf("Data Received, receive_wifi_data...\n");
					//receive_wifi_data();
					gcode->stream->printf("Data Received complete!\n");
				}
			} else if (gcode->subcode == 5) {
			} else if (gcode->subcode == 6) {
				char ip_addr[16] = "192.168.1.2";
				char netmask[16] = "255.255.255.0";
				char broadcast[16];
				// get_broadcast_from_ip_and_netmask(broadcast, sizeof(broadcast), ip_addr, netmask);
				{
					// Inlined get_broadcast_from_ip_and_netmask
					uint32_t i_ip = ip_to_int(ip_addr);
					uint32_t i_mask = ip_to_int(netmask);
					uint32_t i_broadcast = i_ip | (i_mask ^ 0xffffffff);
					// Inlined int_to_ip
					unsigned char bytes[4];
					bytes[0] = i_broadcast & 0xFF;
					bytes[1] = (i_broadcast >> 8) & 0xFF;
					bytes[2] = (i_broadcast >> 16) & 0xFF;
					bytes[3] = (i_broadcast >> 24) & 0xFF;
					snprintf(broadcast, sizeof(broadcast), "%d.%d.%d.%d", bytes[3], bytes[2], bytes[1], bytes[0]);
				}
				gcode->stream->printf("broadcast: %s\n", broadcast);
			} else if (gcode->subcode == 7) {
				gcode->stream->printf("aaaaaaa\n");
			}

		} else if (gcode->m == 482) {
	    	u16 status = 0;
	    	char param[64];
	    	u8 param_len = 0;
			memset(param, 0, sizeof(param));
			STA_PARAM_TYPE param_type;
			// STA_PARAM_TYPE_SSID				= 0
			// STA_PARAM_TYPE_PASSWORD			= 1
			// STA_PARAM_TYPE_CHANNEL			= 2
			// STA_PARAM_TYPE_HOSTNAME			= 3
			// STA_PARAM_TYPE_IP_ADDR			= 7
			// STA_PARAM_TYPE_GATEWAY_ADDR		= 8
			// STA_PARAM_TYPE_NETMASK_ADDR		= 9
			// STA_PARAM_TYPE_MAC				= 11
			switch (gcode->subcode) {
			   case 0:
				   param_type = STA_PARAM_TYPE_SSID;
				   break;
			   case 1:
				   param_type = STA_PARAM_TYPE_PASSWORD;
				   break;
			   case 2:
				   param_type = STA_PARAM_TYPE_CHANNEL;
				   break;
			   case 3:
				   param_type = STA_PARAM_TYPE_HOSTNAME;
				   break;
			   case 4:
				   param_type = STA_PARAM_TYPE_MAC;
				   break;
			   case 5:
				   param_type = STA_PARAM_TYPE_IP_ADDR;
				   break;
			   case 6:
				   param_type = STA_PARAM_TYPE_GATEWAY_ADDR;
				   break;
			   case 7:
				   param_type = STA_PARAM_TYPE_NETMASK_ADDR;
				   break;
			   default:
				   param_type = STA_PARAM_TYPE_SSID;
			}
			if (M8266WIFI_SPI_Query_STA_Param(param_type, (u8 *)param, &param_len, &status) == 0) {
				THEKERNEL->streams->printf("Query WiFi STA parameters ERROR!\n");
			} else {
				if (param_type == STA_PARAM_TYPE_CHANNEL) {
					THEKERNEL->streams->printf("STA param[%d]: %d\n", gcode->subcode, *param);
				} else if (param_type == STA_PARAM_TYPE_MAC) {
					THEKERNEL->streams->printf("STA param[%d]: %X-%X-%X-%X-%X-%X\n", gcode->subcode, *param,*(param+1),*(param+2),*(param+3),*(param+4),*(param+5));
				} else {
					THEKERNEL->streams->printf("STA param[%d]: %s\n", gcode->subcode, param);
				}
			}
		} else if (gcode->m == 483) {
			u16 status = 0;
			char param[64];
			u8 param_len = 0;
			memset(param, 0, sizeof(param));
			AP_PARAM_TYPE param_type;
			// AP_PARAM_TYPE_SSID 					= 0
			// AP_PARAM_TYPE_PASSWORD 				= 1
			// AP_PARAM_TYPE_CHANNEL 				= 2
			// AP_PARAM_TYPE_AUTHMODE 				= 3
			// AP_PARAM_TYPE_IP_ADDR				= 7
			// AP_PARAM_TYPE_GATEWAY_ADDR	  		= 8
			// AP_PARAM_TYPE_NETMASK_ADDR	  		= 9
			// AP_PARAM_TYPE_PHY_MODE			  	= 10
			switch (gcode->subcode) {
			   case 0:
				   param_type = AP_PARAM_TYPE_SSID;
				   break;
			   case 1:
				   param_type = AP_PARAM_TYPE_PASSWORD;
				   break;
			   case 2:
				   param_type = AP_PARAM_TYPE_CHANNEL;
				   break;
			   case 3:
				   param_type = AP_PARAM_TYPE_AUTHMODE;
				   break;
			   case 4:
				   param_type = AP_PARAM_TYPE_IP_ADDR;
				   break;
			   case 5:
				   param_type = AP_PARAM_TYPE_GATEWAY_ADDR;
				   break;
			   case 6:
				   param_type = AP_PARAM_TYPE_NETMASK_ADDR;
				   break;
			   case 7:
				   param_type = AP_PARAM_TYPE_PHY_MODE;
				   break;
			   default:
				   param_type = AP_PARAM_TYPE_SSID;
			}
			if (M8266WIFI_SPI_Query_AP_Param(param_type, (u8 *)param, &param_len, &status) == 0) {
				THEKERNEL->streams->printf("Query WiFi AP parameters ERROR!\n");
			} else {
				if (param_type == AP_PARAM_TYPE_CHANNEL || param_type == AP_PARAM_TYPE_AUTHMODE || param_type == AP_PARAM_TYPE_PHY_MODE) {
					THEKERNEL->streams->printf("AP param[%d]: %d\n", gcode->subcode, *param);
				} else {
					THEKERNEL->streams->printf("AP param[%d]: %s\n", gcode->subcode, param);
				}
			}
		} else if (gcode->m == 489) {
			// query wifi status
			query_wifi_status();
		}
    }
}

void WifiProvider::set_wifi_op_mode(u8 op_mode) {
	u16 status = 0;
	// THEKERNEL->streams->printf("M8266WIFI_Config_Connection_via_SPI...\n");
	if (M8266WIFI_SPI_Set_Opmode(op_mode, 1, &status) == 0) {
		THEKERNEL->streams->printf("M8266WIFI_SPI_Set_Opmode, ERROR, status: %d!\n", status);
	} else if (op_mode == 1) {
		THEKERNEL->streams->printf("WiFi Access Point Disabled...\n");
	} else if (op_mode == 3) {
		THEKERNEL->streams->printf("WiFi Access Point Enabled...\n");
	}
}

void WifiProvider::on_get_public_data(void* argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
    if(!pdr->starts_with(wlan_checksum)) return;
    if(!pdr->second_element_is(get_wlan_checksum)
    	&& !pdr->second_element_is(get_rssi_checksum)) return;
	
	if(pdr->second_element_is(get_wlan_checksum)) {
		u8 signals = 0;
		u16 status = 0;
		char ssid[32];
		u8 ssid_len = 0;
		u8 connection_status = 0;
	
		// get current connected information
		M8266WIFI_SPI_Query_STA_Param(STA_PARAM_TYPE_SSID, (u8 *)ssid, &ssid_len, &status);
	
		M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);
	
		ScannedSigs wlans[MAX_WLAN_SIGNALS];
		M8266WIFI_SPI_STA_Scan_Signals(wlans, MAX_WLAN_SIGNALS, 0xff, 0, &status);
		// wait for scan finish
		while (true) {
			signals = M8266WIFI_SPI_STA_Fetch_Last_Scanned_Signals(wlans, MAX_WLAN_SIGNALS, &status);
			if (signals == 0) {
				// 0x25: If not start scan before
				// 0x26: If currently module is scanning
				// 0x27: If last scan result has failure
				// 0x29: Other failure
				if ((status & 0xff) == 0x26) {
					THEKERNEL->call_event(ON_IDLE, this);
					// wait 1 ms
					M8266WIFI_Module_delay_ms(1);
					continue;
				} else {
					// scan fail
					return;
				}
			} else {
				// NOTE caller must free the returned string when done
				size_t n;
				std::string str;
				std::string ssid_str;
				char buf[10];
				for (int i = 0; i < signals; i ++) {
					ssid_str = "";
					for (size_t j = 0; j < strlen(wlans[i].ssid); j ++ ) {
						if(j <32 ){
							ssid_str += wlans[i].ssid[j] == ' ' ? 0x01 : wlans[i].ssid[j];
						}
					}
					ssid_str.append(",");
					// ignore same ssid
				    if (str.find(ssid_str) != string::npos) {
				    	continue;
				    }
					str.append(ssid_str);
					str.append(wlans[i].authmode == 0 ? "0" : "1");
					str.append(",");
					n = snprintf(buf, sizeof(buf), "%d", wlans[i].rssi);
					if(n > sizeof(buf)) n = sizeof(buf);
					str.append(buf, n);
					str.append(",");
					if (strncmp(ssid, wlans[i].ssid, ssid_len <= 32 ? ssid_len : 32) == 0 && connection_status == 5) {
						str.append("1\n");
					} else {
						str.append("0\n");
					}
				}
				char *temp_buf = (char *)malloc(str.length() + 1);
				memcpy(temp_buf, str.c_str(), str.length());
				temp_buf[str.length()]= '\0';
				pdr->set_data_ptr(temp_buf);
				pdr->set_taken();
				return;
			}
		}
	}
	else if( pdr->second_element_is(get_rssi_checksum) ) {
		u8 ssid[32];
		signed char rssi;
		u16 status;
		if(M8266WIFI_SPI_STA_Query_Current_SSID_And_RSSI(ssid, &rssi, &status))
		{
			s8 *data = static_cast<s8 *>(pdr->get_data_ptr());
			data[0] = rssi;
			pdr->set_taken();
			return;
		}
	}
}

int parse_ip(const char *ip, int fields[4]) {
    char *copy = strdup(ip);
    char *token = strtok(copy, ".");
    for (int i = 0; i < 4; i++) {
        if (!token) { free(copy); return 0; }
       
        for (int j = 0; token[j]; j++) {
            if (!isdigit(token[j])) { free(copy); return 0; }
        }
        
        int num = atoi(token);
        if (num < 0 || num > 255) { free(copy); return 0; }
        fields[i] = num;
        token = strtok(NULL, ".");
    }
    free(copy);
    return 1;
}


void WifiProvider::on_set_public_data(void *argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
    if(!pdr->starts_with(wlan_checksum)) return;
    if(!pdr->second_element_is(set_wlan_checksum)
    		&& !pdr->second_element_is(ap_set_channel_checksum)
			&& !pdr->second_element_is(ap_set_ssid_checksum)
			&& !pdr->second_element_is(ap_set_password_checksum)
			&& !pdr->second_element_is(ap_enable_checksum)) return;

    if (pdr->second_element_is(set_wlan_checksum)) {
        ap_conn_info *s = static_cast<ap_conn_info *>(pdr->get_data_ptr());
    	u16 status = 0;
        u8 connection_status;

    	s->has_error = false;
    	if (s->disconnect) {
    		if (M8266WIFI_SPI_STA_DisConnect_Ap(&status) == 0) {
    			s->has_error = true;
    			snprintf(s->error_info, sizeof(s->error_info), "Disconnect error!");
    		}
    	} else {
    	    // u8 M8266WIFI_SPI_STA_Connect_Ap(u8 ssid[32], u8 password[64], u8 saved, u8 timeout_in_s, u16* status);
    	    M8266WIFI_SPI_STA_Connect_Ap((u8 *)s->ssid, (u8 *)s->password, 1, 0, &status);

    		// wait for connection finish
    		while (true) {
    			M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);
    			if (connection_status == 1) {
    				// connecting, wait
    				THEKERNEL->call_event(ON_IDLE, this);
    				// wait 1 ms
    				M8266WIFI_Module_delay_ms(1);
    				continue;
    			} else if (connection_status == 5) {
    				// connection success
    				s->has_error = false;
    				break;
    			} else {
    				s->has_error = true;
    				if (connection_status == 0) {
    					snprintf(s->error_info, sizeof(s->error_info), "No connecting started!");
    				} else if (connection_status == 2) {
    					snprintf(s->error_info, sizeof(s->error_info), "Wifi password incorrect!");
    				} else if (connection_status == 3) {
    					snprintf(s->error_info, sizeof(s->error_info), "No wifi ssid found: %s!", s->ssid);
    				} else if (connection_status == 4) {
    					snprintf(s->error_info, sizeof(s->error_info), "Other error reason!");
    				}
    				break;
    			}
    		}

    		// get ip address if no error
    		if (!s->has_error) {
    			u16 status = 0;
				u8 param_len = 0;
    			char sta_address[16];
    			char ap_address[16];
    			M8266WIFI_SPI_Get_STA_IP_Addr(sta_address, &status);
    			memcpy(s->ip_address,sta_address,16);
    			
    			if( M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE_IP_ADDR, (u8 *)ap_address, &param_len, &status) )
    			{
    				int ip_fields[4], ap_fields[4];
    				if (parse_ip(sta_address, ip_fields) && parse_ip(ap_address, ap_fields)) 
    				{
				        if ((ip_fields[0] == ap_fields[0]) && (ip_fields[1] == ap_fields[1]) && (ip_fields[2] == ap_fields[2])) 
				        {
					    	ap_fields[2] = (ap_fields[2] + 1) % 256; 
						    snprintf(ap_address, 16, "%d.%d.%d.%d", ap_fields[0], ap_fields[1], ap_fields[2], ap_fields[3]);
						    M8266WIFI_SPI_Config_AP_Param(AP_PARAM_TYPE_IP_ADDR, (u8*)ap_address, strlen(ap_address), 1, &status);
						}
				    }
    			}
    			
    		}

    	}
    } else if (pdr->second_element_is(ap_set_channel_checksum)) {
    	u16 status = 0;
    	u8 ap_channel = *static_cast<u8 *>(pdr->get_data_ptr());
		if (M8266WIFI_SPI_Config_AP_Param(AP_PARAM_TYPE_CHANNEL, &ap_channel, 1, 1, &status) == 0) {
			THEKERNEL->streams->printf("WiFi set AP Channel ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
		} else {
			THEKERNEL->streams->printf("WiFi AP Channel has been changed to %d\n", ap_channel);
		}
    } else if (pdr->second_element_is(ap_set_ssid_checksum)) {
    	u16 status = 0;
    	u16 len =0;
    	u8  ssid2[33];
    	char *ssid = static_cast<char *>(pdr->get_data_ptr());
    	memcpy(ssid2, ssid, 32);
    	ssid2[32] = '\0';
    	len = strlen(ssid);
		if (M8266WIFI_SPI_Config_AP_Param(AP_PARAM_TYPE_SSID, ssid2, len, 1, &status) == 0) {
			THEKERNEL->streams->printf("WiFi set AP SSID ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
		} else {
			THEKERNEL->streams->printf("WiFi AP SSID has been changed to %s\n", ssid);
		}
    } else if (pdr->second_element_is(ap_set_password_checksum)) {
    	u16 status = 0;
    	u8 op_mode;

    	//Before set AP, ensure module has AP mode
		if (M8266WIFI_SPI_Get_Opmode(&op_mode, &status) == 0) {
			THEKERNEL->streams->printf("WiFi get OP mode ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
		} else {
			if (op_mode != 3) {
				THEKERNEL->streams->printf("WiFi can not set password under none AP mode!\n");
			} else {
		    	char *password = static_cast<char *>(pdr->get_data_ptr());
		    	u8 authmode = strlen(password) == 0 ? 0 : 4;
				if (M8266WIFI_SPI_Config_AP_Param(AP_PARAM_TYPE_PASSWORD, (u8 *)password, strlen(password), 1, &status) > 0) {
					THEKERNEL->streams->printf("WiFi AP Password has been changed to %s\n", password);
				}
				if (M8266WIFI_SPI_Config_AP_Param(AP_PARAM_TYPE_AUTHMODE, &authmode, 1, 1, &status) == 0) {
					// THEKERNEL->streams->printf("WiFi set AP Auth Mode ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
				}
			}
		}
    } else if (pdr->second_element_is(ap_enable_checksum)) {
    	bool *enable_op = static_cast<bool *>(pdr->get_data_ptr());
    	if (*enable_op) {
        	set_wifi_op_mode(3);
    	} else {
        	set_wifi_op_mode(1);
    	}
    }
	pdr->set_taken();
}


void WifiProvider::query_wifi_status() {
	u16 status = 0;
	u32 esp8266_id;
	u8 flash_size;
	char fw_ver[24] = "";
	THEKERNEL->streams->printf("M8266WIFI_SPI_Get_Module_Info...\n");
	if (M8266WIFI_SPI_Get_Module_Info(&esp8266_id, &flash_size, fw_ver, &status) == 0) {
		THEKERNEL->streams->printf("M8266WIFI_SPI_Get_Module_Info ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
	} else {
		THEKERNEL->streams->printf("esp8266_id:%ld, flash_size:%d, fw_ver:%s!\n",  esp8266_id, flash_size, fw_ver);
	}
}

void WifiProvider::init_wifi_module(bool reset) {
	u16 status = 0;
	char address[20];
	u8 param_len = 0;


	if (reset) {
		THEKERNEL->streams->printf("M8266WIFI_SPI_Delete_Connections...\n");
		// disconnect current links
		if (M8266WIFI_SPI_Delete_Connection( udp_link_no, &status) == 0){
			THEKERNEL->streams->printf("M8266WIFI_SPI_Delete_Connection ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
		}
		if (M8266WIFI_SPI_Delete_Connection( tcp_link_no, &status) == 0){
			THEKERNEL->streams->printf("M8266WIFI_SPI_Delete_Connection ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
		}

		// remove current stream
		THEKERNEL->streams->remove_stream(this);
	}


	// THEKERNEL->streams->printf("M8266WIFI_Module_Init_Via_SPI...\n");

	M8266HostIf_Init();

	if (M8266WIFI_Module_Init_Via_SPI() == 0) {
		THEKERNEL->streams->printf("M8266WIFI_Module_Init_Via_SPI, ERROR!\n");
	}

	// init udp and tcp server connection
	// THEKERNEL->streams->printf("Init UDP and TCP connection...\n");
	// setup TCP Connection
	snprintf(address, sizeof(address), "192.168.4.10");
	if (M8266WIFI_SPI_Setup_Connection(2, this->tcp_port, address, 0, tcp_link_no, 3, &status) == 0) {
		THEKERNEL->streams->printf("M8266WIFI_SPI_Setup_Connection ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
	}
	// setup UDP Connection
	snprintf(address, sizeof(address), "192.168.4.255");
	if (M8266WIFI_SPI_Setup_Connection(0, this->udp_recv_port, address, 0, udp_link_no, 3, &status) == 0) {
		THEKERNEL->streams->printf("M8266WIFI_SPI_Setup_Connection ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
	}

	// set timeout
	if( M8266WIFI_SPI_Set_TcpServer_Auto_Discon_Timeout(tcp_link_no, tcp_timeout_s, &status) == 0)
	{
		THEKERNEL->streams->printf("M8266WIFI_SPI_Set_TcpServer_Auto_Discon_Timeout ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
	}

	// load current AP IP and Netmask
	if( M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE_IP_ADDR, (u8 *)this->ap_address, &param_len, &status) == 0)
	{
		THEKERNEL->streams->printf("Get AP_PARAM_TYPE_IP_ADDR ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
	}
	if( M8266WIFI_SPI_Query_AP_Param(AP_PARAM_TYPE_NETMASK_ADDR, (u8 *)this->ap_netmask, &param_len, &status) == 0)
	{
		THEKERNEL->streams->printf("Get AP_PARAM_TYPE_NETMASK_ADDR ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
	}

	if (reset) {
		// append stream again
		THEKERNEL->streams->append_stream(this);
	}

	wifi_init_ok = true;
}

void WifiProvider::M8266WIFI_Module_delay_ms(u16 nms)
{
	u16 i, j;
	for(i=0; i<nms; i++)
		for(j=0; j<4; j++)					// delay 1ms. Call 4 times of delay_us(250), as M8266HostIf_delay_us(u8 nus), nus max 255
			M8266HostIf_delay_us(250);
}

void WifiProvider::M8266WIFI_Module_Hardware_Reset(void) // total 800ms  (Chinese: 本例子中这个函数的总共执行时间大约800毫秒)
{
	M8266HostIf_Set_SPI_nCS_Pin(0);   			// Module nCS==ESP8266 GPIO15 as well, Low during reset in order for a normal reset (Chinese: 为了实现正常复位，模块的片选信号nCS在复位期间需要保持拉低)
	M8266WIFI_Module_delay_ms(1); 	    		// delay 1ms, adequate for nCS stable (Chinese: 延迟1毫秒，确保片选nCS设置后有足够的时间来稳定)

	M8266HostIf_Set_nRESET_Pin(0);					// Pull low the nReset Pin to bring the module into reset state (Chinese: 拉低nReset管脚让模组进入复位状态)
	M8266WIFI_Module_delay_ms(5);      		// delay 5ms, adequate for nRESET stable(Chinese: 延迟5毫秒，确保片选nRESER设置后有足够的时间来稳定，也确保nCS和nRESET有足够的时间同时处于低电平状态)
	                                        // give more time especially for some board not good enough
	                                        //(Chinese: 如果主板不是很好，导致上升下降过渡时间较长，或者因为失配存在较长的振荡时间，所以信号到轨稳定的时间较长，那么在这里可以多给一些延时)

	M8266HostIf_Set_nRESET_Pin(1);					// Pull high again the nReset Pin to bring the module exiting reset state (Chinese: 拉高nReset管脚让模组退出复位状态)
	M8266WIFI_Module_delay_ms(300); 	  		// at least 18ms required for reset-out-boot sampling boottrap pin (Chinese: 至少需要18ms的延时来确保退出复位时足够的boottrap管脚采样时间)
	                                        // Here, we use 300ms for adequate abundance, since some board GPIO, (Chinese: 在这里我们使用了300ms的延时来确保足够的富裕量，这是因为在某些主板上，)
																					// needs more time for stable(especially for nRESET) (Chinese: 他们的GPIO可能需要较多的时间来输出稳定，特别是对于nRESET所对应的GPIO输出)
																					// You may shorten the time or give more time here according your board v.s. effiency
																					// (Chinese: 如果你的主机板在这里足够好，你可以缩短这里的延时来缩短复位周期；反之则需要加长这里的延时。
																					//           总之，你可以调整这里的时间在你们的主机板上充分测试，找到一个合适的延时，确保每次复位都能成功。并适当保持一些富裕量，来兼容批量化时主板的个体性差异)
	M8266HostIf_Set_SPI_nCS_Pin(1);         // release/pull-high(defualt) nCS upon reset completed (Chinese: 释放/拉高(缺省)片选信号
	//M8266WIFI_Module_delay_ms(1); 	    		// delay 1ms, adequate for nCS stable (Chinese: 延迟1毫秒，确保片选nCS设置后有足够的时间来稳定)

	M8266WIFI_Module_delay_ms(800-300-5-2); // Delay more than around 500ms for M8266WIFI module bootup and initialization，including bootup information print。No influence to host interface communication. Could be shorten upon necessary. But test for verification required if adjusted.
	                                        // (Chinese: 延迟大约500毫秒，来等待模组成功复位后完成自己的启动过程和自身初始化，包括串口信息打印。但是此时不影响模组和单片主机之间的通信，这里的时间可以根据需要适当调整.如果调整缩短了这里的时间，建议充分测试，以确保系统(时序关系上的)可靠性)
}

u8 WifiProvider::M8266WIFI_Module_Init_Via_SPI()
{
	u16 status = 0;
	uint32_t spi_clk = 24000000;

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//Step 1: To hardware reset the module (with nCS=0 during reset) and wait up the module bootup
	//(Chinese: 步骤1：对模组执行硬复位时序(在片选nCS拉低的时候对nRESET管脚输出低高电平)，并等待模组复位启动完毕
	M8266WIFI_Module_Hardware_Reset();


	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step2: Try SPI clock in a fast one as possible up to 40MHz (M8266WIFI could support only upto 40MHz SPI)
	// (Chinese: 第二步，在确保SPI底层通信可靠的前提下，调整SPI时钟尽可能的快，以支持最快速度通信。本模组最大可以支持40MHz的SPI频率)
	#ifndef SPI_BaudRatePrescaler_2
	#define SPI_BaudRatePrescaler_2         ((u32)0x00000002U)
	#define SPI_BaudRatePrescaler_4         ((u32)0x00000004U)
	#define SPI_BaudRatePrescaler_6         ((u32)0x00000006U)
	#define SPI_BaudRatePrescaler_8         ((u32)0x00000008U)
	#define SPI_BaudRatePrescaler_16        ((u32)0x00000010U)
	#define SPI_BaudRatePrescaler_32        ((u32)0x00000020U)
	#define SPI_BaudRatePrescaler_64        ((u32)0x00000040U)
	#define SPI_BaudRatePrescaler_128       ((u32)0x00000080U)
	#define SPI_BaudRatePrescaler_256       ((u32)0x00000100U)
	#endif

	M8266HostIf_SPI_SetSpeed(SPI_BaudRatePrescaler_4);					// Setup SPI Clock. Here 96/4 = 24MHz for LPC17XX, upto 40MHz
	spi_clk = 24000000;

	// wait clock stable (Chinese: 设置SPI时钟后，延时等待时钟稳定)
	M8266WIFI_Module_delay_ms(1);

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step3: It is very mandatory to call M8266HostIf_SPI_Select() to tell the driver which SPI you used and how faster the SPI clock you used. The function must be called before SPI access
	//(Chinese: 第三步：调用M8266HostIf_SPI_Select()。 在正式调用驱动API函数和模组进行通信之前，调用M8266HostIf_SPI_Select()来告诉驱动使用哪个SPI以及SPI的时钟有多快，这一点非常重要。
	//                  如果没有调用这个API，单片机主机和模组之间将可能将无法通信)
	if(M8266HostIf_SPI_Select((uint32_t)M8266WIFI_INTERFACE_SPI, spi_clk, &status) == 0)
	{
		THEKERNEL->streams->printf("M8266HostIf_SPI_Select ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
		return 0;
	}

#if 0 //只在硬件测试阶段打开，测试SPI总线可靠性
	// Step 4: Used to evaluate the high-speed spi communication. Changed to #if 0 to comment it for formal release
	//(Chinese: 第四步，开发阶段和测试阶段，用于测试评估主机板在当前频率下进行高速SPI读写访问时的可靠性。
	//          如果足够可靠，则可以适当提高SPI频率；如果不可靠，则可能需要检查主机板连线或者降低SPI频率。
    //		       产品研发完毕进入正式产品化发布阶段后，因为在研发阶段已经确立了最佳稳定频率，建议这里改成 #if 0，不必再测试)
	volatile u32  i, j;
	u8   byte;

	if(M8266WIFI_SPI_Interface_Communication_OK(&byte)==0) 	  									//	if SPI logical Communication failed
    {
		THEKERNEL->streams->printf("Communication test ERROR!\n");
		return 0;
    }

	i = 100000;
	j = M8266WIFI_SPI_Interface_Communication_Stress_Test(i);
	if( (j < i) && (i - j > 5)) 		//  if SPI Communication stress test failed (Chinese: SPI底层通信压力测试失败，表明你的主机板或接线支持不了当前这么高的SPI频率设置)
	{
		THEKERNEL->streams->printf("Wifi Module Stress test ERROR!\n");
		return 0;
	}
#endif
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 5: Conifiguration to module
	// (Chinese:第5步：配置模组)

	// 5.1 If you hope to reduce the Max Tx power, you could enable it by change to "#if 1" (Chinese: 5.1 如果你希望减小模组的最大发射功率，可以将这里改成 #if 1，并调整下面的 tx_max_power参数的值)
	//u8 M8266WIFI_SPI_Set_Tx_Max_Power(u8 tx_max_power, u16 *status)
	if(M8266WIFI_SPI_Set_Tx_Max_Power(68, &status)==0)   // tx_max_power=68 to set the max tx power of aroud half of manufacture default, i.e. 50mW or 17dBm. Refer to the API specification for more info
	{
		THEKERNEL->streams->printf("M8266WIFI_SPI_Set_Tx_Max_Power ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
		return 0;                                          // (Chinese: tx_max_power=68表示将发射最大功率设置为出厂缺省数值的一般，即50mW或者17dBm。具体数值含义可以查看这个API函数的头文件声明里的注释
	}

#if 0
    //u8 M8266WIFI_SPI_Set_WebServer(u8 open_not_shutdown, u16 server_port, u8 saved, u16* status)
    if(M8266WIFI_SPI_Set_WebServer(1, 80, 0, &status)==0) 
	{
		THEKERNEL->streams->printf("M8266WIFI_SPI_Set_Tx_Max_Power ERROR, status:%d, high: %d, low: %d!\n", status, int(status >> 8), int(status & 0xff));
		return 0;                                          // (Chinese: tx_max_power=68表示将发射最大功率设置为出厂缺省数值的一般，即50mW或者17dBm。具体数值含义可以查看这个API函数的头文件声明里的注释
	}    
#endif

	return 1;
}

int WifiProvider::type() {
	return 1;
}



