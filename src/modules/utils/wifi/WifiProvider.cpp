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
	this->machine_name = THEKERNEL->config->value(wifi_checksum, machine_name_checksum)->by_default("CARVERA")->as_string();

    // Init Wifi Module
    this->init_wifi_module(false);

    // Add interrupt for WIFI data receving
    Pin *smoothie_pin = new Pin();
    smoothie_pin->from_string(THEKERNEL->config->value(wifi_checksum, wifi_interrupt_pin_checksum)->by_default("2.11")->as_string());
    smoothie_pin->as_input();
    if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
        PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
        wifi_interrupt_pin = new mbed::InterruptIn(pinname);
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

	while (true)
	{
		received = M8266WIFI_SPI_RecvData(WifiData, WIFI_DATA_MAX_SIZE, WIFI_DATA_TIMEOUT_MS, &link_no, &status);
		if (link_no == udp_link_no) {
			return;
		}
		for (int i = 0; i < received; i ++) {
	        if(WifiData[i] == '?') {
	            query_flag = true;
	            continue;
	        }
			if (WifiData[i] == '*') {
				diagnose_flag = true;
				continue;
			}
	        if(WifiData[i] == 'X' - 'A' + 1) { // ^X
	            halt_flag = true;
	            continue;
	        }
	        if(THEKERNEL->is_feed_hold_enabled()) {
	            if(WifiData[i] == '!') { // safe pause
	                THEKERNEL->set_feed_hold(true);
	                continue;
	            }

	            if(WifiData[i] == '~') { // safe resume
	                THEKERNEL->set_feed_hold(false);
	                continue;
	            }
	        }
	        // convert CR to NL (for host OSs that don't send NL)
	        if( WifiData[i] == '\r' ) {
//	        	received = '\n';
				WifiData[i] = '\n';
	        }
	        this->buffer.push_back(char(WifiData[i]));
		}
		if (received < WIFI_DATA_MAX_SIZE) {
			return;
		}
	}
}

bool WifiProvider::ready() {
	return M8266WIFI_SPI_Has_DataReceived();
}

void WifiProvider::get_broadcast_from_ip_and_netmask(char *broadcast_addr, char *ip_addr, char *netmask)
{
	uint32_t i_ip = ip_to_int(ip_addr);
	uint32_t i_mask = ip_to_int(netmask);
	uint32_t i_broadcast = i_ip | (i_mask ^ 0xffffffff);
	int_to_ip(i_broadcast, broadcast_addr);
}

void WifiProvider::int_to_ip(uint32_t i_ip, char *ip_addr) {
    unsigned char bytes[4];
    bytes[0] = i_ip & 0xFF;
    bytes[1] = (i_ip >> 8) & 0xFF;
    bytes[2] = (i_ip >> 16) & 0xFF;
    bytes[3] = (i_ip >> 24) & 0xFF;
	snprintf(ip_addr, 16, "%d.%d.%d.%d", bytes[3], bytes[2], bytes[1], bytes[0]);
}

uint32_t WifiProvider::ip_to_int(char* ip_addr) {
  unsigned char bytes[4];
  sscanf(ip_addr, "%u.%u.%u.%u", &bytes[0], &bytes[1], &bytes[2], &bytes[3]);
  return bytes[0] * 16777216 + bytes[1] * 65536 + bytes[2] * 256 + bytes[3];
}

void WifiProvider::on_second_tick(void *)
{
	u16 status = 0;
	char address[16];
	char udp_buff[100];
	u8 param_len = 0;
	u8 connection_status = 0;
	u8 client_num = 0;
	ClientInfo RemoteClients[15];

	if (!wifi_init_ok || THEKERNEL->is_uploading()) return;

	M8266WIFI_SPI_List_Clients_On_A_TCP_Server(tcp_link_no, &client_num, RemoteClients, &status);

	M8266WIFI_SPI_Get_STA_Connection_Status(&connection_status, &status);
	// THEKERNEL->streams->printf("M8266WIFI_SPI_Get_STA_Connection_Status: [%d]!\n", connection_status);
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
		if (connection_fail_count > 10) {
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
        puts(THEKERNEL->get_query_string().c_str());
    }

    if (diagnose_flag) {
    	diagnose_flag = false;
    	puts(THEKERNEL->get_diagnose_string().c_str(), 0);
    }

    if (halt_flag) {
        halt_flag = false;
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(MANUAL);
        if(THEKERNEL->is_grbl_mode()) {
            puts("ALARM: Abort during cycle\r\n");
        } else {
            puts("HALTED, M999 or $X to exit HALT state\r\n");
        }
    }
}

void WifiProvider::on_main_loop(void *argument)
{
    if( this->has_char('\n') ){
        string received;
        received.reserve(20);
        while(1){
           char c;
           this->buffer.pop_front(c);
           if( c == '\n' ){
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                return;
            }else{
                received += c;
            }
        }
    }
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
	u16 status;
	u8 link_no;
	u16 received = M8266WIFI_SPI_RecvData(WifiData,
			(size == 0 || size > WIFI_DATA_MAX_SIZE) ? WIFI_DATA_MAX_SIZE : size, WIFI_DATA_TIMEOUT_MS, &link_no, &status);
	if (link_no == udp_link_no) {
		// THEKERNEL->streams->printf("gets, data from udp");
		return 0;
	}
	if (int(status & 0xff) == 32 || int(status & 0xff) == 34 || int(status & 0xff) == 47) {
		THEKERNEL->streams->printf("gets, received: %d, status:%d, high: %d, low: %d!\n", received, status, int(status >> 8), int(status & 0xff));
	}
	*buf = (char *)&WifiData;
	return received;
}

// Does the queue have a given char ?
bool WifiProvider::has_char(char letter) {
    int index = this->buffer.tail;
    while( index != this->buffer.head ){
        if( this->buffer.buffer[index] == letter ){
            return true;
        }
        index = this->buffer.next_block_index(index);
    }
    return false;
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
				get_broadcast_from_ip_and_netmask(broadcast, ip_addr, netmask);
				gcode->stream->printf("broadcast: %s\n", broadcast);
			} else if (gcode->subcode == 7) {
				gcode->stream->printf("aaaaaaa\n");
				gcode->stream->printf("test buffer: %s\n", test_buffer.c_str());
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
					THEKERNEL->streams->printf("STA param[%d]: %d\n", gcode->subcode, param_len);
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
    if(!pdr->second_element_is(get_wlan_checksum)) return;

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
					ssid_str += wlans[i].ssid[j] == ' ' ? 0x01 : wlans[i].ssid[j];
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
    			M8266WIFI_SPI_Get_STA_IP_Addr(s->ip_address, &status);
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
    	char *ssid = static_cast<char *>(pdr->get_data_ptr());
		if (M8266WIFI_SPI_Config_AP_Param(AP_PARAM_TYPE_SSID, (u8 *)ssid, strlen(ssid), 1, &status) == 0) {
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
	char address[16];
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

	return 1;
}

int WifiProvider::type() {
	return 1;
}



