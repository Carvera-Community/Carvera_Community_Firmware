/*
 * WifiProvider.h
 *
 *  Created on: 2020年6月10日
 *      Author: josh
 */

#ifndef WIFIPROVIDER_H_
#define WIFIPROVIDER_H_

using namespace std;
#include <vector>
#include <queue>

#include "Pin.h"
#include "Module.h"
#include "StreamOutput.h"

#include "M8266WIFIDrv.h"
#include "libs/RingBuffer.h"

#define WIFI_DATA_MAX_SIZE 1460
#define WIFI_DATA_TIMEOUT_MS 10
#define MAX_WLAN_SIGNALS 8

enum ParseState { WAIT_HEADER, READ_LENGTH, READ_DATA, CHECK_FOOTER };
class WifiProvider : public Module, public StreamOutput
{
public:
	WifiProvider();

    void on_module_loaded();
    void on_gcode_received(void *argument);
    void on_main_loop( void* argument );
    void on_second_tick(void* argument);
    void on_idle(void* argument);
    void on_get_public_data(void* argument);
    void on_set_public_data(void* argument);

    int gets(char** buf, int size = 0);
    int puts(const char*, int size = 0);
    int _putc(int c);
    int _getc(void);
    bool ready();
    bool has_char(char letter);
    int type(); // 0: serial, 1: wifi
    void reset(void){ptrData=0;ptr_xbuff=0;currentState = WAIT_HEADER;};
    int printfcmd(const char cmd, const char *format, ...);
    int printf(const char *format, ...) __attribute__ ((format(printf, 2, 3)));


private:
    void M8266WIFI_Module_delay_ms(u16 nms);
    void set_wifi_op_mode(u8 op_mode);

    void M8266WIFI_Module_Hardware_Reset(void);
    u8 M8266WIFI_Module_Init_Via_SPI();

    void init_wifi_module(bool reset);
    void query_wifi_status();

    uint32_t ip_to_int(char* ip_addr);
    void int_to_ip(uint32_t i_ip, char *ip_addr, size_t buffer_size);
    void get_broadcast_from_ip_and_netmask(char *broadcast_addr, size_t broadcast_buffer_size, char *ip_addr, char *netmask);

    void on_pin_rise();
    void receive_wifi_data();
    void processPacket();
    unsigned int crc16_ccitt(unsigned char *data, unsigned int len);
    int CheckFilePacket(char** buf);
    
    
    void PacketMessage(char cmd, const char* s, int size);

    mbed::InterruptIn *wifi_interrupt_pin; // Interrupt pin for measuring speed
    float probe_slow_rate;

	u8 WifiData[WIFI_DATA_MAX_SIZE];

	int tcp_port;
	int udp_send_port;
	int udp_recv_port;
	int tcp_timeout_s;
	int connection_fail_count;
	char machine_name[64]; // Fixed-size buffer to avoid std::string heap allocation
	char ap_address[16];
	char ap_netmask[16];
	char sta_address[16];
	char sta_netmask[16];

    struct {
    	u8  tcp_link_no;
    	u8  udp_link_no;
    	bool wifi_init_ok:1;
    	volatile bool halt_flag:1;
    	volatile bool query_flag:1;
    	volatile bool diagnose_flag:1;
    	volatile bool has_data_flag:1;
    };
    
    ParseState currentState = WAIT_HEADER;    
    std::vector<uint8_t> packetData;
    int ptrData;
    int ptr_xbuff;
    
};

#endif /* WIFIPROVIDER_H_ */
