/*
 * WifiProvider.h
 *
 *  Created on: 2020年6月10日
 *      Author: josh
 */

#ifndef WIFIPROVIDER_H_
#define WIFIPROVIDER_H_

using namespace std;

#include "Pin.h"
#include "Module.h"

#include "M8266WIFIDrv.h"
#include "CommunicationProtocol.h"
#include "WifiConstants.h"
#include "WifiTcpEndpoint.h"

#include <cstddef>

class WifiProvider : public Module
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

private:
    friend class WifiTcpEndpoint;

    static constexpr size_t TcpEndpointCount = 1;

    struct ReceivedData {
        u16 copied;
        u8 link_no;
        u16 status;
        bool partial;
    };

    void M8266WIFI_Module_delay_ms(u16 nms);
    void set_wifi_op_mode(u8 op_mode);

    void M8266WIFI_Module_Hardware_Reset(void);
    u8 M8266WIFI_Module_Init_Via_SPI();

    void init_wifi_module(bool reset);
    void query_wifi_status();

    uint32_t ip_to_int(const char* ip_addr);
    void int_to_ip(uint32_t i_ip, char *ip_addr, size_t buffer_size);
    void get_broadcast_from_ip_and_netmask(char *broadcast_addr, size_t broadcast_buffer_size, const char *ip_addr, const char *netmask);

    void on_pin_rise();
    void receive_wifi_data();
    void service_realtime_flags();
    void configure_tcp_endpoints();
    void append_tcp_streams();
    void remove_tcp_streams();
    void send_discovery_packet(const char *local_address, const char *netmask, bool client_connected);
    WifiTcpEndpoint *tcp_endpoint(size_t index);
    WifiTcpEndpoint *endpoint_for_link(u8 link_no);
    WifiTcpEndpoint *endpoint_for_protocol(comms::Protocol protocol);
    WifiTcpEndpoint *first_enabled_endpoint();
    int configured_smoothie_port() const;
    ReceivedData receive_from_module(u8 *buffer, u16 capacity);

    int send_on_link(u8 link_no, const char *s, int size);
    int send_byte_on_link(u8 link_no, int c);
    int read_byte_for(WifiTcpEndpoint& endpoint);
    int read_raw_for(WifiTcpEndpoint& endpoint, char **buf, int size);
    int read_framed_file_for(WifiTcpEndpoint& endpoint, char **buf, int size);

    mbed::InterruptIn *wifi_interrupt_pin; // Interrupt pin for measuring speed
    float probe_slow_rate;

    string test_buffer;

    WifiTcpEndpoint smoothie_endpoint;
    WifiTcpEndpoint *discovery_endpoint;
    u8 WifiData[WifiDataMaxSize];
    u8 WifiTxData[WifiDataMaxSize];

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
    	u8  udp_link_no;
        bool wifi_init_ok:1;
        volatile bool has_data_flag:1;
        bool streams_appended:1;
    };

};

#endif /* WIFIPROVIDER_H_ */
