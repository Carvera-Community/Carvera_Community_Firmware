#ifndef WIFIPUBLICACCESS_H
#define WIFIPUBLICACCESS_H

#define wlan_checksum       CHECKSUM("wlan")
#define get_wlan_checksum   CHECKSUM("get_wlan")
#define set_wlan_checksum   CHECKSUM("set_wlan")
#define ap_set_channel_checksum   CHECKSUM("ap_set_channel")
#define ap_set_ssid_checksum   CHECKSUM("ap_set_ssid")
#define ap_set_password_checksum   CHECKSUM("ap_set_password")
#define ap_enable_checksum   CHECKSUM("ap_enable")

struct ap_conn_info {
    char ssid[32];
    char password[64];
    char ip_address[15];
    bool has_error;
    char error_info[64];
    bool disconnect;
};


#endif
