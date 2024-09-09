/********************************************************************
 * M8266HostIf.h
 * .Description
 *     header file of M8266WIFI Host Interface
 * .Copyright(c) Anylinkin Technology 2015.5-
 *     IoT@anylinkin.com
 *     http://www.anylinkin.com
 *     http://anylinkin.taobao.com
 *  Author
 *     wzuo
 *  Date
 *  Version
 ********************************************************************/
#ifndef _M8266_HOST_IF_H_
#define _M8266_HOST_IF_H_

void M8266HostIf_Init(void);
void M8266HostIf_GPIO_CS_RESET_Init(void);

void M8266HostIf_SPI_SetSpeed(u32 SPI_BaudRatePrescaler);

void M8266HostIf_usart_txd_data(char* data, u16 len);
void M8266HostIf_usart_txd_string(char* str);
void M8266HostIf_USART_Reset_RX_BUFFER(void);
u16 M8266HostIf_USART_RX_BUFFER_Data_count(void);
u16 M8266HostIf_USART_get_data(u8* data, u16 len);

#endif

