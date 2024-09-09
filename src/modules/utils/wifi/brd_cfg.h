/********************************************************************
 * brd_cfg.h
 * .Description
 *     header file of main board configuration macros
 * .Copyright(c) Anylinkin Technology 2015.5-
 *     IoT@anylinkin.com
 *     http://www.anylinkin.com
 *     http://anylinkin.taobao.com
 *  Author
 *     wzuo
 *  Date
 *  Version
 ********************************************************************/
#ifndef _BRG_CFG_H_
#define _BRG_CFG_H_

#include "LPC17xx.h"

//#define MCU_IS_STM32F1XX
//#define MCU_IS_STM32F2XX
//#define MCU_IS_STM32F3XX
//#define MCU_IS_STM32F4XX
//#define MCU_IS_STM32F7XX

//#define MCU_IS_STM32L4XX

#define MCU_IS_NXP_LPC17XX
//#define MCU_IS_NXP_MK27_28

//#define MCU_IS_HT32F16XX

//#define M8266WIFI_CONFIG_VIA_USART
#define M8266WIFI_CONFIG_VIA_SPI

//#define M8266WIFI_SPI_ACCESS_USE_DMA

#if defined(MCU_IS_HT32F16XX)
#include "ht32f1655_56.h"
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// M8266WIFI Module Interface Board Usages Macros
//      -- nRESET, nCS
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// nRESET
#if   defined(MCU_IS_STM32F1XX)  // STM's STM32F1XX, using *GPIOB12* as nRESET
#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)
#define  M8266WIFI_nRESET_GPIO											GPIOB
#define  M8266WIFI_nRESET_PIN												GPIO_Pin_12
#elif defined(MCU_IS_STM32F2XX)  // STM's STM32F2XX, //using *GPIOD0* as nRESET
#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define  M8266WIFI_nRESET_GPIO											GPIOD
#define  M8266WIFI_nRESET_PIN												GPIO_Pin_0
#elif defined(MCU_IS_STM32F3XX)  // STM's STM32F3XX, //using *GPIOC9* as nRESET
#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE)
#define  M8266WIFI_nRESET_GPIO											GPIOC
#define  M8266WIFI_nRESET_PIN												GPIO_Pin_9
#elif defined(MCU_IS_STM32F4XX)  // STM's STM32F4XX,
//using *GPIOD13* as nRESET
//#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
//#define  M8266WIFI_nRESET_GPIO											GPIOD
//#define  M8266WIFI_nRESET_PIN												GPIO_Pin_13
//using *GPIOB9* as nRESET
#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define  M8266WIFI_nRESET_GPIO											GPIOB
#define  M8266WIFI_nRESET_PIN												GPIO_Pin_9
//using *GPIOA0* as nRESET
//#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
//#define  M8266WIFI_nRESET_GPIO											GPIOA
//#define  M8266WIFI_nRESET_PIN												GPIO_Pin_0
//using *GPIOE4* as nRESET
//#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
//#define  M8266WIFI_nRESET_GPIO											GPIOE
//#define  M8266WIFI_nRESET_PIN												GPIO_Pin_4
#elif defined(MCU_IS_STM32F7XX)  // STM's STM32F7XX, using *GPIOB2* as nRESET  // avoid to use PE3, boards like Aplolo have problems in output high simutenous to nearby PIO e.g. PE3&PE4
#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  __HAL_RCC_GPIOE_CLK_ENABLE()
#define  M8266WIFI_nRESET_GPIO											GPIOE
#define  M8266WIFI_nRESET_PIN												GPIO_PIN_3
//#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  __HAL_RCC_GPIOB_CLK_ENABLE()
//#define  M8266WIFI_nRESET_GPIO											GPIOB
//#define  M8266WIFI_nRESET_PIN												GPIO_PIN_2
#elif defined(MCU_IS_STM32L4XX)  // STM's STM32L4XX, using *GPIOC3* as nRESET
#define  M8266WIFI_nRESET_GPIO_RCC_CLOCK_EN				  __HAL_RCC_GPIOC_CLK_ENABLE()
#define  M8266WIFI_nRESET_GPIO											GPIOC
#define  M8266WIFI_nRESET_PIN												GPIO_PIN_3
#elif defined(MCU_IS_NXP_LPC17XX) // NXP's LPC17XX,  using *GPIO[2,7]* as nRESET
#define  M8266WIFI_nRESET_GPIO											LPC_GPIO2
#define  M8266WIFI_nRESET_PIN												10
#elif defined(MCU_IS_NXP_MK27_28)   	// NXP's MK27/MK28, using *PTE4* as nRESET
#define  M8266WIFI_nRESET_PORT											PORTE
#define  M8266WIFI_nRESET_PORT_CLOCK								kCLOCK_PortE
#define  M8266WIFI_nRESET_GPIO											GPIOE
#define  M8266WIFI_nRESET_PIN												4
#elif defined(MCU_IS_HT32F16XX)   // Holtek's HT32F16XX, using *GPIOA2* as nRESET
#define  M8266WIFI_nRESET_GPIO_CLK_BIT							CKCUClock.Bit.PA
#define  M8266WIFI_nRESET_GPIO											HT_GPIOA
#define  M8266WIFI_nRESET_PIN												GPIO_PIN_2
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif
/// nCS
#if   defined(MCU_IS_STM32F1XX)  // STM's STM32F1XX, using *GPIOA4* as nCS
#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)
#define  M8266WIFI_SPI_nCS_GPIO											GPIOA
#define  M8266WIFI_SPI_nCS_PIN											GPIO_Pin_4
#elif defined(MCU_IS_STM32F2XX)  // STM's STM32F2XX, // using *GPIOA15* as nCS
#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define  M8266WIFI_SPI_nCS_GPIO											GPIOA
#define  M8266WIFI_SPI_nCS_PIN											GPIO_Pin_15
#elif defined(MCU_IS_STM32F3XX)  // STM's STM32F3XX, // using *GPIOC8* as nCS
#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN				RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE)
#define  M8266WIFI_SPI_nCS_GPIO											GPIOC
#define  M8266WIFI_SPI_nCS_PIN											GPIO_Pin_8
#elif defined(MCU_IS_STM32F4XX)  // STM's STM32F4XX
//using *GPIOA8* as nCS
//#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
//#define  M8266WIFI_SPI_nCS_GPIO											GPIOA
//#define  M8266WIFI_SPI_nCS_PIN											GPIO_Pin_8
//using *GPIOD3* as nCS
#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define  M8266WIFI_SPI_nCS_GPIO											GPIOD
#define  M8266WIFI_SPI_nCS_PIN											GPIO_Pin_3
//using *GPIOA4* as nCS
//#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
//#define  M8266WIFI_SPI_nCS_GPIO										GPIOA
//#define  M8266WIFI_SPI_nCS_PIN										GPIO_Pin_4
//using *GPIOE6* as nCS
//#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
//#define  M8266WIFI_SPI_nCS_GPIO										GPIOE
//#define  M8266WIFI_SPI_nCS_PIN										GPIO_Pin_6
#elif defined(MCU_IS_STM32F7XX)  // STM's STM32F7XX, using *GPIOE4* as nCS
#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN				__HAL_RCC_GPIOE_CLK_ENABLE()
#define  M8266WIFI_SPI_nCS_GPIO											GPIOE
#define  M8266WIFI_SPI_nCS_PIN											GPIO_PIN_4
#elif defined(MCU_IS_STM32L4XX)  // STM's STM32L4XX, using *GPIOC2* as nCS
#define  M8266WIFI_SPI_nCS_GPIO_RCC_CLOCK_EN				__HAL_RCC_GPIOC_CLK_ENABLE()
#define  M8266WIFI_SPI_nCS_GPIO											GPIOC
#define  M8266WIFI_SPI_nCS_PIN											GPIO_PIN_2
#elif defined(MCU_IS_NXP_LPC17XX) // NXP's LPC17XX, using *GPIO[0,16]* as nCS
#define  M8266WIFI_SPI_nCS_GPIO											LPC_GPIO0
#define  M8266WIFI_SPI_nCS_PIN											6
#elif defined(MCU_IS_NXP_MK27_28)   	// NXP's MK27/MK28, using *PTD15* as nCS
#define  M8266WIFI_SPI_nCS_PORT											PORTD
#define  M8266WIFI_SPI_nCS_PORT_CLOCK								kCLOCK_PortD
#define  M8266WIFI_SPI_nCS_GPIO											GPIOD
#define  M8266WIFI_SPI_nCS_PIN											15
#elif defined(MCU_IS_HT32F16XX)   // Holtek's HT32F16XX, using *GPIOA3* as nCS
#define  M8266WIFI_SPI_nCS_GPIO_CLK_BIT							CKCUClock.Bit.PA
#define  M8266WIFI_SPI_nCS_GPIO											HT_GPIOA
#define  M8266WIFI_SPI_nCS_PIN											GPIO_PIN_3
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// M8266WIFI Module Interface Board Usages Macros
//     - MOSI - MISO - CLK
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      PIN   GND - nRST - nCS - MOSI - MISO - CLK
////For STM32, see M8266HostIf_STM32xx.c
//- If uses SPI1, SPI1_MOSI=GPIOA7,  SPI1_MISO=GPIOA6,  SPI1_SCK=GPIOA5   -- this default
//            or  SPI1_MOSI=GPIOB5,  SPI1_MISO=GPIOB4,  SPI1_SCK=GPIOB3
//- If uses SPI2, SPI2_MOSI=GPIOB15, SPI3_MISO=GPIOB14, SPI3_SCK=GPIOB13  -- this default
//            or  SPI2_MOSI=GPIOC3,  SPI1_MISO=GPIOC2,  SPI1_SCK=GPIOB10,
//- If uses SPI3, SPI3_MOSI=GPIOB5,  SPI3_MISO=GPIOB4,  SPI3_SCK=GPIOB3,  -- this default
//            or  SPI3_MOSI=GPIOC12, SPI3_MISO=GPIOC11, SPI3_SCK=GPIOC10
//- If uses SPI4, SPI3_MOSI=GPIOE2,  SPI3_MISO=GPIOE5,  SPI3_SCK=GPIOE6,  -- this default
//            or  SPI3_MOSI=GPIOC12, SPI3_MISO=GPIOC11, SPI3_SCK=GPIOC10

////For LPX17xx
//- If uses SPI0, SPI0_MOSI=P0.18,  SPI0_MISO=P0.17,  SPI0_SCK=P0.15      -- use this currently
//For HT32F16xx
//- If uses SPI0, SPI0_MOSI=PA5,  SPI0_MISO=PA6,  SPI0_SCK=PA4            -- use this currently
//            or  SPI0_MOSI=PD11, SPI0_MISO=PD12, SPI0_SCK=PD10
//- If uses SPI1, SPI1_MOSI=PE10, SPI1_MISO=PE11, SPI1_SCK=PE9
//            or  SPI1_MOSI=PC11, SPI1_MISO=PC12, SPI1_SCK=PC10
#define  M8266WIFI_SPI_INTERFACE_NO							1  // STM32: 1~. for STM32F4xx, try to use SPI1 other than SPI2 and SPI3, since according to datasheet, SPI1 has a better performance than SPI2 and SPI3
																										// NXP:   0~
//- If uses USART1, USART1_TXD=PA9, USART1_RXD=PA10
//- If uses USART2, USART2_TXD=PA2, USART2_RXD=PA3
#define  M8266WIFI_USART_INTERFACE_NO						1 // 1 for USART1, 2 for USART2
#define  M8266WIFI_USART_RX_BUF_SIZE						512

#define  USART_FOR_PRINTF												USART1

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// M8266WIFI Module Interface Board Usages Macros
//     - SPI TX DMA   if DMA used
//     - SPI RX DMA   if DMA used
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
#ifdef M8266WIFI_SPI_ACCESS_USE_DMA
#define M8266WIFI_INTERFACE_SPI_TX_DMA										DMA2
#define M8266WIFI_INTERFACE_SPI_TX_RCC_Periph_DMA 				RCC_AHB1Periph_DMA2
#define M8266WIFI_INTERFACE_SPI_TX_DMA_CHANNEL 						DMA_Channel_3
#define M8266WIFI_INTERFACE_SPI_TX_DMA_STREAM							DMA2_Stream3
#define M8266WIFI_INTERFACE_SPI_RX_DMA										DMA2
#define M8266WIFI_INTERFACE_SPI_RX_RCC_Periph_DMA 				RCC_AHB1Periph_DMA2
#define M8266WIFI_INTERFACE_SPI_RX_DMA_CHANNEL 						DMA_Channel_3
#define M8266WIFI_INTERFACE_SPI_RX_DMA_STREAM							DMA2_Stream2
#endif

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// PLEASE DO NOT MODIFY BELOW UNLESS NECESSITY
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#if defined(MCU_IS_STM32F1XX) || defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F3XX) || defined(MCU_IS_STM32F4XX) || defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX)
#define MCU_IS_STM32
#endif

//////////////////////////////////////////////////////////////////////////////////
#if   defined(MCU_IS_STM32F1XX)  // STM's STM32F1XX,
#define GPIO_InitStructure_AS_GPIO_OUTPUT(gpio,pin)						\
	GPIO_InitStructure.GPIO_Pin 	= pin;												\
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;						\
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						\
	GPIO_Init(gpio, &GPIO_InitStructure)
#elif defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F4XX)  	// STM's STM32F2XX, STM's STM32F4XX
#define GPIO_InitStructure_AS_GPIO_OUTPUT(gpio,pin)						\
	GPIO_InitStructure.GPIO_Pin 	= pin;												\
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;							\
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;							\
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					\
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;								\
	GPIO_Init(gpio, &GPIO_InitStructure)
#elif defined(MCU_IS_STM32F3XX)  // STM's STM32F3X
#define GPIO_InitStructure_AS_GPIO_OUTPUT(gpio,pin)						\
	GPIO_InitStructure.GPIO_Pin 	= pin;												\
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;							\
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;							\
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3 ;				\
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;								\
	GPIO_Init(gpio, &GPIO_InitStructure)
#elif defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX) // STM's STM32F7XX, STM32L4XX, using HAL
#define GPIO_InitStructure_AS_GPIO_OUTPUT(gpio,pin)						\
	GPIO_InitStructure.Pin 				= pin;												\
	GPIO_InitStructure.Mode  			= GPIO_MODE_OUTPUT_PP;				\
	GPIO_InitStructure.Speed 			= GPIO_SPEED_HIGH;						\
	GPIO_InitStructure.Pull  			= GPIO_NOPULL;								\
	HAL_GPIO_Init(gpio,&GPIO_InitStructure)
#elif defined(MCU_IS_NXP_LPC17XX)//NXP's LPC17XX
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

#if   defined(MCU_IS_STM32F1XX)  // STM's STM32F1XX,
#define GPIO_OUTPUT_HIGH(gpio,pin)	(gpio)->BSRR=pin
#define GPIO_OUTPUT_LOW(gpio,pin)		(gpio)->BRR=pin
#elif defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F4XX)  	// STM's STM32F2XX, STM's STM32F4XX
#define GPIO_OUTPUT_HIGH(gpio,pin)	(gpio)->BSRRL=pin
#define GPIO_OUTPUT_LOW(gpio,pin)		(gpio)->BSRRH=pin
#elif   defined(MCU_IS_STM32F3XX) || defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX)  // STM's STM32F3XX, STM32F7XX, STM32L4XX
#define GPIO_OUTPUT_HIGH(gpio,pin)	(gpio)->BSRR=pin  				// HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET)
#define GPIO_OUTPUT_LOW(gpio,pin)		(gpio)->BSRR=(pin<<16)  	// HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET)
#elif defined(MCU_IS_NXP_LPC17XX)//NXP's LPC17XX
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

#if   defined(MCU_IS_STM32F1XX)
#define GPIO_InitStructure_AS_AF(gpio,pins)										\
	GPIO_InitStructure.GPIO_Pin 	= pins;												\
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;						\
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						\
	GPIO_Init(gpio, &GPIO_InitStructure)
#elif defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F3XX) || defined(MCU_IS_STM32F4XX)  	// STM's STM32F2XX, STM32F3XX, STM's STM32F4XX
#define GPIO_InitStructure_AS_AF(gpio,pins)										\
	GPIO_InitStructure.GPIO_Pin 	= pins;												\
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;								\
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;							\
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					\
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;								\
	GPIO_Init(gpio, &GPIO_InitStructure)
#elif defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX)  	// STM's STM32F7XX, STM32L4XX, using HAL
#define GPIO_InitStructure_AS_AF(gpio,pins)										\
	GPIO_InitStructure.Pin       = pins;												\
	GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;							\
	GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;							\
	GPIO_InitStructure.Pull      = GPIO_PULLUP;									\
  GPIO_InitStructure.Alternate = 0x05U;												\
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure)
#elif defined(MCU_IS_NXP_LPC17XX)//NXP's LPC17XX
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

//To define GPIO_PinRemapSPI0
#if   defined(MCU_IS_STM32F1XX)
#elif defined(MCU_IS_STM32F2XX)
#elif defined(MCU_IS_STM32F4XX)
#elif defined(MCU_IS_STM32F3XX)
#elif defined(MCU_IS_STM32F7XX)
#elif defined(MCU_IS_STM32L4XX)
#elif defined(MCU_IS_NXP_LPC17XX)
#define GPIO_PinRemapSPI0																																										\
						LPC_PINCON->PINSEL0 &= ~(0x03UL << 30); LPC_PINCON->PINSEL0 |=  (0x02UL << 30);									\
						LPC_PINCON->PINSEL1 &= ~(0x03UL <<  2); LPC_PINCON->PINSEL1 |=  (0x02UL <<  2);									\
						LPC_PINCON->PINSEL1 &= ~(0x03UL <<  4); LPC_PINCON->PINSEL1 |=  (0x02UL <<  4);									\
						LPC_PINCON->PINSEL1 &= ~(0x03UL << 0);
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

//To define GPIO_PinRemapSPI1
#if   defined(MCU_IS_STM32F1XX)
#define GPIO_PinRemapSPI1																																	\
	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
#elif defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F3XX) || defined(MCU_IS_STM32F4XX) || defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX)
#define GPIO_PinRemapSPI1																																	\
  GPIO_PinAFConfig(M8266WIFI_SPI_GPIO,M8266WIFI_SPI_MOSI_GPIO_PinSource,GPIO_AF_SPI1);		\
	GPIO_PinAFConfig(M8266WIFI_SPI_GPIO,M8266WIFI_SPI_MISO_GPIO_PinSource,GPIO_AF_SPI1); 		\
	GPIO_PinAFConfig(M8266WIFI_SPI_GPIO,M8266WIFI_SPI_CLK_GPIO_PinSource,GPIO_AF_SPI1);
#elif defined(MCU_IS_NXP_LPC17XX)//NXP LPC17XX
#define GPIO_PinRemapSPI1																																		\
						LPC_PINCON->PINSEL0 &= ~(0x03UL << 14); LPC_PINCON->PINSEL0 |=  (0x02UL << 14);	\
						LPC_PINCON->PINSEL0 &= ~(0x03UL << 16); LPC_PINCON->PINSEL0 |=  (0x02UL << 16);	\
						LPC_PINCON->PINSEL0 &= ~(0x03UL << 18); LPC_PINCON->PINSEL0 |=  (0x02UL << 18);	\
						LPC_PINCON->PINSEL0 &= ~(0x03UL << 12);
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

//To define GPIO_PinRemapSPI2
#if   defined(MCU_IS_STM32F1XX)
#elif defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F3XX) || defined(MCU_IS_STM32F4XX) || defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX)
#define GPIO_PinRemapSPI2																																	\
  GPIO_PinAFConfig(M8266WIFI_SPI_GPIO,M8266WIFI_SPI_MOSI_GPIO_PinSource,GPIO_AF_SPI2);		\
	GPIO_PinAFConfig(M8266WIFI_SPI_GPIO,M8266WIFI_SPI_MISO_GPIO_PinSource,GPIO_AF_SPI2); 		\
	GPIO_PinAFConfig(M8266WIFI_SPI_GPIO,M8266WIFI_SPI_CLK_GPIO_PinSource,GPIO_AF_SPI2);
#elif defined(MCU_IS_NXP_LPC17XX)//NXP LPC17XX
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

//To define GPIO_PinRemapSPI3
#if   defined(MCU_IS_STM32F1XX)
#define GPIO_PinRemapSPI3																				\
	GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);
#elif defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F3XX) || defined(MCU_IS_STM32F4XX) || defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX)
#define GPIO_PinRemapSPI3																				\
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI3);					\
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI3); 				\
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI3);
#elif defined(MCU_IS_NXP_LPC17XX)//NXP's LPC17XX
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

#if   defined(MCU_IS_STM32F1XX)
#define GPIO_PinRemapUSART1																				\
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
#elif defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F3XX) || defined(MCU_IS_STM32F4XX) || defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX)
#define GPIO_PinRemapUSART1																				\
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1); 			\
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
#elif defined(MCU_IS_NXP_LPC17XX)//NXP's LPC17XX
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

#if   defined(MCU_IS_STM32F1XX)
#define GPIO_PinRemapUSART2																				\
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#elif defined(MCU_IS_STM32F2XX) || defined(MCU_IS_STM32F3XX) || defined(MCU_IS_STM32F4XX) || defined(MCU_IS_STM32F7XX) || defined(MCU_IS_STM32L4XX)
#define GPIO_PinRemapUSART2																				\
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,  GPIO_AF_USART2); 			\
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,  GPIO_AF_USART2);
#elif defined(MCU_IS_NXP_LPC17XX)//NXP's LPC17XX
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif


#if   defined(MCU_IS_STM32)  // for STM32
#if   (M8266WIFI_SPI_INTERFACE_NO == 1)
  #define  M8266WIFI_INTERFACE_SPI								SPI1
#elif (M8266WIFI_SPI_INTERFACE_NO == 2)
  #define  M8266WIFI_INTERFACE_SPI								SPI2
#elif (M8266WIFI_SPI_INTERFACE_NO == 3)
  #define  M8266WIFI_INTERFACE_SPI								SPI3
#elif (M8266WIFI_SPI_INTERFACE_NO == 4)
  #define  M8266WIFI_INTERFACE_SPI								SPI4
#else
#error Wrong M8266WIFI_SPI_INTERFACE_NO defined near line 137 for STM32 in brd_cfg.h
#endif

#elif defined(MCU_IS_NXP_LPC17XX) // for LPC17XX
#if   (M8266WIFI_SPI_INTERFACE_NO == 0)
  #define  M8266WIFI_INTERFACE_SPI								LPC_SSP0
	#define  LPC_SC_PCON_SSP_BIT										21				// Power Enable bit of SSP0
	#define  LPC_SC_PCLKSELx_FOR_SSP								PCLKSEL1	// PCLK Selection for SSP0
	#define  LPC_SC_PCLKSELx_FOR_SSP_MASK_BIT				10
#elif (M8266WIFI_SPI_INTERFACE_NO == 1)
  #define  M8266WIFI_INTERFACE_SPI								LPC_SSP1
	#define  LPC_SC_PCON_SSP_BIT										10				// Power Enable bit of SSP1
	#define  LPC_SC_PCLKSELx_FOR_SSP								PCLKSEL0	// PCLK Selection for SSP1
	#define  LPC_SC_PCLKSELx_FOR_SSP_MASK_BIT				20
#else
#error Wrong M8266WIFI_SPI_INTERFACE_NO defined near line 137 for NXP_LPC17XX in brd_cfg.h
#endif

#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28
#if   (M8266WIFI_SPI_INTERFACE_NO == 0)
  #define  M8266WIFI_INTERFACE_SPI								SPI0
	#define  M8266WIFI_INTERFACE_SPI_CLOCK					kCLOCK_Spi0
#elif (M8266WIFI_SPI_INTERFACE_NO == 1)
  #define  M8266WIFI_INTERFACE_SPI								SPI1
	#define  M8266WIFI_INTERFACE_SPI_CLOCK					kCLOCK_Spi1
#elif (M8266WIFI_SPI_INTERFACE_NO == 2)
  #define  M8266WIFI_INTERFACE_SPI								SPI2
	#define  M8266WIFI_INTERFACE_SPI_CLOCK					kCLOCK_Spi2
#elif (M8266WIFI_SPI_INTERFACE_NO == 3)
  #define  M8266WIFI_INTERFACE_SPI								SPI3
	#define  M8266WIFI_INTERFACE_SPI_CLOCK					kCLOCK_Spi3
#else
#error Wrong M8266WIFI_SPI_INTERFACE_NO defined near line 137 for NXP_MK27_28 in brd_cfg.h
#endif

#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX
#if   (M8266WIFI_SPI_INTERFACE_NO == 0)
  #define  M8266WIFI_INTERFACE_SPI								HT_SPI0
#elif (M8266WIFI_SPI_INTERFACE_NO == 1)
  #define  M8266WIFI_INTERFACE_SPI								HT_SPI1
#else
#error Wrong M8266WIFI_SPI_INTERFACE_NO defined near line 137 for Holteks HT32F16XX in brd_cfg.h
#endif

#else
#error YOU SHOULD DEFINED MCU_IS_XXX near line 17 in brd_cfg.h
#endif

#if   defined(MCU_IS_STM32)  // for STM32
#if   (M8266WIFI_USART_INTERFACE_NO == 1)
#define  M8266WIFI_INTERFACE_USART							USART1
#define  M8266WIFI_INTERFACE_USART_IRQn					USART1_IRQn
#elif (M8266WIFI_USART_INTERFACE_NO == 2)
#define  M8266WIFI_INTERFACE_USART							USART2
#define  M8266WIFI_INTERFACE_USART_IRQn					USART2_IRQn
#else
#error Wrong M8266WIFI_SPI_INTERFACE_NO defined near line 90 for STM32 in brd_cfg.h
#endif

#elif defined(MCU_IS_NXP_LPC17XX) // for LPC17XX
#elif defined(MCU_IS_NXP_MK27_28)// NXP's MK27/MK28

#elif defined(MCU_IS_HT32F16XX)  //Holtek's HT32F16XX

#else
#error UNSUPPORTED USARTx defined in brd_cfg.h
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//Other board related Macros

#if defined(MCU_IS_STM32F2XX)

#if 1 // for a customer2's Board  ( LED[1-4] - R[49-52] - VCC )  || PF6-LED1-R49-VCC, PB12-LED2-R50-VCC, PE0-LED3-R51-VCC, PB8-LED4-R52-VCC,
#define RCC_AHB1Periph_GPIO_LEDS								RCC_AHB1Periph_GPIOB
#define LED_GPIOS															  GPIOB
#define LED0_GPIO_PIN													  GPIO_Pin_8
#define LED1_GPIO_PIN													  GPIO_Pin_12
//#define LED2_GPIO_PIN													  GPIO_Pin_2
//#define LED3_GPIO_PIN													  GPIO_Pin_3
#endif

#if 1 // for a customer2's Board
#define RCC_AHB1Periph_GPIO_KEYS								RCC_AHB1Periph_GPIOB
#define KEY_GPIOS																GPIOB
#define KEY0_GPIO_PIN														GPIO_Pin_10
#define KEY1_GPIO_PIN														GPIO_Pin_10
#define KEY2_GPIO_PIN														GPIO_Pin_10
#define KEY3_GPIO_PIN														GPIO_Pin_10
#endif

#endif  // end of #if defined(MCU_IS_STM32F2XX)

#if defined(MCU_IS_STM32F3XX)

#if 1 // for STM32F303RE Nucleo Board, LED0/LED2->LD2->PA5
#define RCC_AHBPeriph_GPIO_LEDS									RCC_AHBPeriph_GPIOA
#define LED_GPIOS															  GPIOA
#define LED0_GPIO_PIN													  GPIO_Pin_5
#define LED1_GPIO_PIN													  GPIO_Pin_5

#define RCC_AHBPeriph_GPIO_KEYS									RCC_AHBPeriph_GPIOB
#define KEY_GPIOS																GPIOB
#define KEY0_GPIO_PIN														GPIO_Pin_10
#define KEY1_GPIO_PIN														GPIO_Pin_10
#define KEY2_GPIO_PIN														GPIO_Pin_10
#define KEY3_GPIO_PIN														GPIO_Pin_10

#endif

#endif  // end of #if defined(MCU_IS_STM32F3XX)



#if defined(MCU_IS_STM32F4XX)

#if 0  // for DISCOVERY STM32F4 Board
#define RCC_AHB1Periph_GPIO_LEDS								RCC_AHB1Periph_GPIOF
#define LED_GPIOS															  GPIOF
#define LED0_GPIO_PIN													  GPIO_Pin_9
#define LED1_GPIO_PIN													  GPIO_Pin_10
#elif 0 // for a customer1's Board
#define RCC_AHB1Periph_GPIO_LEDS								RCC_AHB1Periph_GPIOC
#define LED_GPIOS															  GPIOC
#define LED0_GPIO_PIN													  GPIO_Pin_0
#define LED1_GPIO_PIN													  GPIO_Pin_1
//#define LED2_GPIO_PIN													  GPIO_Pin_2
//#define LED3_GPIO_PIN													  GPIO_Pin_3
#elif 0 // for a customer2's Board  ( LED[1-4] - R[49-52] - VCC )  || PF6-LED1-R49-VCC, PB12-LED2-R50-VCC, PE0-LED3-R51-VCC, PB8-LED4-R52-VCC,
#define RCC_AHB1Periph_GPIO_LEDS								RCC_AHB1Periph_GPIOB
#define LED_GPIOS															  GPIOB
#define LED0_GPIO_PIN													  GPIO_Pin_8
#define LED1_GPIO_PIN													  GPIO_Pin_12
//#define LED2_GPIO_PIN													  GPIO_Pin_2
//#define LED3_GPIO_PIN													  GPIO_Pin_3
#elif 1 // for a customer3's Board
#define RCC_AHB1Periph_GPIO_LEDS								RCC_AHB1Periph_GPIOA
#define LED_GPIOS															  GPIOA
#define LED0_GPIO_PIN													  GPIO_Pin_11
#define LED1_GPIO_PIN													  GPIO_Pin_12
#endif


#if 0 // for DISCOVERY STM32F4 Board
#define RCC_AHB1Periph_GPIO_KEYS								RCC_AHB1Periph_GPIOE
#define KEY_GPIOS															  GPIOE
#define KEY0_GPIO_PIN														GPIO_Pin_4
#define KEY1_GPIO_PIN 													GPIO_Pin_3
#define KEY2_GPIO_PIN														GPIO_Pin_2
#elif 0 // for a customer1's Board
#define RCC_AHB1Periph_GPIO_KEYS								RCC_AHB1Periph_GPIOB
#define KEY_GPIOS																GPIOB
#define KEY0_GPIO_PIN														GPIO_Pin_9
#define KEY1_GPIO_PIN														GPIO_Pin_9
#define KEY2_GPIO_PIN														GPIO_Pin_9
#elif 1 // for a customer2's Board
#define RCC_AHB1Periph_GPIO_KEYS								RCC_AHB1Periph_GPIOB
#define KEY_GPIOS																GPIOB
#define KEY0_GPIO_PIN														GPIO_Pin_10
#define KEY1_GPIO_PIN														GPIO_Pin_10
#define KEY2_GPIO_PIN														GPIO_Pin_10
#define KEY3_GPIO_PIN														GPIO_Pin_10
#else
#endif

#endif

#endif
