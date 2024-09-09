/********************************************************************
 * M8266HostIf.c
 * .Description
 *     Source file of M8266WIFI Host Interface
 * .Copyright(c) Anylinkin Technology 2015.5-
 *     IoT@anylinkin.com
 *     http://www.anylinkin.com
 *     http://anylinkin.taobao.com
 *  Author
 *     wzuo
 *  Date
 *  Version
 ********************************************************************/

#include "stdio.h"
#include "string.h"
#include "brd_cfg.h"
#include "M8266WIFIDrv.h"
#include "M8266HostIf.h"

/***********************************************************************************
 * M8266HostIf_GPIO_SPInCS_nRESET_Pin_Init                                         *
 * Description                                                                     *
 *    To initialise the GPIOs for SPI nCS and nRESET output for M8266WIFI module   *
 *    You may update the macros of GPIO PINs usages for nRESET from brd_cfg.h      *
 *    You are not recommended to modify codes below please                         *
 * Parameter(s):                                                                   *
 *    None                                                                         *
 * Return:                                                                         *
 *    None                                                                         *
 ***********************************************************************************/
void M8266HostIf_GPIO_CS_RESET_Init(void)
{
	//Initialise the nRESET pin
	M8266WIFI_nRESET_GPIO->FIODIR |= (1 << M8266WIFI_nRESET_PIN);  				// Set as Output
	//M8266WIFI_nRESET_GPIO->FIOCLR |= (1 << M8266WIFI_nRESET_PIN);
	M8266WIFI_nRESET_GPIO->FIOSET |= (1 << M8266WIFI_nRESET_PIN);

	//Initialise the nCS pin
	M8266WIFI_SPI_nCS_GPIO->FIODIR |= (1 << M8266WIFI_SPI_nCS_PIN);  		// Set as Output
	M8266WIFI_SPI_nCS_GPIO->FIOSET |= (1 << M8266WIFI_SPI_nCS_PIN);  		// Output High initially
}

/***********************************************************************************
 * M8266HostIf_SPI_Init                                                            *
 * Description                                                                     *
 *    To initialise the SPI Interface for M8266WIFI module                         *
 *    You may update the macros of SPI usages for nRESET from brd_cfg.h            *
 *    You are not recommended to modify codes below please                         *
 * Parameter(s):                                                                   *
 *    None                                                                         *
 * Return:                                                                         *
 *    None                                                                         *
 ***********************************************************************************/
void M8266HostIf_SPI_Init(void)
{
	// Initialise the SPI GPIO pins
	GPIO_PinRemapSPI1;


	// To enable the SPI(SSPx) Power and Clock Selection
	LPC_SC->PCON |= (1UL << LPC_SC_PCON_SSP_BIT);  							   // to enable the SSPx
	LPC_SC->LPC_SC_PCLKSELx_FOR_SSP = ((LPC_SC->LPC_SC_PCLKSELx_FOR_SSP)
	                                  & (~(3UL<<LPC_SC_PCLKSELx_FOR_SSP_MASK_BIT)))
	                                  | (1UL<<LPC_SC_PCLKSELx_FOR_SSP_MASK_BIT);   // to Select the PCLK_SSPx = CPU CLK
	// Initialise the SPI Registers
	M8266WIFI_INTERFACE_SPI->CR0 = 0
									|(0x07<<0)				// bits(3~0)=b0111 = 8-bit transfer
									|(0x00<<4)			  // bits(5~4)=b00   = Frame Format-> SPI
									|(0x00<<6)			  // bit 6    =b0    = CPOL->SSP controller maintains the bus clock low between frames/idle
									|(0x00<<7)			  // bit 7    =b0    = CPHA->SSP controller captures serial data on the first clock transition
									|(0x00<<8)				// bits(15-8)=0x00 = SCR(Serial Clock Rate)=0. SSP CLK = PCLK_SSPx / (CPSDVSR x [SCR+1]).
									;										// NXP SSPx always MSB first in a byte
	M8266WIFI_INTERFACE_SPI->CR1 = 0
									|(0x00<<0)				// bit 0 = b0 = LBM-> Normal operation other than Loopback mode
									|(0x01<<1)				// bit 1 = b1 = SSE-> The SSP controller enabled
									|(0x00<<2)				// bit 2 = b0 = MS -> SPI Master
									;
	// Initially SPI_CLK = (PCLK_SSPx=CPU_CLK=96MHz)/(8x[0+1])=12MHz
	M8266WIFI_INTERFACE_SPI->CPSR = 8; // bits 7-0 = CPSDVSR(should be even) = 8.

	M8266WIFI_INTERFACE_SPI->ICR  = 3;  // SSPn Interrupt Clear Register

#ifdef M8266WIFI_SPI_ACCESS_USE_DMA
     M8266WIFI_INTERFACE_SPI->DMACR = 0
									|(1<<0) 					// Receive DMA Enable
									|(1<<1)           // Transmit DMA Enable
									;
#endif

}

/***********************************************************************************
 * M8266HostIf_SPI_SetSpeed                                                        *
 * Description                                                                     *
 *    To setup the SPI Clock Speed for M8266WIFI module                            *
 * Parameter(s):                                                                   *
 *    SPI_BaudRatePrescaler: SPI BaudRate Prescaler                                *
 * Return:                                                                         *
 *    None                                                                         *
 ***********************************************************************************/
void M8266HostIf_SPI_SetSpeed(u32 SPI_BaudRatePrescaler) // SPI_BaudRatePrescaler should be an even number no less than 2.
{
	M8266WIFI_INTERFACE_SPI->CPSR = SPI_BaudRatePrescaler;
}

#ifdef M8266WIFI_SPI_ACCESS_USE_DMA
void M8266HostIf_SPI_DMA_Init(void)
{

	LPC_SC->PCONP  |= ((uint32_t)1 << 29);    				 // enable the GPDMA function power/clock
	LPC_GPDMA->DMACIntTCClear   |=                     // DMA Interrupt Terminal Count Request Clear register
									    (1<<((M8266WIFI_INTERFACE_SPI_RX_DMA_STREAM-LPC_GPDMACH0)>>5))
										 |(1<<((M8266WIFI_INTERFACE_SPI_TX_DMA_STREAM-LPC_GPDMACH0)>>5));
	LPC_GPDMA->DMACIntErrClr   |=                       // DMA Interrupt Error Clear register
									    (1<<((M8266WIFI_INTERFACE_SPI_RX_DMA_STREAM-LPC_GPDMACH0)>>5))
										 |(1<<((M8266WIFI_INTERFACE_SPI_TX_DMA_STREAM-LPC_GPDMACH0)>>5));

	LPC_GPDMA->DMACConfig    =
											(1<<0) 		// enable DMA
										 |(0<<1)		// using little-endian mode
										 ;

	// Set DMA Channel for SPI RX
	M8266WIFI_INTERFACE_SPI_RX_DMA_STREAM->DMACCLLI   = 0;
	M8266WIFI_INTERFACE_SPI_RX_DMA_STREAM->DMACCConfig= 0
										 |(0<<0)              // bit0 = Enable = 0 -> Channel not enable initially
										 |(1<<1)							// bits[5:1]  = SrcPeripheral -> DMA request Input = 1 -> SSP0 RX
	                   |((M8266WIFI_SPI_INTERFACE_NO&1)<<2)             //-> DMA request Input = 3 -> SSP1 RX
										 |(0<<6)							// bits[10:6] = DestPeripheral-> since it is memory, ignored
										 |(2<<11)							// bits[13:11]= TransferType = 2 -> Peripheral to Memory
										 |(0<<14)							// bit14  = Interrupt error mask = 0 -> mask out the error interrupt
										 |(0<<15)							// bit15  = terminal count interrupt mask = 0 -> mask out
										 |(0<<16)							// bit16  = lock = 0 -> not enable the locked transfer
										 |(0<<17)							// bit17  = active, the channel FIFO has data -> read only
										 |(0<<18)							// bit18  = Halt, Halt DMA request = 0 -> enable DMA request
										 |(0<<19)             // bits[31:19] = reserved 0
										 ;
	M8266WIFI_INTERFACE_SPI_RX_DMA_STREAM->DMACCControl    = 0
		                 |(0<<0)					    // bits[11:0] = transfer size, use 0 for the moment during initialization
										 |(2<<12)             // bits[14:12]= Source burst size = 2 -> 8 bits per burst
										 |(2<<15)             // bits[17:15]= Destin burst size = 2 -> 8 bits per burst
                     |(0<<18)             // bits[20:18]= Source transfer width = 0 -> Bytes
                     |(0<<21)             // bits[23:21]= Destin transfer width = 0 -> Bytes
                     |(0<<24)             // bits[25:24]= Reserved 0
                     |(0<<26)             // bit26 = source increament = 0 -> not increase
                     |(1<<27)             // bit27 = destin increament = 1 -> increase
										 |((uint32_t)1<<31)   // bit31 = Terminal count interrupt enable bit = 1 -> enabled
										 ;
	M8266WIFI_INTERFACE_SPI_RX_DMA_STREAM->DMACCSrcAddr    = (uint32_t)(&(M8266WIFI_INTERFACE_SPI->DR));  	// DMA source address,
	M8266WIFI_INTERFACE_SPI_RX_DMA_STREAM->DMACCDestAddr   = (uint32_t)0;  															// DMA destin address, use 0 during initialization for the moment

	// Set DMA Channel for SPI TX
	M8266WIFI_INTERFACE_SPI_TX_DMA_STREAM->DMACCLLI   = 0;

	M8266WIFI_INTERFACE_SPI_TX_DMA_STREAM->DMACCConfig= 0
										 |(0<<0)              // bit0 = Enable = 0 -> Channel not enable initially
										 |(0<<1)              // bits[5:1]  = SrcPeripheral  -> since it is memory, ignored
										 |(0<<6)							// bits[10:6] = DestPeripheral -> DMA request output = 0 -> SSP0 TX
	                   |((M8266WIFI_SPI_INTERFACE_NO&1)<<7)             // -> DMA request output = 2 -> SSP1 TX
										 |(1<<11)							// bits[13:11]= TransferType = 1 -> Memory to Peripheral
										 |(0<<14)							// bit14  = Interrupt error mask = 0 -> mask out the error interrupt
										 |(0<<15)							// bit15  = terminal count interrupt mask = 0 -> mask out
										 |(0<<16)							// bit16  = lock = 0 -> not enable the locked transfer
										 |(0<<17)							// bit17  = active, the channel FIFO has data -> read only
										 |(0<<18)							// bit18  = Halt, Halt DMA request = 0 -> enable DMA request
										 |(0<<19)             // bits[31:19] = reserved 0

										 ;

	M8266WIFI_INTERFACE_SPI_TX_DMA_STREAM->DMACCControl    = 0
		                 |(0<<0)					    // bits[11:0] = transfer size, use 0 for the moment during initialization
										 |(2<<12)             // bits[14:12]= Source burst size = 0 -> 1 byte per burst
										 |(2<<15)             // bits[17:15]= Destin burst size = 0 -> 1 byte per burst
                     |(0<<18)             // bits[20:18]= Source transfer width = 0 -> Bytes
                     |(0<<21)             // bits[23:21]= Destin transfer width = 0 -> Bytes
                     |(0<<24)             // bits[25:24]= Reserved 0
                     |(1<<26)             // bit26 = source increament = 1 -> increase
                     |(0<<27)    					// bit27 = destin increament = 0 -> not increase
										 |((uint32_t)1<<31)   // bit31 = Terminal count interrupt enable bit = 1 -> enabled1<<31)             // bit31 = Terminal count interrupt enable bit = 0 -> interrupt disabled
										 ;
	M8266WIFI_INTERFACE_SPI_TX_DMA_STREAM->DMACCSrcAddr    = (uint32_t)0;  															// DMA source address, use 0 during initialization for the moment
	M8266WIFI_INTERFACE_SPI_TX_DMA_STREAM->DMACCDestAddr   = (uint32_t)(&(M8266WIFI_INTERFACE_SPI->DR));// DMA destin address, use 0 during initialization for the moment

}

#endif

/***********************************************************************************
 * M8266HostIf_Init                                                                *
 * Description                                                                     *
 *    To initialise the Host interface for M8266WIFI module                        *
 * Parameter(s):                                                                   *
 *    baud: baud rate to set                                                       *
 * Return:                                                                         *
 *    None                                                                         *
 ***********************************************************************************/
void M8266HostIf_Init(void)
{
	 M8266HostIf_GPIO_CS_RESET_Init();
	 M8266HostIf_SPI_Init();

#ifdef M8266WIFI_SPI_ACCESS_USE_DMA
	 M8266HostIf_SPI_DMA_Init();
#endif
}

//////////////////////////////////////////////////////////////////////////////////////
// BELOW FUNCTIONS ARE REQUIRED BY M8266WIFIDRV.LIB.
// PLEASE IMPLEMENTE THEM ACCORDING TO YOUR HARDWARE
//////////////////////////////////////////////////////////////////////////////////////
/***********************************************************************************
 * M8266HostIf_Set_nRESET_Pin                                                      *
 * Description                                                                     *
 *    To Outpout HIGH or LOW onto the GPIO pin for M8266WIFI nRESET                *
 *    You may update the macros of GPIO PIN usages for nRESET from brd_cfg.h       *
 *    You are not recommended to modify codes below please                         *
 * Parameter(s):                                                                   *
 *    1. level: LEVEL output to nRESET pin                                         *
 *              0 = output LOW  onto nRESET                                        *
 *              1 = output HIGH onto nRESET                                        *
 * Return:                                                                         *
 *    None                                                                         *
 ***********************************************************************************/
void M8266HostIf_Set_nRESET_Pin(u8 level)
{
	// if (level == 0)
	if (level == 1)
		M8266WIFI_nRESET_GPIO->FIOSET |= (1 << M8266WIFI_nRESET_PIN);
	else
		M8266WIFI_nRESET_GPIO->FIOCLR |= (1 << M8266WIFI_nRESET_PIN);
}

/***********************************************************************************
 * M8266HostIf_Set_SPI_nCS_PIN                                                     *
 * Description                                                                     *
 *    To Outpout HIGH or LOW onto the GPIO pin for M8266WIFI SPI nCS               *
 *    You may update the macros of GPIO PIN usages for SPI nCS from brd_cfg.h      *
 *    You are not recommended to modify codes below please                         *
 * Parameter(s):                                                                   *
 *    1. level: LEVEL output to SPI nCS pin                                        *
 *              0 = output LOW  onto SPI nCS                                       *
 *              1 = output HIGH onto SPI nCS                                       *
 * Return:                                                                         *
 *    None                                                                         *
 ***********************************************************************************/
void M8266HostIf_Set_SPI_nCS_Pin(u8 level)
{
	if (level != 0)
		M8266WIFI_SPI_nCS_GPIO->FIOSET |= (1<<M8266WIFI_SPI_nCS_PIN);
	else
		M8266WIFI_SPI_nCS_GPIO->FIOCLR |= (1<<M8266WIFI_SPI_nCS_PIN);
}

/***********************************************************************************
 * M8266WIFIHostIf_delay_us                                                        *
 * Description                                                                     *
 *    To loop delay some micro seconds.                                            *
 * Parameter(s):                                                                   *
 *    1. nus: the micro seconds to delay                                           *
 * Return:                                                                         *
 *    none                                                                         *
 ***********************************************************************************/
extern uint32_t SystemCoreClock;
void M8266HostIf_delay_us(u8 nus)
{
	volatile u32 temp;
	SysTick->LOAD = SystemCoreClock / 1000000 * nus; 								//  The System Tick will use SystemFrequency. see below SysTick_CTRL_CLKSOURCE_Msk
	SysTick->VAL  = 0x00;        								 								//	Clear the counter value
	SysTick->CTRL = (SysTick->CTRL & (~SysTick_CTRL_TICKINT_Msk))	//  Disable the System Tick interrupt
									| SysTick_CTRL_CLKSOURCE_Msk	//  System Tick Select CPU Clock
									| SysTick_CTRL_ENABLE_Msk ; 	//	Start the down counter
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&SysTick_CTRL_COUNTFLAG_Msk));		//	Wait the time reached

	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; 	//	Stop the counter
	SysTick->VAL =0x00;       								//	Clear the counter value
}

/***********************************************************************************
 * M8266HostIf_SPI_ReadWriteByte                                                   *
 * Description                                                                     *
 *    To write a byte onto the SPI bus from MCU MOSI to the M8266WIFI module       *
 *    and read back a byte from the SPI bus MISO meanwhile                         *
 *    You may update the macros of SPI usage from brd_cfg.h                        *
 * Parameter(s):                                                                   *
 *    1. TxdByte: the byte to be sent over MOSI                                    *
 * Return:                                                                         *
 *    1. The byte read back from MOSI meanwhile                                    *                                                                         *
 ***********************************************************************************/
u8 M8266HostIf_SPI_ReadWriteByte(u8 TxdByte)
{

	// wait the Transmit FIFO empty
	while((M8266WIFI_INTERFACE_SPI->SR &(1<<0))==0) {}  // bit0=TFE(Transmit FIFO empty)

  	// to Txd the data onto MOSI
    M8266WIFI_INTERFACE_SPI->DR = TxdByte;

	// wait the data received from MISO
    while((M8266WIFI_INTERFACE_SPI->SR &(1<<2))==0) {} 	// bit2=RNE(Receive FIFO not empty)

    // return the received data from MISO
  	return (uint8_t) M8266WIFI_INTERFACE_SPI->DR;
}
