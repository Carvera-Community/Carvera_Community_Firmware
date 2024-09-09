/*
**************************************************************************************************************
*                                                 NXP USB Host Stack
*
*                                     (c) Copyright 2008, NXP SemiConductors
*                                     (c) Copyright 2008, OnChip  Technologies LLC
*                                                 All Rights Reserved
*
*                                                  www.nxp.com
*                                               www.onchiptech.com
*
* File           : usbhost_lpc17xx.c
* Programmer(s)  : Ravikanth.P
* Version        :
*
**************************************************************************************************************
*/
 
/*
**************************************************************************************************************
*                                            INCLUDE HEADER FILES
**************************************************************************************************************
*/

#include  "usbhost_lpc17xx.h"

/*
**************************************************************************************************************
*                                              GLOBAL VARIABLES
**************************************************************************************************************
*/
int gUSBConnected;

volatile  USB_INT32U   HOST_RhscIntr = 0;         /* Root Hub Status Change interrupt                       */
volatile  USB_INT32U   HOST_WdhIntr  = 0;         /* Semaphore to wait until the TD is submitted            */
volatile  USB_INT08U   HOST_TDControlStatus = 0;
volatile  HCED        *EDCtrl;                    /* Control endpoint descriptor structure                  */
volatile  HCED        *EDBulkIn;                  /* BulkIn endpoint descriptor  structure                  */
volatile  HCED        *EDBulkOut;                 /* BulkOut endpoint descriptor structure                  */
volatile  HCTD        *TDHead;                    /* Head transfer descriptor structure                     */
volatile  HCTD        *TDTail;                    /* Tail transfer descriptor structure                     */
volatile  HCCA        *Hcca;                      /* Host Controller Communications Area structure          */ 
          USB_INT16U  *TDBufNonVol;               /* Identical to TDBuffer just to reduce compiler warnings */
volatile  USB_INT08U  *TDBuffer;                  /* Current Buffer Pointer of transfer descriptor          */

// USB host structures
// AHB SRAM block 1
#define HOSTBASEADDR 0x2007C000
// reserve memory for the linker
static USB_INT08U HostBuf[0x200] __attribute__((at(HOSTBASEADDR)));
/*
**************************************************************************************************************
*                                         DELAY IN MILLI SECONDS
*
* Description: This function provides a delay in milli seconds
*
* Arguments  : delay    The delay required
*
* Returns    : None
*
**************************************************************************************************************
*/

void  Host_DelayMS (USB_INT32U  delay)
{
    volatile  USB_INT32U  i;


    for (i = 0; i < delay; i++) {
        Host_DelayUS(1000);
    }
}

/*
**************************************************************************************************************
*                                         DELAY IN MICRO SECONDS
*
* Description: This function provides a delay in micro seconds
*
* Arguments  : delay    The delay required
*
* Returns    : None
*
**************************************************************************************************************
*/

void  Host_DelayUS (USB_INT32U  delay)
{
    volatile  USB_INT32U  i;


    for (i = 0; i < (4 * delay); i++) {    /* This logic was tested. It gives app. 1 micro sec delay        */
        ;
    }
}

// bits of the USB/OTG clock control register
#define HOST_CLK_EN     (1<<0)
#define DEV_CLK_EN      (1<<1)
#define PORTSEL_CLK_EN  (1<<3)
#define AHB_CLK_EN      (1<<4)

// bits of the USB/OTG clock status register
#define HOST_CLK_ON     (1<<0)
#define DEV_CLK_ON      (1<<1)
#define PORTSEL_CLK_ON  (1<<3)
#define AHB_CLK_ON      (1<<4)

// we need host clock, OTG/portsel clock and AHB clock
#define CLOCK_MASK (HOST_CLK_EN | PORTSEL_CLK_EN | AHB_CLK_EN)

/*
**************************************************************************************************************
*                                         INITIALIZE THE HOST CONTROLLER
*
* Description: This function initializes lpc17xx host controller
*
* Arguments  : None
*
* Returns    : 
*
**************************************************************************************************************
*/
void  Host_Init (void)
{
    PRINT_Log("In Host_Init\n");
    NVIC_DisableIRQ(USB_IRQn);                           /* Disable the USB interrupt source           */
    
    // turn on power for USB
    LPC_SC->PCONP       |= (1UL<<31);
    // Enable USB host clock, port selection and AHB clock
    LPC_USB->USBClkCtrl |= CLOCK_MASK;
    // Wait for clocks to become available
    while ((LPC_USB->USBClkSt & CLOCK_MASK) != CLOCK_MASK)
        ;
    
    // it seems the bits[0:1] mean the following
    // 0: U1=device, U2=host
    // 1: U1=host, U2=host
    // 2: reserved
    // 3: U1=host, U2=device
    // NB: this register is only available if OTG clock (aka "port select") is enabled!!
    // since we don't care about port 2, set just bit 0 to 1 (U1=host)
    LPC_USB->OTGStCtrl |= 1;
    
    // now that we've configured the ports, we can turn off the portsel clock
    LPC_USB->USBClkCtrl &= ~PORTSEL_CLK_EN;
    
    // power pins are not connected on mbed, so we can skip them
    /* P1[18] = USB_UP_LED, 01 */
    /* P1[19] = /USB_PPWR,     10 */
    /* P1[22] = USB_PWRD, 10 */
    /* P1[27] = /USB_OVRCR, 10 */
    /*LPC_PINCON->PINSEL3 &= ~((3<<4) | (3<<6) | (3<<12) | (3<<22));  
    LPC_PINCON->PINSEL3 |=  ((1<<4)|(2<<6) | (2<<12) | (2<<22));   // 0x00802080
    */

    // configure USB D+/D- pins
    /* P0[29] = USB_D+, 01 */
    /* P0[30] = USB_D-, 01 */
    LPC_PINCON->PINSEL1 &= ~((3<<26) | (3<<28));  
    LPC_PINCON->PINSEL1 |=  ((1<<26)|(1<<28));     // 0x14000000
        
    PRINT_Log("Initializing Host Stack\n");

    Hcca       = (volatile  HCCA       *)(HostBuf+0x000);
    TDHead     = (volatile  HCTD       *)(HostBuf+0x100);
    TDTail     = (volatile  HCTD       *)(HostBuf+0x110);
    EDCtrl     = (volatile  HCED       *)(HostBuf+0x120); 
    EDBulkIn   = (volatile  HCED       *)(HostBuf+0x130);
    EDBulkOut  = (volatile  HCED       *)(HostBuf+0x140);
    TDBuffer   = (volatile  USB_INT08U *)(HostBuf+0x150);
    
    /* Initialize all the TDs, EDs and HCCA to 0  */
    Host_EDInit(EDCtrl);
    Host_EDInit(EDBulkIn);
    Host_EDInit(EDBulkOut);
    Host_TDInit(TDHead);
    Host_TDInit(TDTail);
    Host_HCCAInit(Hcca);
    
    Host_DelayMS(50);                                   /* Wait 50 ms before apply reset              */
    LPC_USB->HcControl       = 0;                       /* HARDWARE RESET                             */
    LPC_USB->HcControlHeadED = 0;                       /* Initialize Control list head to Zero       */
    LPC_USB->HcBulkHeadED    = 0;                       /* Initialize Bulk list head to Zero          */
    
                                                        /* SOFTWARE RESET                             */
    LPC_USB->HcCommandStatus = OR_CMD_STATUS_HCR;
    LPC_USB->HcFmInterval    = DEFAULT_FMINTERVAL;      /* Write Fm Interval and Largest Data Packet Counter */

                                                        /* Put HC in operational state                */
    LPC_USB->HcControl  = (LPC_USB->HcControl & (~OR_CONTROL_HCFS)) | OR_CONTROL_HC_OPER;
    LPC_USB->HcRhStatus = OR_RH_STATUS_LPSC;            /* Set Global Power                           */
    
    LPC_USB->HcHCCA = (USB_INT32U)Hcca;
    LPC_USB->HcInterruptStatus |= LPC_USB->HcInterruptStatus;                   /* Clear Interrrupt Status                    */


    LPC_USB->HcInterruptEnable  = OR_INTR_ENABLE_MIE |
                         OR_INTR_ENABLE_WDH |
                         OR_INTR_ENABLE_RHSC;

    NVIC_SetPriority(USB_IRQn, 0);       /* highest priority */
    /* Enable the USB Interrupt */
    NVIC_EnableIRQ(USB_IRQn);
    PRINT_Log("Host Initialized\n");
}

/*
**************************************************************************************************************
*                                         INTERRUPT SERVICE ROUTINE
*
* Description: This function services the interrupt caused by host controller
*
* Arguments  : None
*
* Returns    : None
*
**************************************************************************************************************
*/

extern "C" void USB_IRQHandler (void)
{
    USB_INT32U   int_status;
    USB_INT32U   ie_status;

    int_status    = LPC_USB->HcInterruptStatus;                          /* Read Interrupt Status                */
    ie_status     = LPC_USB->HcInterruptEnable;                          /* Read Interrupt enable status         */
 
    if (!(int_status & ie_status)) {
        return;
    } else {

        int_status = int_status & ie_status;
        if (int_status & OR_INTR_STATUS_RHSC) {                 /* Root hub status change interrupt     */
            if (LPC_USB->HcRhPortStatus1 & OR_RH_PORT_CSC) {
                if (LPC_USB->HcRhStatus & OR_RH_STATUS_DRWE) {
                    /*
                     * When DRWE is on, Connect Status Change
                     * means a remote wakeup event.
                    */
                    HOST_RhscIntr = 1;// JUST SOMETHING FOR A BREAKPOINT
                }
                else {
                    /*
                     * When DRWE is off, Connect Status Change
                     * is NOT a remote wakeup event
                    */
                    if (LPC_USB->HcRhPortStatus1 & OR_RH_PORT_CCS) {
                        if (!gUSBConnected) {
                            HOST_TDControlStatus = 0;
                            HOST_WdhIntr = 0;
                            HOST_RhscIntr = 1;
                            gUSBConnected = 1;
                        }
                        else
                            PRINT_Log("Spurious status change (connected)?\n");
                    } else {
                        if (gUSBConnected) {
                            LPC_USB->HcInterruptEnable = 0; // why do we get multiple disc. rupts???
                            HOST_RhscIntr = 0;
                            gUSBConnected = 0;
                        }
                        else
                            PRINT_Log("Spurious status change (disconnected)?\n");
                    }
                }
                LPC_USB->HcRhPortStatus1 = OR_RH_PORT_CSC;
            }
            if (LPC_USB->HcRhPortStatus1 & OR_RH_PORT_PRSC) {
                LPC_USB->HcRhPortStatus1 = OR_RH_PORT_PRSC;
            }
        }
        if (int_status & OR_INTR_STATUS_WDH) {                  /* Writeback Done Head interrupt        */
            HOST_WdhIntr = 1;
            HOST_TDControlStatus = (TDHead->Control >> 28) & 0xf;
        }            
        LPC_USB->HcInterruptStatus = int_status;                         /* Clear interrupt status register      */
    }
    return;
}

/*
**************************************************************************************************************
*                                     PROCESS TRANSFER DESCRIPTOR
*
* Description: This function processes the transfer descriptor
*
* Arguments  : ed            Endpoint descriptor that contains this transfer descriptor
*              token         SETUP, IN, OUT
*              buffer        Current Buffer Pointer of the transfer descriptor
*              buffer_len    Length of the buffer
*
* Returns    : OK       if TD submission is successful
*              ERROR    if TD submission fails
*
**************************************************************************************************************
*/

USB_INT32S  Host_ProcessTD (volatile  HCED       *ed,
                            volatile  USB_INT32U  token,
                            volatile  USB_INT08U *buffer,
                                      USB_INT32U  buffer_len)
{
    volatile  USB_INT32U   td_toggle;


    if (ed == EDCtrl) {
        if (token == TD_SETUP) {
            td_toggle = TD_TOGGLE_0;
        } else {
            td_toggle = TD_TOGGLE_1;
        }
    } else {
        td_toggle = 0;
    }
    TDHead->Control = (TD_ROUNDING    |
                      token           |
                      TD_DELAY_INT(0) |                           
                      td_toggle       |
                      TD_CC);
    TDTail->Control = 0;
    TDHead->CurrBufPtr   = (USB_INT32U) buffer;
    TDTail->CurrBufPtr   = 0;
    TDHead->Next         = (USB_INT32U) TDTail;
    TDTail->Next         = 0;
    TDHead->BufEnd       = (USB_INT32U)(buffer + (buffer_len - 1));
    TDTail->BufEnd       = 0;

    ed->HeadTd  = (USB_INT32U)TDHead | ((ed->HeadTd) & 0x00000002);
    ed->TailTd  = (USB_INT32U)TDTail;
    ed->Next    = 0;

    if (ed == EDCtrl) {
        LPC_USB->HcControlHeadED = (USB_INT32U)ed;
        LPC_USB->HcCommandStatus = LPC_USB->HcCommandStatus | OR_CMD_STATUS_CLF;
        LPC_USB->HcControl       = LPC_USB->HcControl       | OR_CONTROL_CLE;
    } else {
        LPC_USB->HcBulkHeadED    = (USB_INT32U)ed;
        LPC_USB->HcCommandStatus = LPC_USB->HcCommandStatus | OR_CMD_STATUS_BLF;
        LPC_USB->HcControl       = LPC_USB->HcControl       | OR_CONTROL_BLE;
    }    

    // Host_WDHWait();
    Host_DelayMS(100);

//    if (!(TDHead->Control & 0xF0000000)) {
    if (!HOST_TDControlStatus) {
        return (OK);
    } else {      
        return (ERR_TD_FAIL);
    }
}

/*
**************************************************************************************************************
*                                       ENUMERATE THE DEVICE
*
* Description: This function is used to enumerate the device connected
*
* Arguments  : None
*
* Returns    : None
*
**************************************************************************************************************
*/

USB_INT32S  Host_EnumDev (void)
{
    USB_INT32S  rc;

    PRINT_Log("Connect a Mass Storage device\n");
    while (!HOST_RhscIntr)
        __WFI();

    Host_DelayMS(100);                             /* USB 2.0 spec says atleast 50ms delay beore port reset */
    LPC_USB->HcRhPortStatus1 = OR_RH_PORT_PRS; // Initiate port reset
    while (LPC_USB->HcRhPortStatus1 & OR_RH_PORT_PRS)
        __WFI(); // Wait for port reset to complete...
    LPC_USB->HcRhPortStatus1 = OR_RH_PORT_PRSC; // ...and clear port reset signal
    Host_DelayMS(200);                                                 /* Wait for 100 MS after port reset  */

    EDCtrl->Control = 8 << 16;                                         /* Put max pkt size = 8              */
                                                                       /* Read first 8 bytes of device desc */
    rc = HOST_GET_DESCRIPTOR(USB_DESCRIPTOR_TYPE_DEVICE, 0, TDBuffer, 8);
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    EDCtrl->Control = TDBuffer[7] << 16;                               /* Get max pkt size of endpoint 0    */
    rc = HOST_SET_ADDRESS(1);                                          /* Set the device address to 1       */
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    Host_DelayMS(2);
    EDCtrl->Control = (EDCtrl->Control) | 1;                          /* Modify control pipe with address 1 */
                                                                      /* Get the configuration descriptor   */
    rc = HOST_GET_DESCRIPTOR(USB_DESCRIPTOR_TYPE_CONFIGURATION, 0, TDBuffer, 9);
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }
                                                                       /* Get the first configuration data  */
    rc = HOST_GET_DESCRIPTOR(USB_DESCRIPTOR_TYPE_CONFIGURATION, 0, TDBuffer, ReadLE16U(&TDBuffer[2]));
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    rc = MS_ParseConfiguration();                                      /* Parse the configuration           */
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    rc = USBH_SET_CONFIGURATION(1);                                    /* Select device configuration 1     */
    if (rc != OK) {
        PRINT_Err(rc);
    }
    Host_DelayMS(100);                                               /* Some devices may require this delay */
    return (rc);
}

/*
**************************************************************************************************************
*                                        RECEIVE THE CONTROL INFORMATION
*
* Description: This function is used to receive the control information
*
* Arguments  : bm_request_type
*              b_request
*              w_value
*              w_index
*              w_length
*              buffer
*
* Returns    : OK       if Success
*              ERROR    if Failed
*
**************************************************************************************************************
*/
   
USB_INT32S  Host_CtrlRecv (         USB_INT08U   bm_request_type,
                                    USB_INT08U   b_request,
                                    USB_INT16U   w_value,
                                    USB_INT16U   w_index,
                                    USB_INT16U   w_length,
                          volatile  USB_INT08U  *buffer)
{
    USB_INT32S  rc;


    Host_FillSetup(bm_request_type, b_request, w_value, w_index, w_length);
    rc = Host_ProcessTD(EDCtrl, TD_SETUP, TDBuffer, 8);
    if (rc == OK) {
        if (w_length) {
            rc = Host_ProcessTD(EDCtrl, TD_IN, TDBuffer, w_length);
        }
        if (rc == OK) {
            rc = Host_ProcessTD(EDCtrl, TD_OUT, NULL, 0);
        }
    }
    return (rc);
}

/*
**************************************************************************************************************
*                                         SEND THE CONTROL INFORMATION
*
* Description: This function is used to send the control information
*
* Arguments  : None
*
* Returns    : OK                      if Success
*              ERR_INVALID_BOOTSIG    if Failed
*
**************************************************************************************************************
*/

USB_INT32S  Host_CtrlSend (          USB_INT08U   bm_request_type,
                                     USB_INT08U   b_request,
                                     USB_INT16U   w_value,
                                     USB_INT16U   w_index,
                                     USB_INT16U   w_length,
                           volatile  USB_INT08U  *buffer)
{
    USB_INT32S  rc;


    Host_FillSetup(bm_request_type, b_request, w_value, w_index, w_length);

    rc = Host_ProcessTD(EDCtrl, TD_SETUP, TDBuffer, 8);
    if (rc == OK) {
        if (w_length) {
            rc = Host_ProcessTD(EDCtrl, TD_OUT, TDBuffer, w_length);
        }
        if (rc == OK) {
            rc = Host_ProcessTD(EDCtrl, TD_IN, NULL, 0);
        }
    }
    return (rc);
}

/*
**************************************************************************************************************
*                                          FILL SETUP PACKET
*
* Description: This function is used to fill the setup packet
*
* Arguments  : None
*
* Returns    : OK                      if Success
*              ERR_INVALID_BOOTSIG    if Failed
*
**************************************************************************************************************
*/

void  Host_FillSetup (USB_INT08U   bm_request_type,
                      USB_INT08U   b_request,
                      USB_INT16U   w_value,
                      USB_INT16U   w_index,
                      USB_INT16U   w_length)
{
    int i;
    for (i=0;i<w_length;i++)
        TDBuffer[i] = 0;
    
    TDBuffer[0] = bm_request_type;
    TDBuffer[1] = b_request;
    WriteLE16U(&TDBuffer[2], w_value);
    WriteLE16U(&TDBuffer[4], w_index);
    WriteLE16U(&TDBuffer[6], w_length);
}



/*
**************************************************************************************************************
*                                         INITIALIZE THE TRANSFER DESCRIPTOR
*
* Description: This function initializes transfer descriptor
*
* Arguments  : Pointer to TD structure
*
* Returns    : None
*
**************************************************************************************************************
*/

void  Host_TDInit (volatile  HCTD *td)
{

    td->Control    = 0;
    td->CurrBufPtr = 0;
    td->Next       = 0;
    td->BufEnd     = 0;
}

/*
**************************************************************************************************************
*                                         INITIALIZE THE ENDPOINT DESCRIPTOR
*
* Description: This function initializes endpoint descriptor
*
* Arguments  : Pointer to ED strcuture
*
* Returns    : None
*
**************************************************************************************************************
*/

void  Host_EDInit (volatile  HCED *ed)
{

    ed->Control = 0;
    ed->TailTd  = 0;
    ed->HeadTd  = 0;
    ed->Next    = 0;
}

/*
**************************************************************************************************************
*                                 INITIALIZE HOST CONTROLLER COMMUNICATIONS AREA
*
* Description: This function initializes host controller communications area
*
* Arguments  : Pointer to HCCA
*
* Returns    : 
*
**************************************************************************************************************
*/

void  Host_HCCAInit (volatile  HCCA  *hcca)
{
    USB_INT32U  i;


    for (i = 0; i < 32; i++) {

        hcca->IntTable[i] = 0;
        hcca->FrameNumber = 0;
        hcca->DoneHead    = 0;
    }

}

/*
**************************************************************************************************************
*                                         WAIT FOR WDH INTERRUPT
*
* Description: This function is infinite loop which breaks when ever a WDH interrupt rises
*
* Arguments  : None
*
* Returns    : None
*
**************************************************************************************************************
*/

void  Host_WDHWait (void)
{
  while (!HOST_WdhIntr)
      __WFI();

  HOST_WdhIntr = 0;
}

/*
**************************************************************************************************************
*                                         READ LE 32U
*
* Description: This function is used to read an unsigned integer from a character buffer in the platform
*              containing little endian processor
*
* Arguments  : pmem    Pointer to the character buffer
*
* Returns    : val     Unsigned integer
*
**************************************************************************************************************
*/

USB_INT32U  ReadLE32U (volatile  USB_INT08U  *pmem)
{
    USB_INT32U val = *(USB_INT32U*)pmem;
#ifdef __BIG_ENDIAN
    return __REV(val);
#else
    return val;
#endif    
}

/*
**************************************************************************************************************
*                                        WRITE LE 32U
*
* Description: This function is used to write an unsigned integer into a charecter buffer in the platform 
*              containing little endian processor.
*
* Arguments  : pmem    Pointer to the charecter buffer
*              val     Integer value to be placed in the charecter buffer
*
* Returns    : None
*
**************************************************************************************************************
*/

void  WriteLE32U (volatile  USB_INT08U  *pmem,
                            USB_INT32U   val)
{
#ifdef __BIG_ENDIAN
    *(USB_INT32U*)pmem = __REV(val);
#else
    *(USB_INT32U*)pmem = val;
#endif
}

/*
**************************************************************************************************************
*                                          READ LE 16U
*
* Description: This function is used to read an unsigned short integer from a charecter buffer in the platform
*              containing little endian processor
*
* Arguments  : pmem    Pointer to the charecter buffer
*
* Returns    : val     Unsigned short integer
*
**************************************************************************************************************
*/

USB_INT16U  ReadLE16U (volatile  USB_INT08U  *pmem)
{
    USB_INT16U val = *(USB_INT16U*)pmem;
#ifdef __BIG_ENDIAN
    return __REV16(val);
#else
    return val;
#endif    
}

/*
**************************************************************************************************************
*                                         WRITE LE 16U
*
* Description: This function is used to write an unsigned short integer into a charecter buffer in the
*              platform containing little endian processor
*
* Arguments  : pmem    Pointer to the charecter buffer
*              val     Value to be placed in the charecter buffer
*
* Returns    : None
*
**************************************************************************************************************
*/

void  WriteLE16U (volatile  USB_INT08U  *pmem,
                            USB_INT16U   val)
{
#ifdef __BIG_ENDIAN
    *(USB_INT16U*)pmem = (__REV16(val) & 0xFFFF);
#else
    *(USB_INT16U*)pmem = val;
#endif
}

/*
**************************************************************************************************************
*                                         READ BE 32U
*
* Description: This function is used to read an unsigned integer from a charecter buffer in the platform
*              containing big endian processor
*
* Arguments  : pmem    Pointer to the charecter buffer
*
* Returns    : val     Unsigned integer
*
**************************************************************************************************************
*/

USB_INT32U  ReadBE32U (volatile  USB_INT08U  *pmem)
{
    USB_INT32U val = *(USB_INT32U*)pmem;
#ifdef __BIG_ENDIAN
    return val;
#else
    return __REV(val);
#endif
}

/*
**************************************************************************************************************
*                                         WRITE BE 32U
*
* Description: This function is used to write an unsigned integer into a charecter buffer in the platform
*              containing big endian processor
*
* Arguments  : pmem    Pointer to the charecter buffer
*              val     Value to be placed in the charecter buffer
*
* Returns    : None
*
**************************************************************************************************************
*/

void  WriteBE32U (volatile  USB_INT08U  *pmem,
                            USB_INT32U   val)
{
#ifdef __BIG_ENDIAN
    *(USB_INT32U*)pmem = val;
#else
    *(USB_INT32U*)pmem = __REV(val);
#endif
}

/*
**************************************************************************************************************
*                                         READ BE 16U
*
* Description: This function is used to read an unsigned short integer from a charecter buffer in the platform
*              containing big endian processor
*
* Arguments  : pmem    Pointer to the charecter buffer
*
* Returns    : val     Unsigned short integer
*
**************************************************************************************************************
*/

USB_INT16U  ReadBE16U (volatile  USB_INT08U  *pmem)
{
    USB_INT16U val = *(USB_INT16U*)pmem;
#ifdef __BIG_ENDIAN
    return val;
#else
    return __REV16(val);
#endif    
}

/*
**************************************************************************************************************
*                                         WRITE BE 16U
*
* Description: This function is used to write an unsigned short integer into the charecter buffer in the
*              platform containing big endian processor
*
* Arguments  : pmem    Pointer to the charecter buffer
*              val     Value to be placed in the charecter buffer
*
* Returns    : None
*
**************************************************************************************************************
*/

void  WriteBE16U (volatile  USB_INT08U  *pmem,
                            USB_INT16U   val)
{
#ifdef __BIG_ENDIAN
    *(USB_INT16U*)pmem = val;
#else
    *(USB_INT16U*)pmem = (__REV16(val) & 0xFFFF);
#endif
}
