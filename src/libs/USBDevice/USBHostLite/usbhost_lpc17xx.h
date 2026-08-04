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
* File           : usbhost_lpc17xx.h
* Programmer(s)  : Ravikanth.P
* Version        :
*
**************************************************************************************************************
*/

#ifndef USBHOST_LPC17xx_H
#define USBHOST_LPC17xx_H

/*
**************************************************************************************************************
*                                       INCLUDE HEADER FILES
**************************************************************************************************************
*/

#include    "usbhost_inc.h"

/*
**************************************************************************************************************
*                                        PRINT CONFIGURATION
**************************************************************************************************************
*/

#define  PRINT_ENABLE         1

#if PRINT_ENABLE
#define  PRINT_Log(...)       printf(__VA_ARGS__)
#define  PRINT_Err(rc)        printf("ERROR: In %s at Line %u - rc = %d\n", __FUNCTION__, __LINE__, rc)

#else 
#define  PRINT_Log(...)       do {} while(0)
#define  PRINT_Err(rc)        do {} while(0)

#endif

/*
**************************************************************************************************************
*                                        GENERAL DEFINITIONS
**************************************************************************************************************
*/

#define  USB_DESCRIPTOR_LENGTH_OFFSET                   0
#define  USB_DESCRIPTOR_TYPE_OFFSET                     1

#define  USB_CONFIGURATION_TOTAL_LENGTH_OFFSET          2

#define  USB_DEVICE_MAX_PACKET_SIZE0_OFFSET             7

#define  USB_INTERFACE_NUMBER_OFFSET                    2
#define  USB_INTERFACE_CLASS_OFFSET                     5

#define  USB_ENDPOINT_ADDRESS_OFFSET                    2
#define  USB_ENDPOINT_ATTRIBUTES_OFFSET                 3
#define  USB_ENDPOINT_MAX_PACKET_SIZE_OFFSET            4
#define  USB_ENDPOINT_DIRECTION_IN                      0x80
#define  USB_ENDPOINT_NUMBER_MASK                       0x7F
#define  USB_ENDPOINT_TRANSFER_TYPE_MASK                0x03
#define  USB_ENDPOINT_TRANSFER_TYPE_BULK                0x02

#define  USB_DEVICE_ADDRESS                             1
#define  USB_CONFIGURATION_VALUE                        1
#define  USB_SET_CONFIGURATION_SETTLE_MS                100
#define  USB_ENDPOINT_ZERO_INITIAL_DESCRIPTOR_BYTES      8
#define  USB_SETUP_PACKET_SIZE                          8
#define  USB_CONFIGURATION_DESCRIPTOR_HEADER_SIZE        9
#define  USB_SETUP_BM_REQUEST_TYPE_OFFSET               0
#define  USB_SETUP_B_REQUEST_OFFSET                     1
#define  USB_SETUP_W_VALUE_OFFSET                       2
#define  USB_SETUP_W_INDEX_OFFSET                       4
#define  USB_SETUP_W_LENGTH_OFFSET                      6

#define  DESC_LENGTH(x)                                 (x)[USB_DESCRIPTOR_LENGTH_OFFSET]
#define  DESC_TYPE(x)                                   (x)[USB_DESCRIPTOR_TYPE_OFFSET]

#define  OHCI_ED_FUNCTION_ADDRESS(address)              ((USB_INT32U)(address))
#define  OHCI_ED_ENDPOINT_NUMBER(endpoint_address)      (((USB_INT32U)((endpoint_address) & USB_ENDPOINT_NUMBER_MASK)) << 7)
#define  OHCI_ED_DIRECTION(direction)                   ((USB_INT32U)(direction) << 11)
#define  OHCI_ED_SKIP                                   (1UL << 14)
#define  OHCI_ED_MAX_PACKET_SIZE(max_packet)            ((USB_INT32U)(max_packet) << 16)
#define  OHCI_ED_DIR_OUT                                1
#define  OHCI_ED_DIR_IN                                 2
#define  OHCI_ED_HEAD_TOGGLE_CARRY                      0x00000002
#define  OHCI_ED_BULK_ENDPOINT_CONTROL(device_address, endpoint_address, direction, max_packet) \
         (OHCI_ED_FUNCTION_ADDRESS(device_address) |                                            \
          OHCI_ED_ENDPOINT_NUMBER(endpoint_address) |                                           \
          OHCI_ED_DIRECTION(direction) |                                                        \
          OHCI_ED_MAX_PACKET_SIZE(max_packet))


#define  HOST_GET_DESCRIPTOR(descType, descIndex, data, length)                      \
         Host_CtrlRecv(USB_DEVICE_TO_HOST | USB_RECIPIENT_DEVICE, GET_DESCRIPTOR,    \
         (descType << 8)|(descIndex), 0, length, data)

#define  HOST_SET_ADDRESS(new_addr)                                                  \
         Host_CtrlSend(USB_HOST_TO_DEVICE | USB_RECIPIENT_DEVICE, SET_ADDRESS,       \
         new_addr, 0, 0, NULL)

#define  USBH_SET_CONFIGURATION(configNum)                                           \
         Host_CtrlSend(USB_HOST_TO_DEVICE | USB_RECIPIENT_DEVICE, SET_CONFIGURATION, \
         configNum, 0, 0, NULL)

#define  USBH_SET_INTERFACE(ifNum, altNum)                                           \
         Host_CtrlSend(USB_HOST_TO_DEVICE | USB_RECIPIENT_INTERFACE, SET_INTERFACE,  \
         altNum, ifNum, 0, NULL)

/*
**************************************************************************************************************
*                                  OHCI OPERATIONAL REGISTER FIELD DEFINITIONS
**************************************************************************************************************
*/

                                            /* ------------------ HcControl Register ---------------------  */
#define  OR_CONTROL_CLE                 0x00000010
#define  OR_CONTROL_BLE                 0x00000020
#define  OR_CONTROL_HCFS                0x000000C0
#define  OR_CONTROL_HC_OPER             0x00000080
                                            /* ----------------- HcCommandStatus Register ----------------- */
#define  OR_CMD_STATUS_HCR              0x00000001
#define  OR_CMD_STATUS_CLF              0x00000002
#define  OR_CMD_STATUS_BLF              0x00000004
                                            /* --------------- HcInterruptStatus Register ----------------- */
#define  OR_INTR_STATUS_WDH             0x00000002
#define  OR_INTR_STATUS_RHSC            0x00000040
                                            /* --------------- HcInterruptEnable Register ----------------- */
#define  OR_INTR_ENABLE_WDH             0x00000002
#define  OR_INTR_ENABLE_RHSC            0x00000040
#define  OR_INTR_ENABLE_MIE             0x80000000
                                            /* ---------------- HcRhDescriptorA Register ------------------ */
#define  OR_RH_STATUS_LPSC              0x00010000
#define  OR_RH_STATUS_DRWE              0x00008000
                                            /* -------------- HcRhPortStatus[1:NDP] Register -------------- */
#define  OR_RH_PORT_CCS                 0x00000001
#define  OR_RH_PORT_PRS                 0x00000010
#define  OR_RH_PORT_CSC                 0x00010000
#define  OR_RH_PORT_PRSC                0x00100000


/*
**************************************************************************************************************
*                                               FRAME INTERVAL
**************************************************************************************************************
*/

#define  FI                     0x2EDF           /* 12000 bits per frame (-1)                               */
#define  DEFAULT_FMINTERVAL     ((((6 * (FI - 210)) / 7) << 16) | FI)

/*
**************************************************************************************************************
*                                       TRANSFER DESCRIPTOR CONTROL FIELDS
**************************************************************************************************************
*/

#define  TD_ROUNDING        (USB_INT32U) (0x00040000)        /* Buffer Rounding                             */
#define  TD_SETUP           (USB_INT32U)(0)                  /* Direction of Setup Packet                   */
#define  TD_IN              (USB_INT32U)(0x00100000)         /* Direction In                                */
#define  TD_OUT             (USB_INT32U)(0x00080000)         /* Direction Out                               */
#define  TD_DELAY_INT(x)    (USB_INT32U)((x) << 21)          /* Delay Interrupt                             */
#define  TD_TOGGLE_0        (USB_INT32U)(0x02000000)         /* Toggle 0                                    */
#define  TD_TOGGLE_1        (USB_INT32U)(0x03000000)         /* Toggle 1                                    */
#define  TD_CC              (USB_INT32U)(0xF0000000)         /* Completion Code                             */
#define  TD_CC_SHIFT        28
#define  TD_CC_MASK         0x0F

/*
**************************************************************************************************************
*                                       USB STANDARD REQUEST DEFINITIONS
**************************************************************************************************************
*/

#define  USB_DESCRIPTOR_TYPE_DEVICE                     1
#define  USB_DESCRIPTOR_TYPE_CONFIGURATION              2
#define  USB_DESCRIPTOR_TYPE_INTERFACE                  4
#define  USB_DESCRIPTOR_TYPE_ENDPOINT                   5
                                                    /*  ----------- Control RequestType Fields  ----------- */
#define  USB_DEVICE_TO_HOST         0x80
#define  USB_HOST_TO_DEVICE         0x00
#define  USB_REQUEST_TYPE_CLASS     0x20
#define  USB_RECIPIENT_DEVICE       0x00
#define  USB_RECIPIENT_INTERFACE    0x01
                                                    /* -------------- USB Standard Requests  -------------- */
#define  SET_ADDRESS                 5
#define  GET_DESCRIPTOR              6
#define  SET_CONFIGURATION           9
#define  SET_INTERFACE              11

/*
**************************************************************************************************************
*                                       TYPE DEFINITIONS
**************************************************************************************************************
*/

typedef struct hcEd {                       /* ----------- HostController EndPoint Descriptor ------------- */
    volatile  USB_INT32U  Control;              /* Endpoint descriptor control                              */
    volatile  USB_INT32U  TailTd;               /* Physical address of tail in Transfer descriptor list     */
    volatile  USB_INT32U  HeadTd;               /* Physcial address of head in Transfer descriptor list     */
    volatile  USB_INT32U  Next;                 /* Physical address of next Endpoint descriptor             */
} HCED;

typedef struct hcTd {                       /* ------------ HostController Transfer Descriptor ------------ */
    volatile  USB_INT32U  Control;              /* Transfer descriptor control                              */
    volatile  USB_INT32U  CurrBufPtr;           /* Physical address of current buffer pointer               */
    volatile  USB_INT32U  Next;                 /* Physical pointer to next Transfer Descriptor             */
    volatile  USB_INT32U  BufEnd;               /* Physical address of end of buffer                        */
} HCTD;

typedef struct hcca {                       /* ----------- Host Controller Communication Area ------------  */
    volatile  USB_INT32U  IntTable[32];         /* Interrupt Table                                          */
    volatile  USB_INT32U  FrameNumber;          /* Frame Number                                             */
    volatile  USB_INT32U  DoneHead;             /* Done Head                                                */
    volatile  USB_INT08U  Reserved[116];        /* Reserved for future use                                  */
    volatile  USB_INT08U  Unknown[4];           /* Unused                                                   */
} HCCA;

/*
**************************************************************************************************************
*                                     EXTERN DECLARATIONS
**************************************************************************************************************
*/

extern  volatile  HCED        *EDBulkIn;        /* BulkIn endpoint descriptor  structure                    */
extern  volatile  HCED        *EDBulkOut;       /* BulkOut endpoint descriptor structure                    */
extern  volatile  HCTD        *TDHead;          /* Head transfer descriptor structure                       */
extern  volatile  HCTD        *TDTail;          /* Tail transfer descriptor structure                       */
extern  volatile  USB_INT08U  *TDBuffer;        /* Current Buffer Pointer of transfer descriptor            */

typedef struct hostPortResetStatus {
    USB_INT32U waited_ms;
    USB_INT32U port_status;
    USB_INT32U interrupt_status;
} HostPortResetStatus;

/*
**************************************************************************************************************
*                                       FUNCTION PROTOTYPES
**************************************************************************************************************
*/

void        Host_Init     (void);

// extern "C" void USB_IRQHandler (void) __irq;

USB_INT32S  Host_WaitForDevice(USB_INT32U timeout_ms);
USB_INT32S  Host_ResetRootPort(USB_INT32U timeout_ms, HostPortResetStatus *status);
USB_INT32S  Host_ReadDeviceDescriptor(void);
USB_INT32S  Host_SetDeviceAddress(USB_INT08U address);
USB_INT32S  Host_ReadConfigurationDescriptor(USB_INT16U max_len, USB_INT16U *fetched_len);

USB_INT32S  Host_ProcessTD(volatile  HCED       *ed,
                           volatile  USB_INT32U  token,
                     const volatile  USB_INT08U *buffer,
                                     USB_INT32U  buffer_len);

void        Host_DelayUS  (          USB_INT32U    delay);
void        Host_DelayMS  (          USB_INT32U    delay);


void        Host_TDInit   (volatile  HCTD *td);
void        Host_EDInit   (volatile  HCED *ed);
void        Host_HCCAInit (volatile  HCCA  *hcca);

USB_INT32S  Host_CtrlRecv (          USB_INT08U   bm_request_type,
                                     USB_INT08U   b_request,
                                     USB_INT16U   w_value,
                                     USB_INT16U   w_index,
                                     USB_INT16U   w_length,
                           volatile  USB_INT08U  *buffer);

USB_INT32S  Host_CtrlSend (          USB_INT08U   bm_request_type,
                                     USB_INT08U   b_request,
                                     USB_INT16U   w_value,
                                     USB_INT16U   w_index,
                                     USB_INT16U   w_length,
                     const volatile  USB_INT08U  *buffer);

void        Host_FillSetup(          USB_INT08U   bm_request_type,
                                     USB_INT08U   b_request,
                                     USB_INT16U   w_value,
                                     USB_INT16U   w_index,
                                     USB_INT16U   w_length);


void        Host_WDHWait  (void);


USB_INT32U  ReadLE32U     (volatile  USB_INT08U  *pmem);
void        WriteLE32U    (volatile  USB_INT08U  *pmem,
                                     USB_INT32U   val);
USB_INT16U  ReadLE16U     (volatile  USB_INT08U  *pmem);
void        WriteLE16U    (volatile  USB_INT08U  *pmem,
                                     USB_INT16U   val);
USB_INT32U  ReadBE32U     (volatile  USB_INT08U  *pmem);
void        WriteBE32U    (volatile  USB_INT08U  *pmem,
                                     USB_INT32U   val);
USB_INT16U  ReadBE16U     (volatile  USB_INT08U  *pmem);
void        WriteBE16U    (volatile  USB_INT08U  *pmem,
                                     USB_INT16U   val);

#endif
