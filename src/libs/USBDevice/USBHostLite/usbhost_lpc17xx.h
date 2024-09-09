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

#define  DESC_LENGTH(x)  x[0]
#define  DESC_TYPE(x)    x[1]


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

/*
**************************************************************************************************************
*                                       FUNCTION PROTOTYPES
**************************************************************************************************************
*/

void        Host_Init     (void);

// extern "C" void USB_IRQHandler (void) __irq;

USB_INT32S  Host_EnumDev  (void);

USB_INT32S  Host_ProcessTD(volatile  HCED       *ed,
                           volatile  USB_INT32U  token,
                           volatile  USB_INT08U *buffer,
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
                           volatile  USB_INT08U  *buffer);

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
