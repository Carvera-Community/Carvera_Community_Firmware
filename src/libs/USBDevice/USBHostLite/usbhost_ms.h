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
* File           : usbhost_ms.h
* Programmer(s)  : Ravikanth.P
* Version        :
*
**************************************************************************************************************
*/

#ifndef  USBHOST_MS_H
#define  USBHOST_MS_H

/*
**************************************************************************************************************
*                                       INCLUDE HEADER FILES
**************************************************************************************************************
*/

#include  "usbhost_inc.h"

/*
**************************************************************************************************************
*                               MASS STORAGE SPECIFIC DEFINITIONS
**************************************************************************************************************
*/

#define    MS_GET_MAX_LUN_REQ            0xFE
#define    MASS_STORAGE_CLASS            0x08
#define    MASS_STORAGE_SUBCLASS_SCSI    0x06
#define    MASS_STORAGE_PROTOCOL_BO      0x50

#define    INQUIRY_LENGTH                36
/*
**************************************************************************************************************
*                                  SCSI SPECIFIC DEFINITIONS
**************************************************************************************************************
*/

#define  CBW_SIGNATURE               0x43425355
#define  CSW_SIGNATURE               0x53425355
#define  CBW_SIZE                      31
#define  CSW_SIZE                      13
#define  CSW_CMD_PASSED              0x00
#define  SCSI_CMD_REQUEST_SENSE      0x03
#define  SCSI_CMD_TEST_UNIT_READY    0x00
#define  SCSI_CMD_INQUIRY            0x12
#define  SCSI_CMD_READ_10            0x28
#define  SCSI_CMD_READ_CAPACITY      0x25
#define  SCSI_CMD_WRITE_10           0x2A

/*
**************************************************************************************************************
*                                       TYPE DEFINITIONS
**************************************************************************************************************
*/

typedef enum  ms_data_dir {

    MS_DATA_DIR_IN     = 0x80,
    MS_DATA_DIR_OUT    = 0x00,
    MS_DATA_DIR_NONE   = 0x01

} MS_DATA_DIR;

/*
**************************************************************************************************************
*                                     FUNCTION PROTOTYPES
**************************************************************************************************************
*/

USB_INT32S  MS_BulkRecv          (          USB_INT32U    block_number,
                                            USB_INT16U    num_blocks,
                                  volatile  USB_INT08U   *user_buffer);

USB_INT32S  MS_BulkSend          (          USB_INT32U    block_number,
                                            USB_INT16U    num_blocks,
                                  volatile  USB_INT08U   *user_buffer);
USB_INT32S  MS_ParseConfiguration(void);
USB_INT32S  MS_TestUnitReady     (void);
USB_INT32S  MS_ReadCapacity (USB_INT32U *numBlks, USB_INT32U *blkSize);
USB_INT32S  MS_GetMaxLUN         (void);
USB_INT32S  MS_GetSenseInfo      (void);
USB_INT32S  MS_Init (USB_INT32U *blkSize, USB_INT32U *numBlks, USB_INT08U *inquiryResult);
USB_INT32S  MS_Inquire (USB_INT08U *response);

void        Fill_MSCommand       (          USB_INT32U    block_number,
                                            USB_INT32U    block_size,
                                            USB_INT16U    num_blocks,
                                            MS_DATA_DIR   direction,
                                            USB_INT08U    scsi_cmd,
                                            USB_INT08U    scsi_cmd_len);
#endif
