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
* File           : usbhost_ms.c
* Programmer(s)  : Ravikanth.P
* Version        :
*
**************************************************************************************************************
*/

/*
**************************************************************************************************************
*                                       INCLUDE HEADER FILES
**************************************************************************************************************
*/

#include  "usbhost_ms.h"

/*
**************************************************************************************************************
*                                         GLOBAL VARIABLES
**************************************************************************************************************
*/

USB_INT32U  MS_BlkSize;

/*
**************************************************************************************************************
*                                      INITIALIZE MASS STORAGE INTERFACE
*
* Description: This function initializes the mass storage interface
*
* Arguments  : None
*
* Returns    : OK                      if Success
*              ERR_INVALID_BOOTSIG    if Failed
*
**************************************************************************************************************
*/

USB_INT32S MS_Init (USB_INT32U *blkSize, USB_INT32U *numBlks, USB_INT08U *inquiryResult)
{
    USB_INT08U  retry;
    USB_INT32S  rc;

    MS_GetMaxLUN();                                                    /* Get maximum logical unit number   */
    retry  = 80;
    while(retry) {
        rc = MS_TestUnitReady();                                       /* Test whether the unit is ready    */
        if (rc == OK) {
            break;
        }
        MS_GetSenseInfo();                                             /* Get sense information             */
        retry--;
    }
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }
    rc = MS_ReadCapacity(numBlks, blkSize);                         /* Read capacity of the disk         */
    MS_BlkSize = *blkSize;                        // Set global
    rc = MS_Inquire (inquiryResult);
    return (rc);
}

USB_INT32S MS_Enumerate(void)
{
    USB_INT32S rc;

    PRINT_Log("USBHostLite: waiting for USB device\n");
    rc = Host_WaitForDevice(0);
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    rc = Host_ResetRootPort(0, NULL);
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    rc = Host_ReadDeviceDescriptor();
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    rc = Host_SetDeviceAddress(1);
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    USB_INT16U descriptor_len = 0;
    rc = Host_ReadConfigurationDescriptor(HOST_TD_BUFFER_SIZE, &descriptor_len);
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    rc = MS_ParseConfiguration(descriptor_len);
    if (rc != OK) {
        PRINT_Err(rc);
        return (rc);
    }

    rc = USBH_SET_CONFIGURATION(1);
    if (rc != OK) {
        PRINT_Err(rc);
    }
    Host_DelayMS(100);
    return (rc);
}

/*
**************************************************************************************************************
*                                         PARSE THE CONFIGURATION
*
* Description: This function is used to parse the configuration
*
* Arguments  : None
*
* Returns    : OK                      if Success
*              ERR_INVALID_BOOTSIG    if Failed
*
**************************************************************************************************************
*/

USB_INT32S  MS_ParseConfiguration (USB_INT16U descriptor_len)
{
    volatile  USB_INT08U  *desc_ptr;
    volatile  USB_INT08U  *desc_end;
              USB_INT08U   ms_int_found;
              USB_INT08U   current_ms_interface;
              USB_INT08U   bulk_in_found;
              USB_INT08U   bulk_out_found;


    desc_ptr     = TDBuffer;
    desc_end     = TDBuffer + descriptor_len;
    ms_int_found = 0;
    current_ms_interface = 0;
    bulk_in_found = 0;
    bulk_out_found = 0;

    if (descriptor_len < USB_CONFIGURATION_DESCRIPTOR_HEADER_SIZE ||
        desc_ptr[1] != USB_DESCRIPTOR_TYPE_CONFIGURATION ||
        desc_ptr[0] < USB_CONFIGURATION_DESCRIPTOR_HEADER_SIZE ||
        desc_ptr[0] > descriptor_len) {
        return (ERR_BAD_CONFIGURATION);
    }
    desc_ptr += desc_ptr[0];

    while (desc_ptr < desc_end) {
        USB_INT16U remaining = desc_end - desc_ptr;
        if (remaining < 2 || desc_ptr[0] < 2 || desc_ptr[0] > remaining) {
            return (ERR_BAD_CONFIGURATION);
        }

        switch (desc_ptr[1]) {

            case USB_DESCRIPTOR_TYPE_INTERFACE: {                     /* If it is an interface descriptor   */
                 USB_INT08U is_ms_interface = desc_ptr[0] >= 8 &&
                     desc_ptr[5] == MASS_STORAGE_CLASS &&
                     desc_ptr[6] == MASS_STORAGE_SUBCLASS_SCSI &&
                     desc_ptr[7] == MASS_STORAGE_PROTOCOL_BO;
                 current_ms_interface = is_ms_interface && !ms_int_found;
                 if (current_ms_interface) {
                     ms_int_found = 1;
                 }
                 break;
            }

            case USB_DESCRIPTOR_TYPE_ENDPOINT:                        /* If it is an endpoint descriptor    */
                 if (current_ms_interface && desc_ptr[0] >= 6 &&
                     (desc_ptr[3] & USB_ENDPOINT_TRANSFER_TYPE_MASK) == USB_ENDPOINT_TRANSFER_TYPE_BULK) {
                     if (desc_ptr[2] & 0x80) {                        /* If it is In endpoint               */
                         EDBulkIn->Control =  1                             |      /* USB address           */
                                              ((desc_ptr[2] & 0x7F) << 7)   |      /* Endpoint address      */
                                              (2 << 11)                     |      /* direction             */
                                              (ReadLE16U(&desc_ptr[4]) << 16);     /* MaxPkt Size           */
                         bulk_in_found = 1;
                     } else {                                         /* If it is Out endpoint              */
                         EDBulkOut->Control = 1                             |      /* USB address           */
                                              ((desc_ptr[2] & 0x7F) << 7)   |      /* Endpoint address      */
                                              (1 << 11)                     |      /* direction             */
                                              (ReadLE16U(&desc_ptr[4]) << 16);     /* MaxPkt Size           */
                         bulk_out_found = 1;
                     }
                 }
                 break;

            default:
                 break;
        }
        desc_ptr += desc_ptr[0];
    }
    if (ms_int_found && bulk_in_found && bulk_out_found) {
        PRINT_Log("Mass Storage device connected\n");
        return (OK);
    } else {
        PRINT_Log("Not a Mass Storage device\n");
        return (ERR_NO_MS_INTERFACE);
    }
}

/*
**************************************************************************************************************
*                                         GET MAXIMUM LOGICAL UNIT
*
* Description: This function returns the maximum logical unit from the device
*
* Arguments  : None
*
* Returns    : OK                      if Success
*              ERR_INVALID_BOOTSIG    if Failed
*
**************************************************************************************************************
*/

USB_INT32S  MS_GetMaxLUN (void)
{
    USB_INT32S  rc;


    rc = Host_CtrlRecv(USB_DEVICE_TO_HOST | USB_REQUEST_TYPE_CLASS | USB_RECIPIENT_INTERFACE,
                       MS_GET_MAX_LUN_REQ,
                       0,
                       0,
                       1,
                       TDBuffer);
    return (rc); 
}

/*
**************************************************************************************************************
*                                          GET SENSE INFORMATION
*
* Description: This function is used to get sense information from the device
*
* Arguments  : None
*
* Returns    : OK       if Success
*              ERROR    if Failed
*
**************************************************************************************************************
*/

USB_INT32S  MS_GetSenseInfo (void)
{
    USB_INT32S  rc;


    Fill_MSCommand(0, 0, 0, MS_DATA_DIR_IN, SCSI_CMD_REQUEST_SENSE, 6);
    rc = Host_ProcessTD(EDBulkOut, TD_OUT, TDBuffer, CBW_SIZE);
    if (rc == OK) {
        rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, 18);
        if (rc == OK) {
            rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, CSW_SIZE);
            if (rc == OK) {
                if (TDBuffer[12] != 0) {
                    rc = ERR_MS_CMD_FAILED;
                }
            }
        }
    }
    return (rc);
}

/*
**************************************************************************************************************
*                                           TEST UNIT READY
*
* Description: This function is used to test whether the unit is ready or not
*
* Arguments  : None
*
* Returns    : OK       if Success
*              ERROR    if Failed
*
**************************************************************************************************************
*/

USB_INT32S  MS_TestUnitReady (void)
{
    USB_INT32S  rc;


    Fill_MSCommand(0, 0, 0, MS_DATA_DIR_NONE, SCSI_CMD_TEST_UNIT_READY, 6);
    rc = Host_ProcessTD(EDBulkOut, TD_OUT, TDBuffer, CBW_SIZE);
    if (rc == OK) {
        rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, CSW_SIZE);
        if (rc == OK) {        
            if (TDBuffer[12] != 0) {
                rc = ERR_MS_CMD_FAILED;
            }
        }
    }
    return (rc);
}

/*
**************************************************************************************************************
*                                            READ CAPACITY
*
* Description: This function is used to read the capacity of the mass storage device
*
* Arguments  : None
*
* Returns    : OK       if Success
*              ERROR    if Failed
*
**************************************************************************************************************
*/

USB_INT32S MS_ReadCapacity (USB_INT32U *numBlks, USB_INT32U *blkSize)
{
    USB_INT32S  rc;


    Fill_MSCommand(0, 0, 0, MS_DATA_DIR_IN, SCSI_CMD_READ_CAPACITY, 10);
    rc = Host_ProcessTD(EDBulkOut, TD_OUT, TDBuffer, CBW_SIZE);
    if (rc == OK) {
        rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, 8);
        if (rc == OK) {
            if (numBlks)
                *numBlks = ReadBE32U(&TDBuffer[0]);
            if (blkSize)
                *blkSize = ReadBE32U(&TDBuffer[4]);
            rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, CSW_SIZE);
            if (rc == OK) {
                if (TDBuffer[12] != 0) {
                    rc = ERR_MS_CMD_FAILED;
                }
            }
        }
    }
    return (rc);
}



USB_INT32S MS_Inquire (USB_INT08U *response)
{
    USB_INT32S rc;
    USB_INT32U i;

    Fill_MSCommand(0, 0, 0, MS_DATA_DIR_IN, SCSI_CMD_INQUIRY, 6);
    rc = Host_ProcessTD(EDBulkOut, TD_OUT, TDBuffer, CBW_SIZE);
    if (rc == OK) {
        rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, INQUIRY_LENGTH);
        if (rc == OK) {
            if (response) {
                for ( i = 0; i < INQUIRY_LENGTH; i++ )
                    *response++ = *TDBuffer++;
#if 0
                MemCpy (response, TDBuffer, INQUIRY_LENGTH);
                StrNullTrailingSpace (response->vendorID, SCSI_INQUIRY_VENDORCHARS);
                StrNullTrailingSpace (response->productID, SCSI_INQUIRY_PRODUCTCHARS);
                StrNullTrailingSpace (response->productRev, SCSI_INQUIRY_REVCHARS);
#endif
            }
            rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, CSW_SIZE);
            if (rc == OK) {
                if (TDBuffer[12] != 0) {    // bCSWStatus byte
                    rc = ERR_MS_CMD_FAILED;
                }
            }
        }
    }
    return (rc);
}

/*
**************************************************************************************************************
*                                         RECEIVE THE BULK DATA
*
* Description: This function is used to receive the bulk data
*
* Arguments  : None
*
* Returns    : OK                      if Success
*              ERR_INVALID_BOOTSIG    if Failed
*
**************************************************************************************************************
*/
    
USB_INT32S  MS_BulkRecv (          USB_INT32U   block_number,
                                   USB_INT16U   num_blocks,
                         volatile  USB_INT08U  *user_buffer)
{
    if(num_blocks != 1 || MS_BlkSize > 0x200) return ERR_MS_CMD_FAILED;
    USB_INT32S  rc;
    unsigned int i;
    volatile USB_INT08U *c = user_buffer;
    for (i=0;i<MS_BlkSize*num_blocks;i++)
        *c++ = 0;


    Fill_MSCommand(block_number, MS_BlkSize, num_blocks, MS_DATA_DIR_IN, SCSI_CMD_READ_10, 10);

    rc = Host_ProcessTD(EDBulkOut, TD_OUT, TDBuffer, CBW_SIZE);
    if (rc == OK) {
        rc = Host_ProcessTD(EDBulkIn, TD_IN, MSBuffer, MS_BlkSize);
        if (rc == OK) {
            for(i = 0; i < MS_BlkSize; i++) user_buffer[i] = MSBuffer[i];
            rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, CSW_SIZE);
            if (rc == OK) {
                if (TDBuffer[12] != 0) {
                    rc = ERR_MS_CMD_FAILED;
                }
            }
        }
    }
    return (rc);
}

/*
**************************************************************************************************************
*                                         SEND BULK DATA
*
* Description: This function is used to send the bulk data
*
* Arguments  : None
*
* Returns    : OK                      if Success
*              ERR_INVALID_BOOTSIG    if Failed
*
**************************************************************************************************************
*/

USB_INT32S  MS_BulkSend (          USB_INT32U   block_number,
                                   USB_INT16U   num_blocks,
                   const volatile  USB_INT08U  *user_buffer)
{
    if(num_blocks != 1 || MS_BlkSize > 0x200) return ERR_MS_CMD_FAILED;
    USB_INT32S  rc;
    for(unsigned int i = 0; i < MS_BlkSize; i++) MSBuffer[i] = user_buffer[i];


    Fill_MSCommand(block_number, MS_BlkSize, num_blocks, MS_DATA_DIR_OUT, SCSI_CMD_WRITE_10, 10);

    rc = Host_ProcessTD(EDBulkOut, TD_OUT, TDBuffer, CBW_SIZE);
    if (rc == OK) {
        rc = Host_ProcessTD(EDBulkOut, TD_OUT, MSBuffer, MS_BlkSize);
        if (rc == OK) {
            rc = Host_ProcessTD(EDBulkIn, TD_IN, TDBuffer, CSW_SIZE);
            if (rc == OK) {
                if (TDBuffer[12] != 0) {
                    rc = ERR_MS_CMD_FAILED;
                }
            }
        }
    }
    return (rc);
}

/*
**************************************************************************************************************
*                                         FILL MASS STORAGE COMMAND
*
* Description: This function is used to fill the mass storage command
*
* Arguments  : None
*
* Returns    : OK                      if Success
*              ERR_INVALID_BOOTSIG    if Failed
*
**************************************************************************************************************
*/

void  Fill_MSCommand (USB_INT32U   block_number,
                      USB_INT32U   block_size,
                      USB_INT16U   num_blocks,
                      MS_DATA_DIR  direction,
                      USB_INT08U   scsi_cmd,
                      USB_INT08U   scsi_cmd_len)
{
            USB_INT32U  data_len;
    static  USB_INT32U  tag_cnt = 0;
            USB_INT32U  cnt;


    for (cnt = 0; cnt < CBW_SIZE; cnt++) {
         TDBuffer[cnt] = 0;
    }
    switch(scsi_cmd) {

        case SCSI_CMD_TEST_UNIT_READY:
             data_len = 0;
             break;
        case SCSI_CMD_READ_CAPACITY:
             data_len = 8;
             break;
        case SCSI_CMD_REQUEST_SENSE:
             data_len = 18;
             break;
        case SCSI_CMD_INQUIRY:
             data_len = 36;
             break;
        default:
             data_len = block_size * num_blocks;
             break;
    }
    WriteLE32U(TDBuffer, CBW_SIGNATURE);
    WriteLE32U(&TDBuffer[4], tag_cnt);
    WriteLE32U(&TDBuffer[8], data_len);
    TDBuffer[12]     = (direction == MS_DATA_DIR_NONE) ? 0 : direction;
    TDBuffer[14]     = scsi_cmd_len;                                   /* Length of the CBW                 */
    TDBuffer[15]     = scsi_cmd;
    if ((scsi_cmd     == SCSI_CMD_REQUEST_SENSE)
     || (scsi_cmd     == SCSI_CMD_INQUIRY)) {
        TDBuffer[19] = (USB_INT08U)data_len;
    } else {
        WriteBE32U(&TDBuffer[17], block_number);
    }
    WriteBE16U(&TDBuffer[22], num_blocks);
}
