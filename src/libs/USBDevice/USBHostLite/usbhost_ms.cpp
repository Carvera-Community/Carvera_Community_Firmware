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

USB_INT32S  MS_ParseConfiguration (void)
{
    volatile  USB_INT08U  *desc_ptr;
              USB_INT08U   ms_int_found;


    desc_ptr     = TDBuffer;
    ms_int_found = 0;

    if (desc_ptr[1] != USB_DESCRIPTOR_TYPE_CONFIGURATION) {    
        return (ERR_BAD_CONFIGURATION);
    }
    desc_ptr += desc_ptr[0];

    while (desc_ptr != TDBuffer + ReadLE16U(&TDBuffer[2])) {

        switch (desc_ptr[1]) {

            case USB_DESCRIPTOR_TYPE_INTERFACE:                       /* If it is an interface descriptor   */
                 if (desc_ptr[5] == MASS_STORAGE_CLASS &&             /* check if the class is mass storage */
                     desc_ptr[6] == MASS_STORAGE_SUBCLASS_SCSI &&     /* check if the subclass is SCSI      */
                     desc_ptr[7] == MASS_STORAGE_PROTOCOL_BO) {       /* check if the protocol is Bulk only */
                     ms_int_found = 1;
                     desc_ptr    += desc_ptr[0];                      /* Move to next descriptor start      */
                 }
                 break;

            case USB_DESCRIPTOR_TYPE_ENDPOINT:                        /* If it is an endpoint descriptor    */
                 if ((desc_ptr[3] & 0x03) == 0x02) {                  /* If it is Bulk endpoint             */
                     if (desc_ptr[2] & 0x80) {                        /* If it is In endpoint               */
                         EDBulkIn->Control =  1                             |      /* USB address           */
                                              ((desc_ptr[2] & 0x7F) << 7)   |      /* Endpoint address      */
                                              (2 << 11)                     |      /* direction             */
                                              (ReadLE16U(&desc_ptr[4]) << 16);     /* MaxPkt Size           */
                         desc_ptr += desc_ptr[0];                     /* Move to next descriptor start      */
                     } else {                                         /* If it is Out endpoint              */
                         EDBulkOut->Control = 1                             |      /* USB address           */
                                              ((desc_ptr[2] & 0x7F) << 7)   |      /* Endpoint address      */
                                              (1 << 11)                     |      /* direction             */
                                              (ReadLE16U(&desc_ptr[4]) << 16);     /* MaxPkt Size           */
                         desc_ptr += desc_ptr[0];                     /* Move to next descriptor start      */
                     }
                 } else {                                             /* If it is not bulk end point        */
                     desc_ptr += desc_ptr[0];                         /* Move to next descriptor start      */
                 }
                 break;

            default:                                 /* If the descriptor is neither interface nor endpoint */
                 desc_ptr += desc_ptr[0];                             /* Move to next descriptor start      */
                 break;
        }
    }
    if (ms_int_found) {
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
    USB_INT32S  rc;
    int i;
    volatile USB_INT08U *c = user_buffer;
    for (i=0;i<MS_BlkSize*num_blocks;i++)
        *c++ = 0;


    Fill_MSCommand(block_number, MS_BlkSize, num_blocks, MS_DATA_DIR_IN, SCSI_CMD_READ_10, 10);

    rc = Host_ProcessTD(EDBulkOut, TD_OUT, TDBuffer, CBW_SIZE);
    if (rc == OK) {
        rc = Host_ProcessTD(EDBulkIn, TD_IN, user_buffer, MS_BlkSize * num_blocks);
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
                         volatile  USB_INT08U  *user_buffer)
{
    USB_INT32S  rc;


    Fill_MSCommand(block_number, MS_BlkSize, num_blocks, MS_DATA_DIR_OUT, SCSI_CMD_WRITE_10, 10);

    rc = Host_ProcessTD(EDBulkOut, TD_OUT, TDBuffer, CBW_SIZE);
    if (rc == OK) {
        rc = Host_ProcessTD(EDBulkOut, TD_OUT, user_buffer, MS_BlkSize * num_blocks);
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
