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
* File           : usbhost_err.h
* Programmer(s)  : Ravikanth.P
* Version        :
*
**************************************************************************************************************
*/

#ifndef  USBHOST_ERR_H
#define  USBHOST_ERR_H


/*
**************************************************************************************************************
*                                        GENERAL DEFINITIONS
**************************************************************************************************************
*/

#define  OK                        0
#define  MATCH_FOUND               0

/*
**************************************************************************************************************
*                                HOST CONTROLLER SPECIFIC ERROR CODES
**************************************************************************************************************
*/

#define  ERR_TD_FAIL              -1

/*
**************************************************************************************************************
*                                  MASS STORAGE SPECIFIC ERROR CODES
**************************************************************************************************************
*/

#define  ERR_MS_CMD_FAILED       -10
#define  ERR_BAD_CONFIGURATION   -11
#define  ERR_NO_MS_INTERFACE     -12

/*
**************************************************************************************************************
*                                      FAT SPECIFIC ERROR CODES
**************************************************************************************************************
*/

#define  MATCH_NOT_FOUND         -20
#define  ERR_FAT_NOT_SUPPORTED   -21
#define  ERR_OPEN_LIMIT_REACHED  -22
#define  ERR_INVALID_BOOT_SIG    -23
#define  ERR_INVALID_BOOT_SEC    -24
#define  ERR_ROOT_DIR_FULL       -25

#endif
