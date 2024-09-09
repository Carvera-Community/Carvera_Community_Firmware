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
* File           : usbhost_inc.h
* Programmer(s)  : Ravikanth.P
* Version        :
*
**************************************************************************************************************
*/

#ifndef  USBHOST_INC_H
#define  USBHOST_INC_H

/*
**************************************************************************************************************
*                                       INCLUDE HEADER FILES
**************************************************************************************************************
*/

#include  "usbhost_cpu.h"
#include  "usbhost_err.h"
#include  "usbhost_lpc17xx.h"
#include  "usbhost_ms.h"
#include  "mbed.h"


#ifdef TARGET_LPC2368
#error "There is no USB host on the LPC2368!"
#endif

#endif
