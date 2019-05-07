/*! *********************************************************************************
 * \defgroup Wireless UART Application
 * @{
 ********************************************************************************** */
/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* 
*
* This file is the interface file for the Wireless UART application
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef _MEMFLASH_H_
#define _MEMFLASH_H_
#include "fsl_debug_console.h"
#include "fsl_flash.h"
#include "fsl_clock.h"
void MemFlashInit();

void MemFlashRead(uint32_t length);
void printMemFlash();
void MemFlashWrite(uint32_t *s_buffer, uint32_t Length);

#endif /* _APP_H_ */

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
