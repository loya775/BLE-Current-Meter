/*! *********************************************************************************
 * \addtogroup Wireless UART Application
 * @{
 ********************************************************************************** */
/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* 
*
* This file is the source file for the Wireless UART Application
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
 *************************************************************************************
 * Include
 *************************************************************************************
 ************************************************************************************/
/* Framework / Drivers */
#include "MEMFLASH.h"
#include "fsl_debug_console.h"
#include "fsl_flash.h"
#include "fsl_clock.h"

static flash_config_t s_flashDriver;

uint32_t pflashBlockBase = 0;
uint32_t pflashTotalSize = 0;
uint32_t pflashSectorSize = 0;
status_t result;    /* Return code from each flash driver function */
uint32_t destAdrss; /* Address of the target location */
uint8_t ReadData[20];

void MemFlashInit()
{
	CLOCK_EnableClock(kCLOCK_Mscm);
    memset(&s_flashDriver, 0, sizeof(flash_config_t));

    /* Setup flash driver structure for device and initialize variables. */
    result = FLASH_Init(&s_flashDriver);

    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);
	#ifndef SECTOR_INDEX_FROM_END
  	  #define SECTOR_INDEX_FROM_END 1U
	#endif

#if defined(FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP) && FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
        /* Note: we should make sure that the sector shouldn't be swap indicator sector*/
        destAdrss = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize * 2));
#else
        destAdrss = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize));
#endif
        //result = FLASH_Erase(&s_flashDriver, destAdrss, pflashSectorSize, kFLASH_ApiEraseKey);

}

void MemFlashRead(uint32_t length)
{
	uint32_t s_buffeRead[0];
	uint8_t* ReadDirection;
	uint32_t failAddr, failDat;
	result = FLASH_VerifyProgram(&s_flashDriver, destAdrss, length, s_buffeRead, kFLASH_MarginValueUser,&failAddr, &failDat);
	for (uint32_t i = 0; i < length; i++)
    {
    	ReadData[i] = *(volatile uint8_t *)(destAdrss + i * 4);
    }
}

void MemFlashWrite(uint32_t *s_buffer, uint32_t Length)
{
	result = FLASH_Program(&s_flashDriver, destAdrss, s_buffer, Length);
}
void printMemFlash()
{
	MemFlashRead(10);
	uint8_t* Dic;
	Dic= (uint8_t*)ReadData;
	printingTime(Dic);
}
