/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcmgr_internal_core_api.h"
#include <stdio.h>
#include <string.h>
#include "fsl_device_registers.h"
#include "fsl_mu.h"

/* Count of cores in the system */
#define MCMGR_CORECOUNT 2

/* Count of memory regions in the system */
#define MCMGR_MEMREGCOUNT 2

/* MCMGR MU channel index - used for passing startupData */
#define MCMGR_MU_CHANNEL 3

/* MU TR/RR $MCMGR_MU_CHANNEL is managed by MCMGR */
#define MU_RX_ISR_Handler(x) MU_RX_ISR(x)
#define MU_RX_ISR(number) MU_Rx##number##FullFlagISR
#define mcmgr_mu_channel_handler MU_RX_ISR_Handler(MCMGR_MU_CHANNEL)
#define MU_RX_ISR_FLAG_Mask(x) MU_RX_ISR_FLAG(x)
#define MU_RX_ISR_FLAG(number) kMU_Rx##number##FullInterruptEnable
#define mcmgr_mu_channel_flag MU_RX_ISR_FLAG_Mask(MCMGR_MU_CHANNEL)

/* MCMGR boot address of RAM */
#define MCMGR_BOOT_ADDRESS_RAM (0x09000000U)

/* MCMGR boot address of flash memory */
#define MCMGR_BOOT_ADDRESS_FLASH (0x01000000U)

volatile mcmgr_core_context_t s_mcmgrCoresContext[MCMGR_CORECOUNT] = {{.state = kMCMGR_ResetCoreState, .startupData = 0},
                                                                      {.state = kMCMGR_ResetCoreState, .startupData = 0}};

/* Initialize structure with informations of all cores */
static const mcmgr_core_info_t s_mcmgrCores[MCMGR_CORECOUNT] = {
    {.coreType = kMCMGR_CoreTypeCortexM4, .coreName = "Main"},
    {.coreType = kMCMGR_CoreTypeCortexM0Plus, .coreName = "Secondary"}};

const mcmgr_system_info_t g_mcmgrSystem = {
    .coreCount = MCMGR_CORECOUNT, .memRegCount = MCMGR_MEMREGCOUNT, .cores = s_mcmgrCores};

mcmgr_status_t mcmgr_early_init_internal(mcmgr_core_t coreNum)
{
    switch (coreNum)
    {
        case kMCMGR_Core0:
/* MUA clk enable */
#if defined(FSL_FEATURE_MU_SIDE_A)
            MU_Init(MUA);
#endif
            break;
        case kMCMGR_Core1:
#if defined(FSL_FEATURE_MU_SIDE_B)
            MU_Init(MUB);
#endif
            break;
        default:
            return kStatus_MCMGR_Error;
    }

    /* Trigger core up event here, core is starting! */
    MCMGR_TriggerEvent(kMCMGR_RemoteCoreUpEvent, 0);

    return kStatus_MCMGR_Success;
}

mcmgr_status_t mcmgr_late_init_internal(mcmgr_core_t coreNum)
{
#if defined(FSL_FEATURE_MU_SIDE_A)
    MU_EnableInterrupts(MUA, mcmgr_mu_channel_flag);

#if (defined(FSL_FEATURE_MU_HAS_RESET_INT) && FSL_FEATURE_MU_HAS_RESET_INT)
    MU_EnableInterrupts(MUA, kMU_ResetAssertInterruptEnable);
#endif

#if defined(__riscv)
    EVENT_SetIRQPriority(MUA_IRQn, 4);
#else
    NVIC_SetPriority(MUA_IRQn, 2);
#endif

#if defined(__riscv)
    EnableIRQ(MUA_IRQn);
#else
    NVIC_EnableIRQ(MUA_IRQn);
#endif

#elif defined(FSL_FEATURE_MU_SIDE_B)
    MU_EnableInterrupts(MUB, mcmgr_mu_channel_flag);

#if (defined(FSL_FEATURE_MU_HAS_RESET_INT) && FSL_FEATURE_MU_HAS_RESET_INT)
    MU_EnableInterrupts(MUB, kMU_ResetAssertInterruptEnable);
#endif

#if defined(__riscv)
    EVENT_SetIRQPriority(MUB_IRQn, 4);
#else
    NVIC_SetPriority(MUB_IRQn, 2);
#endif

#if defined(__riscv)
    EnableIRQ(MUB_IRQn);
#else
    NVIC_EnableIRQ(MUB_IRQn);
#endif

#endif

    return kStatus_MCMGR_Success;
}

mcmgr_status_t mcmgr_start_core_internal(mcmgr_core_t coreNum, void *bootAddress)
{
    if (coreNum != kMCMGR_Core1)
    {
        return kStatus_MCMGR_Error;
    }
#if defined(FSL_FEATURE_MU_SIDE_A)
    if (MU_GetStatusFlags(MUA) & MU_SR_RDIP_MASK)
    {
        /* cannot start already started core... */
        return kStatus_MCMGR_Error;
    }

    if (bootAddress == (void *)MCMGR_BOOT_ADDRESS_RAM)
    {
        MU_BootCoreB(MUA, (mu_core_boot_mode_t)2);
    }
    else if (bootAddress == (void *)MCMGR_BOOT_ADDRESS_FLASH)
    {
        MU_BootCoreB(MUA, (mu_core_boot_mode_t)0);
    }
    else
    {
        /* invalid boot address */
        return kStatus_MCMGR_Error;
    }
#endif
    return kStatus_MCMGR_Success;
}

mcmgr_status_t mcmgr_get_startup_data_internal(mcmgr_core_t coreNum, uint32_t *startupData)
{
    if (coreNum != kMCMGR_Core1)
    {
        return kStatus_MCMGR_Error;
    }
    if (!startupData)
    {
        return kStatus_MCMGR_Error;
    }

    if (s_mcmgrCoresContext[coreNum].state >= kMCMGR_RunningCoreState)
    {
        *startupData = s_mcmgrCoresContext[coreNum].startupData;
        return kStatus_MCMGR_Success;
    }
    else
    {
        return kStatus_MCMGR_NotReady;
    }
}

mcmgr_status_t mcmgr_stop_core_internal(mcmgr_core_t coreNum)
{
    if (coreNum != kMCMGR_Core1)
    {
        return kStatus_MCMGR_Error;
    }
#if defined(FSL_FEATURE_MU_SIDE_A)
    if (!(MU_GetStatusFlags(MUA) & MU_SR_RDIP_MASK))
    {
        /* cannot stop already stopped core... */
        return kStatus_MCMGR_Error;
    }

    MU_HardwareResetOtherCore(MUA, false, true, kMU_CoreBootFromDflashBase);
#endif
    return kStatus_MCMGR_Success;
}

mcmgr_status_t mcmgr_get_core_property_internal(mcmgr_core_t coreNum,
                                                mcmgr_core_property_t property,
                                                void *value,
                                                uint32_t *length)
{
    if (value == NULL)
    {
        /* If value is null set only length */
        switch (property)
        {
            case kMCMGR_CoreStatus:
                *length = sizeof(mcmgr_core_status_t);
                break;

            case kMCMGR_CorePowerMode:
                *length = sizeof(mu_power_mode_t);
                break;

            case kMCMGR_CoreType:
                *length = sizeof(mcmgr_core_type_t);
                break;

            default:
                return kStatus_MCMGR_Error;
        }
    }
    else
    {
        switch (property)
        {
            case kMCMGR_CoreStatus:
                if (*length < sizeof(mcmgr_core_status_t))
                {
                    return kStatus_MCMGR_Error;
                }
                if (coreNum == kMCMGR_Core0)
                {
                    *((mcmgr_core_status_t *)value) = kMCMGR_NotInReset;
                }
                else if (coreNum == kMCMGR_Core1)
                {
/* Read BRS value from MU_SR, 0-processor is not in reset, 1-processor is in reset */
#if defined(FSL_FEATURE_MU_SIDE_A)
                    uint32_t reg = MUA->CR;
                    if (reg & (1 << 7))
                    {
                        *((mcmgr_core_status_t *)value) = kMCMGR_InReset;
                    }
                    else
                    {
                        *((mcmgr_core_status_t *)value) = kMCMGR_NotInReset;
                    }
#endif
                }
                break;

            case kMCMGR_CorePowerMode:
                if (*length < sizeof(mu_power_mode_t))
                {
                    return kStatus_MCMGR_Error;
                }
/* Read BPM value from MU_SR - power mode */
#if defined(FSL_FEATURE_MU_SIDE_A)
                *((mu_power_mode_t *)value) = (mu_power_mode_t)MU_GetOtherCorePowerMode(MUA);
#endif
                break;

            case kMCMGR_CoreType:
                if (*length < sizeof(mcmgr_core_type_t))
                {
                    return kStatus_MCMGR_Error;
                }
                *((mcmgr_core_type_t *)value) = g_mcmgrSystem.cores[coreNum].coreType;
                break;

            default:
                return kStatus_MCMGR_Error;
        }
    }

    return kStatus_MCMGR_Success;
}

mcmgr_core_t mcmgr_get_current_core_internal(void)
{
    /* MSCM peripheral clock needs to be enabled before */
    CLOCK_EnableClock(kCLOCK_Mscm);
    return (mcmgr_core_t)MSCM->CPXNUM;
}

mcmgr_status_t mcmgr_trigger_event_internal(uint32_t remoteData, bool forcedWrite)
{
    /* When forcedWrite is false, execute the blocking call, i.e. wait until previously
       sent data is processed. Otherwise, run the non-blocking version of the MU send function. */
    if(false == forcedWrite)
    {
        /* This is a blocking call */
#if defined(FSL_FEATURE_MU_SIDE_A)
        MU_SendMsg(MUA, MCMGR_MU_CHANNEL, remoteData);
#elif defined(FSL_FEATURE_MU_SIDE_B)
        MU_SendMsg(MUB, MCMGR_MU_CHANNEL, remoteData);
#endif
    } else {
        /* This is a non-blocking call */
#if defined(FSL_FEATURE_MU_SIDE_A)
        MU_SendMsgNonBlocking(MUA, MCMGR_MU_CHANNEL, remoteData);
#elif defined(FSL_FEATURE_MU_SIDE_B)
        MU_SendMsgNonBlocking(MUB, MCMGR_MU_CHANNEL, remoteData);
#endif
    }
    return kStatus_MCMGR_Success;
}

/*!
 * @brief ISR handler
 *
 * This function is called when data from MU is received
 */
void mcmgr_mu_channel_handler(void)
{
    uint32_t data;
    uint16_t eventType;
    uint16_t eventData;

    /* Non-blocking version of the receive function needs to be called here to avoid
       deadlock in ISR. The RX register must contain the payload now because the RX flag/event 
       has been identified before reaching this point (mcmgr_mu_channel_handler function). */
#if defined(FSL_FEATURE_MU_SIDE_A)
    data = MU_ReceiveMsgNonBlocking(MUA, MCMGR_MU_CHANNEL);
#elif defined(FSL_FEATURE_MU_SIDE_B)
    data = MU_ReceiveMsgNonBlocking(MUB, MCMGR_MU_CHANNEL);
#endif

    eventType = data >> 16;
    eventData = data & 0xFFFF;

    if (eventType < kMCMGR_EventTableLength)
    {
        if (MCMGR_eventTable[eventType].callback != NULL)
        {
            MCMGR_eventTable[eventType].callback(eventData, MCMGR_eventTable[eventType].callbackData);
        }
    }
}

#if defined(MCMGR_HANDLE_EXCEPTIONS) && (MCMGR_HANDLE_EXCEPTIONS == 1)
#if defined(__riscv)
/* This general implementation for exception handler */
static void DefaultISR(void)
{
    uint32_t exceptionNumber;

    __ASM volatile("csrr %0, 0x342" : "=r"(exceptionNumber)); /* MCAUSE */

    exceptionNumber &= 0x1F;
    MCMGR_TriggerEvent(kMCMGR_RemoteExceptionEvent, (uint16_t)exceptionNumber);
    while (1)
        ; /* stop here */
}

void LSU_Handler(void)
{
    DefaultISR();
}

void Ecall_Handler(void)
{
    DefaultISR();
}

void IllegalInstruction_Handler(void)
{
    DefaultISR();
}

#else /* __riscv */
/* This overrides the weak DefaultISR implementation from startup file */
void DefaultISR(void)
{
    uint32_t exceptionNumber = __get_IPSR();
    MCMGR_TriggerEvent(kMCMGR_RemoteExceptionEvent, (uint16_t)exceptionNumber);
    while (1)
        ; /* stop here */
}

void HardFault_Handler(void)
{
    DefaultISR();
}

void NMI_Handler(void)
{
    DefaultISR();
}

#if defined(__CM4_CMSIS_VERSION)
/* Cortex-M4 contains additional exception handlers */
void MemManage_Handler(void)
{
    DefaultISR();
}

void BusFault_Handler(void)
{
    DefaultISR();
}

void UsageFault_Handler(void)
{
    DefaultISR();
}
#endif

#endif /* __riscv */

#endif
