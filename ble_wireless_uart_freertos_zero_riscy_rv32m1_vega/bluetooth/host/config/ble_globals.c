/*! *********************************************************************************
* \addtogroup BLE
* @{
********************************************************************************** */
/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
* 
* 
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* DO NOT MODIFY THIS FILE!
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "ble_general.h"
#include "att_errors.h"
#include "ble_config.h"
#include "gap_types.h"

#include "ModuleInfo.h"


/************************************************************************************
*************************************************************************************
* Private constants & macros
*************************************************************************************
************************************************************************************/
/* WARNING: Do not change these defines */
#define gAttConnStorageSize_c            6
#define gActiveDevicesStorageSize_c      552
#define gProcedureDataStorageSize_c      288

/************************************************************************************
*************************************************************************************
* Public memory declarations - external references from Host library
*************************************************************************************
************************************************************************************/
uint8_t gcGapMaximumBondedDevices_d = gMaxBondedDevices_c;
uint8_t gGapControllerResolvingListSize = gMaxResolvingListSize_c;
bleBondIdentityHeaderBlob_t gaBondIdentityHeaderBlobs[gMaxBondedDevices_c];

gapIdentityInformation_t mControllerPrivacyIdentities[gMaxResolvingListSize_c];
gapCarSupport_t mCAR_Support[gMaxResolvingListSize_c];
uint8_t gcGattMaxHandleCountForWriteNotifications_c = gMaxWriteNotificationHandles_c;
uint16_t gGattWriteNotificationHandles[gMaxWriteNotificationHandles_c];
uint8_t gcGattMaxHandleCountForReadNotifications_c = gMaxReadNotificationHandles_c;
uint16_t gGattReadNotificationHandles[gMaxReadNotificationHandles_c];

uint8_t gcGattDbMaxPrepareWriteOperationsInQueue_c = gPrepareWriteQueueSize_c;
attPrepareWriteRequestParams_t gPrepareWriteQueues[gcGattDbMaxPrepareWriteClients_c][gPrepareWriteQueueSize_c];

uint16_t gGapDefaultTxOctets = gBleDefaultTxOctets_c;
uint16_t gGapDefaultTxTime = gBleDefaultTxTime_c;

uint16_t gGapHostPrivacyTimeout = gBleHostPrivacyTimeout_c;
uint16_t gGapControllerPrivacyTimeout = gBleControllerPrivacyTimeout_c;

bool_t gGapLeSecureConnectionsOnlyMode = gBleLeSecureConnectionsOnlyMode_c;
bool_t gGapLeScOobHasMitmProtection = gBleLeScOobHasMitmProtection_c;

/* The following definitions are required by the VERSION_TAGS. DO NOT MODIFY or REMOVE */
extern const moduleInfo_t BLE_HOST_VERSION;
#if defined ( __IAR_SYSTEMS_ICC__ )
#pragma required=BLE_HOST_VERSION /* force the linker to keep the symbol in the current compilation unit */
uint8_t ble_dummy; /* symbol suppressed by the linker as it is unused in the compilation unit, but necessary because 
                             to avoid warnings related to #pragma required */
#elif defined(__GNUC__)
static const moduleInfo_t *const dummy __attribute__((__used__)) = &BLE_HOST_VERSION;
#endif /* __IAR_SYSTEMS_ICC__ */

/* BLE Host connection storage */
const uint8_t gBleMaxActiveConnections = gAppMaxConnections_c;
uint32_t gAttConnStorage[(gAttConnStorageSize_c * gAppMaxConnections_c + 3) /4];
uint32_t gActiveDevicesStorage[(gActiveDevicesStorageSize_c * gAppMaxConnections_c + 3) /4];
uint32_t gProcedureDataStorage[(gProcedureDataStorageSize_c * gAppMaxConnections_c + 3) /4];
/* Private structure sizes used for run-time checks */
extern const uint16_t gAttConnEntrySize;
extern const uint16_t gActiveDevicesEntrySize;
#if gGattClientSupported_d
extern const uint16_t gProcedureDataEntrySize;
#endif

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
bool_t Ble_CheckMemoryStorage(void)
{
    bool_t status = TRUE;
    
    if (gAttConnEntrySize * gAppMaxConnections_c > sizeof(gAttConnStorage))
    {
        status = FALSE;
    }
    
    if (gActiveDevicesEntrySize * gAppMaxConnections_c > sizeof(gActiveDevicesStorage))
    {
        status = FALSE;
    }
#if gGattClientSupported_d
    if (gProcedureDataEntrySize * gAppMaxConnections_c > sizeof(gProcedureDataStorage))
    {
        status = FALSE;
    }
#endif
    
    return status;
}

/*! *********************************************************************************
* @}
********************************************************************************** */
