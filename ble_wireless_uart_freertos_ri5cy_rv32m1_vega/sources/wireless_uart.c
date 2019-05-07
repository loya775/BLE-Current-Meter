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
#include "RNG_Interface.h"
#include "Keyboard.h"
#include "LED.h"
#include "TimersManager.h"
#include "FunctionLib.h"
#include "Panic.h"
#include "SerialManager.h"
#include "MemManager.h"
#include "board.h"

/* BLE Host Stack */
#include "gatt_interface.h"
#include "gatt_server_interface.h"
#include "gatt_client_interface.h"
#include "gatt_database.h"
#include "gap_interface.h"
#include "gatt_db_app_interface.h"

#if !MULTICORE_HOST
#include "gatt_db_handles.h"
#else
#define UUID128(name, ...)\
    extern uint8_t name[16];
#include "gatt_uuid128.h"
#undef UUID128
#endif

/* Profile / Services */
#include "wireless_uart_interface.h"
#include "battery_interface.h"
/* Wrappers */
#include "ble_conn_manager.h"
#include "ble_service_discovery.h"

#include "board.h"
#include "ApplMain.h"
#include "wireless_uart.h"

#if MULTICORE_HOST
#include "erpc_host.h"
#include "dynamic_gatt_database.h"
#endif

#include "fsl_lptmr.h"
#include "fsl_lpadc.h"
#include "fsl_intmux.h"
#include "fsl_rtc.h"

#include <math.h>

#include "ili9341.h"
#include "currentSensor.h"
#include "memflash.h"
/************************************************************************************
 *************************************************************************************
 * Private macros
 *************************************************************************************
 ************************************************************************************/

#define mAppUartBufferSize_c   			gAttMaxWriteDataSize_d(gAttMaxMtu_c) /* Local Buffer Size */

#define mAppUartFlushIntervalInMs_c   	(7) 	/* Flush Timeout in Ms */

#define mBatteryLevelReportInterval_c   (10)	/* battery level report interval in seconds  */

#define DEMO_LPADC_IRQ_HANDLER_FUNC ADC0_IRQHandler
#define DEMO_LPADC_BASE ADC0

#define DEMO_LPTMR_BASE LPTMR0
#define DEMO_LPTMR_IRQn LPTMR0_IRQn
#define LPTMR_ADC_HANDLER LPTMR0_IRQHandler
//#define LPTMR0_IRQn LPTMR0_LPTMR1_IRQn
/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
#define LPTMR_USEC_COUNT (1000000U / 100U)

#define adc_n_samples 12u
/************************************************************************************
 *************************************************************************************
 * Private type definitions
 *************************************************************************************
 ************************************************************************************/
typedef enum appEvent_tag{
    mAppEvt_PeerConnected_c,
    mAppEvt_PairingComplete_c,
    mAppEvt_ServiceDiscoveryComplete_c,
    mAppEvt_ServiceDiscoveryNotFound_c,
    mAppEvt_ServiceDiscoveryFailed_c,
    mAppEvt_GattProcComplete_c,
    mAppEvt_GattProcError_c
}appEvent_t;

typedef enum appState_tag
{
    mAppIdle_c, 
    mAppExchangeMtu_c,	
    mAppServiceDisc_c,
    mAppServiceDiscRetry_c,
    mAppRunning_c
} appState_t;

typedef enum menuState_tag
{
    menuAppIdle_c,
    menuAppPrint_c,
	consumo,
	anomalias,
	configuracion,
	modFecha,
	SetAlarm,
	RegPag
} menuState_t;

typedef struct appPeerInfo_tag
{
    deviceId_t  deviceId;
    bool_t      isBonded;
    wucConfig_t clientInfo;
    appState_t  appState;
    gapRole_t   gapRole;
} appPeerInfo_t;

typedef struct advState_tag
{
    bool_t advOn;
} advState_t;
/************************************************************************************
 *************************************************************************************
 * Private memory declarations
 *************************************************************************************
 ************************************************************************************/

static appPeerInfo_t maPeerInformation[gAppMaxConnections_c];
static gapRole_t     mGapRole;

/* Adv Parmeters */
static advState_t mAdvState;
static bool_t   mScanningOn = FALSE;

static uint16_t mCharMonitoredHandles[1] = { value_uart_stream };

/* Service Data*/
static wusConfig_t mWuServiceConfig;
static bool_t      mBasValidClientList[gAppMaxConnections_c] = {FALSE};
static basConfig_t mBasServiceConfig = {service_battery, 0, mBasValidClientList, gAppMaxConnections_c};

static tmrTimerID_t mAppTimerId = gTmrInvalidTimerID_c;
static tmrTimerID_t mUartStreamFlushTimerId = gTmrInvalidTimerID_c;
static tmrTimerID_t mBatteryMeasurementTimerId = gTmrInvalidTimerID_c;

static uint8_t gAppSerMgrIf;
static uint16_t mAppUartBufferSize = mAppUartBufferSize_c;
static volatile bool_t mAppUartNewLine = FALSE;

uint8_t sizeOfMenu;

static menuState_t currentMenuState = menuAppPrint_c;

static volatile bool g_LpadcConversionCompletedFlag = false;
static volatile uint32_t g_LpadcInterruptCounter = 0U;
static lpadc_conv_result_t g_LpadcResultConfigStruct;

static uint16_t adc_values[adc_n_samples];
static uint8_t adc_index = 0;

static volatile bool busyWait;
static uint16_t sumCurrent=0;
static float currentTotal;
static uint8_t count;
static uint8_t Flag = 1;
static uint8_t resultado[17];
static uint8_t enableStateMachine=1;
static uint8_t u1;
static uint8_t u2;
static uint8_t d1;
static uint8_t d2;
static uint8_t enable;
static uint8_t corriente[6];

uint8_t const appMenu[]= "\r\n[0] Consumo \r\n "
						 "[1] Configuracion\r\n";
uint8_t const consumos[]= "\r\nUltima Fecha de Pago:";
uint8_t const consumos1[]= "\r\nConsumo Actual: ";
uint8_t const consumos2[]= "\r\nProxima Fecha de Pago: ";
uint8_t const consumos3[]= "\r\nPotencia:";
uint8_t const consumos4[]= "\r\n[0] Recargar";
uint8_t const consumos5[]= "\r\n[1] Configuraciones\r\n";

uint8_t const config[]= "\r\n[1] Modificar Fecha";
uint8_t const config2[]= "\r\n[2] Pago Realizado\r\n ";

uint8_t const RegPago[]= "Ingresa Fecha en que se realizo el pago\r\n ";
uint8_t const RegPago1[]= "Salir\r\n";


uint8_t const modFec[]= "Ingresa Nueva Fecha\r\n ";
uint8_t const modFec1[]= "Salir\r\n";

static rtc_datetime_t date;
/************************************************************************************
 *************************************************************************************
 * Private functions prototypes
 *************************************************************************************
 ************************************************************************************/

/* Gatt and Att callbacks */
static void BleApp_AdvertisingCallback(gapAdvertisingEvent_t* pAdvertisingEvent);
static void BleApp_ScanningCallback (gapScanningEvent_t* pScanningEvent);
static void BleApp_ConnectionCallback
(
    deviceId_t peerDeviceId,
    gapConnectionEvent_t* pConnectionEvent
);
static void BleApp_GattServerCallback 
(
   deviceId_t deviceId,
   gattServerEvent_t* pServerEvent
);

static void BleApp_GattClientCallback 
(
    deviceId_t              serverDeviceId,
    gattProcedureType_t     procedureType,
    gattProcedureResult_t   procedureResult,
    bleResult_t             error
);

static void BleApp_ServiceDiscoveryCallback
(
    deviceId_t deviceId,
    servDiscEvent_t* pEvent
);

static void BleApp_Config (void);
static void BleApp_Advertise (void);

void BleApp_StateMachineHandler
(
    deviceId_t peerDeviceId,
    uint8_t event
);

void Menu_StateMachineHandler();

static void BleApp_StoreServiceHandles
(
    deviceId_t 	     peerDeviceId,
    gattService_t   *pService
);
static bool_t BleApp_CheckScanEvent (gapScannedDevice_t* pData);

/* Timer Callbacks */
static void ScaningTimerCallback (void *);
static void UartStreamFlushTimerCallback(void *);
static void BatteryMeasurementTimerCallback (void *);

/* Uart Tx/Rx Callbacks*/
static void Uart_RxCallBack(void *pData);
static void Uart_TxCallBack(void *pBuffer);

static void BleApp_FlushUartStream(void *pParam);
static void BleApp_ReceivedUartStream(deviceId_t peerDeviceId, uint8_t *pStream, uint16_t streamLength);
static void BleApp_SendUartStream(uint8_t *pRecvStream, uint8_t streamSize);
void RTC_IRQHandler(void);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief    Initializes application specific functionality before the BLE stack init.
*
********************************************************************************** */
void BleApp_Init(void)
{
	LCD_ILI9341_init();

    /* Initialize application support for drivers */
    BOARD_InitAdc();
    LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* 1U is trigger0 mask. */

    /* UI */
    SerialManager_Init();

    /* Register Serial Manager interface */
    Serial_InitInterface(&gAppSerMgrIf, APP_SERIAL_INTERFACE_TYPE, APP_SERIAL_INTERFACE_INSTANCE);

    Serial_SetBaudRate(gAppSerMgrIf, gUARTBaudRate115200_c);

    /* Install Controller Events Callback handler */
    Serial_SetRxCallBack(gAppSerMgrIf, Uart_RxCallBack, NULL);
    
    /*Timer initialization*/
    lptmr_config_t lptmrConfig;
    INTMUX_Init(INTMUX0);
    INTMUX_EnableInterrupt(INTMUX0, 0, DEMO_LPTMR_IRQn);
    LPTMR_GetDefaultConfig(&lptmrConfig);

    /* Initialize the LPTMR */
    LPTMR_Init(DEMO_LPTMR_BASE, &lptmrConfig);

	/*
	 * Set timer period.
	 * Note : the parameter "ticks" of LPTMR_SetTimerPeriod should be equal or greater than 1.
	*/
	LPTMR_SetTimerPeriod(DEMO_LPTMR_BASE, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

	/* Enable timer interrupt */
	LPTMR_EnableInterrupts(DEMO_LPTMR_BASE, kLPTMR_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(DEMO_LPTMR_IRQn);

	/* Start counting */
	LPTMR_StartTimer(DEMO_LPTMR_BASE);

#if MULTICORE_HOST
    /* Init eRPC host */
    init_erpc_host();
#endif    
}

/*! *********************************************************************************
 * \brief    Starts the BLE application.
 *
 * \param[in]    mGapRole    GAP Start Role (Central or Peripheral).
 ********************************************************************************** */
void BleApp_Start (gapRole_t mGapRole)
{
    switch (mGapRole)
    {
        case gGapCentral_c:
        {
            Serial_Print(gAppSerMgrIf, "\n\rScanning...\n\r", gAllowToBlock_d);
            mAppUartNewLine = TRUE;
            gPairingParameters.localIoCapabilities = gIoKeyboardDisplay_c;
            App_StartScanning(&gScanParams, BleApp_ScanningCallback, TRUE);
            break;
        }
        case gGapPeripheral_c:
        {
            Serial_Print(gAppSerMgrIf, "\n\rAdvertising...\n\r", gAllowToBlock_d);
            mAppUartNewLine = TRUE;
            gPairingParameters.localIoCapabilities = gIoDisplayOnly_c;
            BleApp_Advertise();
            break;
        }
        default:
            break;
    }
}

/*! *********************************************************************************
* \brief        Handles keyboard events.
*
* \param[in]    events    Key event structure.
********************************************************************************** */
void BleApp_HandleKeys(key_event_t events)
{
    uint8_t mPeerId = 0;
    switch (events)
    {
        case gKBD_EventPressPB1_c:
        {
            LED_StopFlashingAllLeds();
            Led1Flashing();

            BleApp_Start(mGapRole);
            break;
        }
        case gKBD_EventLongPB1_c:
        {
            for(mPeerId = 0; mPeerId < gAppMaxConnections_c; mPeerId++)
            {
                if (maPeerInformation[mPeerId].deviceId != gInvalidDeviceId_c)
                    Gap_Disconnect(maPeerInformation[mPeerId].deviceId);
            }
            break;
        }
        case gKBD_EventPressPB2_c:
        {
            /* Switch current role */
            if( mGapRole == gGapCentral_c )
            {
                Serial_Print(gAppSerMgrIf, "\n\rSwitched role to GAP Peripheral.\n\r", gAllowToBlock_d);
                mAppUartNewLine = TRUE;
                mGapRole = gGapPeripheral_c;
            }
            else
            {
                Serial_Print(gAppSerMgrIf, "\n\rSwitched role to GAP Central.\n\r", gAllowToBlock_d);
                mAppUartNewLine = TRUE;
                mGapRole = gGapCentral_c;
            }
            break;
        }
        case gKBD_EventLongPB2_c:
            break;
    default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE generic callback.
*
* \param[in]    pGenericEvent    Pointer to gapGenericEvent_t.
********************************************************************************** */
void BleApp_GenericCallback (gapGenericEvent_t* pGenericEvent)
{
    /* Call BLE Conn Manager */
    BleConnManager_GenericEvent(pGenericEvent);
    
    switch (pGenericEvent->eventType)
    {
        case gInitializationComplete_c:    
        {
            BleApp_Config();
            BleApp_Start(mGapRole);
        }
        break;    
        
        case gAdvertisingParametersSetupComplete_c:
        {
            App_StartAdvertising(BleApp_AdvertisingCallback, BleApp_ConnectionCallback);
        }
        break;    
        
        default: 
            break;
    }
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief        Configures BLE Stack after initialization. Usually used for
*               configuring advertising, scanning, white list, services, et al.
*
********************************************************************************** */
static void BleApp_Config()
{
    uint8_t mPeerId = 0;
    
#if MULTICORE_HOST
    if (GattDbDynamic_CreateDatabase() != gBleSuccess_c)
    {
        panic(0,0,0,0);
        return;
    }
#endif /* MULTICORE_HOST */

    /* Configure as GAP Dual Role */
    BleConnManager_GapDualRoleConfig();
        
    /* Register for callbacks */
    App_RegisterGattServerCallback(BleApp_GattServerCallback);
    App_RegisterGattClientProcedureCallback(BleApp_GattClientCallback);
    GattServer_RegisterHandlesForWriteNotifications(NumberOfElements(mCharMonitoredHandles), mCharMonitoredHandles);
    BleServDisc_RegisterCallback(BleApp_ServiceDiscoveryCallback);
	
    for(mPeerId = 0; mPeerId < gAppMaxConnections_c; mPeerId++)
    {
        maPeerInformation[mPeerId].appState = mAppIdle_c;
        maPeerInformation[mPeerId].deviceId = gInvalidDeviceId_c;
        maPeerInformation[mPeerId].clientInfo.hService = gGattDbInvalidHandleIndex_d;
        maPeerInformation[mPeerId].clientInfo.hUartStream = gGattDbInvalidHandleIndex_d;
    }
    
    /* By default, always start node as GAP central */
    mGapRole = gGapPeripheral_c;
    
    Serial_Print(gAppSerMgrIf, "\n\rWireless UART starting as GAP Central, press the role switch to change it.\n\r", gAllowToBlock_d);
    
    mAdvState.advOn = FALSE;
    mScanningOn = FALSE;
    
    /* Start services */
    Wus_Start(&mWuServiceConfig);

    mBasServiceConfig.batteryLevel = BOARD_GetBatteryLevel();
    Bas_Start(&mBasServiceConfig);

    /* Allocate application timer */
    mAppTimerId = TMR_AllocateTimer();
    mUartStreamFlushTimerId = TMR_AllocateTimer();
    mBatteryMeasurementTimerId = TMR_AllocateTimer();
}

/*! *********************************************************************************
* \brief        Configures GAP Advertise parameters. Advertise will satrt after
*               the parameters are set.
*
********************************************************************************** */
static void BleApp_Advertise(void)
{
    /* Set advertising parameters*/
    Gap_SetAdvertisingParameters(&gAdvParams);
}

/*! *********************************************************************************
 * \brief        Handles BLE Scanning callback from host stack.
 *
 * \param[in]    pScanningEvent    Pointer to gapScanningEvent_t.
 ********************************************************************************** */
static void BleApp_ScanningCallback (gapScanningEvent_t* pScanningEvent)
{
    switch (pScanningEvent->eventType)
    {
        case gDeviceScanned_c:
        {
            if (BleApp_CheckScanEvent(&pScanningEvent->eventData.scannedDevice))
            {        
                gConnReqParams.peerAddressType = pScanningEvent->eventData.scannedDevice.addressType;
                FLib_MemCpy(gConnReqParams.peerAddress, 
                            pScanningEvent->eventData.scannedDevice.aAddress,
                            sizeof(bleDeviceAddress_t));
                
                Gap_StopScanning();
#if gAppUsePrivacy_d
                gConnReqParams.usePeerIdentityAddress = pScanningEvent->eventData.scannedDevice.advertisingAddressResolved;
#endif
                App_Connect(&gConnReqParams, BleApp_ConnectionCallback);
            }
        }        
        break;
        
        case gScanStateChanged_c:
        {
            mScanningOn = !mScanningOn;
            
            /* Node starts scanning */
            if (mScanningOn)
            { 
                /* Start advertising timer */
                TMR_StartLowPowerTimer(mAppTimerId, 
                           gTmrLowPowerSecondTimer_c,
                           TmrSeconds(gScanningTime_c),
                           ScaningTimerCallback, NULL);  

                Led1Flashing();
            }
            /* Node is not scanning */
            else
            {                
                TMR_StopTimer(mAppTimerId);
                
                Led1Flashing();
                Led2Flashing();
                Led3Flashing();
                Led4Flashing();
            }
        }
        break;
    case gScanCommandFailed_c:
    {
        panic(0, 0, 0, 0);
        break;
    }
    default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE Advertising callback from host stack.
*
* \param[in]    pAdvertisingEvent    Pointer to gapAdvertisingEvent_t.
********************************************************************************** */
static void BleApp_AdvertisingCallback (gapAdvertisingEvent_t* pAdvertisingEvent)
{
    switch (pAdvertisingEvent->eventType)
    {
        case gAdvertisingStateChanged_c:
        {
            mAdvState.advOn = !mAdvState.advOn;
            LED_StopFlashingAllLeds();
            Led1Flashing();

            if(!mAdvState.advOn)
            {
                Led2Flashing();
                Led3Flashing();
                Led4Flashing();
            }
        }
        break;

        case gAdvertisingCommandFailed_c:
        {
            panic(0,0,0,0);
        }
        break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE Connection callback from host stack.
*
* \param[in]    peerDeviceId        Peer device ID.
* \param[in]    pConnectionEvent    Pointer to gapConnectionEvent_t.
********************************************************************************** */
static void BleApp_ConnectionCallback (deviceId_t peerDeviceId, gapConnectionEvent_t* pConnectionEvent)
{
    switch (pConnectionEvent->eventType)
    {
        case gConnEvtConnected_c:
        {
            if(peerDeviceId < gAppMaxConnections_c)
            {
                maPeerInformation[peerDeviceId].deviceId = peerDeviceId;
            }
            else
            {
                Gap_Disconnect(peerDeviceId);
                break;
            }

            /* Advertising stops when connected */
            mAdvState.advOn = FALSE;

            /* Subscribe client*/
            Wus_Subscribe(peerDeviceId);
            Bas_Subscribe(&mBasServiceConfig, peerDeviceId);

            /* UI */
            LED_StopFlashingAllLeds();
            Led1On();

            /* Stop Advertising Timer*/
            mAdvState.advOn = FALSE;
            TMR_StopTimer(mAppTimerId);
            
            if(!TMR_IsTimerActive(mBatteryMeasurementTimerId))
            {
                /* Start battery measurements */
                TMR_StartLowPowerTimer(mBatteryMeasurementTimerId, gTmrLowPowerIntervalMillisTimer_c,
                                       TmrSeconds(mBatteryLevelReportInterval_c), BatteryMeasurementTimerCallback, NULL);
            }

#if gAppUsePairing_d
#if gAppUseBonding_d            
            Gap_CheckIfBonded(peerDeviceId, &maPeerInformation[peerDeviceId].isBonded);
            
            if ((maPeerInformation[peerDeviceId].isBonded) &&
                (gBleSuccess_c == Gap_LoadCustomPeerInformation(peerDeviceId,
                    (void*) &maPeerInformation[peerDeviceId].clientInfo, 0, sizeof (wucConfig_t))))
            {
                /* Restored custom connection information. Encrypt link */
                Gap_EncryptLink(peerDeviceId);
            }
#endif /* gAppUseBonding_d*/ 
#endif /* gAppUsePairing_d */

            Serial_Print(gAppSerMgrIf, "Connected to device ", gAllowToBlock_d);
            Serial_PrintDec(gAppSerMgrIf, peerDeviceId);
            if( mGapRole == gGapCentral_c )
            {
                Serial_Print(gAppSerMgrIf, " as master.\n\r", gAllowToBlock_d);
            }
            else
            {
                Serial_Print(gAppSerMgrIf, " as slave.\n\r", gAllowToBlock_d);
            }
            mAppUartNewLine = TRUE;
            
            maPeerInformation[peerDeviceId].gapRole = mGapRole;

            /* run the state machine */
            BleApp_StateMachineHandler(peerDeviceId, mAppEvt_PeerConnected_c);
        }
        break;

        case gConnEvtDisconnected_c:
        {
            Serial_Print(gAppSerMgrIf, "Disconnected from device ", gAllowToBlock_d);
            Serial_PrintDec(gAppSerMgrIf, peerDeviceId);
            Serial_Print(gAppSerMgrIf, ".\n\r", gAllowToBlock_d);
            
            maPeerInformation[peerDeviceId].appState = mAppIdle_c;
            maPeerInformation[peerDeviceId].clientInfo.hService = gGattDbInvalidHandleIndex_d;
            maPeerInformation[peerDeviceId].clientInfo.hUartStream = gGattDbInvalidHandleIndex_d;
            
            /* Unsubscribe client */
            Wus_Unsubscribe();
            Bas_Unsubscribe(&mBasServiceConfig, peerDeviceId); 
            
            TMR_StopTimer(mBatteryMeasurementTimerId);

            /* Reset Service Discovery to be sure*/
            BleServDisc_Stop(peerDeviceId);

            /* UI */
            LED_TurnOffAllLeds();
            LED_StartFlash(LED_ALL);
 
            /* mark device id as invalid */
            maPeerInformation[peerDeviceId].deviceId = gInvalidDeviceId_c;
            
            /* recalculate minimum of maximum MTU's of all connected devices */
            mAppUartBufferSize                       = mAppUartBufferSize_c;
            
            for(uint8_t mPeerId = 0; mPeerId < gAppMaxConnections_c; mPeerId++)
            {
                if(gInvalidDeviceId_c != maPeerInformation[mPeerId].deviceId)
                {
                    uint16_t tempMtu;
                    
                    Gatt_GetMtu(mPeerId, &tempMtu);
                    tempMtu = gAttMaxWriteDataSize_d(tempMtu);
                    
                    if(tempMtu < mAppUartBufferSize)
                    {
                        mAppUartBufferSize = tempMtu;
                    }
                }
            }
            BleApp_Start(mGapRole);
        }
        break;

#if gAppUsePairing_d		
        case gConnEvtPairingComplete_c:
        {
            if (pConnectionEvent->eventData.pairingCompleteEvent.pairingSuccessful)
            {
                BleApp_StateMachineHandler(peerDeviceId, 
                                           mAppEvt_PairingComplete_c);
            }
        }
        break;
#endif /* gAppUsePairing_d */

    default:
        break;
    }
    
    /* Connection Manager to handle Host Stack interactions */
    if (maPeerInformation[peerDeviceId].gapRole == gGapCentral_c)
    {
        BleConnManager_GapCentralEvent(peerDeviceId, pConnectionEvent);
    }
    else if (maPeerInformation[peerDeviceId].gapRole == gGapPeripheral_c)
    {
        BleConnManager_GapPeripheralEvent(peerDeviceId, pConnectionEvent);
    }
}

static void BleApp_ServiceDiscoveryCallback(deviceId_t peerDeviceId, servDiscEvent_t* pEvent)
{
    switch(pEvent->eventType)
    {
        case gServiceDiscovered_c:
        {
            if (pEvent->eventData.pService->uuidType == gBleUuidType128_c)
            {
                if(FLib_MemCmp((void*)&uuid_service_wireless_uart, (void*)&pEvent->eventData.pService->uuid, sizeof(bleUuid_t)))
                { 
                    BleApp_StoreServiceHandles(peerDeviceId, pEvent->eventData.pService); 
                }
            }
        }
        break;

        case gDiscoveryFinished_c:
        {
            if (pEvent->eventData.success)
            {
                if (gGattDbInvalidHandleIndex_d != maPeerInformation[peerDeviceId].clientInfo.hService)
                {
                    BleApp_StateMachineHandler(peerDeviceId, 
                                               mAppEvt_ServiceDiscoveryComplete_c);
                }
                else
                {
                    BleApp_StateMachineHandler(peerDeviceId, 
                                               mAppEvt_ServiceDiscoveryNotFound_c);                
                }
            }
            else
            {
                BleApp_StateMachineHandler(peerDeviceId, 
                                           mAppEvt_ServiceDiscoveryFailed_c);
            }
        }
        break;

        default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles GATT client callback from host stack.
*
* \param[in]    serverDeviceId      GATT Server device ID.
* \param[in]    procedureType    	Procedure type.
* \param[in]    procedureResult    	Procedure result.
* \param[in]    error    			Callback result.
********************************************************************************** */
static void BleApp_GattClientCallback(
    deviceId_t              serverDeviceId,
    gattProcedureType_t     procedureType,
    gattProcedureResult_t   procedureResult,
    bleResult_t             error
)
{  
    if (procedureResult == gGattProcError_c)
    {    
        BleApp_StateMachineHandler(serverDeviceId, 
                                   mAppEvt_GattProcError_c);
    }
    else if (procedureResult == gGattProcSuccess_c)
    {        
    	BleApp_StateMachineHandler(serverDeviceId, 
                                   mAppEvt_GattProcComplete_c);
    }

    /* Signal Service Discovery Module */
    BleServDisc_SignalGattClientEvent(serverDeviceId, 
                                      procedureType,procedureResult, error);
}

/*! *********************************************************************************
 * \brief        Handles GATT server callback from host stack.
 *
 * \param[in]    deviceId        Client peer device ID.
 * \param[in]    pServerEvent    Pointer to gattServerEvent_t.
 ********************************************************************************** */
static void BleApp_GattServerCallback (
                                       deviceId_t deviceId,
                                       gattServerEvent_t* pServerEvent)
{
    uint16_t tempMtu = 0;
    
    switch (pServerEvent->eventType)
    {
        case gEvtAttributeWrittenWithoutResponse_c:
        {
            if (pServerEvent->eventData.attributeWrittenEvent.handle == value_uart_stream)
            {
            	BleApp_ReceivedUartStream(deviceId, pServerEvent->eventData.attributeWrittenEvent.aValue,
                            pServerEvent->eventData.attributeWrittenEvent.cValueLength);
            }
            break;
        }
        case gEvtMtuChanged_c:
        {
            /* update stream length with minimum of  new MTU */
            Gatt_GetMtu(deviceId, &tempMtu);
            tempMtu = gAttMaxWriteDataSize_d(tempMtu);
            
            mAppUartBufferSize = mAppUartBufferSize <= tempMtu? mAppUartBufferSize : tempMtu;
        }
        break;
    default:
        break;
    }
}


static bool_t MatchDataInAdvElementList (
                                         gapAdStructure_t *pElement,
                                         void *pData,
                                         uint8_t iDataLen)
{
    uint8_t i;

    for (i = 0; i < pElement->length; i += iDataLen)
    {
        if (FLib_MemCmp(pData, &pElement->aData[i], iDataLen))
        {
            return TRUE;
        }
    }
    return FALSE;
}

/*! *********************************************************************************
 * \brief        Checks Scan data for a device to connect.
 *
 * \param[in]    pData    Pointer to gapScannedDevice_t.
 ********************************************************************************** */
static bool_t BleApp_CheckScanEvent (gapScannedDevice_t* pData)
{
    uint8_t index = 0;
    bool_t foundMatch = FALSE;

    while (index < pData->dataLength)
    {
        gapAdStructure_t adElement;

        adElement.length = pData->data[index];
        adElement.adType = (gapAdType_t) pData->data[index + 1];
        adElement.aData = &pData->data[index + 2];

        /* Search for Wireless UART Service */
        if ((adElement.adType == gAdIncomplete128bitServiceList_c)
            || (adElement.adType == gAdComplete128bitServiceList_c))
        {
            foundMatch = MatchDataInAdvElementList(&adElement,
                &uuid_service_wireless_uart, 16);
        }

        /* Move on to the next AD element type */
        index += adElement.length + sizeof(uint8_t);
    }

    return foundMatch;
}

/*! *********************************************************************************
 * \brief        Stores handles used by the application.
 *
 * \param[in]    pService    Pointer to gattService_t.
 ********************************************************************************** */
static void BleApp_StoreServiceHandles (deviceId_t peerDeviceId, gattService_t *pService)
{
    /* Found Wireless UART Service */
    maPeerInformation[peerDeviceId].clientInfo.hService = pService->startHandle;

    if (pService->cNumCharacteristics > 0 &&
        pService->aCharacteristics != NULL)
    {
        /* Found Uart Characteristic */
        maPeerInformation[peerDeviceId].clientInfo.hUartStream =
            pService->aCharacteristics[0].value.handle;
    }
}

static void BleApp_SendUartStream(uint8_t *pRecvStream, uint8_t streamSize)
{
    gattCharacteristic_t characteristic = {gGattCharPropNone_c, {0}, 0, 0};
    uint8_t              mPeerId = 0;

    /* send UART stream to all peers */
    for(mPeerId = 0; mPeerId < gAppMaxConnections_c; mPeerId++)
    {
        if(gInvalidDeviceId_c != maPeerInformation[mPeerId].deviceId && 
           mAppRunning_c == maPeerInformation[mPeerId].appState)
        {
            characteristic.value.handle = maPeerInformation[mPeerId].clientInfo.hUartStream;
            GattClient_WriteCharacteristicValue(mPeerId, &characteristic,
                                                streamSize, pRecvStream, TRUE,
                                                FALSE, FALSE, NULL);
        }
    }
}

void BleApp_StateMachineHandler(deviceId_t peerDeviceId, uint8_t event)
{
	if(Flag == 1)
	{
		//TMR_ADC();
		InitRTC();
		MemFlashInit();
		Flag = 0;
	}

    uint16_t tempMtu = 0;
    
    /* invalid client information */
    if(gInvalidDeviceId_c == maPeerInformation[peerDeviceId].deviceId)
    {
        return;
    }
    
    switch (maPeerInformation[peerDeviceId].appState)
    {
        case mAppIdle_c:
        {
            if (event == mAppEvt_PeerConnected_c)
            {
                /* Let the central device initiate the Exchange MTU procedure*/
                if (mGapRole == gGapCentral_c)
                {
                    /* Moving to Exchange MTU State */
                    maPeerInformation[peerDeviceId].appState = mAppExchangeMtu_c;
                    GattClient_ExchangeMtu(peerDeviceId);
                }
                else
                {
                    /* Moving to Service Discovery State*/
                    maPeerInformation[peerDeviceId].appState = mAppServiceDisc_c;

                    /* Start Service Discovery*/
                    BleServDisc_FindService(peerDeviceId, 
                                            gBleUuidType128_c,
                                            (bleUuid_t*) &uuid_service_wireless_uart);                  
                }
            }
        }
        break;

        case mAppExchangeMtu_c:
        {
            if (event == mAppEvt_GattProcComplete_c)
            {
                /* update stream length with minimum of maximum MTU's of connected devices */
                Gatt_GetMtu(peerDeviceId, &tempMtu);
                tempMtu = gAttMaxWriteDataSize_d(tempMtu);
                
                mAppUartBufferSize = mAppUartBufferSize <= tempMtu? mAppUartBufferSize : tempMtu;
                
                /* Moving to Service Discovery State*/
                maPeerInformation[peerDeviceId].appState = mAppServiceDisc_c;

                /* Start Service Discovery*/
                BleServDisc_FindService(peerDeviceId, 
                                        gBleUuidType128_c,
                                        (bleUuid_t*) &uuid_service_wireless_uart);
            }
            else if (event == mAppEvt_GattProcError_c) 
            {
                Gap_Disconnect(peerDeviceId);
            }
        }
        break;

        case mAppServiceDisc_c:
        {
            if (event == mAppEvt_ServiceDiscoveryComplete_c)
            {            	
                /* Moving to Running State*/
                maPeerInformation[peerDeviceId].appState = mAppRunning_c;
                Menu_StateMachineHandler();

#if gAppUseBonding_d
                /* Write data in NVM */
                Gap_SaveCustomPeerInformation(maPeerInformation[peerDeviceId].deviceId,
                                              (void*) &maPeerInformation[peerDeviceId].clientInfo, 0,
                                              sizeof (wucConfig_t));
#endif
            }
            else if (event == mAppEvt_ServiceDiscoveryNotFound_c)
            {
                /* Moving to Service discovery Retry State*/
                maPeerInformation[peerDeviceId].appState = mAppServiceDiscRetry_c;              
                /* Retart Service Discovery for all services */
                BleServDisc_Start(peerDeviceId);              
            }
            else if (event == mAppEvt_ServiceDiscoveryFailed_c)
            {
                Gap_Disconnect(peerDeviceId);
                currentMenuState = menuAppPrint_c;
            }
        }
        break;
        
        case mAppServiceDiscRetry_c:
        {
            if (event == mAppEvt_ServiceDiscoveryComplete_c)
            {            	
                /* Moving to Running State*/
                maPeerInformation[peerDeviceId].appState = mAppRunning_c;
                Menu_StateMachineHandler();
            }
            else if( (event == mAppEvt_ServiceDiscoveryNotFound_c) ||
                     (event == mAppEvt_ServiceDiscoveryFailed_c) )
            {
                Gap_Disconnect(peerDeviceId);
                currentMenuState = menuAppPrint_c;
            }
        }
        break;        

        case mAppRunning_c:
        	Menu_StateMachineHandler();
			break;
        default:
                break;
    }
}

void Menu_StateMachineHandler()
{

	//float sum=00.00f;

	if (enableStateMachine==1)
	{
	switch(currentMenuState)
	{
	case menuAppPrint_c:
		currentMenuState = menuAppIdle_c;
		printTime();
		BleApp_SendUartStream(&resultado[0],SizeOfString(resultado)+1);
		BleApp_SendUartStream(&appMenu[0],SizeOfString(appMenu));
		enableStateMachine=0;
		break;
	case consumo:
		currentMenuState = consumo;
		printTime();
		BleApp_SendUartStream(&resultado[0],SizeOfString(resultado)+1);
		BleApp_SendUartStream(&consumos[0],SizeOfString(consumos));
		printMemFlash();
		BleApp_SendUartStream(&consumos1[0],SizeOfString(consumos1));
		enable=separar_2d_enteros_2d_decimales(currentTotal,&u1,&u2,&d1,&d2);
		corriente[0]=Converter_IntToChar(u1);
		corriente[1]= Converter_IntToChar(u2);
		corriente[2]=46;
		corriente[3]=Converter_IntToChar(d1);
		corriente[4]=Converter_IntToChar(d2);
		corriente[5] = 0;
		LCD_ILI9341_sendString(30,80,ILI9341_GREEN,&corriente[0]);

		BleApp_SendUartStream(&corriente[0],SizeOfString(corriente));
		BleApp_SendUartStream(&consumos2[0],SizeOfString(consumos2));
		printMemFlash();
		BleApp_SendUartStream(&consumos3[0],SizeOfString(consumos3));
		enable=separar_2d_enteros_2d_decimales(getWatts(currentTotal),&u1,&u2,&d1,&d2);
		corriente[0]=Converter_IntToChar(u1);
		corriente[1]= Converter_IntToChar(u2);
		corriente[2]=46;
		corriente[3]=Converter_IntToChar(d1);
		corriente[4]=Converter_IntToChar(d2);

		LCD_ILI9341_sendString(30,160,ILI9341_GREEN,&corriente[0]);

		BleApp_SendUartStream(&corriente[0],SizeOfString(corriente));
		BleApp_SendUartStream(&consumos4[0],SizeOfString(consumos4));
		BleApp_SendUartStream(&consumos5[0],SizeOfString(consumos5));
		enableStateMachine=0;
		break;
	case anomalias:
		currentMenuState = anomalias;
		printTime();

		enableStateMachine=0;
		break;
	case configuracion:
		currentMenuState = configuracion;
		printTime();
		BleApp_SendUartStream(&resultado[0],SizeOfString(resultado));
		BleApp_SendUartStream(&config[0],SizeOfString(config));
		BleApp_SendUartStream(&config2[0],SizeOfString(config2));
		enableStateMachine=0;
		break;
	case RegPag:
		currentMenuState = RegPag;
		BleApp_SendUartStream(&RegPago[0],SizeOfString(RegPago));
		BleApp_SendUartStream(&RegPago1[0],SizeOfString(RegPago1));
		enableStateMachine=0;
		break;
	case modFecha:
		currentMenuState = modFecha;
		BleApp_SendUartStream(&modFec[0],SizeOfString(modFec));
		BleApp_SendUartStream(&modFec1[0],SizeOfString(modFec1));
		enableStateMachine=0;
		break;
	case SetAlarm:
		currentMenuState = SetAlarm;
		enableStateMachine=0;
		break;
	case menuAppIdle_c:
		enableStateMachine=0;
		break;
	}
	}
}

/*! *********************************************************************************
 * \brief        Handles scanning timer callback.
 *
 * \param[in]    pParam        Calback parameters.
 ********************************************************************************** */
static void ScaningTimerCallback (void * pParam)
{
    /* Stop scanning */
    Gap_StopScanning();
}

static void BleApp_FlushUartStream(void *pParam)
{
    uint8_t *pMsg = NULL;
    uint16_t bytesRead = 0;
    uint8_t  mPeerId = 0;
    bool_t   mValidDevices = FALSE;
    
    /* Valid devices are in Running state */
    for(mPeerId = 0; mPeerId < gAppMaxConnections_c; mPeerId++)
    {
        if( gInvalidDeviceId_c != maPeerInformation[mPeerId].deviceId && 
           mAppRunning_c == maPeerInformation[mPeerId].appState)
        {
            mValidDevices = TRUE;
            break;
        }
    }
    
    if(!mValidDevices)
    {
        return;
    }

    /* Allocate buffer for GATT Write */
    pMsg = MEM_BufferAlloc(mAppUartBufferSize);
    
    if (pMsg == NULL)
    {
    	return;
    }

    /* Collect the data from the serial manager buffer */
    if ( Serial_Read( gAppSerMgrIf, pMsg, mAppUartBufferSize, &bytesRead) == gSerial_Success_c )
    {
        if (bytesRead != 0)
        {
            /* Send data over the air */
            BleApp_SendUartStream(pMsg, bytesRead);
        }
    }    
    /* Free Buffer */
    MEM_BufferFree(pMsg);
}

static void BleApp_ReceivedUartStream(deviceId_t peerDeviceId, uint8_t *pStream, uint16_t streamLength)
{
    static deviceId_t previousDeviceId = gInvalidDeviceId_c;
    
    uint8_t  additionalInfoBuff[10] = { '\r','\n','[', '0', '0', '-', 'M', ']', ':', ' '};
    uint8_t *pBuffer = NULL;
    uint8_t  messageHeaderSize = 0;
    uint8_t FlagRecent=0;
    enableStateMachine=1;

    if(gInvalidDeviceId_c == peerDeviceId)
    {
        return;
    }

    if (currentMenuState == modFecha && streamLength == 11)
    {
    	NuevaFecha(pStream);
    }

    if (currentMenuState == RegPag && streamLength == 11)
    {
        UltPag(pStream);
        currentMenuState = menuAppIdle_c;
        Menu_StateMachineHandler();
    }



	if(currentMenuState == menuAppIdle_c && streamLength == 2 && *pStream == '0')
	{
		printTime();
		currentMenuState = consumo;
		Menu_StateMachineHandler();
		FlagRecent=1;
	}else if(currentMenuState == menuAppIdle_c && streamLength == 2 && *pStream == '1')
	{
		currentMenuState = configuracion;
		Menu_StateMachineHandler();
		FlagRecent=1;
	}
/*Menu Configuración cambio de maquinas de estado*/
	if(currentMenuState == configuracion && *pStream == '1' && FlagRecent==0)
		{
			currentMenuState = modFecha;
			Menu_StateMachineHandler();
			FlagRecent=1;
		}else if(streamLength == 2 && *pStream == '2')
		{
			currentMenuState = RegPag;
			Menu_StateMachineHandler();
			FlagRecent=1;
		}
/*Fin menu Cambio Maquina de estados Configuración*/
	if(currentMenuState == consumo && streamLength == 2 && *pStream == '0')
	{
		currentMenuState = menuAppIdle_c;
		//Menu_StateMachineHandler();
	}
	if(currentMenuState == modFecha && streamLength == 2 && *pStream == '0')
	{
		currentMenuState = menuAppPrint_c;
		Menu_StateMachineHandler();
	}

	if (currentMenuState == RegPag && streamLength == 2 && *pStream == '0')
	    {
		currentMenuState = menuAppPrint_c;
		Menu_StateMachineHandler();
	    }

	FlagRecent = 0;
    if( (previousDeviceId != peerDeviceId) || mAppUartNewLine )
    {
        streamLength += sizeof(additionalInfoBuff);
    }
    
    /* Allocate buffer for asynchronous write */
    pBuffer = MEM_BufferAlloc(streamLength);

    if (pBuffer != NULL)
    {
        /* if this is a message from a previous device, print device ID */
        if( (previousDeviceId != peerDeviceId) || mAppUartNewLine )
        {
            messageHeaderSize = sizeof(additionalInfoBuff);
            
            if( mAppUartNewLine )
            {
                mAppUartNewLine = FALSE;
            }
            
            if(peerDeviceId > 9)
            {
                additionalInfoBuff[3] = '0' + (peerDeviceId / 10);
            }
            additionalInfoBuff[4] = '0' + (peerDeviceId % 10);
            
            if( gGapCentral_c != maPeerInformation[peerDeviceId].gapRole )
            {
                additionalInfoBuff[6] = 'S';
            }
            
            FLib_MemCpy(pBuffer, additionalInfoBuff, sizeof(additionalInfoBuff));
        }
        
        FLib_MemCpy(pBuffer + messageHeaderSize, pStream, streamLength - messageHeaderSize);
        Serial_AsyncWrite(gAppSerMgrIf, pBuffer, streamLength, Uart_TxCallBack, pBuffer);
    }

    /* update the previous device ID */
    previousDeviceId = peerDeviceId;

}
static void UartStreamFlushTimerCallback(void *pData)
{
    App_PostCallbackMessage(BleApp_FlushUartStream, NULL);
}

/*! *********************************************************************************
* \brief        Handles UART Receive callback.
*
* \param[in]    pData        Parameters.
********************************************************************************** */
static void Uart_RxCallBack(void *pData)
{
    static uint8_t bAppMsgPosted = 0;
    static uint16_t msgByteCount = 0;
    uint16_t byteCount;

    Serial_RxBufferByteCount(gAppSerMgrIf, &byteCount);
    
    if (byteCount < mAppUartBufferSize)
    {
        /* Clear App Msg if posted earlier */
        if( 1 == bAppMsgPosted )
        {
            bAppMsgPosted = 0;
        }
        
        /* Restart flush timer */
        TMR_StartLowPowerTimer(mUartStreamFlushTimerId,
                gTmrLowPowerSingleShotMillisTimer_c,
                mAppUartFlushIntervalInMs_c,
                UartStreamFlushTimerCallback, NULL);
    }
    else
    {
        /* Post App Msg only one at a time */
        if( 0 == bAppMsgPosted )
        {
            bAppMsgPosted = 1;
            msgByteCount = byteCount;
            App_PostCallbackMessage(BleApp_FlushUartStream, NULL);
        }
        else
        {
            /* Check if application has read bytes */
            if( byteCount < msgByteCount )
            {
                bAppMsgPosted = 0;
            }
        }
    }
}

/*! *********************************************************************************
* \brief        Handles UART Transmit callback.
*
* \param[in]    pData        Parameters.
********************************************************************************** */
static void Uart_TxCallBack(void *pBuffer)
{
    MEM_BufferFree(pBuffer);
}


/*! *********************************************************************************
* \brief        Handles battery measurement timer callback.
*
* \param[in]    pParam        Calback parameters.
********************************************************************************** */
static void BatteryMeasurementTimerCallback(void * pParam)
{
    mBasServiceConfig.batteryLevel = BOARD_GetBatteryLevel();
    Bas_RecordBatteryMeasurement(&mBasServiceConfig);
}

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
void InitRTC()
{
    uint32_t sec;
    uint32_t currSeconds;
    uint8_t index;

    rtc_config_t rtcConfig;

    RTC_GetDefaultConfig(&rtcConfig);
    RTC_Init(RTC, &rtcConfig);
    /* Select RTC clock source */
    RTC_SetClockSource(RTC);

	resultado[0] = 70;
	resultado[1] = 101;
	resultado[2] = 99;
	resultado[3] = 104;
	resultado[4] = 97;
	resultado[5] = 58;
	resultado[6] = 32;

    date.year = 2014U;
    date.month = 12U;
    date.day = 25U;
    date.hour = 19U;
    date.minute = 0;
    date.second = 0;
    RTC_StopTimer(RTC);

    /* Set RTC time to default */
    RTC_SetDatetime(RTC, &date);
    RTC_StartTimer(RTC);
    RTC_GetDatetime(RTC, &date);

}



void printTime()
{
    RTC_GetDatetime(RTC, &date);
	resultado[7] = ((date.year/1000));
	resultado[8] = ((date.year-(resultado[7]*1000))/100);
	resultado[9] = ((date.year-(resultado[7]*1000)-(resultado[8]*100))/10);
	resultado[10] = ((date.year-(resultado[7]*1000)-(resultado[8]*100)-(resultado[9]*10)));
	resultado[11] = 45;
	resultado[12] = ((date.month/10));
	resultado[13] = ((date.month-(resultado[12]*10)));
	resultado[14] = 45;
	resultado[15] = ((date.day/10));
	resultado[16] = ((date.day-(resultado[15]*10)));

	for (int i=7;i<17;i++)
	{
		if (resultado[i]!= 45)
		{
			resultado[i] += 48;
		}
	}
}

void printingTime(uint8_t *Send)
{
	uint8_t ForPrint[10];
	for(int i=0;i<10;i++)
	{
		ForPrint[i] = *(Send + i);
	}
	BleApp_SendUartStream(&ForPrint[0],SizeOfString(ForPrint)+1);
}

void NuevaFecha(uint8_t* Fecha)
{
	uint8_t Fechita[10];
	for (int i=0;i<10;i++)
	{
		Fechita[i] = *(Fecha+i);
	}

	date.year = ((Fechita[0]-48)*1000)+((Fechita[1]-48)*100)+((Fechita[2]-48)*10)+(Fechita[3]-48);
	date.month = ((Fechita[5]-48)*10)+(Fechita[6]-48);
	date.day = ((Fechita[8]-48)*10)+(Fechita[9]-48);
	PRINTF("SO: %c",date.month);
	RTC_StopTimer(RTC);
	/* Set RTC time to default */
	RTC_SetDatetime(RTC, &date);
	RTC_StartTimer(RTC);
	RTC_GetDatetime(RTC, &date);

}
void UltPag(uint8_t* Fecha)
{
	uint32_t Fechita[10];
	for (int i=0;i<10;i++)
	{
		Fechita[i] = *(Fecha+i);
	}

	MemFlashWrite(Fechita, sizeof(Fechita));
}

void DEMO_LPADC_IRQ_HANDLER_FUNC(void)
{
    g_LpadcInterruptCounter++;
    if (LPADC_GetConvResult(DEMO_LPADC_BASE, &g_LpadcResultConfigStruct))
    {
        g_LpadcConversionCompletedFlag = true;
        adc_values[adc_index] = g_LpadcResultConfigStruct.convValue >> 3u;
        adc_index = (adc_index + 1) % adc_n_samples;

        if(adc_index<(adc_n_samples-1))
        {
        	if(adc_values[adc_index]>2800)
        	{
        		sumCurrent=sumCurrent+((.080566*adc_values[adc_index])*(.080566*adc_values[adc_index]));
        	}
        }else if(adc_values[adc_index]>2800)
        {
        	currentTotal=(sqrt(sumCurrent)/adc_n_samples)/10;
        	sumCurrent=(.080566*adc_values[adc_index])*(.080566*adc_values[adc_index]);
        }else
        {
        	sumCurrent=0;
        }
    }
}

void LPTMR_ADC_HANDLER(void)
{
    LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);

    if(g_LpadcConversionCompletedFlag)
    {
    	g_LpadcConversionCompletedFlag = false;
    	LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* 1U is trigger0 mask. */
    }

    else
    {
    	g_LpadcConversionCompletedFlag = false;
    }

    __DSB();
    __ISB();
}

void TMR_ADC()
{
	/*Timer initialization*/
	    lptmr_config_t lptmrConfig;
	    INTMUX_Init(INTMUX0);
	    INTMUX_EnableInterrupt(INTMUX0, 0, DEMO_LPTMR_IRQn);
	    LPTMR_GetDefaultConfig(&lptmrConfig);

	    /* Initialize the LPTMR */
	    LPTMR_Init(DEMO_LPTMR_BASE, &lptmrConfig);

		/*
		 * Set timer period.
		 * Note : the parameter "ticks" of LPTMR_SetTimerPeriod should be equal or greater than 1.
		*/
		LPTMR_SetTimerPeriod(DEMO_LPTMR_BASE, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

		/* Enable timer interrupt */
		LPTMR_EnableInterrupts(DEMO_LPTMR_BASE, kLPTMR_TimerInterruptEnable);

		/* Enable at the NVIC */
		EnableIRQ(DEMO_LPTMR_IRQn);

		/* Start counting */
		LPTMR_StartTimer(DEMO_LPTMR_BASE);

}


void RTC_IRQHandler(void)
{
    if (RTC_GetStatusFlags(RTC) & kRTC_AlarmFlag)
    {
        busyWait = false;

        /* Clear alarm flag */
        RTC_ClearStatusFlags(RTC, kRTC_AlarmInterruptEnable);
    }
}
