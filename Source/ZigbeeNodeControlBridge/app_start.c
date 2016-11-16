/*****************************************************************************
 *
 * MODULE: ZigbeeNodeControlBridge
 *
 * COMPONENT: app_start.c
 *
 * $AUTHOR: Faisal Bhaiyat $
 *
 * DESCRIPTION:
 *
 * $HeadURL:  $
 *
 * $Revision: 54887 $
 *
 * $LastChangedBy: nxp29741 $
 *
 * $LastChangedDate:  $
 *
 * $Id: app_start.c  $
 *
 *****************************************************************************
*
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "pwrm.h"
#include "pdum_apl.h"
#include "pdum_nwk.h"
#include "pdum_gen.h"
#include "pdm.h"
#include "dbg_uart.h"
#include "dbg.h"
#include "zps_gen.h"
#include "zps_apl_af.h"
#include "appapi.h"
#include "zps_nwk_pub.h"
#include "zps_mac.h"
#include "rnd_pub.h"
#include "Htsdriver.h"
#include "Button.h"
#include <string.h>
#include "SerialLink.h"
#include "app_Znc_cmds.h"
#include "Uart.h"
#include "mac_pib.h"
#include "PDM_IDs.h"
#include "app_common.h"
#include "Log.h"
#include "app_events.h"
#include "zcl_common.h"
#ifdef STACK_MEASURE
#include "StackMeasure.h"
#endif

#ifdef CLD_OTA
#include "app_ota_server.h"
#endif
#if (JENNIC_CHIP_FAMILY == JN517x)
#include "AHI_ModuleConfiguration.h"
#endif
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define UART_DEBUGGING TRUE//forrest
#ifndef DEBUG_WDR
#define DEBUG_WDR                                                 TRUE
#endif

#ifndef UART_DEBUGGING
#define UART_DEBUGGING                                            FALSE
#endif

#ifndef TRACE_APPSTART
#define TRACE_APPSTART                                            FALSE
#endif


#ifndef TRACE_EXC
#define TRACE_EXC                                                 TRUE
#endif

#if (JENNIC_CHIP_FAMILY == JN516x)
#define LED1_DIO_PIN                                              ( 1 << 16 )
#define LED2_DIO_PIN                                              ( 1 << 17 )

#define LED_DIO_PINS                                             ( LED1_DIO_PIN |\
                                                                   LED2_DIO_PIN )

#endif

#if (JENNIC_CHIP_FAMILY == JN517x)

#define LED1_DIO_PIN                                              ( 1 << 14 )
#define LED2_DIO_PIN                                              ( 1 << 15 )

#define LED_DIO_PINS                                             ( LED1_DIO_PIN |\
                                                                   LED2_DIO_PIN )
#endif

#define TIMER_QUEUE_SIZE                                           8
#define MLME_QUEQUE_SIZE                                           8
#define MCPS_QUEUE_SIZE                                            30
#define ZPS_QUEUE_SIZE                                             2
#define APP_QUEUE_SIZE                                             8
#define TX_QUEUE_SIZE                                              150
#define RX_QUEUE_SIZE                                              150
#define BDB_QUEUE_SIZE                                             2
//#define APP_ZTIMER_STORAGE                                         4
#define APP_ZTIMER_STORAGE                                         20   //forrest

#if JENNIC_CHIP_FAMILY == JN517x

#define NVIC_INT_PRIO_LEVEL_BBC                                    ( 7 )
#define NVIC_INT_PRIO_LEVEL_UART0                                  ( 5 )
#endif


/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PUBLIC void app_vFormatAndSendUpdateLists ( void );
PUBLIC void APP_vMainLoop ( void );
PUBLIC void APP_vInitResources ( void );
PUBLIC void APP_vSetUpHardware ( void );
#if (JENNIC_CHIP_FAMILY == JN516x)
/* Run time exception handlers */
void vBusErrorHandler ( void );
void vAlignmentErrorHandler (void );
void vIllegalInstructionHandler ( void );
void vUnclaimedException ( void );
void vStackOverflowHandler ( void );
void vReportException ( char*    sExStr );
#endif

void vfExtendedStatusCallBack ( ZPS_teExtendedStatus    eExtendedStatus );
PRIVATE void vInitialiseApp ( void );
#if (defined PDM_EEPROM)
#if TRACE_APPSTART
PRIVATE void vPdmEventHandlerCallback ( uint32                  u32EventNumber,
                                        PDM_eSystemEventCode    eSystemEventCode );
#endif
#endif
PRIVATE void APP_cbTimerZclTick (void*    pvParam);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

extern bool_t g_bStopZdpAcks;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/



PUBLIC tsLedState         s_sLedState =  { LED1_DIO_PIN,  ZTIMER_TIME_MSEC(1),  FALSE };
#ifdef FULL_FUNC_DEVICE
tsZllEndpointInfoTable    sEndpointTable;
tsZllGroupInfoTable       sGroupTable;
#endif

tsZllState sZllState = {
        .eState             = FACTORY_NEW,
        .eNodeState         = E_STARTUP,
        .u8DeviceType       = 0,
        .u8MyChannel        = 11,
        .u16MyAddr          = TL_MIN_ADDR,
#ifdef FULL_FUNC_DEVICE
        .u16FreeAddrLow     = TL_MIN_ADDR,
        .u16FreeAddrHigh    = TL_MAX_ADDR,
        .u16FreeGroupLow    = TL_MIN_GROUP,
        .u16FreeGroupHigh   = TL_MAX_GROUP,
#endif
#ifdef CLD_OTA
        .bValid                 = FALSE,
        .u64IeeeAddrOfServer    = 0,
        .u16NwkAddrOfServer     = 0,
        .u8OTAserverEP          = 0
#endif
};

PUBLIC tszQueue           APP_msgSerialRx;
PUBLIC tszQueue           APP_msgSerialTx;
PUBLIC tszQueue           APP_msgBdbEvents;
PUBLIC tszQueue           APP_msgAppEvents;

ZTIMER_tsTimer            asTimers[APP_ZTIMER_STORAGE + BDB_ZTIMER_STORAGE];
zps_tsTimeEvent           asTimeEvent [ TIMER_QUEUE_SIZE ];
MAC_tsMcpsVsDcfmInd       asMacMcpsDcfmInd [ MCPS_QUEUE_SIZE ];
MAC_tsMlmeVsDcfmInd       asMacMlmeVsDcfmInd [ MLME_QUEQUE_SIZE ];
BDB_tsZpsAfEvent          asBdbEvent [ BDB_QUEUE_SIZE ];
APP_tsEvent               asAppMsg [ APP_QUEUE_SIZE ];
uint8                     au8AtRxBuffer [ RX_QUEUE_SIZE ];
uint8                     au8AtTxBuffer [ TX_QUEUE_SIZE ];
uint8                     u8IdTimer;
uint8                     u8TmrToggleLED;
uint8                     u8HaModeTimer;
uint8                     u8TickTimer;
uint8                     u8JoinedDevice =  0;
uint8                     au8LinkRxBuffer[256];
ZPS_tsAfFlashInfoSet      sSet;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
#ifdef FULL_FUNC_DEVICE
extern PUBLIC void APP_cbTimerCommission ( void*    pvParam );
#endif
#if (defined PDM_EEPROM)
#if TRACE_APPSTART
extern PUBLIC uint8 u8PDM_CalculateFileSystemCapacity ( void );
extern PUBLIC uint8 u8PDM_GetFileSystemOccupancy ( void );
#endif
#endif

#if JENNIC_CHIP_FAMILY == JN517x
extern void APP_isrUart ( uint32    u32Device ,
                          uint32    u32ItemBitmap );
extern void vWatchdogHandler ( uint32    u32Device ,
                               uint32    u32ItemBitmap );
#endif
//forrest
uint8 					  u8SerialCmdTimer;

DeviceAnnounceAction iDeviceAnnounceAction[MAX_DEV_ANNC_INST];
PUBLIC void App_DeviceAnnounceCallback( void*   pvParam );
PUBLIC void App_DeviceAnnounceTimeout (void*   pvParam );

//forrest

/****************************************************************************
 *
 * NAME: vAppMain
 *
 * DESCRIPTION:
 * Entry point for application from a cold start.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/

PUBLIC void vAppMain(void)
{
    uint8    u8FormJoin;
    uint8    au8LinkTxBuffer[50];
    // Wait until FALSE i.e. on XTAL  - otherwise uart data will be at wrong speed
    while ( bAHI_GetClkSource ( ) == TRUE );

    // Now we are running on the XTAL, optimise the flash memory wait states.
    vAHI_OptimiseWaitStates ( );
    g_bStopZdpAcks =  TRUE;
    vAHI_DioSetDirection ( 0, LED_DIO_PINS );
    vAHI_DioSetOutput ( LED_DIO_PINS, 0 );
    /* set up storage in flash for the keys */
    sSet.u16CredNodesCount = ZNC_MAX_TCLK_DEVICES;
    sSet.u16SectorSize = 32 * 1024;
    sSet.u8SectorSet   = 7;
#ifdef STACK_MEASURE
    vInitStackMeasure ( );
#endif


#ifdef HWDEBUG
    vAHI_WatchdogStart ( 12 );
#endif

#if (JENNIC_CHIP_FAMILY == JN516x)
    vAHI_WatchdogException ( TRUE );
#else
    vAHI_WatchdogException ( TRUE , vWatchdogHandler );
#endif
    /* Initialise debugging */
#if  (UART_DEBUGGING == TRUE)
    /* Send debug output to DBG_UART */
    DBG_vUartInit ( DBG_E_UART_1, DBG_E_UART_BAUD_RATE_115200 );//forrest
#else
    /* Send debug output through SerialLink to host */
    vSL_LogInit ( );
#endif
    DBG_vPrintf ( TRACE_APPSTART, "APP: Entering APP_vInitResources()\n" );
    APP_vInitResources ( );

    DBG_vPrintf ( TRACE_APPSTART, "APP: Entering APP_vSetUpHardware()\n" );
    APP_vSetUpHardware ( );
#if (JENNIC_CHIP_FAMILY == JN517x)
    vAHI_ModuleConfigure(E_MODULE_DEFAULT);
#endif
    UART_vInit ( );
    UART_vRtsStartFlow ( );
    vLog_Printf ( TRACE_APPSTART,LOG_DEBUG, "\n\nInitialising \n" );
    extern void*    _stack_low_water_mark;
    vSL_LogFlush ( );
    vLog_Printf ( TRACE_APPSTART,LOG_INFO, "Stack low water mark = %08x\n", &_stack_low_water_mark );

#if (JENNIC_CHIP_FAMILY == JN516x)
    vAHI_SetStackOverflow ( TRUE, ( uint32 ) &_stack_low_water_mark );
#endif
    vLog_Printf ( TRACE_EXC, LOG_INFO, "\n** Control Bridge Reset** " );
    if ( bAHI_WatchdogResetEvent ( ) )
    {
        /* Push out anything that might be in the log buffer already */
        vLog_Printf ( TRACE_EXC, LOG_CRIT, "\n\n\n%s WATCHDOG RESET @ %08x ", "WDR", ( ( uint32* ) &_heap_location) [ 0 ] );
        vSL_LogFlush ( );
    }
    vInitialiseApp ();
    app_vFormatAndSendUpdateLists ( );

    
    if (sZllState.eNodeState == E_RUNNING)
    {
        /* Declared within if statement. If it is declared at the top
         * the function, the while loop will cause the data to be on
         * the stack forever.
         */

        if( sZllState.u8DeviceType >=  1 )
        {
            u8FormJoin = 0;
        }
        else
        {
            u8FormJoin = 1;
        }
        APP_vSendJoinedFormEventToHost ( u8FormJoin, au8LinkTxBuffer );
        vSL_WriteMessage ( E_SL_MSG_NODE_NON_FACTORY_NEW_RESTART,
                           1,
                           ( uint8* ) &sZllState.eNodeState ) ;
        BDB_vStart();
    }
    else
    {
        vSL_WriteMessage( E_SL_MSG_NODE_FACTORY_NEW_RESTART,
                          1,
                          ( uint8* ) &sZllState.eNodeState );
    }
    ZTIMER_eStart ( u8TickTimer, ZCL_TICK_TIME );
    ZTIMER_eStart( u8SerialCmdTimer, ZTIMER_TIME_MSEC(3000)); //forrest
    DBG_vPrintf( TRACE_APPSTART, "APP: Entering APP_vMainLoop()\n");
    APP_vMainLoop();

}

void vAppRegisterPWRMCallbacks(void)
{

}

void APP_cbToggleLED ( void* pvParam )
{
    tsLedState*    psLedState =  ( tsLedState* ) pvParam;

    if( ZPS_vNwkGetPermitJoiningStatus ( ZPS_pvAplZdoGetNwkHandle ( ) ) )
    {
        vAHI_DioSetOutput ( LED2_DIO_PIN, (psLedState->u32LedState) & LED_DIO_PINS );
        vAHI_DioSetOutput ( LED1_DIO_PIN, (psLedState->u32LedState) & LED_DIO_PINS );
    }
    else
    {
        vAHI_DioSetOutput ( ( ~psLedState->u32LedState ) & LED_DIO_PINS, psLedState->u32LedState & LED_DIO_PINS );

    }
    psLedState->u32LedState =  ( ~psLedState->u32LedState ) & LED_DIO_PINS;

    if( u8JoinedDevice == 10 )
    {
        if(  !ZPS_vNwkGetPermitJoiningStatus ( ZPS_pvAplZdoGetNwkHandle ( ) ) )
        {
            psLedState->u32LedToggleTime =  ZTIMER_TIME_MSEC ( 1 );
        }

        if( ZPS_vNwkGetPermitJoiningStatus ( ZPS_pvAplZdoGetNwkHandle ( ) ) )
        {
            psLedState->u32LedToggleTime =  ZTIMER_TIME_MSEC ( 500 );
        }
        u8JoinedDevice =  0;
    }
    u8JoinedDevice++;

    ZTIMER_eStart( u8TmrToggleLED, psLedState->u32LedToggleTime );
}

#ifdef FULL_FUNC_DEVICE

/****************************************************************************
 *
 * NAME: APP_vInitialiseNode
 *
 * DESCRIPTION:
 * Initialises the application related functions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC tsZllEndpointInfoTable * psGetEndpointRecordTable(void)
{
    return &sEndpointTable;
}


/****************************************************************************
 *
 * NAME: psGetGroupRecordTable
 *
 * DESCRIPTION:
 * return the address of the group record table
 *
 * RETURNS:
 * pointer to the group record table
 *
 ****************************************************************************/
PUBLIC tsZllGroupInfoTable * psGetGroupRecordTable(void)
{
    return &sGroupTable;
}

#endif
/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vInitialiseApp
 *
 * DESCRIPTION:
 * Initialises Zigbee stack, hardware and application.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vInitialiseApp ( void )
{
    uint16            u16DataBytesRead;
    BDB_tsInitArgs    sArgs;
    uint8             u8DeviceType;


    PDM_eInitialise ( 63 );
    PDUM_vInit ( );
    PWRM_vInit ( E_AHI_SLEEP_OSCON_RAMON );
#if (defined PDM_EEPROM)
#if TRACE_APPSTART
    PDM_vRegisterSystemCallback ( vPdmEventHandlerCallback );
#endif
#endif
    sZllState.eNodeState =  E_STARTUP;
    /* Restore any application data previously saved to flash */
    PDM_eReadDataFromRecord ( PDM_ID_APP_ZLL_CMSSION,
                              &sZllState,
                              sizeof ( tsZllState ),
                              &u16DataBytesRead );
#ifdef FULL_FUNC_DEVICE
    PDM_eReadDataFromRecord ( PDM_ID_APP_END_P_TABLE,
                              &sEndpointTable,
                              sizeof ( tsZllEndpointInfoTable ),
                              &u16DataBytesRead );
    PDM_eReadDataFromRecord ( PDM_ID_APP_GROUP_TABLE,
                              &sGroupTable,
                              sizeof ( tsZllGroupInfoTable ),
                              &u16DataBytesRead );
#endif
    ZPS_eMiniMacSetTxBuffers ( 5 );

    if ( sZllState.eNodeState == E_RUNNING )
    {
        u8DeviceType = ( sZllState.u8DeviceType >=  2 ) ? 1 : sZllState.u8DeviceType;
        APP_vConfigureDevice ( u8DeviceType );
        ZPS_eAplAfInit ( );
    }
    else
    {

        ZPS_eAplAfInit ( );
        sZllState.u8DeviceType =  0;
        ZPS_vNwkNibSetChannel ( ZPS_pvAplZdoGetNwkHandle(), DEFAULT_CHANNEL);
        ZPS_vNwkNibSetPanId (ZPS_pvAplZdoGetNwkHandle(), (uint16) RND_u32GetRand ( 1, 0xfff0 ) );

    }
    /* If the device state has been restored from flash, re-start the stack
     * and set the application running again.
     */
    sBDB.sAttrib.bbdbNodeIsOnANetwork      =  ( ( sZllState.eNodeState >= E_RUNNING ) ? ( TRUE ) : ( FALSE ) );
    sBDB.sAttrib.bTLStealNotAllowed        =  !sBDB.sAttrib.bbdbNodeIsOnANetwork;
    sArgs.hBdbEventsMsgQ                   =  &APP_msgBdbEvents;
    BDB_vInit ( &sArgs );

    ZPS_vExtendedStatusSetCallback(vfExtendedStatusCallBack);
    APP_ZCL_vInitialise();
    /* Needs to be after we initialise the ZCL and only if we are already
     * running. If we are not running we will send the notify after we
     * have a network formed notification.
     */
    if (sZllState.eNodeState == E_RUNNING)
    {
#ifdef CLD_OTA
        vAppInitOTA();
#endif
    }
}


/****************************************************************************
 *
 * NAME: APP_vMainLoop
 *
 * DESCRIPTION:
 * Main application loop
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vMainLoop(void)
{
    bSetPermitJoinForever = TRUE;
    while (TRUE)
    {
        
        zps_taskZPS ( );
        bdb_taskBDB ( );
        APP_vHandleAppEvents ( );
        APP_vProcessRxData ( );
        ZTIMER_vTask ( );

#ifdef DBG_ENABLE
        vSL_LogFlush ( ); /* flush buffers */
#endif
        /* Re-load the watch-dog timer. Execution must return through the idle
         * task before the CPU is suspended by the power manager. This ensures
         * that at least one task / ISR has executed within the watchdog period
         * otherwise the system will be reset.
         */
        vAHI_WatchdogRestart ( );

        /*
         * suspends CPU operation when the system is idle or puts the device to
         * sleep if there are no activities in progress
         */
        PWRM_vManagePower();
    }
}


/****************************************************************************
 *
 * NAME: APP_vSetUpHardware
 *
 * DESCRIPTION:
 * Set up interrupts
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vSetUpHardware ( void )
{
#if (JENNIC_CHIP_FAMILY == JN517x)
    vAHI_Uart0RegisterCallback ( APP_isrUart );
    u32AHI_Init ( );
    vAHI_InterruptSetPriority ( MICRO_ISR_MASK_BBC,        NVIC_INT_PRIO_LEVEL_BBC );
    vAHI_InterruptSetPriority ( MICRO_ISR_MASK_UART0,      NVIC_INT_PRIO_LEVEL_UART0 );
#else
    TARGET_INITIALISE ( );
    /* clear interrupt priority level  */
    SET_IPL ( 0 );
    portENABLE_INTERRUPTS ( );
#endif
}


/****************************************************************************
 *
 * NAME: APP_vInitResources
 *
 * DESCRIPTION:
 * Initialise resources (timers, queue's etc)
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vInitResources ( void )
{

    vLog_Printf ( TRACE_APPSTART,LOG_DEBUG, "APP: Initialising resources...\n");
    vLog_Printf ( TRACE_APPSTART,LOG_DEBUG, "APP: ZPS_tsAfEvent = %d bytes\n",    sizeof ( ZPS_tsAfEvent ) );
    vLog_Printf ( TRACE_APPSTART,LOG_DEBUG, "APP: zps_tsTimeEvent = %d bytes\n",  sizeof ( zps_tsTimeEvent ) );

    /* Initialise the Z timer module */
    ZTIMER_eInit ( asTimers, sizeof(asTimers) / sizeof(ZTIMER_tsTimer));
#if (JENNIC_CHIP_FAMILY == JN517x)
    /* Create Z timers */
    ZTIMER_eOpen ( &u8TickTimer,       APP_cbTimerZclTick,          NULL,                      ZTIMER_FLAG_PREVENT_SLEEP );
    ZTIMER_eOpen ( &u8IdTimer,         APP_vIdentifyEffectEnd,      NULL,                      ZTIMER_FLAG_PREVENT_SLEEP );
    ZTIMER_eOpen ( &u8TmrToggleLED,    APP_cbToggleLED,             &s_sLedState,              ZTIMER_FLAG_PREVENT_SLEEP );
    ZTIMER_eOpen ( &u8HaModeTimer,     App_TransportKeyCallback,    &u64CallbackMacAddress,    ZTIMER_FLAG_PREVENT_SLEEP );
#else
    /* Create Z timers */
    ZTIMER_eOpen ( &u8TickTimer,       APP_cbTimerZclTick,          NULL );
    ZTIMER_eOpen ( &u8IdTimer,         APP_vIdentifyEffectEnd,         NULL );
    ZTIMER_eOpen ( &u8TmrToggleLED,    APP_cbToggleLED,             &s_sLedState );
    ZTIMER_eOpen ( &u8HaModeTimer,     App_TransportKeyCallback,    &u64CallbackMacAddress );
#endif
    /* Create all the queues */
    ZQ_vQueueCreate ( &APP_msgBdbEvents,      BDB_QUEUE_SIZE,         sizeof ( BDB_tsZpsAfEvent ),       (uint8*)asBdbEvent );
    ZQ_vQueueCreate ( &zps_msgMlmeDcfmInd,    MLME_QUEQUE_SIZE,       sizeof ( MAC_tsMlmeVsDcfmInd ),    (uint8*)asMacMlmeVsDcfmInd );
    ZQ_vQueueCreate ( &zps_msgMcpsDcfmInd,    MCPS_QUEUE_SIZE,        sizeof ( MAC_tsMcpsVsDcfmInd ),    (uint8*)asMacMcpsDcfmInd );
    ZQ_vQueueCreate ( &zps_TimeEvents,        TIMER_QUEUE_SIZE,       sizeof ( zps_tsTimeEvent ),        (uint8*)asTimeEvent );
    ZQ_vQueueCreate ( &APP_msgAppEvents,      APP_QUEUE_SIZE,         sizeof ( APP_tsEvent ),            (uint8*)asAppMsg );
    ZQ_vQueueCreate ( &APP_msgSerialTx,       TX_QUEUE_SIZE,          sizeof ( uint8 ),                  (uint8*)au8AtTxBuffer );
    ZQ_vQueueCreate ( &APP_msgSerialRx,       RX_QUEUE_SIZE,          sizeof ( uint8 ),                  (uint8*)au8AtRxBuffer );

	//forrest
	ZTIMER_eOpen (&u8SerialCmdTimer,  App_SerialCmdCallback,NULL);
#if 1

	//ZTIMER_eOpen ( &u8ActiveEndpointReqTimer, App_ActiveEndpointCallback,  NULL);
	//Timer index : [4,5], [6,7],  ......
	uint8 i;
	for(i=0;i<MAX_DEV_ANNC_INST;i++)
	{
		iDeviceAnnounceAction[i].u8Params = i*2;
		iDeviceAnnounceAction[i].u8ParamsTimeout = i*2+1;
		iDeviceAnnounceAction[i].eState = LeTV_NONE;
		iDeviceAnnounceAction[i].bStatus = FALSE;
		iDeviceAnnounceAction[i].u16ShortAddress = 0x0000;
		ZTIMER_eOpen ( &iDeviceAnnounceAction[i].u8Timer, App_DeviceAnnounceCallback,  &iDeviceAnnounceAction[i].u8Params);
		ZTIMER_eOpen ( &iDeviceAnnounceAction[i].u8Timeout, App_DeviceAnnounceTimeout,  &iDeviceAnnounceAction[i].u8ParamsTimeout);
		
		DBG_vPrintf(TRUE, "\n ZTIMER_eOpen : 0x%x 0x%x 0x%x\n",i,iDeviceAnnounceAction[i].u8Timer,iDeviceAnnounceAction[i].u8Timeout);
	}
	//forrest
#endif
    vZCL_RegisterHandleGeneralCmdCallBack (APP_vProfileWideCommandSupportedForCluster );
    DBG_vPrintf(TRACE_APPSTART, "APP: Initialising resources complete\n");

}

#if (JENNIC_CHIP_FAMILY == JN516x)
/****************************************************************************
 *
 * NAME: vUnclaimedException
 *
 * DESCRIPTION:
 * Initialises Zigbee stack, hardware and application.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vUnclaimedException ( void )
{

    register uint32    u32PICSR;
    register uint32    u32PICMR;

    asm volatile ("l.mfspr %0,r0,0x4800" :"=r"(u32PICMR) : );
    asm volatile ("l.mfspr %0,r0,0x4802" :"=r"(u32PICSR) : );

    vLog_Printf ( TRACE_APPSTART,LOG_EMERG, "Unclaimed interrupt : %x : %x\n", u32PICSR, u32PICSR );
    vSL_LogFlush ( );
    /* Log the exception */
    vLog_Printf ( TRACE_EXC, LOG_CRIT, "\n\n\n%s EXCEPTION @ %08x (PICMR: %08x HP: %08x)", "UCMI",
                                       u32PICMR, u32PICSR, ( ( uint32* ) &_heap_location) [ 0 ] );
    vSL_LogFlush ( );
    /* Software reset */
    vAHI_SwReset ( );
}

void vStackOverflowHandler ( void )
{

    vReportException ( "EXS" );
}


void vAlignmentErrorHandler ( void )
{
    vReportException ( "EXA" );
}

void vBusErrorHandler ( void )
{
    vReportException ( "EXB" );
}

void vIllegalInstructionHandler ( void )
{
    vReportException ( "EXI" );
}

void vReportException ( char*    sExStr )
{

    register uint32     u32EPCR;
    register uint32     u32EEAR;
    volatile uint32*    pu32Stack;

    /* TODO - add reg dump too */

    asm volatile ("l.mfspr %0,r0,0x0020" :"=r"(u32EPCR) : );
    asm volatile ("l.mfspr %0,r0,0x0030" :"=r"(u32EEAR) : );
    asm volatile ("l.or %0,r0,r1" :"=r"(pu32Stack) : );


    vSL_LogFlush();
    /* Log the exception */
    vLog_Printf(TRACE_EXC, LOG_CRIT, "\n\n\n%s EXCEPTION @ %08x (EA: %08x SK: %08x HP: %08x)",
                               sExStr, u32EPCR, u32EEAR, pu32Stack, ( ( uint32* ) &_heap_location) [ 0 ] );
    vSL_LogFlush ( );

    vLog_Printf ( TRACE_EXC,LOG_CRIT, "Stack dump:\n" );
    vSL_LogFlush ( );
#if (DEBUG_WDR == TRUE)
    vAHI_WatchdogStop ( );
#endif
    /* loop until we hit a 32k boundary. should be top of stack */
    while ( ( uint32 ) pu32Stack & 0x7fff )
    {
#if (DEBUG_WDR == TRUE)
        volatile uint32 u32Delay;
#endif
        vLog_Printf ( TRACE_EXC,LOG_CRIT, "% 8x : %08x\n", pu32Stack, *pu32Stack );
        vSL_LogFlush ( );
        pu32Stack++;
#if (DEBUG_WDR == TRUE)
        for (u32Delay = 0; u32Delay < 100000; u32Delay++);
            vAHI_DioSetOutput(LED_DIO_PINS, 0);
        for (u32Delay = 0; u32Delay < 100000; u32Delay++);
            vAHI_DioSetOutput(0, LED_DIO_PINS);
#endif
    }
    vSL_LogFlush ( );
#if (DEBUG_WDR == FALSE)
    /* Software reset */
    vAHI_SwReset ( );
#endif
#if (DEBUG_WDR == TRUE)
    while(1)
    {
        volatile uint32    u32Delay;
        for (u32Delay = 0; u32Delay < 100000; u32Delay++);
            vAHI_DioSetOutput(LED_DIO_PINS, 0);
        for (u32Delay = 0; u32Delay < 100000; u32Delay++);
            vAHI_DioSetOutput(0, LED_DIO_PINS);
    }
#endif
}

#endif

/****************************************************************************
 *
 * NAME: app_vFormatAndSendUpdateLists
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void app_vFormatAndSendUpdateLists ( void )
{
    typedef struct
    {
        uint16     u16Clusterid;
        uint16*    au16Attibutes;
        uint32     u32Asize;
        uint8*     au8command;
        uint32     u32Csize;
    }    tsAttribCommand;

    uint16                 u16Length =  0;
    uint8                  u8Count = 0 ;
    uint8                  u8BufferLoop =  0;
    uint8                  au8LinkTxBuffer[256];

    /*List of clusters per endpoint */
    uint16    u16ClusterListHA [ ]  =  { 0x0000, 0x0001, 0x0003, 0x0004, 0x0005, 0x0006,
                                         0x0008, 0x0019, 0x0101, 0x1000, 0x0300, 0x0201, 0x0204 };
    uint16    u16ClusterListHA2 [ ] =  { 0x0405, 0x0500, 0x0400, 0x0402, 0x0403, 0x0405, 0x0406,
                                         0x0702, 0x0b03, 0x0b04 , 0x1000 };

    /*list of attributes per cluster */

    uint16    u16AttribBasic [ ] =  { 0x0000, 0x0001, 0x0002, 0x0003, 0x0004,
                                      0x0005, 0x0006, 0x0007, 0x4000 };
    uint16    u16AttribIdentify [ ] =  { 0x000 };
    uint16    u16AttribGroups [ ] =  { 0x000 };
    uint16    u16AttribScenes [ ] =  { 0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005 };
    uint16    u16AttribOnOff [ ] =  { 0x0000, 0x4000, 0x4001, 0x4002 };
    uint16    u16AttribLevel [ ] =  { 0x0000, 0x0001, 0x0010, 0x0011 };
    uint16    u16AttribColour [ ] =  { 0x0000, 0x0001, 0x0002, 0x0007, 0x0008,
                                       0x0010, 0x0011, 0x0012, 0x0013, 0x0015,
                                       0x0016, 0x0017, 0x0019, 0x001A, 0x0020,
                                       0x0021, 0x0022, 0x0024, 0x0025, 0x0026,
                                       0x0028, 0x0029, 0x002A, 0x4000, 0x4001,
                                       0x4002, 0x4003, 0x4004, 0x4006, 0x400A,
                                       0x400B, 0x400C };
    uint16    u16AttribThermostat [ ] =  { 0x0000, 0x0003, 0x0004, 0x0011, 0x0012,
                                           0x001B, 0x001C };
    uint16    u16AttribHum [ ] =  { 0x0000, 0x0001, 0x0002, 0x0003 };
    uint16    u16AttribPower [ ] =  { 0x0020, 0x0034 };
    uint16    u16AttribIllumM [ ] =  { 0x000, 0x0001, 0x0002, 0x0003, 0x0004 };
    uint16    u16AttribIllumT [ ] =  { 0x000, 0x0001, 0x0002 };
    uint16    u16AttribSM [ ] =  { 0x0000, 0x0300, 0x0301, 0x0302, 0x0306, 0x0400 };
    /*list of commands per cluster */
    uint8     u8CommandBasic [ ] =  { 0 };
    uint8     u8CommandIdentify [ ] =  { 0, 1, 0x40 };
    uint8     u8CommandGroups [ ] =  { 0, 1, 2, 3, 4, 5 };
    uint8     u8CommandScenes [ ] =  { 0, 1, 2, 3, 4, 5, 6,
                                    0x40, 0x41, 0x42 };
    uint8     u8CommandsOnOff [ ] =  { 0, 1, 2, 0x40, 0x41, 0x42 };
    uint8     u8CommandsLevel [ ] =  { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8     u8CommandsColour [ ] =  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                     0xa, 0x40, 0x41, 0x42, 0x43, 0x44, 0x47,
                                     0x4b, 0x4c, 0xfe, 0xff };
    uint8     u8CommandThermostat [ ] = {0};
    uint8     u8CommandHum [ ] =  { 0 };
    uint8     u8CommandPower [ ] =  { 0 };
    uint8     u8CommandIllumM [ ] =  { 0 };
    uint8     u8CommandIllumT [ ] =  { 0 };
    uint8     u8CommandSM [ ] =  { 0 };

    tsAttribCommand    asAttribCommand[13] =  {  { 0x0000, u16AttribBasic, ( sizeof( u16AttribBasic ) / sizeof ( u16AttribBasic [ 0 ] ) ),
                                                  u8CommandBasic, ( sizeof ( u8CommandBasic ) / sizeof ( u8CommandBasic [ 0 ] )  )},
                                                 { 0x0003, u16AttribIdentify, ( sizeof ( u16AttribIdentify ) / sizeof ( u16AttribIdentify [ 0 ]  ) ),
                                                   u8CommandIdentify, ( sizeof ( u8CommandIdentify ) / sizeof ( u8CommandIdentify [ 0 ]  ) ) },
                                                 { 0x0004, u16AttribGroups, ( sizeof ( u16AttribGroups ) / sizeof ( u16AttribGroups [ 0 ]  ) ),
                                                   u8CommandGroups, ( sizeof ( u8CommandGroups ) / sizeof ( u8CommandGroups [ 0 ]  ) ) },
                                                 { 0x0005, u16AttribScenes, ( sizeof ( u16AttribScenes ) / sizeof ( u16AttribScenes [ 0 ]  ) ),
                                                   u8CommandScenes, ( sizeof( u8CommandScenes )  / sizeof ( u8CommandScenes [ 0 ]  ) ) },
                                                 { 0x0006, u16AttribOnOff,  ( sizeof ( u16AttribOnOff ) / sizeof ( u16AttribOnOff [ 0 ]  ) ),
                                                   u8CommandsOnOff, ( sizeof ( u8CommandsOnOff )/ sizeof ( u8CommandsOnOff [ 0 ]  ) ) },
                                                 { 0x0008, u16AttribLevel,  ( sizeof ( u16AttribLevel ) / sizeof ( u16AttribLevel [ 0 ]  ) ),
                                                   u8CommandsLevel, ( sizeof ( u8CommandsLevel ) / sizeof ( u8CommandsLevel [ 0 ]  ) ) },
                                                 { 0x0300, u16AttribColour, ( sizeof ( u16AttribColour ) / sizeof ( u16AttribColour [ 0 ]  ) ),
                                                   u8CommandsColour, ( sizeof ( u8CommandsColour ) / sizeof ( u8CommandsColour [ 0 ]  ) ) },
                                                 { 0x0201, u16AttribThermostat, ( sizeof ( u16AttribThermostat ) / sizeof ( u16AttribThermostat [ 0 ]  ) ),
                                                   u8CommandThermostat, ( sizeof ( u8CommandThermostat ) / sizeof ( u8CommandThermostat [ 0 ]  ) ) },
                                                 { 0x0405, u16AttribHum, ( sizeof ( u16AttribHum ) / sizeof ( u16AttribHum [ 0 ]  ) ),
                                                   u8CommandHum, ( sizeof ( u8CommandHum ) / sizeof ( u8CommandHum [ 0 ]  ) ) },
                                                 { 0x0001, u16AttribPower, ( sizeof ( u16AttribPower ) /sizeof( u16AttribPower [ 0 ]  ) ),
                                                   u8CommandPower, ( sizeof ( u8CommandPower ) /sizeof( u8CommandPower [ 0 ]  ) ) },
                                                 { 0x0400, u16AttribIllumM, ( sizeof ( u16AttribIllumM ) /sizeof( u16AttribIllumM [ 0 ]  ) ),
                                                   u8CommandIllumM, ( sizeof ( u8CommandIllumM ) /sizeof( u8CommandIllumM [ 0 ]  ) ) },
                                                 { 0x0402, u16AttribIllumT, ( sizeof ( u16AttribIllumT ) /sizeof( u16AttribIllumT [ 0 ]  ) ),
                                                   u8CommandIllumT, ( sizeof ( u8CommandIllumT ) /sizeof( u8CommandIllumT [ 0 ]  ) ) },
                                                 { 0x0702, u16AttribSM, ( sizeof ( u16AttribSM ) / sizeof ( u16AttribSM [ 0 ]  ) ),
                                                   u8CommandSM, ( sizeof ( u8CommandSM ) / sizeof ( u8CommandSM [ 0 ]  ) )} };



    /* Cluster list endpoint HA */
    ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],            ZIGBEENODECONTROLBRIDGE_ZLO_ENDPOINT,    u16Length );
    ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    0x0104,                                  u16Length );
    while ( u8BufferLoop < ( sizeof ( u16ClusterListHA ) /  sizeof( u16ClusterListHA [ 0 ] ) ) )
    {
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    u16ClusterListHA [ u8BufferLoop ],    u16Length );
        u8BufferLoop++;
    }
    vSL_WriteMessage ( E_SL_MSG_NODE_CLUSTER_LIST,
                       u16Length,
                       au8LinkTxBuffer );

        /* Cluster list endpoint HA */
    u16Length = u8BufferLoop =  0;
    ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],         ZIGBEENODECONTROLBRIDGE_ZLO_ENDPOINT,    u16Length );
    ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ], 0x0104,                                  u16Length );
    while ( u8BufferLoop <  ( sizeof ( u16ClusterListHA2 ) /  sizeof ( u16ClusterListHA2 [ 0 ] ) )  )
    {

        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ], u16ClusterListHA2 [ u8BufferLoop ],    u16Length );
        u8BufferLoop++;
    }
    vSL_WriteMessage ( E_SL_MSG_NODE_CLUSTER_LIST,
                       u16Length,
                       au8LinkTxBuffer );

        /* Attribute list basic cluster HA EP*/
    for ( u8Count =  0; u8Count < 13; u8Count++ )
    {
        u16Length =  0;
        u8BufferLoop =  0;
        ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],            ZIGBEENODECONTROLBRIDGE_ZLO_ENDPOINT,     u16Length );
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    0x0104,                                   u16Length );
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    asAttribCommand[u8Count].u16Clusterid,    u16Length );
        while ( u8BufferLoop <   asAttribCommand [ u8Count ].u32Asize   )
        {
            ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    asAttribCommand[u8Count].au16Attibutes [ u8BufferLoop ],    u16Length );
            u8BufferLoop++;
        }
        vSL_WriteMessage ( E_SL_MSG_NODE_ATTRIBUTE_LIST,
                           u16Length,
                           au8LinkTxBuffer );
        u16Length =  0;
        u8BufferLoop =  0;
        ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],            ZIGBEENODECONTROLBRIDGE_ZLO_ENDPOINT,     u16Length );
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    0x0104,                                   u16Length );
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    asAttribCommand[u8Count].u16Clusterid,    u16Length );
        while ( u8BufferLoop <   asAttribCommand [ u8Count ].u32Csize  )
        {
            ZNC_BUF_U8_UPD ( &au8LinkTxBuffer [ u16Length ],    asAttribCommand[u8Count].au8command [ u8BufferLoop ],    u16Length );
            u8BufferLoop++;
        }
        vSL_WriteMessage ( E_SL_MSG_NODE_COMMAND_ID_LIST,
                           u16Length,
                           au8LinkTxBuffer );
    }
    
}

void vfExtendedStatusCallBack ( ZPS_teExtendedStatus    eExtendedStatus )
{
    vLog_Printf ( TRACE_EXC,LOG_DEBUG, "ERROR: Extended status %x\n", eExtendedStatus );
}

#if (defined PDM_EEPROM)
#if TRACE_APPSTART
/****************************************************************************
 *
 * NAME: vPdmEventHandlerCallback
 *
 * DESCRIPTION:
 * Handles PDM callback, information the application of PDM conditions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vPdmEventHandlerCallback ( uint32                  u32EventNumber,
                                        PDM_eSystemEventCode    eSystemEventCode )
{

    switch ( eSystemEventCode )
    {
        /*
         * The next three events will require the application to take some action
         */
        case E_PDM_SYSTEM_EVENT_WEAR_COUNT_TRIGGER_VALUE_REACHED:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG, "PDM: Segment %d reached trigger wear level\n", u32EventNumber);
            break;
        case E_PDM_SYSTEM_EVENT_DESCRIPTOR_SAVE_FAILED:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Record Id %d failed to save\n", u32EventNumber);
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Capacity %d\n", u8PDM_CalculateFileSystemCapacity() );
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Occupancy %d\n", u8PDM_GetFileSystemOccupancy() );
            break;
        case E_PDM_SYSTEM_EVENT_PDM_NOT_ENOUGH_SPACE:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Record %d not enough space\n", u32EventNumber);
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Capacity %d\n", u8PDM_CalculateFileSystemCapacity() );
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Occupancy %d\n", u8PDM_GetFileSystemOccupancy() );
            break;

        /*
         *  The following events are really for information only
         */
        case E_PDM_SYSTEM_EVENT_EEPROM_SEGMENT_HEADER_REPAIRED:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG, "PDM: Segment %d header repaired\n", u32EventNumber);
            break;
        case E_PDM_SYSTEM_EVENT_SYSTEM_INTERNAL_BUFFER_WEAR_COUNT_SWAP:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Segment %d buffer wear count swap\n", u32EventNumber);
            break;
        case E_PDM_SYSTEM_EVENT_SYSTEM_DUPLICATE_FILE_SEGMENT_DETECTED:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Segement %d duplicate selected\n", u32EventNumber);
            break;
        default:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Unexpected call back Code %d Number %d\n", eSystemEventCode, u32EventNumber);
            break;
    }
}
#endif
#endif

PRIVATE void APP_cbTimerZclTick (void*    pvParam)
{
    tsZCL_CallBackEvent    sCallBackEvent;
//    vLog_Printf ( TRACE_EXC, LOG_DEBUG, "\nE_ZCL_CBET_TIMER" );
    sCallBackEvent.eEventType =  E_ZCL_CBET_TIMER;
    vZCL_EventHandler ( &sCallBackEvent );
    ZTIMER_eStart ( u8TickTimer, ZCL_TICK_TIME );
}

bool_t APP_vProfileWideCommandSupportedForCluster ( uint16 u16Clusterid )
{
    if ( u16Clusterid == MEASUREMENT_AND_SENSING_CLUSTER_ID_OCCUPANCY_SENSING)
    {
        return TRUE;
    }
    return FALSE;
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

//forrest
PUBLIC void App_DeviceAnnounceTimeout (void*   pvParam )
{
   ZPS_teStatus estatus;
   int temp;
   temp = *( ( uint8* ) pvParam );
   temp = (temp - 1)/2;
   
   DBG_vPrintf (TRUE, "\n-----App_DeviceAnnounceTimeout : TimeOut 0x%x 0x%x\n",temp,iDeviceAnnounceAction[temp].u16ShortAddress);
   if( iDeviceAnnounceAction[temp].bStatus != FALSE)
   {
   		iDeviceAnnounceAction[temp].eState = LeTV_NONE;
   		iDeviceAnnounceAction[temp].bStatus = FALSE;
		DBG_vPrintf (TRUE, "\n-----App_DeviceAnnounceTimeout : Clean 0x%x 0x%x\n",temp,iDeviceAnnounceAction[temp].u16ShortAddress);
   }
   ZTIMER_eStop(iDeviceAnnounceAction[temp].u8Timeout);
}

PUBLIC void App_DeviceAnnounceCallback( void*   pvParam )
{
   ZPS_teStatus estatus;
   int temp;
   temp = *( ( uint8* ) pvParam );
   temp = (temp)/2;
   //vLog_Printf ( TRUE,LOG_DEBUG, "\n-----App_ActiveEndpointCallback : 0x%x CALLBACK\n",*u8Instance);
   //vSL_LogFlush ( );
//   DBG_vPrintf(TRUE,"\n----- App_DeviceAnnounceCallback : %d 0x%x CALLBACK\n",*u8Instance,DeviceAnnounceAction[*u8Instance].u16ShortAddress);
	
   //DBG_vPrintf (TRUE, "\n-----App_DeviceAnnounceCallback : 0x%x 0x%x CALLBACK\n",*u8Instance,iDeviceAnnounceAction[*u8Instance].u16ShortAddress);
   switch(iDeviceAnnounceAction[temp].eState)
   {
   	 case LeTV_TRANSPORT_KEY:
	 		//DBG_vPrintf (TRUE, "\n-----SendActiveEndpointsRequest : 0x%x CALLBACK\n",u16ActiveEndpointReqAddr );
			//vLog_Printf ( TRUE,LOG_DEBUG, "\n-----SendActiveEndpointsRequest : 0x%x CALLBACK\n",iDeviceAnnounceAction[*u8Instance].u16ShortAddress );
			estatus = SendActiveEndpointsRequest(iDeviceAnnounceAction[temp].u16ShortAddress);

   			DBG_vPrintf (TRUE, "\n-----SendActiveEndpointsRequest : 0x%x 0x%x\n",temp,iDeviceAnnounceAction[temp].u16ShortAddress);
   			 //vLog_Printf (TRUE,LOG_DEBUG, "\n-----status : 0x%x \n",estatus );
     	  break;

	 case LeTV_ACTIVE_ENDPOINT_RSP:
#if 1
	 		iDeviceAnnounceAction[temp].sEndpoint.u8SimpleReqSent = 0;
		 	DBG_vPrintf (TRUE, "\n-----SendSimpleDescriptorRequest : 0x%x 0x%x CALLBACK\n",iDeviceAnnounceAction[temp].u16ShortAddress,iDeviceAnnounceAction[temp].sEndpoint.u8SimpleReqSent);
			estatus = SendSimpleDescriptorRequest(iDeviceAnnounceAction[temp].u16ShortAddress,iDeviceAnnounceAction[temp].sEndpoint.u8EndpointNumber[0]);

			iDeviceAnnounceAction[temp].sEndpoint.u8SimpleReqSent++;
			//DBG_vPrintf (TRUE, "\n-----status : 0x%x \n",estatus );
#endif
		  break;

	case LeTV_SIMPLE_DESCRIPTOR_RSP:
#if 1
		    if(iDeviceAnnounceAction[temp].sEndpoint.u8SimpleReqSent < iDeviceAnnounceAction[temp].sEndpoint.u8NumOfSize)
		  	{
		 		//DBG_vPrintf (TRUE, "\n-----SendSimpleDescriptorRequest : 0x%x 0x%x CALLBACK\n",iDeviceAnnounceAction[temp].u16ShortAddress,iDeviceAnnounceAction[temp].sEndpoint.u8SimpleReqSent);
   				estatus = SendSimpleDescriptorRequest(iDeviceAnnounceAction[temp].u16ShortAddress,iDeviceAnnounceAction[temp].sEndpoint.u8EndpointNumber[iDeviceAnnounceAction[temp].sEndpoint.u8SimpleReqSent]);

				iDeviceAnnounceAction[temp].sEndpoint.u8SimpleReqSent++;
				//DBG_vPrintf (TRUE, "\n-----status : 0x%x \n",estatus );
		  	}
			else if(iDeviceAnnounceAction[temp].sEndpoint.u8SimpleReqSent == iDeviceAnnounceAction[temp].sEndpoint.u8NumOfSize)
			{
				//DBG_vPrintf (TRUE, "\n-----SendNodeDescRequest : 0x%x CALLBACK\n",iDeviceAnnounceAction[temp].u16ShortAddress );
   				estatus = SendNodeDescRequest(iDeviceAnnounceAction[temp].u16ShortAddress);

   				//DBG_vPrintf (TRUE, "\n-----status : 0x%x \n",estatus );
			}
#endif
		  break;
	case LeTV_NODE_DESCRIPTOR_RSP:
		 iDeviceAnnounceAction[temp].eState = LeTV_NONE;
		 iDeviceAnnounceAction[temp].bStatus = FALSE;
		 ZTIMER_eStop(iDeviceAnnounceAction[temp].u8Timer);
		 //DBG_vPrintf (TRUE, "\n-----ZTIMER_eStop : 0x%x CALLBACK\n",iDeviceAnnounceAction[*u8Instance].u16ShortAddress );
		 DBG_vPrintf (TRUE, "\n-----App_DeviceAnnounceCallback ZTIMER_eStop : 0x%x 0x%x 0x%x CALLBACK\n",temp,iDeviceAnnounceAction[temp].u8Timer,iDeviceAnnounceAction[temp].u16ShortAddress);
		 break;
   	}
}
//forrest
