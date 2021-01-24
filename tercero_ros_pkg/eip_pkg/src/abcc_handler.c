/*******************************************************************************
********************************************************************************
**                                                                            **
** ABCC Driver version 4.01.01 (2015-12-14)                                   **
**                                                                            **
** Delivered with:                                                            **
**    ABP         7.16.01 (2015-10-14)                                        **
**                                                                            */
/*******************************************************************************
********************************************************************************
** COPYRIGHT NOTIFICATION (c) 2013 HMS Industrial Networks AB                 **
**                                                                            **
** This code is the property of HMS Industrial Networks AB.                   **
** The source code may not be reproduced, distributed, or used without        **
** permission. When used together with a product from HMS, permission is      **
** granted to modify, reproduce and distribute the code in binary form        **
** without any restrictions.                                                  **
**                                                                            **
** THE CODE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND. HMS DOES NOT    **
** WARRANT THAT THE FUNCTIONS OF THE CODE WILL MEET YOUR REQUIREMENTS, OR     **
** THAT THE OPERATION OF THE CODE WILL BE UNINTERRUPTED OR ERROR-FREE, OR     **
** THAT DEFECTS IN IT CAN BE CORRECTED.                                       **
********************************************************************************
********************************************************************************
** File Description:
** Implementation of operation mode independent parts of the abcc handler.
********************************************************************************
********************************************************************************
*/

#include "abcc_drv_cfg.h"
#include "abcc_td.h"
#include "abcc_drv_if.h"
#include "abp.h"
#include "abcc.h"
#include "abcc_link.h"
#include "abcc_mem.h"
#include "abcc_sys_adapt.h"
#include "abcc_debug_err.h"
#include "abcc_handler.h"
#include "abcc_timer.h"
#include "abcc_setup.h"
#include "abcc_port.h"

#include "IDLTp.h"

#include "abcc_drv_par_if.h"

/*
** Registerd handler functions
*/
EXTFUNC ABCC_ErrorCodeType ABCC_ParRunDriver( IDL_CTRL_HDL hCtrl );
EXTFUNC void ABCC_ParISR( IDL_CTRL_HDL hCtrl );

/*
 ** Default USER defines
 */
#ifndef ABCC_CFG_STARTUP_TIME_MS
#define ABCC_CFG_STARTUP_TIME_MS ( 1500 )
#endif

/********************************************************************************
** Public Globals
********************************************************************************
*/

/*
** Registered handler functions
*/

ABCC_ErrorCodeType ( *ABCC_RunDriver )( IDL_CTRL_HDL hCtrl );
void ( *ABCC_ISR )( IDL_CTRL_HDL hCtrl );
void ( *ABCC_TriggerWrPdUpdate )( IDL_CTRL_HDL hCtrl );

/*
** The interrupt mask that has been set to the ABCC at start up.
*/
UINT16 ABCC_iInterruptEnableMask;

/*
** Registerd driver functions
*/

void  ( *pnABCC_DrvInit )( IDL_CTRL_HDL hCtrl, UINT8 bOpmode );
UINT16 ( *pnABCC_DrvISR )( IDL_CTRL_HDL hCtrl );
void ( *pnABCC_DrvRunDriverTx )( IDL_CTRL_HDL hCtrl );
ABP_MsgType* ( *pnABCC_DrvRunDriverRx )( IDL_CTRL_HDL hCtrl );
BOOL ( *pnABCC_DrvWriteMessage) ( IDL_CTRL_HDL hCtrl, ABP_MsgType* psWriteMsg );
void ( *pnABCC_DrvWriteProcessData )( IDL_CTRL_HDL hCtrl, void* pbProcessData );
BOOL ( *pnABCC_DrvISReadyForWrPd )( IDL_CTRL_HDL hCtrl );
BOOL ( *pnABCC_DrvISReadyForWriteMessage )( IDL_CTRL_HDL hCtrl );
BOOL ( *pnABCC_DrvISReadyForCmd )( IDL_CTRL_HDL hCtrl );
void ( *pnABCC_DrvSetNbrOfCmds )( IDL_CTRL_HDL hCtrl, UINT8 bNbrOfCmds );
void ( *pnABCC_DrvSetAppStatus )( IDL_CTRL_HDL hCtrl, ABP_AppStatusType eAppStatus );
void ( *pnABCC_DrvSetPdSize )( IDL_CTRL_HDL hCtrl, const UINT16 iReadPdSize, const UINT16 iWritePdSize );
void ( *pnABCC_DrvSetMsgReceiverBuffer )( IDL_CTRL_HDL hCtrl, ABP_MsgType* const psReadMsg );
void ( *pnABCC_DrvSetIntMask )( IDL_CTRL_HDL hCtrl, const UINT16 iIntMask );
void* ( *pnABCC_DrvGetWrPdBuffer )( IDL_CTRL_HDL hCtrl );
UINT16 ( *pnABCC_DrvGetModCap )( IDL_CTRL_HDL hCtrl );
UINT16 ( *pnABCC_DrvGetLedStatus )( IDL_CTRL_HDL hCtrl );
UINT16 ( *pnABCC_DrvGetIntStatus )( IDL_CTRL_HDL hCtrl );
UINT8 ( *pnABCC_DrvGetAnybusState )( IDL_CTRL_HDL hCtrl );
void* ( *pnABCC_DrvReadProcessData )( IDL_CTRL_HDL hCtrl );
ABP_MsgType* ( *pnABCC_DrvReadMessage )( IDL_CTRL_HDL hCtrl );
BOOL ( *pnABCC_DrvIsSupervised )( IDL_CTRL_HDL hCtrl );
UINT8 ( *pnABCC_DrvGetAnbStatus )( IDL_CTRL_HDL hCtrl );

#if( ABCC_CFG_SYNC_MEASUREMENT_IP )
BOOL fAbccUserSyncMeasurementIp[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];
#endif

/*******************************************************************************
** Private Globals
********************************************************************************
*/

static volatile UINT8 abcc_bAnbState[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{0xff}};

static ABCC_MainStateType abcc_eMainState[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{ABCC_DRV_INIT}};

/*
** Pointer to WRPD buffer.
*/
static void* abcc_pbWrPdBuffer[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];

/*
 ** Tmo handler for
 */
static ABCC_TimerHandle abcc_TmoHandle[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];

/*
 ** Indicate ready for communication
 */
static BOOL abcc_fReadyForCommunicationTmo[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{FALSE}};
static BOOL abcc_fReadyForCommunication[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{FALSE}};

/*
 ** Current operation mode
 */
static UINT8 abcc_bOpmode[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{0}};

#if( ABCC_CFG_DRV_SPI || ABCC_CFG_DRV_PARALLEL_30 || ABCC_CFG_DRV_SERIAL )
/*
 ** Flag to indicate that WrPD update shall be done
 */
static BOOL abcc_fDoWrPdUpdate[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{FALSE}};
#endif

/*
** The Application status register value of the Anybus module
*/
static volatile ABP_AppStatusType abcc_eAppStatus[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{ABP_APPSTAT_NO_ERROR}};

/*******************************************************************************
** Private Services
********************************************************************************
*/

static void TriggerWrPdUpdateNow( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   if( ABCC_GetMainState(hCtrl) == ABCC_DRV_RUNNING )
   {
      /*
      ** Send new "write process data" to the Anybus-CC.
      ** The data format of the process data is network specific.
      ** The application converts the data accordingly.
      */
      if( pnABCC_DrvISReadyForWrPd(hCtrl) )
      {
         if( ABCC_CbfUpdateWriteProcessData( hCtrl, abcc_pbWrPdBuffer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ) )
         {
            pnABCC_DrvWriteProcessData( hCtrl, abcc_pbWrPdBuffer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
#if( ABCC_CFG_SYNC_MEASUREMENT_IP )
            if( ABCC_GetOpmode(hCtrl) == ABP_OP_MODE_SPI )
            {
               fAbccUserSyncMeasurementIp[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = TRUE;
            }
            else
            {
               ABCC_SYS_GpioReset(hCtrl);
            }
#endif
         }
      }
   }
}

#if( ABCC_CFG_DRV_SPI || ABCC_CFG_DRV_PARALLEL_30 || ABCC_CFG_DRV_SERIAL )
static void TriggerWrPdUpdateLater( void )
{
   abcc_fDoWrPdUpdate[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = TRUE;
}
#endif

static BOOL IsInterruptInUse( IDL_CTRL_HDL hCtrl )
{
   BOOL fReturn;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   fReturn = FALSE;
#if( ABCC_CFG_INT_ENABLED )
   switch( abcc_bOpmode[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] )
   {
   case ABP_OP_MODE_16_BIT_PARALLEL:
   case ABP_OP_MODE_8_BIT_PARALLEL:
   case ABP_OP_MODE_SPI:
      fReturn = TRUE;
      break;

   default:
      break;
   }

   return( fReturn );
#else
   return( fReturn );
#endif /* End of #if ABCC_CFG_INT_ENABLED */
}

static BOOL IsPolledInterruptInUse( IDL_CTRL_HDL hCtrl )
{
   BOOL fReturn;

   fReturn = FALSE;
#if( ABCC_CFG_POLL_ABCC_IRQ_PIN )
   switch( abcc_bOpmode[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] )
   {
   case ABP_OP_MODE_16_BIT_PARALLEL:
   case ABP_OP_MODE_8_BIT_PARALLEL:
   case ABP_OP_MODE_SPI:
      fReturn = TRUE;
      break;

   default:

      break;
   }

   return( fReturn );
#else
   return( fReturn );
#endif /* End of #if ABCC_CFG_POLL_ABCC_IRQ_PIN */
}

static void SetReadyForCommunicationTmo( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   abcc_fReadyForCommunicationTmo[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = TRUE;
}

/*******************************************************************************
** Public Service
********************************************************************************
*/

#if( ABCC_CFG_DRV_SPI || ABCC_CFG_DRV_PARALLEL_30 || ABCC_CFG_DRV_SERIAL )
void ABCC_CheckWrPdUpdate( IDL_CTRL_HDL hCtrl )
{
   if( abcc_fDoWrPdUpdate )
   {
      abcc_fDoWrPdUpdate = FALSE;
      TriggerWrPdUpdateNow(hCtrl);
   }
}
#endif

void ABCC_SetReadyForCommunication( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   ABCC_PORT_UseCritical(hCtrl);
   ABCC_PORT_EnterCritical(hCtrl);
   abcc_fReadyForCommunication[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = TRUE;
   ABCC_PORT_ExitCritical(hCtrl);
}

void ABCC_SetMainStateError( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_DRV_ERROR;
}

ABCC_MainStateType ABCC_GetMainState( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   return( abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
}

void ABCC_TriggerAnbStatusUpdate( IDL_CTRL_HDL hCtrl )
{
   UINT8 bAnbState;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   bAnbState = pnABCC_DrvGetAnybusState(hCtrl);
   if( bAnbState != abcc_bAnbState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] )
   {
      abcc_bAnbState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = bAnbState;
      ABCC_CbfAnbStateChanged( hCtrl, (ABP_AnbStateType)bAnbState );
   }
}

void ABCC_TriggerTransmitMessage( IDL_CTRL_HDL hCtrl )
{
   ABCC_LinkCheckSendMessage(hCtrl);
}

#if ( ABCC_CFG_SYNC_MEASUREMENT_OP || ABCC_CFG_SYNC_MEASUREMENT_IP )
void ABCC_GpioReset( IDL_CTRL_HDL hCtrl )
{
   ABCC_SYS_GpioReset(hCtrl);
}
#endif

#if ( ABCC_CFG_SYNC_MEASUREMENT_OP || ABCC_CFG_SYNC_MEASUREMENT_IP )
void ABCC_GpioSet( IDL_CTRL_HDL hCtrl )
{
   ABCC_SYS_GpioSet(hCtrl);
}
#endif

ABCC_ErrorCodeType ABCC_HwInit( IDL_CTRL_HDL hCtrl )
{
   if( !ABCC_SYS_HwInit(hCtrl) )
   {
      return( ABCC_EC_HW_INIT_FAILED );
   }
   return( ABCC_EC_NO_ERROR );
}


ABCC_ErrorCodeType ABCC_StartDriver( IDL_CTRL_HDL hCtrl, UINT32 lMaxStartupTimeMs )
{
   UINT8 bModuleId;
   UINT8 bOpmode;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   if( lMaxStartupTimeMs == 0 )
   {
      lMaxStartupTimeMs = ABCC_CFG_STARTUP_TIME_MS;
   }

   SetErrorReporter( ABCC_CbfDriverError );

   bModuleId = ABCC_ReadModuleId(hCtrl);

#if( ABCC_CFG_DRV_SERIAL || ABCC_CFG_DRV_PARALLEL_30 )
   if( ( bModuleId != ABP_MODULE_ID_ACTIVE_ABCC40 ) && ( bModuleId != ABP_MODULE_ID_ACTIVE_ABCC30 ) )
#elif( ABCC_CFG_DRV_SPI || ABCC_CFG_DRV_PARALLEL )
   if( bModuleId != ABP_MODULE_ID_ACTIVE_ABCC40 )
#endif
   {
      ABCC_ERROR( hCtrl, ABCC_SEV_FATAL, ABCC_EC_MODULE_ID_NOT_SUPPORTED, (UINT32)bModuleId );

      return( ABCC_EC_MODULE_ID_NOT_SUPPORTED );
   }

   bOpmode = ABCC_GetOpmode(hCtrl);
   abcc_bOpmode[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = bOpmode;

   switch( bOpmode )
   {
#if( ABCC_CFG_DRV_PARALLEL || ABCC_CFG_DRV_PARALLEL_30 )
   case ABP_OP_MODE_8_BIT_PARALLEL:
#if( ABCC_CFG_DRV_PARALLEL )
      if( bModuleId == ABP_MODULE_ID_ACTIVE_ABCC30 )
#endif /* End of #if ABCC_CFG_DRV_PARALLEL */
      {
#if( ABCC_CFG_DRV_PARALLEL_30 )
         ABCC_ISR                   = &ABCC_Par30ISR;
         ABCC_RunDriver             = &ABCC_Par30RunDriver;
         ABCC_TriggerWrPdUpdate     = &TriggerWrPdUpdateLater;

         pnABCC_DrvInit               = &ABCC_DrvPar30Init;
         pnABCC_DrvISR                = &ABCC_DrvPar30ISR;
         pnABCC_DrvRunDriverTx        = &ABCC_DrvPar30RunDriverTx;;
         pnABCC_DrvRunDriverRx        = &ABCC_DrvPar30RunDriverRx;
         pnABCC_DrvWriteMessage       = &ABCC_DrvPar30WriteMessage;
         pnABCC_DrvWriteProcessData   = &ABCC_DrvPar30WriteProcessData;
         pnABCC_DrvISReadyForWrPd     = &ABCC_DrvPar30IsReadyForWrPd;
         pnABCC_DrvISReadyForWriteMessage = &ABCC_DrvPar30IsReadyForWriteMessage;
         pnABCC_DrvISReadyForCmd      = &ABCC_DrvPar30IsReadyForCmd;
         pnABCC_DrvSetNbrOfCmds       = &ABCC_DrvPar30SetNbrOfCmds;
         pnABCC_DrvSetAppStatus       = &ABCC_DrvPar30SetAppStatus;
         pnABCC_DrvSetPdSize          = &ABCC_DrvPar30SetPdSize;
         pnABCC_DrvSetIntMask         = &ABCC_DrvPar30SetIntMask;
         pnABCC_DrvGetWrPdBuffer      = &ABCC_DrvPar30GetWrPdBuffer;
         pnABCC_DrvGetModCap          = &ABCC_DrvPar30GetModCap;
         pnABCC_DrvGetLedStatus       = &ABCC_DrvPar30GetLedStatus;
         pnABCC_DrvGetIntStatus       = &ABCC_DrvPar30GetIntStatus;
         pnABCC_DrvGetAnybusState     = &ABCC_DrvPar30GetAnybusState;
         pnABCC_DrvReadProcessData    = &ABCC_DrvPar30ReadProcessData;
         pnABCC_DrvReadMessage        = &ABCC_DrvPar30ReadMessage;
         pnABCC_DrvIsSupervised       = &ABCC_DrvPar30IsSupervised;
         pnABCC_DrvGetAnbStatus       = &ABCC_DrvPar30GetAnbStatus;

         ABCC_iInterruptEnableMask = ABCC_CFG_INT_ENABLE_MASK_PAR30;
#else
         ABCC_ERROR( hCtrl, ABCC_SEV_FATAL, ABCC_EC_INCORRECT_OPERATING_MODE, (UINT32)bOpmode );
#endif /* End of #if ABCC_CFG_DRV_PARALLEL_30 */
         break;
      }

      /*
      ** If event driven parallel operating mode is enabled and an ABCC 40
      ** module is mounted fall through to the 16-bit parallel operating mode
      ** case which sets up the event driven parallel operating mode.
      */
#endif /* End of #if ABCC_CFG_DRV_PARALLEL or ABCC_CFG_DRV_PARALLEL_30 */
#if( ABCC_CFG_DRV_PARALLEL )
   case ABP_OP_MODE_16_BIT_PARALLEL:

      if( bModuleId == ABP_MODULE_ID_ACTIVE_ABCC30 )
      {
         ABCC_ERROR( hCtrl, ABCC_SEV_FATAL, ABCC_EC_INCORRECT_OPERATING_MODE, (UINT32)bOpmode );

         break;
      }

      ABCC_ISR                   = &ABCC_ParISR;
      ABCC_RunDriver             = &ABCC_ParRunDriver;
      ABCC_TriggerWrPdUpdate     = &TriggerWrPdUpdateNow;

      pnABCC_DrvInit               = &ABCC_DrvParInit;
      pnABCC_DrvISR                = &ABCC_DrvParISR;
      pnABCC_DrvRunDriverTx        = NULL;
      pnABCC_DrvRunDriverRx        = &ABCC_DrvParRunDriverRx;
      pnABCC_DrvWriteMessage       = &ABCC_DrvParWriteMessage;
      pnABCC_DrvWriteProcessData   = &ABCC_DrvParWriteProcessData;
      pnABCC_DrvISReadyForWrPd     = &ABCC_DrvParIsReadyForWrPd;
      pnABCC_DrvISReadyForWriteMessage = &ABCC_DrvParIsReadyForWriteMessage;
      pnABCC_DrvISReadyForCmd      = &ABCC_DrvParIsReadyForCmd;
      pnABCC_DrvSetNbrOfCmds       = &ABCC_DrvParSetNbrOfCmds;
      pnABCC_DrvSetAppStatus       = &ABCC_DrvParSetAppStatus;
      pnABCC_DrvSetPdSize          = &ABCC_DrvParSetPdSize;
      pnABCC_DrvSetIntMask         = &ABCC_DrvParSetIntMask;
      pnABCC_DrvGetWrPdBuffer      = &ABCC_DrvParGetWrPdBuffer;
      pnABCC_DrvGetModCap          = &ABCC_DrvParGetModCap;
      pnABCC_DrvGetLedStatus       = &ABCC_DrvParGetLedStatus;
      pnABCC_DrvGetIntStatus       = &ABCC_DrvParGetIntStatus;
      pnABCC_DrvGetAnybusState     = &ABCC_DrvParGetAnybusState;
      pnABCC_DrvReadProcessData    = &ABCC_DrvParReadProcessData;
      pnABCC_DrvReadMessage        = &ABCC_DrvParReadMessage;
      pnABCC_DrvIsSupervised       = &ABCC_DrvParIsSupervised;
      pnABCC_DrvGetAnbStatus       = &ABCC_DrvParGetAnbStatus;

#if ABCC_CFG_INT_ENABLED
      ABCC_iInterruptEnableMask = ABCC_CFG_INT_ENABLE_MASK_PAR;

#if ABCC_CFG_SYNC_ENABLE && !ABCC_CFG_USE_ABCC_SYNC_SIGNAL
      ABCC_iInterruptEnableMask |= ABP_INTMASK_SYNCIEN;
#endif
#else
      ABCC_iInterruptEnableMask = 0;
#endif

      break;
#endif /* End of #if ABCC_CFG_DRV_PARALLEL */
   default:

      ABCC_ERROR( hCtrl, ABCC_SEV_FATAL, ABCC_EC_INCORRECT_OPERATING_MODE, (UINT32)bOpmode );

      return( ABCC_EC_INCORRECT_OPERATING_MODE );
   }

   if ( !( ( abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == ABCC_DRV_INIT )  ||
           ( abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == ABCC_DRV_SHUTDOWN ) ) )
   {
      ABCC_ERROR(hCtrl, ABCC_SEV_FATAL, ABCC_EC_INCORRECT_STATE, (UINT32)abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
      abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_DRV_ERROR;

      return( ABCC_EC_INCORRECT_STATE );
   }

   if ( !ABCC_SYS_Init(hCtrl) )
   {
      return( ABCC_EC_INTERNAL_ERROR );
   }

   ABCC_TimerInit(hCtrl);
   pnABCC_DrvInit( hCtrl, bOpmode );

   ABCC_LinkInit(hCtrl);
   ABCC_SetupInit(hCtrl);

   abcc_bAnbState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = 0xff;

   abcc_TmoHandle[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_TimerCreate( hCtrl, SetReadyForCommunicationTmo );

   abcc_pbWrPdBuffer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = pnABCC_DrvGetWrPdBuffer(hCtrl);

   if( !ABCC_ModuleDetect(hCtrl) )
   {
      ABCC_ERROR(hCtrl, ABCC_SEV_WARNING, ABCC_EC_MODULE_NOT_DECTECTED, 0);

      return( ABCC_EC_MODULE_NOT_DECTECTED );
   }

#if( ABCC_CFG_OP_MODE_SETTABLE )
   ABCC_SYS_SetOpmode( hCtrl, bOpmode );
#endif

   abcc_fReadyForCommunicationTmo[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = FALSE;
   abcc_fReadyForCommunication[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = FALSE;

#if( ABCC_CFG_SYNC_ENABLE && ABCC_CFG_USE_ABCC_SYNC_SIGNAL )
   ABCC_SYS_SyncInterruptEnable(hCtrl);
#endif

#if( ABCC_CFG_INT_ENABLED )
   if( IsInterruptInUse(hCtrl) )
   {
         ABCC_SYS_AbccInterruptEnable(hCtrl);
   }
#endif /* End of #if ABCC_CFG_INT_ENABLED */

   ABCC_PORT_UseCritical(hCtrl);
   ABCC_PORT_EnterCritical(hCtrl);
   abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_DRV_WAIT_COMMUNICATION_RDY;
   ABCC_PORT_ExitCritical(hCtrl);

   ABCC_TimerStart( hCtrl, abcc_TmoHandle[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex], lMaxStartupTimeMs );

   return( ABCC_EC_NO_ERROR );
}

ABCC_CommunicationStateType ABCC_isReadyForCommunication( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   if( abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] > ABCC_DRV_WAIT_COMMUNICATION_RDY )
   {
      return( ABCC_READY_FOR_COMMUNICATION );
   }

   if( abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] < ABCC_DRV_WAIT_COMMUNICATION_RDY )
   {
      return( ABCC_NOT_READY_FOR_COMMUNICATION );
   }

   if( abcc_fReadyForCommunicationTmo[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == TRUE )
   {
      if( IsInterruptInUse(hCtrl) || IsPolledInterruptInUse(hCtrl) )
      {
         return( ABCC_COMMUNICATION_ERROR );
      }
      else
      {
         abcc_fReadyForCommunication[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = TRUE;
      }
   }

#if( !ABCC_CFG_INT_ENABLED ) && ( ABCC_CFG_POLL_ABCC_IRQ_PIN )
   if( IsPolledInterruptInUse(hCtrl) )
   {
      abcc_fReadyForCommunication[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_SYS_IsAbccInterruptActive(hCtrl);
   }
#endif

   ABCC_PORT_UseCritical(hCtrl);
   ABCC_PORT_EnterCritical(hCtrl);
   if( abcc_fReadyForCommunication[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == TRUE )
   {
      pnABCC_DrvSetIntMask( hCtrl, ABCC_iInterruptEnableMask );
      abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_DRV_SETUP;
      pnABCC_DrvSetNbrOfCmds( hCtrl, ABCC_CFG_MAX_NUM_APPL_CMDS );

      ABCC_PORT_ExitCritical(hCtrl);
      ABCC_SetupCommands(hCtrl);
      return( ABCC_READY_FOR_COMMUNICATION );
   }
   ABCC_PORT_ExitCritical(hCtrl);

   return( ABCC_NOT_READY_FOR_COMMUNICATION );
}


void ABCC_NewWrPdEvent( IDL_CTRL_HDL hCtrl )
{
   if( ABCC_GetMainState(hCtrl) == ABCC_DRV_RUNNING )
   {
      /*
      ** Send new "write process data" to the Anybus-CC.
      ** The data format of the process data is network specific.
      ** The application converts the data accordingly.
      */
      if ( pnABCC_DrvISReadyForWrPd(hCtrl) )
      {
         if( ABCC_CbfUpdateWriteProcessData( hCtrl, abcc_pbWrPdBuffer ) )
         {
            pnABCC_DrvWriteProcessData( hCtrl, abcc_pbWrPdBuffer );
#if( ABCC_CFG_SYNC_MEASUREMENT_IP )
            if( ABCC_GetOpmode(hCtrl) == ABP_OP_MODE_SPI )
            {
               fAbccUserSyncMeasurementIp[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = TRUE;
            }
            else
            {
               ABCC_SYS_GpioReset(hCtrl);
            }
#endif
         }
      }
   }
}



void ABCC_TriggerRdPdUpdate( IDL_CTRL_HDL hCtrl )
{
   void* bpRdPd;

#if( ABCC_CFG_SYNC_MEASUREMENT_OP )
   ABCC_SYS_GpioSet(hCtrl);
#endif

   bpRdPd = pnABCC_DrvReadProcessData(hCtrl);

   if( bpRdPd )
   {
      if( pnABCC_DrvGetAnybusState(hCtrl) == ABP_ANB_STATE_PROCESS_ACTIVE  )
      {
         /*
         ** The "read process data" is only valid in the PROCESS_ACTIVE state.
         ** Retrieve the new "read process data" from the Anybus-CC.
         ** The data format of the process data is network specific.
         ** Convert it to our native format.
         */
         ABCC_CbfNewReadPd( hCtrl, bpRdPd );
      }
   }

#if( ABCC_CFG_SYNC_MEASUREMENT_OP )
   /*
   ** This is the Output Valid point (for OuputValidTime = 0). The
   ** applications has received data and handled it. Thus we reset the
   ** ABCC_CFG_SYNC_MEASUREMENT_OP measurement.
   */
   ABCC_SYS_GpioReset(hCtrl);
#endif
}

void ABCC_TriggerReceiveMessage ( IDL_CTRL_HDL hCtrl )
{
   ABCC_MsgType sRdMsg;
   ABCC_MemBufferStatusType eBufferStatus;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   sRdMsg.psMsg = ABCC_LinkReadMessage(hCtrl);

   if( sRdMsg.psMsg == NULL )
   {
      return;
   }

   ABCC_DEBUG_MSG_DATA( "Msg received", sRdMsg.psMsg );

   eBufferStatus = ABCC_MemGetBufferStatus( hCtrl, sRdMsg.psMsg );

   /*
   ** A new message is available.
   */
   if( ABCC_GetLowAddrOct( sRdMsg.psMsg16->sHeader.iCmdReserved ) & ABP_MSG_HEADER_C_BIT )
   {
      /*
      ** The message is a command, let the application respond.
      */
      ABCC_CbfReceiveMsg( hCtrl, sRdMsg.psMsg );
   }
   else
   {
      if( abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == ABCC_DRV_SETUP )
      {
         /*
         ** The message is a reponse to a setup command we have sent.
         */
         ABCC_DEBUG_MSG_EVENT( "Routing response to setup state machine",
                               sRdMsg.psMsg );
         ABCC_SetupResponses( hCtrl, sRdMsg.psMsg );

         if ( ABCC_SetupCommands(hCtrl) )
         {
            abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_DRV_RUNNING;
         }
      }
      else
      {
         ABCC_MsgHandlerFuncType pnMsgHandler = 0;
         pnMsgHandler = ABCC_LinkGetMsgHandler( hCtrl, ABCC_GetLowAddrOct( sRdMsg.psMsg16->sHeader.iSourceIdDestObj ) );

         if( pnMsgHandler )
         {
            ABCC_DEBUG_MSG_EVENT( "Routing response to registered response handler", sRdMsg.psMsg );
            pnMsgHandler( hCtrl, sRdMsg.psMsg );
         }
         else
         {
            ABCC_DEBUG_MSG_EVENT( "No response handler found", sRdMsg.psMsg );
            ABCC_CbfReceiveMsg( hCtrl, sRdMsg.psMsg );
         }
      }
   }

   if( ABCC_MemGetBufferStatus( hCtrl, sRdMsg.psMsg ) == eBufferStatus )
   {
      /*
      ** The status has not been changed while the user processed the response
      ** message. Then this buffer shall be freed by the driver.
      */
      ABCC_ReturnMsgBuffer( hCtrl, &sRdMsg.psMsg );
   }
}

ABCC_ErrorCodeType ABCC_SendCmdMsg( IDL_CTRL_HDL hCtrl, ABP_MsgType*  psCmdMsg, ABCC_MsgHandlerFuncType pnMsgHandler )
{
   ABCC_ErrorCodeType eResult;
   ABCC_MsgType sMsg;

   sMsg.psMsg = psCmdMsg;

   /*
   ** Register function to handle response.
   ** Must be done before sending the message to avoid race condition.
   */
   if( ABCC_LinkMapMsgHandler( hCtrl, ABCC_GetLowAddrOct( sMsg.psMsg16->sHeader.iSourceIdDestObj ),
                               pnMsgHandler ) == ABCC_EC_NO_ERROR )
   {
      eResult = ABCC_LinkWriteMessage( hCtrl, sMsg.psMsg );
      if( eResult != ABCC_EC_NO_ERROR )
      {
         /*
         ** Free message handler resource
         */
         (void)ABCC_LinkGetMsgHandler( hCtrl, ABCC_GetLowAddrOct( sMsg.psMsg16->sHeader.iSourceIdDestObj ) );
      }
   }
   else
   {
      eResult = ABCC_EC_NO_RESOURCES;

      /*
      ** Report error
      */
      ABCC_ASSERT( hCtrl, FALSE );
   }

   return( eResult );
}

UINT16 ABCC_GetCmdQueueSize( IDL_CTRL_HDL hCtrl )
{
   return( ABCC_LinkGetNumCmdQueueEntries(hCtrl) );
}


ABCC_ErrorCodeType ABCC_SendRespMsg( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsgResp )
{
   return( ABCC_LinkWriteMessage( hCtrl, psMsgResp ) );
}

ABP_MsgType* ABCC_GetCmdMsgBuffer( IDL_CTRL_HDL hCtrl )
{
   if( ABCC_GetCmdQueueSize(hCtrl) == 0 )
   {
      return( NULL );
   }
   return( ABCC_MemAlloc(hCtrl) );
}

ABCC_ErrorCodeType ABCC_ReturnMsgBuffer( IDL_CTRL_HDL hCtrl, ABP_MsgType** ppsBuffer )
{
   ABCC_LinkFree( hCtrl, ppsBuffer );

   return( ABCC_EC_NO_ERROR );
}

void ABCC_TakeMsgBufferOwnership( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg )
{
   ABCC_MemSetBufferStatus( hCtrl, psMsg, ABCC_MEM_BUFSTAT_OWNED );
}

void ABCC_SetPdSize( IDL_CTRL_HDL hCtrl, const UINT16 iReadPdSize, const UINT16 iWritePdSize )
{
   DEBUG_EVENT((" New process data sizes RdPd %d WrPd %d\n", iReadPdSize, iWritePdSize));
   pnABCC_DrvSetPdSize( hCtrl, iReadPdSize, iWritePdSize );
}


void ABCC_HWReset( IDL_CTRL_HDL hCtrl )
{
   DEBUG_EVENT((" HW Reset\n"));
   ABCC_ShutdownDriver(hCtrl);
   ABCC_SYS_HWReset(hCtrl);
}


void ABCC_ShutdownDriver( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   DEBUG_EVENT( ( " Enter Shutdown state\n" ) );

#if( ABCC_CFG_SYNC_ENABLE && ABCC_CFG_USE_ABCC_SYNC_SIGNAL )
   ABCC_SYS_SyncInterruptDisable(hCtrl);
#endif

#if( ABCC_CFG_INT_ENABLED )
   ABCC_SYS_AbccInterruptDisable(hCtrl);
#endif
   ABCC_SYS_Close(hCtrl);
   ABCC_TimerDisable(hCtrl);
   abcc_eMainState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_DRV_SHUTDOWN;
}


BOOL ABCC_ModuleDetect( IDL_CTRL_HDL hCtrl )
{
#if( ABCC_CFG_MOD_DETECT_PINS_CONN )
   return( ABCC_SYS_ModuleDetect(hCtrl) );
#else
   return( TRUE );
#endif
}

UINT16 ABCC_ModCap( IDL_CTRL_HDL hCtrl )
{
   return( pnABCC_DrvGetModCap(hCtrl) );
}

UINT16 ABCC_LedStatus( IDL_CTRL_HDL hCtrl )
{
   return( pnABCC_DrvGetLedStatus(hCtrl) );
}

UINT8 ABCC_AnbState( IDL_CTRL_HDL hCtrl )
{
   return( pnABCC_DrvGetAnybusState(hCtrl) );
}

BOOL ABCC_IsSupervised( IDL_CTRL_HDL hCtrl )
{
   return( pnABCC_DrvIsSupervised(hCtrl) );
}

void ABCC_HWReleaseReset( IDL_CTRL_HDL hCtrl )
{
   ABCC_SYS_HWReleaseReset(hCtrl);
}

ABP_AppStatusType ABCC_GetAppStatus( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   return( abcc_eAppStatus[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
}

void ABCC_SetAppStatus( IDL_CTRL_HDL hCtrl, ABP_AppStatusType eAppStatus )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   if( abcc_eAppStatus[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] != eAppStatus )
   {
      abcc_eAppStatus[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = eAppStatus;
      pnABCC_DrvSetAppStatus( hCtrl, eAppStatus );
   }
}

UINT8 ABCC_ReadModuleId( IDL_CTRL_HDL hCtrl )
{
#ifdef ABCC_CFG_ABCC_MODULE_ID
   return( ABCC_CFG_ABCC_MODULE_ID );
#else
   return( ABCC_SYS_ReadModuleId(hCtrl) );
#endif
}

void ABCC_RunTimerSystem( IDL_CTRL_HDL hCtrl, const INT16 iDeltaTimeMs )
{
   ABCC_TimerTick( hCtrl, iDeltaTimeMs );
}


UINT8 ABCC_GetNewSourceId( IDL_CTRL_HDL hCtrl )
{
   static UINT8 bSourceId = 0;
   UINT8 bTempSrcId;
   ABCC_PORT_UseCritical(hCtrl);

   do
   {
      ABCC_PORT_EnterCritical(hCtrl);
      bTempSrcId = ++bSourceId;
      ABCC_PORT_ExitCritical(hCtrl);
   }  while (  ABCC_LinkIsSrcIdUsed( hCtrl, bTempSrcId ) );

   return( bTempSrcId );
}

UINT8 ABCC_GetOpmode( IDL_CTRL_HDL hCtrl )
{
#if( ABCC_CFG_OP_MODE_GETTABLE )
   return( ABCC_SYS_GetOpmode(hCtrl) );
#elif ( defined( ABCC_CFG_ABCC_OP_MODE_30 ) &&                                \
        defined( ABCC_CFG_ABCC_OP_MODE_40 ) )
   UINT8 bModuleId;

   bModuleId = ABCC_ReadModuleId(hCtrl);

   if( bModuleId == ABP_MODULE_ID_ACTIVE_ABCC30 )
   {
      return( ABCC_CFG_ABCC_OP_MODE_30 );
   }
   else
   {
      return( ABCC_CFG_ABCC_OP_MODE_40 );
   }
#elif defined( ABCC_CFG_ABCC_OP_MODE_30 )
   return( ABCC_CFG_ABCC_OP_MODE_30 );
#elif defined( ABCC_CFG_ABCC_OP_MODE_40 )
   return( ABCC_CFG_ABCC_OP_MODE_40 );
#else
   /*
   ** The user has not configured any way to determine the operating mode
   */
   #error "No method to determine the operating mode is available. Either set ABCC_CFG_OP_MODE_GETTABLE to TRUE or any of ABCC_CFG_ABCC_OP_MODE_X. See descriptions in abcc_cfg.h for details."
#endif /* End of #if defined( ABCC_CFG_OP_MODE_HW_CONF ) */
}


void ABCC_GetAttribute( ABP_MsgType* psMsg,
                        UINT8 bObject,
                        UINT16 iInstance,
                        UINT8 bAttribute,
                        UINT8 bSourceId )
{
   ABCC_MsgType sMsg;
   sMsg.psMsg = psMsg;

   ABCC_SetLowAddrOct( sMsg.psMsg16->sHeader.iSourceIdDestObj, bSourceId ); /* SourceId */
   ABCC_SetHighAddrOct( sMsg.psMsg16->sHeader.iSourceIdDestObj, bObject );  /* bObject */
   psMsg->sHeader.iInstance = iTOiLe( iInstance );                          /* Instance */
   ABCC_SetLowAddrOct(  sMsg.psMsg16->sHeader.iCmdReserved,
                  ABP_MSG_HEADER_C_BIT | ABP_CMD_GET_ATTR );                /* Command */

   sMsg.psMsg16->sHeader.iDataSize = 0;                                     /* Data size           */
   ABCC_SetLowAddrOct( sMsg.psMsg16->sHeader.iCmdExt0CmdExt1, bAttribute ); /* CmdExt0 (Attribute) */
   ABCC_SetHighAddrOct( sMsg.psMsg16->sHeader.iCmdExt0CmdExt1, 0 );         /* CmdExt1 (reserved)  */
}

void ABCC_SetByteAttribute(ABP_MsgType* psMsg,
                           UINT8 bObject,
                           UINT16 iInstance,
                           UINT8 bAttribute,
                           UINT8 bVal,
                           UINT8 bSourceId )
{
   ABCC_MsgType sMsg;
   sMsg.psMsg = psMsg;

   ABCC_SetLowAddrOct( sMsg.psMsg16->sHeader.iSourceIdDestObj, bSourceId );  /* SourceId */
   ABCC_SetHighAddrOct( sMsg.psMsg16->sHeader.iSourceIdDestObj, bObject );   /* bObject */
   psMsg->sHeader.iInstance = iTOiLe( iInstance );                           /* Instance */
   ABCC_SetLowAddrOct(  sMsg.psMsg16->sHeader.iCmdReserved,
                  ABP_MSG_HEADER_C_BIT | ABP_CMD_SET_ATTR );                 /* Command */

   sMsg.psMsg16->sHeader.iDataSize = iTOiLe( 1 );                            /* Data size           */
   ABCC_SetLowAddrOct( sMsg.psMsg16->sHeader.iCmdExt0CmdExt1, bAttribute );  /* CmdExt0 (Attribute) */
   ABCC_SetHighAddrOct( sMsg.psMsg16->sHeader.iCmdExt0CmdExt1, 0 );          /* CmdExt1 (reserved)  */
   ABCC_SetLowAddrOct( sMsg.psMsg16->aiData[ 0 ], bVal );                    /* Data                */
}

void ABCC_SetMsgHeader( ABP_MsgType* psMsg,
                        UINT8 bObject,
                        UINT16 iInstance,
                        UINT8 bAttribute,
                        ABP_MsgCmdType eService,
                        UINT16 iDataSize,
                        UINT8 bSourceId )
{
   ABCC_MsgType sMsg;
   sMsg.psMsg = psMsg;

   ABCC_SetLowAddrOct( sMsg.psMsg16->sHeader.iSourceIdDestObj, bSourceId );  /* SourceId */
   ABCC_SetHighAddrOct( sMsg.psMsg16->sHeader.iSourceIdDestObj, bObject );   /* bObject */
   psMsg->sHeader.iInstance = iTOiLe( iInstance );                           /* Instance */
   ABCC_SetLowAddrOct(  sMsg.psMsg16->sHeader.iCmdReserved,
                        ABP_MSG_HEADER_C_BIT | eService );                   /* Command */

   sMsg.psMsg16->sHeader.iDataSize = iTOiLe( iDataSize );                    /* Data size           */
   ABCC_SetLowAddrOct( sMsg.psMsg16->sHeader.iCmdExt0CmdExt1, bAttribute );  /* CmdExt0 (Attribute) */
   ABCC_SetHighAddrOct( sMsg.psMsg16->sHeader.iCmdExt0CmdExt1, 0 );          /* CmdExt1 (reserved)  */
}

ABCC_ErrorCodeType ABCC_VerifyMessage( const ABP_MsgType* psMsg )
{
   const ABP_MsgType16* psMsg16 = (const ABP_MsgType16* )psMsg;
   if( ABCC_GetLowAddrOct( psMsg16->sHeader.iCmdReserved ) & ABP_MSG_HEADER_E_BIT )
   {
      return( ABCC_EC_RESP_MSG_E_BIT_SET );
   }
   return( ABCC_EC_NO_ERROR );
}


void SetErrorReporter( ABCC_ErrorReporter nFunc )
{
   ErrorReporter = nFunc;
}


/*------------------------------------------------------------------------------
** ABCC_GetDataTypeSizeInBits()
**------------------------------------------------------------------------------
*/
UINT16 ABCC_GetDataTypeSizeInBits( UINT8 bDataType )
{
   UINT16 iSetBitSize;

   if( ABP_Is_PADx( bDataType ) )
   {
      iSetBitSize = bDataType - ABP_PAD0;
   }
   else if ( ABP_Is_BITx( bDataType ) )
   {
      iSetBitSize = ( ( bDataType - ABP_BIT1 ) + 1 );
   }
   else
   {
      iSetBitSize = (UINT16)ABCC_GetDataTypeSize( bDataType );
      iSetBitSize *= 8;
   }

   return( iSetBitSize );
}



/*------------------------------------------------------------------------------
** ABCC_GetDataTypeSize()
**------------------------------------------------------------------------------
*/

UINT8 ABCC_GetDataTypeSize( UINT8 bDataType )
{
   UINT8 bSize;
   switch( bDataType )
   {

   case ABP_UINT8:
   case ABP_BOOL:
   case ABP_SINT8:
   case ABP_ENUM:
   case ABP_BITS8:
   case ABP_CHAR:
   case ABP_OCTET:
      bSize = ABP_UINT8_SIZEOF;
      break;

   case ABP_UINT16:
   case ABP_BITS16:
   case ABP_SINT16:
      bSize = ABP_UINT16_SIZEOF;
      break;
   case ABP_UINT32:
   case ABP_SINT32:
   case ABP_BITS32:
   case ABP_FLOAT:
      bSize = ABP_UINT32_SIZEOF;
      break;

   case ABP_SINT64:
   case ABP_UINT64:
      bSize = ABP_UINT64_SIZEOF;
      break;

   case ABP_BIT1:
   case ABP_BIT2:
   case ABP_BIT3:
   case ABP_BIT4:
   case ABP_BIT5:
   case ABP_BIT6:
   case ABP_BIT7:
      bSize = ABP_UINT8_SIZEOF;
      break;


   case ABP_PAD1:
   case ABP_PAD2:
   case ABP_PAD3:
   case ABP_PAD4:
   case ABP_PAD5:
   case ABP_PAD6:
   case ABP_PAD7:
   case ABP_PAD8:
      bSize = ABP_UINT8_SIZEOF;
      break;

   case ABP_PAD9:
   case ABP_PAD10:
   case ABP_PAD11:
   case ABP_PAD12:
   case ABP_PAD13:
   case ABP_PAD14:
   case ABP_PAD15:
   case ABP_PAD16:
      bSize = ABP_UINT16_SIZEOF;
      break;
   default:
      bSize = 0;
      break;
   }

   return( bSize );
}

void ABCC_GetMsgString( ABP_MsgType* psMsg, char* pcString, UINT16 iNumChar, UINT16 iOctetOffset )
{
   ABCC_PORT_StrCpyToNative( pcString,
                             ABCC_GetMsgDataPtr( psMsg ),
                             iOctetOffset,
                             iNumChar );
}

void ABCC_SetMsgString( ABP_MsgType* psMsg, const char* pcString, UINT16 iNumChar, UINT16 iOctetOffset )
{
   ABCC_PORT_StrCpyToPacked( ABCC_GetMsgDataPtr( psMsg ),
                             iOctetOffset,
                             pcString,
                             iNumChar );
}

void ABCC_GetMsgData8( ABP_MsgType* psMsg, UINT8* pbData, UINT16 iOctetOffset )
{
#ifdef ABCC_SYS_16_BIT_CHAR
   *pbData = 0;
#endif
   ABCC_PORT_Copy8( pbData, 0, ABCC_GetMsgDataPtr( psMsg ), iOctetOffset );
}

void ABCC_SetMsgData8( ABP_MsgType* psMsg, UINT8 bData, UINT16 iOctetOffset )
{
   ABCC_PORT_Copy8( ABCC_GetMsgDataPtr( psMsg ), iOctetOffset, &bData, 0 );
}

void ABCC_GetMsgData16( ABP_MsgType* psMsg, UINT16* piData, UINT16 iOctetOffset )
{
   ABCC_PORT_Copy16( piData, 0, ABCC_GetMsgDataPtr( psMsg ), iOctetOffset );
   *piData = iLeTOi( *piData );
}

void ABCC_SetMsgData16( ABP_MsgType* psMsg, UINT16 iData, UINT16 iOctetOffset )
{
   iData = iTOiLe( iData );
   ABCC_PORT_Copy16( ABCC_GetMsgDataPtr( psMsg ), iOctetOffset, &iData, 0 );
}

void ABCC_GetMsgData32( ABP_MsgType* psMsg, UINT32* plData, UINT16 iOctetOffset )
{
   ABCC_PORT_Copy32( plData, 0, ABCC_GetMsgDataPtr( psMsg ), iOctetOffset );
   *plData = lLeTOl( *plData );
}

void ABCC_SetMsgData32( ABP_MsgType* psMsg, UINT32 lData, UINT16 iOctetOffset )
{
   lData = lTOlLe( lData );
   ABCC_PORT_Copy32( ABCC_GetMsgDataPtr( psMsg ), iOctetOffset, &lData, 0 );
}

#if( ABCC_CFG_64BIT_ADI_SUPPORT )
void ABCC_GetMsgData64( ABP_MsgType* psMsg, UINT64* plData, UINT16 iOctetOffset )
{
    ABCC_PORT_Copy64( plData, 0, ABCC_GetMsgDataPtr( psMsg ), iOctetOffset );
    *plData = lLeTOl( *plData );
}

void ABCC_SetMsgData64( ABP_MsgType* psMsg, UINT64 lData, UINT16 iOctetOffset )
{
   lData = lTOlLe64( lData );
   ABCC_PORT_Copy64( ABCC_GetMsgDataPtr( psMsg ), iOctetOffset, &lData, 0 );
}
#endif
