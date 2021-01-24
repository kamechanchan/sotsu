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
** Timer implementation.
********************************************************************************
********************************************************************************
*/

#include "abcc_timer.h"
#include "abcc_debug_err.h"
#include "abcc_port.h"

#include "IDLTp.h"
/*******************************************************************************
** Constants
********************************************************************************
*/


/*******************************************************************************
** Typedefs
********************************************************************************
*/
/*
 * Timer resource structure
 */
typedef struct ABCC_TimerTimeoutType
{
   BOOL  fActive;
   BOOL  fTmoOccured;
   INT32 lTimeLeft;
   ABCC_TimerTimeoutCallbackType pnHandleTimeout;
}
ABCC_TimerTimeoutType;


/*******************************************************************************
** Public Globals
********************************************************************************
*/


/*******************************************************************************
** Private Globals
********************************************************************************
*/
#define MAX_NUM_TIMERS 3
static ABCC_TimerTimeoutType sTimer[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT][ MAX_NUM_TIMERS ];
static BOOL fTimerEnabled[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{FALSE}};

/*******************************************************************************
** Private Services
********************************************************************************
*/


/*******************************************************************************
** Public Services
********************************************************************************
*/
void ABCC_TimerInit( IDL_CTRL_HDL hCtrl )
{
   ABCC_TimerHandle xHandle;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   for ( xHandle = 0; xHandle < MAX_NUM_TIMERS; xHandle++ )
   {
      sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].pnHandleTimeout = NULL;
   }
   fTimerEnabled[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = TRUE;
}


/*
** ABCC_TimerCreateTimer()
*/
ABCC_TimerHandle ABCC_TimerCreate( IDL_CTRL_HDL hCtrl, ABCC_TimerTimeoutCallbackType pnHandleTimeout )
{
   ABCC_TimerHandle xHandle = ABCC_TIMER_NO_HANDLE;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   
   ABCC_PORT_UseCritical(hCtrl);

   ABCC_PORT_EnterCritical(hCtrl);

   for ( xHandle = 0; xHandle < MAX_NUM_TIMERS; xHandle++ )
   {
      if ( sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].pnHandleTimeout == NULL )
      {
         sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fActive = FALSE;
         sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fTmoOccured = FALSE;
         sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].pnHandleTimeout = pnHandleTimeout;
         break;
      }
   }
   ABCC_PORT_ExitCritical(hCtrl);

   if ( xHandle >=  MAX_NUM_TIMERS )
   {
      xHandle = ABCC_TIMER_NO_HANDLE;
   }
   return( xHandle );
}

BOOL ABCC_TimerStart( IDL_CTRL_HDL hCtrl, ABCC_TimerHandle xHandle,
                      UINT32 lTimeoutMs )
{
   BOOL fTmo;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   
   ABCC_PORT_UseCritical(hCtrl);

   ABCC_ASSERT( hCtrl, sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].pnHandleTimeout );

   ABCC_PORT_EnterCritical(hCtrl);
   fTmo = sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fTmoOccured;
   sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].lTimeLeft = (INT32)lTimeoutMs;
   sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fTmoOccured = FALSE;
   sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fActive = TRUE;

   ABCC_PORT_ExitCritical(hCtrl);
   return( fTmo );
}


/*
** ABCC_TimerStopTimer()
*/
BOOL ABCC_TimerStop( IDL_CTRL_HDL hCtrl, ABCC_TimerHandle xHandle )
{
   BOOL fTmo;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   
   ABCC_PORT_UseCritical(hCtrl);

   ABCC_PORT_EnterCritical(hCtrl);
   fTmo = sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fTmoOccured;

   sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fActive = FALSE;
   sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fTmoOccured = FALSE;

   ABCC_PORT_ExitCritical(hCtrl);
   return( fTmo );
}

/*
** ABCC_TimerTick()
*/
void ABCC_TimerTick(IDL_CTRL_HDL hCtrl, const INT16 iDeltaTimeMs)
{
   ABCC_TimerHandle xHandle;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   
   ABCC_PORT_UseCritical(hCtrl);

   if( !fTimerEnabled[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] )
   {
      return;
   }

   ABCC_PORT_EnterCritical(hCtrl);

   for ( xHandle = 0; xHandle < MAX_NUM_TIMERS; xHandle++ )
   {
       if ( ( sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].pnHandleTimeout != NULL ) &&
             ( sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fActive == TRUE ) )
       {
          sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].lTimeLeft -= (INT32)iDeltaTimeMs;
          if( sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].lTimeLeft <= 0 )
          {
             sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fTmoOccured = TRUE;
             sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].fActive = FALSE;
             sTimer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ xHandle ].pnHandleTimeout(hCtrl);
          }
       }
   }

   ABCC_PORT_ExitCritical(hCtrl);
}

void ABCC_TimerDisable( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   fTimerEnabled[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = FALSE;
}

/*******************************************************************************
** Tasks
********************************************************************************
*/
