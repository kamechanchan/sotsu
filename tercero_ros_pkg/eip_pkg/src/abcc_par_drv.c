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
** Parallel (PARI) driver implementation.
********************************************************************************
********************************************************************************
*/

#include "abcc_drv_cfg.h"

#if( ABCC_CFG_DRV_PARALLEL )

#include "abcc_td.h"
#include "abcc_debug_err.h"
#include "abcc_sys_adapt.h"
#include "abcc_timer.h"
#include "abcc_drv_if.h"
#include "abp.h"
#include "abcc_sys_adapt_par.h"
#include "abcc_handler.h"
#include "abcc_port.h"
#include "abcc_mem.h"

#include "IDLTp.h"

/*******************************************************************************
** Constants
********************************************************************************
*/

/*******************************************************************************
** Typedefs
********************************************************************************
*/

/*******************************************************************************
** Private Globals
********************************************************************************
*/

static ABCC_MsgType par_drv_uReadMessageData[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{{0}}};
static  void* par_drv_pbRdPdBuffer[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];

static UINT16   par_drv_iSizeOfReadPd[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];
static UINT16   par_drv_iSizeOfWritePd[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];

static   UINT8    par_drv_bNbrOfCmds[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];          /* Number of commands supported by the application. */

static const    UINT16   iWRPDFlag     = 0x01;
static const    UINT16   iRDPDFlag     = 0x02;
static const    UINT16   iWRMSGFlag    = 0x04;
static const    UINT16   iRDMSGFlag    = 0x08;
static const    UINT16   iANBRFlag     = 0x10;
static const    UINT16   iAPPRFlag     = 0x20;
static const    UINT16   iAPPRCLRFlag  = 0x40;

#define ABCC_MSG_HEADER_TYPE_SIZEOF 12

#ifdef ABCC_SYS_16_BIT_CHAR
static const UINT16 iWrMsgAdrOffset =        ABP_WRMSG_ADR_OFFSET / 2;
static const UINT16 iRdMsgAdrOffset =        ABP_RDMSG_ADR_OFFSET / 2;
static const UINT16 iModCapAdrOffset =       ABP_MODCAP_ADR_OFFSET / 2;
static const UINT16 iLedStatusAdrOffset =    ABP_LEDSTATUS_ADR_OFFSET / 2;
static const UINT16 iAppStatusAdrOffset =    ABP_APPSTATUS_ADR_OFFSET / 2;
static const UINT16 iAnbStatusAdrOffset =    ABP_ANBSTATUS_ADR_OFFSET / 2;
static const UINT16 iBufCtrlAdrOffset =      ABP_BUFCTRL_ADR_OFFSET / 2;
static const UINT16 iMsgHdrEndAdrOffset =    ABCC_MSG_HEADER_TYPE_SIZEOF / 2;
static const UINT16 iIntMaskAdrOffset =      ABP_INTMASK_ADR_OFFSET / 2;
static const UINT16 iIntStatusAdrOffset =    ABP_INTSTATUS_ADR_OFFSET / 2;
#else
static const UINT16 iWrMsgAdrOffset =        ABP_WRMSG_ADR_OFFSET;
static const UINT16 iRdMsgAdrOffset =        ABP_RDMSG_ADR_OFFSET;
static const UINT16 iModCapAdrOffset =       ABP_MODCAP_ADR_OFFSET;
static const UINT16 iLedStatusAdrOffset =    ABP_LEDSTATUS_ADR_OFFSET;
static const UINT16 iAppStatusAdrOffset =    ABP_APPSTATUS_ADR_OFFSET;
static const UINT16 iAnbStatusAdrOffset =    ABP_ANBSTATUS_ADR_OFFSET;
static const UINT16 iBufCtrlAdrOffset =      ABP_BUFCTRL_ADR_OFFSET;
static const UINT16 iMsgHdrEndAdrOffset =    ABCC_MSG_HEADER_TYPE_SIZEOF;
static const UINT16 iIntMaskAdrOffset =      ABP_INTMASK_ADR_OFFSET;
static const UINT16 iIntStatusAdrOffset =    ABP_INTSTATUS_ADR_OFFSET;

#endif

/*******************************************************************************
** Private forward declarations
********************************************************************************
*/

/*******************************************************************************
** Private Services
********************************************************************************
*/


/*******************************************************************************
** Public Services
********************************************************************************
*/
void ABCC_DrvParInit( IDL_CTRL_HDL hCtrl, UINT8 bOpmode )
{
   /*
   ** Initialize privates and states.
   */
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   ABCC_ASSERT_ERR( hCtrl, ( bOpmode == 7) || ( bOpmode == 8) , ABCC_SEV_FATAL, ABCC_EC_INCORRECT_OPERATING_MODE, (UINT32)bOpmode );

   par_drv_iSizeOfReadPd[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]  = 0;
   par_drv_iSizeOfWritePd[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = 0;
   par_drv_bNbrOfCmds[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]     = 0;
   par_drv_pbRdPdBuffer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]   = ABCC_DrvParallelGetRdPdBuffer(hCtrl);
}


void ABCC_DrvParSetIntMask( IDL_CTRL_HDL hCtrl, const UINT16 iIntMask )
{
   ABCC_DrvWrite16( hCtrl, iIntMaskAdrOffset, iTOiLe( iIntMask ) ) ;
}

#if( ABCC_CFG_INT_ENABLED )
UINT16 ABCC_DrvParISR( IDL_CTRL_HDL hCtrl )
{
   UINT16 iIntStatus;
   UINT16 iIntToHandle;

   /*---------------------------------------------------------------------------
   ** Read the interrupt status register and acknowledge all interrupts.
   ** Read interrupt status until all enabled interrupts are acknowledged.
   ** This will make sure that edge triggered interrupt always will trigger
   ** even if a new event has occurred between the int status read the
   ** acknowledge.
   **---------------------------------------------------------------------------
   */
   iIntStatus = ABCC_DrvRead16( hCtrl, iIntStatusAdrOffset );
   ABCC_DrvWrite16( hCtrl, iIntStatusAdrOffset, iIntStatus );
   iIntStatus = ( iLeTOi( iIntStatus ) ) & ABCC_iInterruptEnableMask;
   iIntToHandle = iIntStatus;

   while( iIntStatus != 0 )
   {
    iIntStatus = ABCC_DrvRead16( hCtrl, iIntStatusAdrOffset );
    ABCC_DrvWrite16( hCtrl, iIntStatusAdrOffset, iIntStatus );
    iIntStatus = ( iLeTOi( iIntStatus ) ) & ABCC_iInterruptEnableMask;
    iIntToHandle |= iIntStatus;
   }

   return iIntToHandle;
}
#else
UINT16 ABCC_DrvParISR( IDL_CTRL_HDL hCtrl )
{
   ABCC_ERROR( ABCC_SEV_WARNING, ABCC_EC_INTERNAL_ERROR, 0);
   return 0;
}
#endif



ABP_MsgType* ABCC_DrvParRunDriverRx( IDL_CTRL_HDL hCtrl )
{
   /*
   ** Always NULL for the parallel interface.
   */
   return NULL;
}


BOOL ABCC_DrvParWriteMessage( IDL_CTRL_HDL hCtrl, ABP_MsgType* psWriteMsg )
{
   UINT16 iBufControlWriteFlags;
   ABCC_MsgType uMsg;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   iBufControlWriteFlags = 0;
   ABCC_ASSERT( hCtrl, psWriteMsg );
   uMsg.psMsg = psWriteMsg;

   /*
   ** Write message data.
   **
   ** Note: The application is only allowed to write to the write message
   ** area when the WRMSG bit it set to 0.
   ** However, the !( iBufControl & iWRMSGFlag ) check should have
   ** already been performed inside link.c, and is thus not required here.
   */
#ifdef MSG_TIMING
   /*Toggle led for timing measurement*/
   GPIO_OUT0  = 0;
#endif
   ABCC_DrvParallelWrite( hCtrl, iWrMsgAdrOffset, psWriteMsg, ABCC_MSG_HEADER_TYPE_SIZEOF + iLeTOi( psWriteMsg->sHeader.iDataSize ) );

   iBufControlWriteFlags |= iWRMSGFlag;

   /*
   ** Determine if command messages (instead of response messages) can be sent.
   */
   if( !( ABCC_GetLowAddrOct( (uMsg.psMsg16->sHeader.iCmdReserved ) & ABP_MSG_HEADER_C_BIT ) ) )
   {
      /*
      ** A command message has been received by the host application and a
      ** response message (not a command message) will be sent back to the
      ** Anybus. The number of commands the host application can receive
      ** shall be increased by one, as a previous received command is now
      ** handled.
      */
      par_drv_bNbrOfCmds[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]++;

      /*
      ** When a change of the number of commands which the host application
      ** can receive is made from 0 to 1, it means that we can set the APPRF
      ** flag again to indicate for the Anybus that the host is now ready to
      ** receive a new command.
      */
      if( par_drv_bNbrOfCmds[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == 1 )
      {
         iBufControlWriteFlags |= iAPPRFlag;
      }
   }
   /*
   ** Update the buffer control register.
   */

   ABCC_DrvWrite16( hCtrl, iBufCtrlAdrOffset, iTOiLe( iBufControlWriteFlags ) );
#ifdef MSG_TIMING
   /*Toggle led for timing measurement*/
   GPIO_OUT0  = 1;
#endif
   return( TRUE );
}


void ABCC_DrvParWriteProcessData( IDL_CTRL_HDL hCtrl, void* pxProcessData )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   if( par_drv_iSizeOfWritePd[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] )
   {
      /*
      ** Write process data.
      */
      ABCC_DrvWriteWrPd( hCtrl, pxProcessData, par_drv_iSizeOfWritePd[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );

      /*
      ** Update the buffer control register.
      */
      ABCC_DrvWrite16( hCtrl, iBufCtrlAdrOffset, iTOiLe( iWRPDFlag ) );
#ifdef PD_TIMING
      /*Toggle led for timing measurement*/
      GPIO_OUT0  = 1;
#endif
   }
}


BOOL ABCC_DrvParIsReadyForWriteMessage( IDL_CTRL_HDL hCtrl )
{
   UINT16 iBufControl;
   iBufControl = ABCC_DrvRead16( hCtrl, iBufCtrlAdrOffset );

   return( !( iLeTOi( iBufControl ) & iWRMSGFlag ) );
}


BOOL ABCC_DrvParIsReadyForCmd( IDL_CTRL_HDL hCtrl )
{
   UINT16 iBufControl;
   iBufControl = ABCC_DrvRead16( hCtrl, iBufCtrlAdrOffset );
   iBufControl = iLeTOi( iBufControl );
   return( !( iBufControl & iWRMSGFlag ) && ( iBufControl & iANBRFlag ) );
}


void ABCC_DrvParSetNbrOfCmds( IDL_CTRL_HDL hCtrl, UINT8 bNbrOfCmds )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   par_drv_bNbrOfCmds[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = bNbrOfCmds;

   /*
   ** Acknowledge that we are ready to accept the first command message.
   */
   ABCC_DrvWrite16( hCtrl, iBufCtrlAdrOffset, iTOiLe( iAPPRFlag ) );
}


void ABCC_DrvParSetAppStatus( IDL_CTRL_HDL hCtrl, ABP_AppStatusType eAppStatus )
{
   ABCC_DrvWrite16( hCtrl, iAppStatusAdrOffset, iTOiLe( (UINT16)eAppStatus ) );
}


void ABCC_DrvParSetPdSize( IDL_CTRL_HDL hCtrl, const UINT16 iReadPdSize, const UINT16 iWritePdSize )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   par_drv_iSizeOfReadPd[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = iReadPdSize;
   par_drv_iSizeOfWritePd[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = iWritePdSize;

   ABCC_SYS_SetPDSize (hCtrl, iReadPdSize, iWritePdSize);
}


static void DrvParSetMsgReceiverBuffer( IDL_CTRL_HDL hCtrl, ABP_MsgType* const psReadMsg )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   par_drv_uReadMessageData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].psMsg = psReadMsg;
}


UINT16 ABCC_DrvParGetIntStatus( IDL_CTRL_HDL hCtrl )
{
   UINT16 iIntStatus;

   iIntStatus = ABCC_DrvRead16( hCtrl, iIntStatusAdrOffset );

   return  iLeTOi( iIntStatus );
}

UINT8 ABCC_DrvParGetAnybusState( IDL_CTRL_HDL hCtrl )
{
   UINT16 iAnbStatus;

   /*
   ** Reading out the Anybus status.
   */
   iAnbStatus = ABCC_DrvRead16( hCtrl, iAnbStatusAdrOffset );

   /*
   ** The Anybus state is stored in bits 0-2 of the read register.
   */
   return( (UINT8)( iLeTOi( iAnbStatus ) & 0x07 ) );
}


void* ABCC_DrvParReadProcessData( IDL_CTRL_HDL hCtrl )
{
   UINT16 iBufctrl;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   /*
   ** Check if the Anybus has updated the read process data.
   */
   iBufctrl = ABCC_DrvRead16( hCtrl, iBufCtrlAdrOffset );

   if( iLeTOi( iBufctrl ) & iRDPDFlag  )
   {
      /*
      ** The RDPD flag must be set before we try to read the process data.
      ** Otherwise the buffers won't be switched and we won't have any process
      ** data available.
      */
      ABCC_DrvWrite16( hCtrl, iBufCtrlAdrOffset, iTOiLe( iRDPDFlag ) );

      /*
      ** We have process data to read.
      */
      ABCC_DrvReadRdPd( hCtrl, par_drv_pbRdPdBuffer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex], par_drv_iSizeOfReadPd[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );

      return( par_drv_pbRdPdBuffer[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
   }
   else
   {
      return NULL;
   }
}


ABP_MsgType* ABCC_DrvParReadMessage( IDL_CTRL_HDL hCtrl )
{
   UINT16 iBufctrl;
   UINT16 iMsgSize;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   iBufctrl = ABCC_DrvRead16( hCtrl, iBufCtrlAdrOffset );

   if( iLeTOi( iBufctrl ) & iRDMSGFlag  )
   {
      DrvParSetMsgReceiverBuffer( hCtrl, ABCC_MemAlloc(hCtrl) );

      if( par_drv_uReadMessageData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].psMsg == NULL )
      {
         ABCC_ERROR( hCtrl, ABCC_SEV_WARNING, ABCC_EC_OUT_OF_MSG_BUFFERS, 0 );
         return( NULL );
      }

      /*
      ** We have message data to read. First read the header and check the size
      ** of data area.
      */
      ABCC_DrvParallelRead( hCtrl, iRdMsgAdrOffset, par_drv_uReadMessageData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].psMsg16, ABCC_MSG_HEADER_TYPE_SIZEOF );

      iMsgSize = iLeTOi( par_drv_uReadMessageData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].psMsg16->sHeader.iDataSize );

      if( iMsgSize > ABCC_CFG_MAX_MSG_SIZE )
      {
         /*
         ** Message size exceeds buffer.
         */
         ABCC_ERROR( hCtrl, ABCC_SEV_FATAL, ABCC_EC_RDMSG_SIZE_ERR, 0 );

         ABCC_DrvWrite16( hCtrl, iBufCtrlAdrOffset, iTOiLe( iRDMSGFlag ) );
         return( NULL );
      }

      /*
      ** Continue reading data area if > 0.
      */
      if( iMsgSize > 0 )
      {
         ABCC_DrvParallelRead( hCtrl, 
                               iRdMsgAdrOffset + iMsgHdrEndAdrOffset,
                               par_drv_uReadMessageData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].psMsg16->aiData,
                               iMsgSize );
      }

      ABCC_DrvWrite16( hCtrl, iBufCtrlAdrOffset, iTOiLe( iRDMSGFlag ) );

      /*
      ** Determine if command messages (instead of response messages) can be read.
      */
      if( ABCC_GetLowAddrOct( par_drv_uReadMessageData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].psMsg16->sHeader.iCmdReserved ) & ABP_MSG_HEADER_C_BIT )
      {
         /*
         ** A command messages has been sent by the Anybus and it has been read
         ** by the host application. The number of commands allowed by the host
         ** application must be decreased by one.
         */
         par_drv_bNbrOfCmds[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]--;

         /*
         ** Indicates that the host application is not ready to receive a new
         ** command from the Anybus. Writing to this register must only be done
         ** when the RDMSG bit is set to 1. A check is not required however,
         ** since the RDMSG bit is set to 1 a few lines higher up in the code.
         */
         if( par_drv_bNbrOfCmds[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == 0 )
         {
            /*
            ** Update the buffer control register.
            */
            ABCC_DrvWrite16( hCtrl, iBufCtrlAdrOffset, iTOiLe( iAPPRCLRFlag ) );
         }
      }

      return( par_drv_uReadMessageData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].psMsg );
   }
   else
   {
      return( NULL );
   }
}

void* ABCC_DrvParGetWrPdBuffer( IDL_CTRL_HDL hCtrl )
{
   return ABCC_DrvParallelGetWrPdBuffer(hCtrl);
}

UINT16 ABCC_DrvParGetModCap( IDL_CTRL_HDL hCtrl )
{
   UINT16 iModCap;
   iModCap = ABCC_DrvRead16( hCtrl, iModCapAdrOffset );
   return iLeTOi( iModCap );
}


UINT16 ABCC_DrvParGetLedStatus( IDL_CTRL_HDL hCtrl )
{
   UINT16 iLedStatus;
   iLedStatus = ABCC_DrvRead16( hCtrl, iLedStatusAdrOffset );
   return iLeTOi( iLedStatus );
}

BOOL ABCC_DrvParIsReadyForWrPd( IDL_CTRL_HDL hCtrl )
{
   return TRUE;
}


BOOL ABCC_DrvParIsSupervised( IDL_CTRL_HDL hCtrl )
{
   UINT16 iAnbStatus;
   /*
   ** Reading out the Anybus status.
   */
   iAnbStatus = ABCC_DrvRead16( hCtrl, iAnbStatusAdrOffset );
   iAnbStatus = iLeTOi( iAnbStatus );

   /*
   ** The Anybus supervision bis is stored in bit 3
   */
   return( ( iAnbStatus >> 3 ) & 1 );

}

UINT8 ABCC_DrvParGetAnbStatus( IDL_CTRL_HDL hCtrl )
{
   UINT16 iAnbStatus;
   /*
   ** Reading out the Anybus status.
   */
   iAnbStatus = ABCC_DrvRead16( hCtrl, iAnbStatusAdrOffset );
   iAnbStatus = iLeTOi( iAnbStatus );
   return (UINT8)iAnbStatus & 0xf ;

}
#endif



/*******************************************************************************
** End of par_drv.c
********************************************************************************
*/
