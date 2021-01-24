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
** Implements message queuing and write message flow control.
**. and response handler functionality
**
********************************************************************************
********************************************************************************
*/

#include "abcc_drv_cfg.h"
#include "abcc_td.h"
#include "abcc_debug_err.h"
#include "abcc_link.h"
#include "abcc_drv_if.h"
#include "abcc_mem.h"
#include "abcc_sys_adapt.h"
#include "abcc_timer.h"
#include "abcc_handler.h"
#include "abcc_port.h"

#include "IDLTp.h"



/*******************************************************************************
** Constants
********************************************************************************
*/

/*
** Max number of messages in each send queue.
*/
#define LINK_MAX_NUM_CMDS_IN_Q            ABCC_CFG_MAX_NUM_APPL_CMDS
#define LINK_MAX_NUM_RESP_IN_Q            ABCC_CFG_MAX_NUM_ABCC_CMDS

/*
** Total number of message resources.
*/
#define LINK_NUM_MSG_IN_POOL              ABCC_CFG_MAX_NUM_MSG_RESOURCES

/*
** Max num message handlers for responses.
** The max number of outstanding commands counter is decremented
** before the handler is invoked therefore we need to handle 1 extra
** handler
*/
#define LINK_MAX_NUM_MSG_HDL ( LINK_MAX_NUM_CMDS_IN_Q + 1 )


/*******************************************************************************
** Typedefs
********************************************************************************
*/

/*
** Message queue type for queueing cmds and responses.
*/
typedef struct MsgQueueType
{
   ABP_MsgType** queue;
   INT8 bReadIndex;
   INT8 bQueueSize;
   INT8 bNumInQueue;
} MsgQueueType;


/*******************************************************************************
** Public Globals
********************************************************************************
*/


/*******************************************************************************
** Private Globals
********************************************************************************
*/

/*
 * Largest supported message size.
 */
static UINT16 link_iMaxMsgSize;

/*
 ** Command and response queues
 */
static ABP_MsgType* link_psCmds[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT][LINK_MAX_NUM_CMDS_IN_Q ];
static ABP_MsgType* link_psResponses[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT][LINK_MAX_NUM_RESP_IN_Q];

static MsgQueueType link_sCmdQueue[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];
static MsgQueueType link_sRespQueue[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];

/*
 ** Response handlers
 */
static ABCC_MsgHandlerFuncType link_pnMsgHandler[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT][ LINK_MAX_NUM_MSG_HDL ];
static UINT8              link_bMsgSrcId[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT][ LINK_MAX_NUM_MSG_HDL ];

static ABCC_LinkNotifyIndType pnMsgSentHandler[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];
static ABP_MsgType* link_psNotifyMsg[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];


/*
 ** Max number of outstanding commands ( no received response yet )
 */
static UINT8 link_bNumberOfOutstandingCommands[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{0}};

/*******************************************************************************
** Private Services
********************************************************************************
*/
static ABP_MsgType* link_DeQueue( IDL_CTRL_HDL hCtrl, MsgQueueType* psMsgQueue )
{
   ABP_MsgType* psMsg = NULL;
   if ( psMsgQueue->bNumInQueue != 0 )
   {
      psMsg = psMsgQueue->queue[ psMsgQueue->bReadIndex++];
      psMsgQueue->bNumInQueue--;
      psMsgQueue->bReadIndex %= psMsgQueue->bQueueSize;
   }
   ABCC_ASSERT(hCtrl, psMsgQueue->bNumInQueue >= 0);
   return psMsg;
}


static BOOL link_EnQueue( IDL_CTRL_HDL hCtrl, MsgQueueType* psMsgQueue, ABP_MsgType* psMsg )
{
   if ( psMsgQueue->bNumInQueue <  psMsgQueue->bQueueSize )
   {
      psMsgQueue->queue[  ( psMsgQueue->bNumInQueue + psMsgQueue->bReadIndex ) %  psMsgQueue->bQueueSize ] = psMsg;
      psMsgQueue->bNumInQueue++;
      return TRUE;
   }
   return FALSE;
}

static ABP_MsgType* link_Peek( MsgQueueType* psMsgQueue )
{
   ABP_MsgType* psMsg;

   psMsg = NULL;

   if( psMsgQueue->bNumInQueue != 0 )
   {
      psMsg = psMsgQueue->queue[ psMsgQueue->bReadIndex ];
   }

   return( psMsg );
}


static void link_CheckNotification( IDL_CTRL_HDL hCtrl, const ABP_MsgType* const psMsg )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   ABCC_DEBUG_MSG_DATA( "Msg sent", (ABP_MsgType*)psMsg );
   if( ( pnMsgSentHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] != NULL ) &&  ( psMsg == link_psNotifyMsg[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ) )
   {
      pnMsgSentHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex](hCtrl);
      pnMsgSentHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = NULL;
   }
}

/*******************************************************************************
** Public Services
********************************************************************************
*/
void ABCC_LinkInit( IDL_CTRL_HDL hCtrl )
{
   UINT16 iCount;

   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   /*
   ** Init Queue structures.
   */
   link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue = 0;
   link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize = LINK_MAX_NUM_CMDS_IN_Q;
   link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bReadIndex = 0;
   link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].queue = link_psCmds[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];

   link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue = 0;
   link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize = LINK_MAX_NUM_RESP_IN_Q;
   link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bReadIndex = 0;
   link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].queue = link_psResponses[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];

   ABCC_MemCreatePool(hCtrl);

   for( iCount = 0; iCount < LINK_MAX_NUM_CMDS_IN_Q; iCount++  )
   {
      link_pnMsgHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iCount ] = 0;
      link_bMsgSrcId[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][iCount ] = 0;
   }

   /*
   ** Initialize driver privates and states to default values.
   */
   link_bNumberOfOutstandingCommands[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = 0;

   pnMsgSentHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = NULL;
   link_psNotifyMsg[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = NULL;

   link_iMaxMsgSize = ABCC_CFG_MAX_MSG_SIZE;
   /*
    ** Limit ABCC 30 messages to 255 bytes
    */
   if( ABCC_ReadModuleId(hCtrl) == ABP_MODULE_ID_ACTIVE_ABCC30 )
   {
      if ( link_iMaxMsgSize > ABP_MAX_MSG_255_DATA_BYTES )
      {
         link_iMaxMsgSize = ABP_MAX_MSG_255_DATA_BYTES;
      }
   }
}


ABP_MsgType* ABCC_LinkReadMessage( IDL_CTRL_HDL hCtrl )
{
   ABCC_MsgType psReadMessage;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   ABCC_PORT_UseCritical(hCtrl);

   psReadMessage.psMsg = pnABCC_DrvReadMessage(hCtrl);

   if( psReadMessage.psMsg != NULL )
   {
      if( ( ABCC_GetLowAddrOct( psReadMessage.psMsg16->sHeader.iCmdReserved ) & ABP_MSG_HEADER_C_BIT ) == 0 )
      {
         /*
         ** Decrement number of outstanding commands if a response is received
         */
         ABCC_PORT_EnterCritical(hCtrl);
         link_bNumberOfOutstandingCommands[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]--;
         ABCC_PORT_ExitCritical(hCtrl);
         ABCC_DEBUG_MSG_GENERAL( ( "Outstanding commands: %d\n",
                                 link_bNumberOfOutstandingCommands[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ) );
      }
   }
   return psReadMessage.psMsg;
}


void ABCC_LinkCheckSendMessage( IDL_CTRL_HDL hCtrl )
{
   BOOL fCmdMsg;
   BOOL fMsgWritten;
   BOOL fSendMsg;
   ABP_MsgType* psWriteMessage = NULL;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   ABCC_PORT_UseCritical(hCtrl);

   fSendMsg = FALSE;

   ABCC_PORT_EnterCritical(hCtrl);

   /*
   ** Check if any messages are queued.
   ** If the queue index > 0 then there are messages in the qeueue.
   ** Response messages are prioritised before command messages.
   **
   ** Note: Only a reference to the message is retrieved from the queue before
   ** the transmission. It must stay in the queue until the transmission is
   ** completed to guarantee the thread protection that is implemented in
   ** ABCC_LinkWriteMessage().
   */
   if ( link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue > 0 )
   {
      fCmdMsg = FALSE;
      fSendMsg = TRUE;

      psWriteMessage = link_Peek( &link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
   }
   else if ( link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue > 0 )
   {
      fCmdMsg = TRUE;
      fSendMsg = TRUE;

      psWriteMessage = link_Peek( &link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
   }

   ABCC_PORT_ExitCritical(hCtrl);

   if ( fSendMsg && ( ( fCmdMsg && pnABCC_DrvISReadyForCmd(hCtrl) ) ||
                      ( !fCmdMsg && pnABCC_DrvISReadyForWriteMessage(hCtrl) ) ) )
   {
      fMsgWritten = pnABCC_DrvWriteMessage( hCtrl, psWriteMessage );

      /*
      ** The message has been transmitted, now it is time to dequeue it.
      */
      ABCC_PORT_EnterCritical(hCtrl);

      if( fCmdMsg )
      {
         link_DeQueue( hCtrl, &link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
         ABCC_DEBUG_MSG_EVENT( "Command dequeued", psWriteMessage );
         ABCC_DEBUG_MSG_GENERAL( ( "CmdQ status: %d(%d)\n", link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue, link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize ) );
      }
      else
      {
         link_DeQueue( hCtrl, &link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
         ABCC_DEBUG_MSG_EVENT( "Response dequeued", psWriteMessage );
         ABCC_DEBUG_MSG_GENERAL( ( "RespQ status: %d(%d)\n", link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue, link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize ) );
      }

      ABCC_PORT_ExitCritical(hCtrl);

      if( fMsgWritten )
      {
         /*
         ** The message was successfully written and can be deallocated now.
         */
         link_CheckNotification( hCtrl, psWriteMessage );
         ABCC_LinkFree( hCtrl, &psWriteMessage );
      }
   }
}


void ABCC_LinkRunDriverRx( IDL_CTRL_HDL hCtrl )
{
   ABP_MsgType* psSentMsg;

   psSentMsg = pnABCC_DrvRunDriverRx(hCtrl);
   /*
   ** If a write message was sent, free the buffer.
   */
   if( psSentMsg )
   {
      link_CheckNotification( hCtrl ,psSentMsg );
      ABCC_LinkFree( hCtrl, &psSentMsg );
   }
}


UINT16 ABCC_LinkGetNumCmdQueueEntries( IDL_CTRL_HDL hCtrl )
{
   UINT16 iQEntries;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   iQEntries =  ABCC_CFG_MAX_NUM_APPL_CMDS - link_bNumberOfOutstandingCommands[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];
   return iQEntries;
}


ABCC_ErrorCodeType ABCC_LinkWriteMessage( IDL_CTRL_HDL hCtrl, ABP_MsgType* psWriteMsg )
{
   BOOL fSendMsg;
   ABCC_MsgType uWriteMsg;
   BOOL fMsgWritten;
   BOOL fCmdMsg;
   ABCC_ErrorCodeType eErrorCode;
   UINT32 lAddErrorInfo;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   ABCC_PORT_UseCritical(hCtrl);

   eErrorCode = ABCC_EC_NO_ERROR;
   lAddErrorInfo = 0;

   fSendMsg = FALSE;
   uWriteMsg.psMsg = psWriteMsg;

   if( iLeTOi( psWriteMsg->sHeader.iDataSize ) > link_iMaxMsgSize )
   {
      eErrorCode = ABCC_EC_WRMSG_SIZE_ERR;
      ABCC_CbfDriverError( hCtrl, ABCC_SEV_WARNING, eErrorCode,
                           iLeTOi( psWriteMsg->sHeader.iDataSize ) );
      return( eErrorCode );
   }

   ABCC_PORT_EnterCritical(hCtrl);

   /*
   ** Check if there are any messages already queued. If both queues are empty
   ** that means there is no ongoing transmission and this message can be
   ** transmitted right now. If any of the queues are populated there is an
   ** ongoing transmission, in this case only queue the message.
   */
   if( ( link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue == 0 ) && ( link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue == 0 ) )
   {
      fSendMsg = TRUE;
   }

   /*
   ** Always queue the message even if it can be transmitted directly. This is
   ** the lock to inhibit other threads to interrupt the ongoing transmission.
   */
   if( ABCC_GetLowAddrOct( uWriteMsg.psMsg16->sHeader.iCmdReserved ) & ABP_MSG_HEADER_C_BIT )
   {
      fCmdMsg = TRUE;

      if( LINK_MAX_NUM_CMDS_IN_Q == link_bNumberOfOutstandingCommands[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] )
      {
         fSendMsg = FALSE;
         lAddErrorInfo = (UINT32)link_bNumberOfOutstandingCommands[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];
         eErrorCode = ABCC_EC_LINK_CMD_QUEUE_FULL;
      }
      else if( link_EnQueue(hCtrl, &link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex], psWriteMsg ) )
      {
         ABCC_DEBUG_MSG_EVENT( "Command queued", psWriteMsg  );
         ABCC_DEBUG_MSG_GENERAL( ( "CmdQ status: %d(%d)\n", link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue, link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize ) );

         ABCC_MemSetBufferStatus( hCtrl, psWriteMsg, ABCC_MEM_BUFSTAT_SENT );
         link_bNumberOfOutstandingCommands[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]++;
         ABCC_DEBUG_MSG_GENERAL( ( "Outstanding commands: %d\n", link_bNumberOfOutstandingCommands[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ) );
      }
      else
      {
         ABCC_DEBUG_MSG_EVENT("Command queue full", psWriteMsg );
         ABCC_DEBUG_MSG_GENERAL( ( "CmdQ status: %d(%d)\n", link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue, link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize ) );
         fSendMsg = FALSE;
         eErrorCode = ABCC_EC_LINK_CMD_QUEUE_FULL;
         ABCC_ASSERT( hCtrl,FALSE );
      }
   }
   else
   {
      fCmdMsg = FALSE;

      if( link_EnQueue( hCtrl, &link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex], psWriteMsg ) )
      {
         ABCC_DEBUG_MSG_EVENT("Response msg queued ", psWriteMsg );
         ABCC_DEBUG_MSG_GENERAL( ( "RespQ status: %d(%d)\n", link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue, link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize ) );
         ABCC_MemSetBufferStatus( hCtrl, psWriteMsg, ABCC_MEM_BUFSTAT_SENT );
      }
      else
      {
         ABCC_DEBUG_MSG_EVENT("Response queue full", psWriteMsg );
         ABCC_DEBUG_MSG_GENERAL( ( "RespQ status: %d(%d)\n", link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue, link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize ) );
         fSendMsg = FALSE;
         eErrorCode = ABCC_EC_LINK_RESP_QUEUE_FULL;
         lAddErrorInfo = (UINT32)psWriteMsg;
      }
   }

   ABCC_PORT_ExitCritical(hCtrl);

   /*
   ** Continue the transmission of the message if both queues were empty.
   ** Else the message will be transmitted at the next ABCC write message event.
   */
   if( fSendMsg && ( ( fCmdMsg && pnABCC_DrvISReadyForCmd(hCtrl) ) ||
                     ( !fCmdMsg && pnABCC_DrvISReadyForWriteMessage(hCtrl) ) ) )
   {
      fMsgWritten = pnABCC_DrvWriteMessage( hCtrl, psWriteMsg );

      ABCC_PORT_EnterCritical(hCtrl);

      /*
      ** The message has been transmitted, now it is time to dequeue it.
      */
      if( fCmdMsg )
      {
         link_DeQueue( hCtrl, &link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
         ABCC_DEBUG_MSG_EVENT( "Command dequeued", psWriteMsg );
         ABCC_DEBUG_MSG_GENERAL( ( "CmdQ status: %d(%d)\n", link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue, link_sCmdQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize ) );
      }
      else
      {
         link_DeQueue( hCtrl, &link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
         ABCC_DEBUG_MSG_EVENT( "Response dequeued", psWriteMsg );
         ABCC_DEBUG_MSG_GENERAL( ( "RespQ status: %d(%d)\n", link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bNumInQueue, link_sRespQueue[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex].bQueueSize ) );
      }

      ABCC_PORT_ExitCritical(hCtrl);

      if( fMsgWritten )
      {
         /*
         ** The message was successfully written and can be deallocated now.
         */
         link_CheckNotification( hCtrl, psWriteMsg );
         ABCC_LinkFree( hCtrl, &psWriteMsg );
      }
   }

   if( eErrorCode != ABCC_EC_NO_ERROR )
   {
      ABCC_CbfDriverError( hCtrl, ABCC_SEV_WARNING, eErrorCode, lAddErrorInfo );
   }

   return( eErrorCode );
}

ABCC_ErrorCodeType ABCC_LinkWrMsgWithNotification( IDL_CTRL_HDL hCtrl, ABP_MsgType* psWriteMsg,
                                                   ABCC_LinkNotifyIndType pnHandler )
{
   ABCC_ErrorCodeType eResult;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   /*
   ** Save callback function to call when message is successfully sent.
   */
   ABCC_ASSERT( hCtrl, pnMsgSentHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == NULL );
   pnMsgSentHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = pnHandler;
   link_psNotifyMsg[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = psWriteMsg;

   eResult = ABCC_LinkWriteMessage( hCtrl, psWriteMsg );

   return( eResult );
}


void ABCC_LinkFree( IDL_CTRL_HDL hCtrl, ABP_MsgType** ppsBuffer )
{
   ABCC_ASSERT_ERR( hCtrl, *ppsBuffer != 0, ABCC_SEV_WARNING, ABCC_EC_TRYING_TO_FREE_NULL_POINTER, 0 );

   ABCC_MemFree( hCtrl, ppsBuffer );
}

ABCC_ErrorCodeType ABCC_LinkMapMsgHandler( IDL_CTRL_HDL hCtrl, UINT8 bSrcId, ABCC_MsgHandlerFuncType  pnMSgHandler )
{
   UINT16 iIndex;
   ABCC_ErrorCodeType eResult = ABCC_EC_NO_RESOURCES;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   ABCC_PORT_UseCritical(hCtrl);

   /*
   ** Find free spot.
   */
   ABCC_PORT_EnterCritical(hCtrl);
   for ( iIndex = 0; iIndex < LINK_MAX_NUM_MSG_HDL ; iIndex++ )
   {
      if (link_pnMsgHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ] == 0 )
      {
         link_pnMsgHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ] = pnMSgHandler;
         link_bMsgSrcId[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ] = bSrcId;
         eResult = ABCC_EC_NO_ERROR;
         break;
      }
   }
   ABCC_PORT_ExitCritical(hCtrl);
   return( eResult );
}

ABCC_MsgHandlerFuncType  ABCC_LinkGetMsgHandler( IDL_CTRL_HDL hCtrl, UINT8 bSrcId )
{
   UINT16 iIndex;
   ABCC_MsgHandlerFuncType pnHandler = NULL;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   ABCC_PORT_UseCritical(hCtrl);

   /*
   ** Find message handler. If not found return NULL.
   */
   ABCC_PORT_EnterCritical(hCtrl);
   for ( iIndex = 0; iIndex < LINK_MAX_NUM_MSG_HDL ; iIndex++ )
   {
      if ( ( link_pnMsgHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ] != NULL ) && ( link_bMsgSrcId[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ] == bSrcId ) )
      {
         pnHandler = link_pnMsgHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ];
         link_pnMsgHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ] = NULL;
         break;
      }
   }
   ABCC_PORT_ExitCritical(hCtrl);
   return pnHandler;
}

BOOL ABCC_LinkIsSrcIdUsed( IDL_CTRL_HDL hCtrl, UINT8 bSrcId )
{
   BOOL fFound = FALSE;
   UINT16 iIndex;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   for ( iIndex = 0; iIndex < LINK_MAX_NUM_MSG_HDL ; iIndex++ )
   {
      if ( ( link_pnMsgHandler[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ] != NULL ) &&
          ( link_bMsgSrcId[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iIndex ] == bSrcId ) )
      {
         fFound = TRUE;
         break;
      }
   }
   return fFound;
}
