/*******************************************************************************
********************************************************************************
**                                                                            **
** ABCC Starter Kit version 2.01.01 (2015-12-14)                              **
**                                                                            **
** Delivered with:                                                            **
**    ABCC Driver 4.01.01 (2015-12-14)                                        **
**    ABP         7.16.01 (2015-10-14)                                        **
**                                                                            */
/*******************************************************************************
********************************************************************************
** COPYRIGHT NOTIFICATION (c) 2015 HMS Industrial Networks AB                 **
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
** This is an example implementation of the application abcc handler.
** It includes the following section:
** 1. ADI definition and default mappings
** 2. ABCC driver callback implementations (process data, default mapping ...)
** 3. State machine implementation for message sequencing, e.g. during user init
**    or reading exception state
** 4. ABCC handler main function to be called from the main loop, including a main
**    state machine for handling startup and restart of the ABCC.
********************************************************************************
********************************************************************************
*/

#include <stdio.h>
#include "abcc_td.h"
#include "abp.h"
#include "abcc.h"
#include "ad_obj.h"
#include "abcc_port.h"

#include "ad_obj.h"         /* Application data object:   254                 */
#include "app_obj.h"        /* Application object:        255                 */
#include "appl_abcc_handler.h"
#include "abcc_port.h"
#include "etn_obj.h"
#include "sync_obj.h"

#include "eip.h"
#include "prt.h"
#include "epl.h"
#include "dpv1.h"
#include "ect.h"
#include "dev.h"
#include "mod.h"
#include "cop.h"

#include "abcc_obj_cfg.h"
#include "appl_adi_config.h"

/*******************************************************************************
** Constants
********************************************************************************
*/

/*------------------------------------------------------------------------------
** Maximum start up time when the module is upgrading its firmware
**------------------------------------------------------------------------------
*/
#define APPL_FW_UPGRADE_STARTUP_TIME_MS     ( 2 * 60 * (UINT32)1000 )

/*******************************************************************************
** Typedefs
********************************************************************************
*/

/*------------------------------------------------------------------------------
** ABCC Handler states
**------------------------------------------------------------------------------
*/
typedef enum appl_AbccHandlerState
{
   APPL_INIT,
   APPL_WAITCOM,
   APPL_RUN,
   APPL_SHUTDOWN,
   APPL_ABCCRESET,
   APPL_DEVRESET,
   APPL_HALT
}
appl_AbccHandlerStateType;

/*------------------------------------------------------------------------------
** States of the User initialization state machine
**------------------------------------------------------------------------------
*/
typedef enum APPL_AbccUserInitState
{
   APPL_ABCC_USER_INIT_IDLE,
   APPL_ABCC_USER_NW_TYPE,
   APPL_ABCC_USER_FIRMWARE,
   APPL_ABCC_USER_SERIAL_NO,
   APPL_ABCC_USER_INIT_NODE_ID,
   APPL_ABCC_USER_INIT_IP_ADDR,
   APPL_ABCC_USER_INIT_BAUD_RATE,
   APPL_ABCC_USER_INIT_COMPLETE,
   APPL_ABCC_USER_INIT_DONE
}
appl_AbccUserInitStateType;

/*------------------------------------------------------------------------------
** States of the Exception state machine
**------------------------------------------------------------------------------
*/
typedef enum appl_ReadExceptionState
{
   APPL_NO_EXCEPTION = 0,
   APPL_READ_EXCEPTION,
   APPL_READ_EXCEPTION_INFO,
   APPL_READ_EXCEPTION_DONE
}
appl_ReadExceptionStateType;

/*******************************************************************************
** Public Globals
********************************************************************************
*/

/*******************************************************************************
** Private Globals
********************************************************************************
*/

/*------------------------------------------------------------------------------
** Current state of the ABCC
**------------------------------------------------------------------------------
*/
static appl_AbccHandlerStateType appl_eAbccHandlerState = APPL_INIT;

/*------------------------------------------------------------------------------
** Current anybus state
**------------------------------------------------------------------------------
*/
static volatile ABP_AnbStateType appl_eAnbState = ABP_ANB_STATE_SETUP;

/*------------------------------------------------------------------------------
** Network Node address.
**------------------------------------------------------------------------------
*/
static UINT8 appl_bNwNodeAddress;
static UINT8 appl_bLastSetNwNodeAddress;

/*------------------------------------------------------------------------------
** Network IP address.
**------------------------------------------------------------------------------
*/
static UINT8 appl_abNwIpAddress[ 4 ] = { 192, 168, 0, 0 };
static UINT8 appl_bLastSetNwIpAddress;

/*------------------------------------------------------------------------------
** Common address variables (used for both node address and IP address depending
** on network type).
**------------------------------------------------------------------------------
*/
static BOOL appl_fSetAddr = FALSE;

/*------------------------------------------------------------------------------
** Network baud rate.
**------------------------------------------------------------------------------
*/
static UINT8 appl_bNwBaudRate;
static UINT8 appl_bLastSetNwBaudRate;
static BOOL  appl_fSetBaudRate = FALSE;

/*------------------------------------------------------------------------------
** Flags to keep track of if the network type supports node ID/IP address
** and/or baud rate to be set.
**------------------------------------------------------------------------------
*/
static BOOL appl_fNwSupportsNodeId;
static BOOL appl_fNwSupportsBaudRate;

/*------------------------------------------------------------------------------
** ABCC User init state machine globals
**------------------------------------------------------------------------------
*/
static BOOL appl_fUserInitWaitForResp = FALSE;
static appl_AbccUserInitStateType appl_eUserInitState = APPL_ABCC_USER_INIT_IDLE;

/*------------------------------------------------------------------------------
** ABCC Exception state machine globals
**------------------------------------------------------------------------------
*/
static BOOL appl_fExceptionWaitForResp = FALSE;
static volatile appl_ReadExceptionStateType appl_eReadExceptionState = APPL_NO_EXCEPTION;

/*------------------------------------------------------------------------------
** Set to TRUE when an unexpected error occur. The main state machine will
** return APPL_MODULE_UNEXPECTED_ERROR when this flag is set.
**------------------------------------------------------------------------------
*/
static BOOL appl_fUnexpectedError = FALSE;

/*------------------------------------------------------------------------------
** Event flags used by application to invoke the corresponding
** ABCC_Trigger<event_action> function from the desired context. The flag will
** be set to TRUE in ABCC_CbfEvent().
**------------------------------------------------------------------------------
*/
static volatile BOOL appl_fMsgReceivedEvent = FALSE;
static volatile BOOL appl_fRdPdReceivedEvent = FALSE;
static volatile BOOL appl_fTransmitMsgEvent = FALSE;
static volatile BOOL appl_fAbccStatusEvent = FALSE;

/*------------------------------------------------------------------------------
** Forward declarations
**------------------------------------------------------------------------------
*/
static void ReadException( IDL_CTRL_HDL hCtrl );
static void HandleExceptionResp( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg );
static void HandleExceptionInfoResp( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg );
static void HandleUserInitResp( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg );
static void HandleResp( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg );
static BOOL SetSw1Sw2NcInstance( IDL_CTRL_HDL hCtrl,
                                 UINT16 iInstance,
                                 UINT8 bValue,
                                 ABCC_MsgHandlerFuncType pnMsgHandler );
static BOOL SetIpAddressNcInstance( IDL_CTRL_HDL hCtrl,
                                    ABCC_MsgHandlerFuncType pnMsgHandler );

/*******************************************************************************
** Private Services
********************************************************************************
*/

/*------------------------------------------------------------------------------
** This is the trigger call for reading exception information from the ABCC.
** It is triggered when ABP_ANB_STATE_EXCEPTION is entered in
** ABCC_CbfAnbStatusChanged().
** Both Exception and Exception info attribute is read from the Anybus object.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl - Handle to the hardware.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void ReadException( IDL_CTRL_HDL hCtrl )
{
   appl_eReadExceptionState = APPL_READ_EXCEPTION;
}

/*------------------------------------------------------------------------------
** State machine used to execute the reading of exception cause
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl - Handle to the hardware.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void RunExceptionSM( IDL_CTRL_HDL hCtrl )
{
   ABP_MsgType* psMsg;

   if( !appl_fExceptionWaitForResp )
   {
      switch( appl_eReadExceptionState )
      {
      case APPL_NO_EXCEPTION:
      case APPL_READ_EXCEPTION_DONE:

         break;

      case APPL_READ_EXCEPTION:

         psMsg = ABCC_GetCmdMsgBuffer(hCtrl);

         if( psMsg != NULL )
         {
            appl_fExceptionWaitForResp = TRUE;
            ABCC_GetAttribute( psMsg, ABP_OBJ_NUM_ANB, 1, ABP_ANB_IA_EXCEPTION, ABCC_GetNewSourceId(hCtrl) );
            if( ABCC_SendCmdMsg( hCtrl, psMsg, HandleExceptionResp ) != ABCC_EC_NO_ERROR )
            {
               APPL_UnexpectedError();
               appl_fExceptionWaitForResp = FALSE;

               return;
            }
         }

         break;

      case APPL_READ_EXCEPTION_INFO:

         psMsg = ABCC_GetCmdMsgBuffer(hCtrl);

         if( psMsg != NULL )
         {
            appl_fExceptionWaitForResp = TRUE;
            ABCC_GetAttribute( psMsg, ABP_OBJ_NUM_NW, 1, ABP_NW_IA_EXCEPTION_INFO, ABCC_GetNewSourceId(hCtrl) );
            if( ABCC_SendCmdMsg( hCtrl, psMsg, HandleExceptionInfoResp ) != ABCC_EC_NO_ERROR )
            {
               APPL_UnexpectedError();
               appl_fExceptionWaitForResp = FALSE;

               return;
            }
         }

         break;

      default:

         break;
      }
   }
}

/*------------------------------------------------------------------------------
** Response handler to the command reading the Exception attribute of the
** Anybus object.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl - Handle to the hardware.
**    psMsg - Pointer to the received response message
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void HandleExceptionResp( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg )
{
   UINT8 bException;

   if( ABCC_VerifyMessage( psMsg ) != ABCC_EC_NO_ERROR )
   {
      APPL_UnexpectedError();
      return;
   }

   ABCC_GetMsgData8( psMsg, &bException, 0 );
   ABCC_PORT_DebugPrint( ( "Exception Code:    %X:\n\n", bException ) );

   appl_eReadExceptionState++;
   appl_fExceptionWaitForResp = FALSE;

   (void)bException;
}

/*------------------------------------------------------------------------------
** Response handler to the command reading the Exception info attribute of the
** Anybus object.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl - Handle to the hardware.
**    psMsg - Pointer to the received response message
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void HandleExceptionInfoResp( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg )
{
   UINT8 bExceptionInfo;

   if( ABCC_VerifyMessage( psMsg ) != ABCC_EC_NO_ERROR )
   {
      APPL_UnexpectedError();
      return;
   }

   ABCC_GetMsgData8( psMsg, &bExceptionInfo, 0 );
   ABCC_PORT_DebugPrint( ( "Exception Info:    %X:\n\n", bExceptionInfo ) );

   appl_eReadExceptionState++;
   appl_fExceptionWaitForResp = FALSE;

   (void)bExceptionInfo;
}

/*------------------------------------------------------------------------------
** State machine used to execute the user initialization during the Anybus
** Setup state.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl - Handle to the hardware.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void RunUserInitSM( IDL_CTRL_HDL hCtrl )
{
   if( !appl_fUserInitWaitForResp )
   {
      ABP_MsgType* psMsgCmd;

      switch( appl_eUserInitState )
      {
      case APPL_ABCC_USER_INIT_IDLE:
      case APPL_ABCC_USER_INIT_DONE:

         break;

      case APPL_ABCC_USER_NW_TYPE:
         /*
          ** Get the 'network type string' request sent.
          */
         psMsgCmd = ABCC_GetCmdMsgBuffer(hCtrl);
         ABCC_GetAttribute( psMsgCmd, ABP_OBJ_NUM_NW, 1, ABP_NW_IA_NW_TYPE_STR, ABCC_GetNewSourceId(hCtrl) );
         ABCC_SendCmdMsg( hCtrl, psMsgCmd, HandleUserInitResp );
         appl_fUserInitWaitForResp = TRUE;

         break;

      case APPL_ABCC_USER_FIRMWARE:
         /*
         ** Get the 'firmware version' request sent.
         */
         psMsgCmd = ABCC_GetCmdMsgBuffer(hCtrl);
         ABCC_GetAttribute( psMsgCmd, ABP_OBJ_NUM_ANB, 1, ABP_ANB_IA_FW_VERSION, ABCC_GetNewSourceId(hCtrl) );
         ABCC_SendCmdMsg( hCtrl, psMsgCmd, HandleUserInitResp );
         appl_fUserInitWaitForResp = TRUE;

         break;

      case APPL_ABCC_USER_SERIAL_NO:
         /*
         ** Get serial number.
         */
         psMsgCmd = ABCC_GetCmdMsgBuffer(hCtrl);
         ABCC_GetAttribute( psMsgCmd, ABP_OBJ_NUM_ANB, 1, ABP_ANB_IA_SERIAL_NUM, ABCC_GetNewSourceId(hCtrl) );
         ABCC_SendCmdMsg( hCtrl, psMsgCmd, HandleUserInitResp );
         appl_fUserInitWaitForResp = TRUE;

         printf("Network type %x",ABCC_NetworkType(hCtrl));

         break;


      case APPL_ABCC_USER_INIT_NODE_ID:

         if( appl_fNwSupportsNodeId &&
             appl_fSetAddr )
         {
            appl_fUserInitWaitForResp = TRUE;

            if( SetSw1Sw2NcInstance( hCtrl,
                                     ABP_NC_INST_NUM_SW1,
                                     appl_bNwNodeAddress,
                                     HandleUserInitResp ) )
            {
               appl_bLastSetNwNodeAddress = appl_bNwNodeAddress;
            }
            else
            {
               appl_fUserInitWaitForResp = FALSE;
            }
         }
         else
         {
            appl_eUserInitState++;
         }

         break;

      case APPL_ABCC_USER_INIT_IP_ADDR:

         if( ( !appl_fNwSupportsNodeId ) &&
             ( appl_fSetAddr ) &&
             ( appl_abNwIpAddress[ 3 ] != 0 ) )
         {
            appl_fUserInitWaitForResp = TRUE;

            if( SetIpAddressNcInstance( hCtrl,
                                        HandleUserInitResp ) )
            {
               appl_bLastSetNwIpAddress = appl_abNwIpAddress[ 3 ];
            }
            else
            {
               appl_fUserInitWaitForResp = FALSE;
            }
         }
         else
         {
            appl_eUserInitState++;
         }

         break;

      case APPL_ABCC_USER_INIT_BAUD_RATE:

         if( appl_fNwSupportsBaudRate &&
             appl_fSetBaudRate )
         {
            appl_fUserInitWaitForResp = TRUE;

            if( SetSw1Sw2NcInstance( hCtrl,
                                     ABP_NC_INST_NUM_SW2,
                                     appl_bNwBaudRate,
                                     HandleUserInitResp ) )
            {
               appl_bLastSetNwBaudRate = appl_bNwBaudRate;
            }
            else
            {
               appl_fUserInitWaitForResp = FALSE;
            }
         }
         else
         {
            appl_eUserInitState++;
         }

         break;

      case APPL_ABCC_USER_INIT_COMPLETE:

         ABCC_UserInitComplete(hCtrl);
         appl_eUserInitState++;

         break;

      default:

         break;
      }
   }
}

/*------------------------------------------------------------------------------
** Use this function to set a value to NC instance 1 (Switch 1) or 2 (Switch 2).
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl        - Handle to the hardware.
**    iInstance    - NC instance to set (ABP_NC_INST_NUM_SW1 or
**                   ABP_NC_INST_NUM_SW2).
**    bValue       - Value to be set.
**    pnMsgHandler - Pointer to the function to handle the response message.
**
** Returns:
**    TRUE  - Set command was successfully sent.
**    FALSE - Failed to send command.
**------------------------------------------------------------------------------
*/
static BOOL SetSw1Sw2NcInstance( IDL_CTRL_HDL hCtrl,
                                 UINT16 iInstance,
                                 UINT8 bValue,
                                 ABCC_MsgHandlerFuncType pnMsgHandler )
{
   ABP_MsgType* psMsg;

   psMsg = ABCC_GetCmdMsgBuffer(hCtrl);

   if( psMsg != NULL )
   {
      appl_fUserInitWaitForResp = TRUE;
      ABCC_SetByteAttribute( psMsg, ABP_OBJ_NUM_NC,
                             iInstance,
                             ABP_NC_VAR_IA_VALUE,
                             bValue,
                             ABCC_GetNewSourceId(hCtrl) );

      if( ABCC_SendCmdMsg( hCtrl, psMsg, pnMsgHandler ) !=
          ABCC_EC_NO_ERROR )
      {
         APPL_UnexpectedError();

         return( FALSE );
      }
   }
   else
   {
      return( FALSE );
   }

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Use this function to set the value stored in appl_abNwIpAddress to the IP
** address NC instance (instance 3).
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl        - Handle to the hardware.
**    pnMsgHandler - Pointer to the function to handle the response message.
**
** Returns:
**    TRUE  - Set command was successfully sent.
**    FALSE - Failed to send command.
**------------------------------------------------------------------------------
*/
static BOOL SetIpAddressNcInstance( IDL_CTRL_HDL hCtrl,
                                    ABCC_MsgHandlerFuncType pnMsgHandler )
{
   ABP_MsgType* psMsg;

   psMsg = ABCC_GetCmdMsgBuffer(hCtrl);

   if( psMsg != NULL )
   {
      ABCC_SetMsgHeader( psMsg,
                         ABP_OBJ_NUM_NC,
                         3,
                         ABP_NC_VAR_IA_VALUE,
                         ABP_CMD_SET_ATTR,
                         4,
                         ABCC_GetNewSourceId(hCtrl) );

      ABCC_SetMsgString( psMsg, (char*)&appl_abNwIpAddress[ 0 ], 4, 0 );

      if( ABCC_SendCmdMsg( hCtrl, psMsg, pnMsgHandler ) !=
          ABCC_EC_NO_ERROR )
      {
         APPL_UnexpectedError();

         return( FALSE );
      }
   }
   else
   {
      return( FALSE );
   }

   return( TRUE );
}

/*------------------------------------------------------------------------------
** Checks if the IP address/node address and/or baud rate has changes
** and sets the updated values to the corresponding network configuration
** instances if it has.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl        - Handle to the hardware.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void UpdateSwitchValues( IDL_CTRL_HDL hCtrl )
{
   /*
   ** Check if the IP address/node address has been changed and take appropriate
   ** action depending on network type.
   */
   if( ( appl_bLastSetNwIpAddress != appl_abNwIpAddress[ 3 ] ) &&
       ( appl_abNwIpAddress[ 3 ] != 0 ) &&
       ( !appl_fNwSupportsNodeId ) &&
       ( appl_fSetAddr ) )
   {
      if( SetIpAddressNcInstance( hCtrl, HandleResp ) )
      {
         appl_bLastSetNwIpAddress = appl_abNwIpAddress[ 3 ];
      }
   }
   else if( ( appl_bLastSetNwNodeAddress != appl_bNwNodeAddress ) &&
            ( appl_fNwSupportsNodeId ) &&
            ( appl_fSetAddr ) )
   {
      if( SetSw1Sw2NcInstance( hCtrl,
                               ABP_NC_INST_NUM_SW1,
                               appl_bNwNodeAddress,
                               HandleResp ) )
      {
         appl_bLastSetNwNodeAddress = appl_bNwNodeAddress;
      }
   }

   /*
   ** Check if baud rate has changed and if a baud rate can be set for the
   ** current network type.
   */
   if( ( appl_bLastSetNwBaudRate != appl_bNwBaudRate ) &&
       ( appl_fNwSupportsBaudRate ) &&
       ( appl_fSetBaudRate ) )
   {
      if( SetSw1Sw2NcInstance( hCtrl,
                               ABP_NC_INST_NUM_SW2,
                               appl_bNwBaudRate,
                               HandleResp ) )
      {
         appl_bLastSetNwBaudRate = appl_bNwBaudRate;
      }
   }
}

/*------------------------------------------------------------------------------
** Response handler to the commands sent by the User initialization state
** machine.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl - Handle to the hardware.
**    psMsg - Pointer to the received response message
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void HandleUserInitResp( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg )
{
   if( ABCC_VerifyMessage( psMsg ) != ABCC_EC_NO_ERROR )
   {
      APPL_UnexpectedError();
      return;
   }

   switch(appl_eUserInitState) {
   case APPL_ABCC_USER_NW_TYPE:
       psMsg->abData[ psMsg->sHeader.iDataSize ] = 0;
       printf( "\nNetwork type:     %s\n", psMsg->abData );
       break;

   case APPL_ABCC_USER_FIRMWARE:
       printf( "\nFirmware version: %d.%.2d build %d\n",
               psMsg->abData[ 0 ] ,
               psMsg->abData[ 1 ] ,
               psMsg->abData[ 2 ] );
       break;

   case APPL_ABCC_USER_SERIAL_NO:
       printf( "\nSerial number:    %X:%X:%X:%X\n\n",
               psMsg->abData[ 3 ],
               psMsg->abData[ 2 ],
               psMsg->abData[ 1 ],
               psMsg->abData[ 0 ] );
       break;
   default:
       break;
   };

   appl_fUserInitWaitForResp = FALSE;
   appl_eUserInitState++;
}

/*------------------------------------------------------------------------------
** Generic response handler, log error if an error was returned by the ABCC.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl - Handle to the hardware
**    psMsg - Pointer to the received response message
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void HandleResp( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsg )
{
   if( ABCC_VerifyMessage( psMsg ) != ABCC_EC_NO_ERROR )
   {
      APPL_UnexpectedError();
      return;
   }
}

/*******************************************************************************
** Public Services
********************************************************************************
*/

APPL_AbccHandlerStatusType APPL_HandleAbcc( IDL_CTRL_HDL hCtrl )
{
   static APPL_AbccHandlerStatusType eModuleStatus = APPL_MODULE_NO_ERROR;
   UINT32 lStartupTimeMs;
   ABCC_CommunicationStateType eAbccComState;

   switch( appl_eAbccHandlerState )
   {
   case APPL_INIT:

      eModuleStatus = APPL_MODULE_NO_ERROR;
      appl_eUserInitState = APPL_ABCC_USER_INIT_IDLE;
      appl_fUserInitWaitForResp = FALSE;
      appl_eReadExceptionState = APPL_NO_EXCEPTION;
      appl_fExceptionWaitForResp = FALSE;
      appl_fMsgReceivedEvent = FALSE;
      appl_fRdPdReceivedEvent = FALSE;
      appl_fTransmitMsgEvent = FALSE;
      appl_fAbccStatusEvent = FALSE;

      if( !ABCC_ModuleDetect(hCtrl) )
      {
         eModuleStatus = APPL_MODULE_NOT_DETECTED;
      }

      if( eModuleStatus == APPL_MODULE_NO_ERROR )
      {
         /*
         ** Init application data object
         */
         if( AD_Init( APPL_asAdiEntryList,
                      APPL_GetNumAdi(),
                      APPL_asAdObjDefaultMap ) != AD_NO_ERROR )
         {
            eModuleStatus = APPL_MODULE_UNEXPECTED_ERROR ;
         }

#if APP_OBJ_ENABLE
         if( APP_GetCandidateFwAvailable() == TRUE )
         {
            lStartupTimeMs = APPL_FW_UPGRADE_STARTUP_TIME_MS;
         }
         else
#endif
         {
            /*
            ** Default time will be used
            */
            lStartupTimeMs = 0;
         }

         if( ABCC_StartDriver( hCtrl, lStartupTimeMs ) == ABCC_EC_NO_ERROR )
         {
            ABCC_HWReleaseReset(hCtrl);
            appl_eAbccHandlerState = APPL_WAITCOM;
         }
         else
         {
            eModuleStatus = APPL_MODULE_NOT_ANSWERING;
         }
      }

      if( eModuleStatus != APPL_MODULE_NO_ERROR )
      {
         appl_eAbccHandlerState = APPL_HALT;
      }

      break;

   case APPL_WAITCOM:

      eAbccComState = ABCC_isReadyForCommunication(hCtrl);

      if( eAbccComState == ABCC_READY_FOR_COMMUNICATION )
      {
         appl_eAbccHandlerState = APPL_RUN;
      }
      else if( eAbccComState == ABCC_COMMUNICATION_ERROR )
      {
         appl_eAbccHandlerState = APPL_HALT;
         eModuleStatus = APPL_MODULE_NOT_ANSWERING;
      }
      break;

   case APPL_RUN:
      /*------------------------------------------------------------------------
      ** Handle events indicated in ABCC_CbfEvent()callback function.
      ** Note that these events could be handled from any chosen context but in
      ** in this application it is done from main loop context.
      **------------------------------------------------------------------------
      */
      if( appl_fRdPdReceivedEvent )
      {
         appl_fRdPdReceivedEvent = FALSE;
         ABCC_TriggerRdPdUpdate(hCtrl);
      }

      if( appl_fMsgReceivedEvent )
      {
         appl_fMsgReceivedEvent = FALSE;
         ABCC_TriggerReceiveMessage(hCtrl);
      }

      if( appl_fTransmitMsgEvent )
      {
         appl_fTransmitMsgEvent = FALSE;
         ABCC_TriggerTransmitMessage(hCtrl);
      }

      if( appl_fAbccStatusEvent )
      {
         appl_fAbccStatusEvent = FALSE;
         ABCC_TriggerAnbStatusUpdate(hCtrl);
      }
      /*
      ** End event handling.
      */

#if SYNC_OBJ_ENABLE
      if( SYNC_GetMode(hCtrl) == SYNC_MODE_NONSYNCHRONOUS )
      {
         ABCC_TriggerWrPdUpdate(hCtrl);
      }
#else
      /*
      ** Always update write process data
      */
      ABCC_TriggerWrPdUpdate(hCtrl);
#endif

      ABCC_RunDriver(hCtrl);

      if( appl_eAnbState == ABP_ANB_STATE_SETUP )
      {
         RunUserInitSM(hCtrl);
      }
      else if( appl_eAnbState == ABP_ANB_STATE_EXCEPTION )
      {
         RunExceptionSM(hCtrl);
      }
      else
      {
         UpdateSwitchValues(hCtrl);
      }

      break;

   case APPL_SHUTDOWN:

      ABCC_HWReset(hCtrl);
      eModuleStatus = APPL_MODULE_SHUTDOWN;
      appl_eAbccHandlerState = APPL_HALT;

      break;

   case APPL_ABCCRESET:

      ABCC_HWReset(hCtrl);
      appl_eAbccHandlerState = APPL_INIT;
      eModuleStatus = APPL_MODULE_NO_ERROR;
      appl_eAnbState = ABP_ANB_STATE_SETUP;
      appl_eReadExceptionState = APPL_NO_EXCEPTION;
      appl_fExceptionWaitForResp = FALSE;
      appl_fUserInitWaitForResp = FALSE;
      break;

   case APPL_DEVRESET:

      ABCC_HWReset(hCtrl);
      eModuleStatus = APPL_MODULE_RESET;
      appl_eAbccHandlerState = APPL_HALT;

      break;

   case APPL_HALT:

      break;

   default:

      break;
   }

   if( appl_fUnexpectedError )
   {
      return( APPL_MODULE_UNEXPECTED_ERROR );
   }

   return( eModuleStatus );
}

void APPL_SetAddress( IDL_CTRL_HDL hCtrl, UINT8 bSwitchValue )
{
   /*
   ** HW switch 1 will the last octet in the IP address
   ** for applicable networks ( 192.168.0.X )
   */
   appl_abNwIpAddress[ 3 ] = bSwitchValue;
   appl_fSetAddr = TRUE;

   /*
   ** Switch 1 is node address for applicable networks
   */
   appl_bNwNodeAddress = bSwitchValue;

   /*
   ** Indicate to application object that the address is set by HW switches
   */
#if APP_OBJ_ENABLE
   APP_HwConfAddress( TRUE );
#endif
}

void APPL_SetBaudrate( IDL_CTRL_HDL hCtrl, UINT8 bSwitchValue )
{
   appl_bNwBaudRate = bSwitchValue;

   appl_fSetBaudRate = TRUE;
}

void APPL_UnexpectedError( void )
{
   appl_fUnexpectedError = TRUE;
}

void APPL_RestartAbcc( IDL_CTRL_HDL hCtrl )
{
   appl_eAbccHandlerState = APPL_ABCCRESET;
}

void APPL_Shutdown( IDL_CTRL_HDL hCtrl )
{
   appl_eAbccHandlerState = APPL_SHUTDOWN;
}


void APPL_Reset( IDL_CTRL_HDL hCtrl )
{
   appl_eAbccHandlerState = APPL_DEVRESET;
}

UINT16  ABCC_CbfAdiMappingReq( IDL_CTRL_HDL hCtrl,
                               const AD_AdiEntryType**  const ppsAdiEntry,
                               const AD_DefaultMapType** const ppsDefaultMap )
{
   return AD_AdiMappingReq( hCtrl, ppsAdiEntry, ppsDefaultMap );
}

BOOL ABCC_CbfUpdateWriteProcessData( IDL_CTRL_HDL hCtrl, void* pxWritePd )
{
   /*
   ** AD_UpdatePdWriteData is a general function that updates all ADI:s according
   ** to current map.
   ** If the ADI mapping is fixed there is potential for doing that in a more
   ** optimized way, for example by using memcpy.
   */
   return( AD_UpdatePdWriteData( pxWritePd ) );
}

#if( ABCC_CFG_REMAP_SUPPORT_ENABLED )
void ABCC_CbfRemapDone( IDL_CTRL_HDL hCtrl )
{
   AD_RemapDone(hCtrl);
}
#endif

void ABCC_CbfNewReadPd( IDL_CTRL_HDL hCtrl,void* pxReadPd )
{
   /*
   ** AD_UpdatePdReadData is a general function that updates all ADI:s according
   ** to current map.
   ** If the ADI mapping is fixed there is potential for doing that in a more
   ** optimized way, for example by using memcpy.
   */
   AD_UpdatePdReadData( pxReadPd );
}

void ABCC_CbfDriverError( IDL_CTRL_HDL hCtrl,
                          ABCC_SeverityType eSeverity,
                          ABCC_ErrorCodeType iErrorCode,
                          UINT32 lAddInfo )
{
   (void)eSeverity;
   (void)iErrorCode;
   (void)lAddInfo;
   ABCC_PORT_DebugPrint( ( "ERROR: Severity:%d Error code:%d, AddInfo:%d\n",
                           eSeverity, iErrorCode, lAddInfo ) );
}

void ABCC_CbfReceiveMsg( IDL_CTRL_HDL hCtrl, ABP_MsgType* psReceivedMsg )
{
   switch(  ABCC_GetMsgDestObj( psReceivedMsg ) )
   {
#if EPL_OBJ_ENABLE
   case ABP_OBJ_NUM_EPL:

      EPL_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if EIP_OBJ_ENABLE
   case ABP_OBJ_NUM_EIP:

      EIP_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if PRT_OBJ_ENABLE
   case ABP_OBJ_NUM_PNIO:

      PRT_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if DPV1_OBJ_ENABLE
   case ABP_OBJ_NUM_DPV1:

      DPV1_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if DEV_OBJ_ENABLE
   case ABP_OBJ_NUM_DEV:

      DEV_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if MOD_OBJ_ENABLE
   case ABP_OBJ_NUM_MOD:

      MOD_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if COP_OBJ_ENABLE
   case ABP_OBJ_NUM_COP:

      COP_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if ETN_OBJ_ENABLE
   case ABP_OBJ_NUM_ETN:

      ETN_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if ECT_OBJ_ENABLE
   case ABP_OBJ_NUM_ECT:

      ECT_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
   case ABP_OBJ_NUM_APPD:

      AD_ProcObjectRequest( hCtrl, psReceivedMsg );
      break;

#if APP_OBJ_ENABLE
   case ABP_OBJ_NUM_APP:

      APP_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif
#if SYNC_OBJ_ENABLE
   case ABP_OBJ_NUM_SYNC:

      SYNC_ProcessCmdMsg( hCtrl, psReceivedMsg );
      break;
#endif

   default:

      /*
      ** We have received a command to an unsupported object.
      */
      ABP_SetMsgErrorResponse( psReceivedMsg, 1, ABP_ERR_UNSUP_OBJ );
      ABCC_SendRespMsg( hCtrl, psReceivedMsg );
      break;
   }
}

void ABCC_CbfWdTimeout( IDL_CTRL_HDL hCtrl )
{
   ABCC_PORT_DebugPrint( ( "ABCC watchdog timeout" ) );
}

void ABCC_CbfWdTimeoutRecovered( IDL_CTRL_HDL hCtrl )
{
   ABCC_PORT_DebugPrint( ( "ABCC watchdog recovered" ) );
}

#if ABCC_CFG_SYNC_ENABLE
void ABCC_CbfSyncIsr( IDL_CTRL_HDL hCtrl )
{
   /*
   ** Call application specific handling of sync event
   */
   APPL_SyncIsr(hCtrl);
}
#endif

void ABCC_CbfEvent( IDL_CTRL_HDL hCtrl, UINT16 iEvents )
{

   /*
   ** Set flag to indicate that an event has occurred and the corresponding
   ** ABCC_Trigger<event_action> must be called. In the sample code the the
   ** trigger function is called from main loop context.
   */
   if( iEvents & ABCC_ISR_EVENT_RDPD )
   {
      appl_fRdPdReceivedEvent = TRUE;
   }

   if( iEvents & ABCC_ISR_EVENT_RDMSG )
   {
      appl_fMsgReceivedEvent = TRUE;
   }

   if( iEvents & ABCC_ISR_EVENT_WRMSG )
   {
      appl_fTransmitMsgEvent = TRUE;
   }

   if( iEvents & ABCC_ISR_EVENT_STATUS  )
   {
      appl_fAbccStatusEvent = TRUE;
   }
}

void ABCC_CbfAnbStateChanged( IDL_CTRL_HDL hCtrl, ABP_AnbStateType eNewAnbState )
{
   static const char* AnbStateString[ 8 ] = { "ABP_ANB_STATE_SETUP",
                                              "ABP_ANB_STATE_NW_INIT",
                                              "ABP_ANB_STATE_WAIT_PROCESS",
                                              "ABP_ANB_STATE_IDLE",
                                              "ABP_ANB_STATE_PROCESS_ACTIVE",
                                              "ABP_ANB_STATE_ERROR",
                                              "",
                                              "ABP_ANB_STATE_EXCEPTION" };
   (void)AnbStateString[ 0 ];

   appl_eAnbState = eNewAnbState;
   ABCC_PORT_DebugPrint( ( "ANB_STATUS: %s \n" ,
                           AnbStateString[ appl_eAnbState ] ) );

   switch( appl_eAnbState )
   {
   case ABP_ANB_STATE_PROCESS_ACTIVE:

      ABCC_TriggerWrPdUpdate(hCtrl);
      break;

   case ABP_ANB_STATE_EXCEPTION:

      /* Trigger message sequence for reading exception data */
      ReadException(hCtrl);
      break;

   case ABP_ANB_STATE_ERROR:

      /* Trigger message sequence for reading exception data */
      break;

   default:

      break;
   }
}

void ABCC_CbfUserInitReq( IDL_CTRL_HDL hCtrl )
{
   UINT16 iNetworkType;

   iNetworkType = ABCC_NetworkType(hCtrl);

   if( ( iNetworkType == ABP_NW_TYPE_DEV ) ||
       ( iNetworkType == ABP_NW_TYPE_PDPV0 ) ||
       ( iNetworkType == ABP_NW_TYPE_PDPV1 ) ||
       ( iNetworkType == ABP_NW_TYPE_COP ) ||
       ( iNetworkType == ABP_NW_TYPE_CNT ) ||
       ( iNetworkType == ABP_NW_TYPE_CCL ) ||
       ( iNetworkType == ABP_NW_TYPE_CPN ) ||
       ( iNetworkType == ABP_NW_TYPE_ECT ) ||
       ( iNetworkType == ABP_NW_TYPE_EPL ) )
   {
      appl_fNwSupportsNodeId = TRUE;
   }
   else
   {
      appl_fNwSupportsNodeId = FALSE;
   }

   if( ( iNetworkType == ABP_NW_TYPE_DEV ) ||
       ( iNetworkType == ABP_NW_TYPE_COP ) ||
       ( iNetworkType == ABP_NW_TYPE_CCL ) )
   {
      appl_fNwSupportsBaudRate = TRUE;
   }
   else
   {
      appl_fNwSupportsBaudRate = FALSE;
   }

   appl_eUserInitState++;
}
