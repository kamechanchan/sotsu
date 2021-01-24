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
** Common defines error codes and functions for abcc driver and application.
********************************************************************************
********************************************************************************
** Services :
********************************************************************************
********************************************************************************
*/

#ifndef ABCC_COMMON_H_
#define ABCC_COMMON_H_

#include "abcc_drv_cfg.h"
#include "abcc_td.h"
#include "abp.h"
#include "abcc_sys_adapt.h"
#include "abcc.h"
#include "abcc_port.h"
#include "abcc_handler.h"

typedef void (*ABCC_ErrorReporter)( IDL_CTRL_HDL hCtrl,
                                    ABCC_SeverityType eSeverity,
                                    ABCC_ErrorCodeType  iErrorCode,
                                    UINT32  lAddInfo );

void SetErrorReporter( ABCC_ErrorReporter nFunc );
EXTFUNC ABCC_ErrorReporter ErrorReporter;

/*------------------------------------------------------------------------------
** Help macros for debugging and error reporting.
**------------------------------------------------------------------------------
*/

/*
** Create constant strings for file and line information.
*/
#define STR( x )        #x
#define XSTR( x )       STR( x )
#define FileLine        "Line nr: " XSTR( __LINE__ ) "\nFile: " __FILE__

/*
** Error reporting.
*/
#if( ABCC_CFG_ERR_REPORTING_ENABLED )
 #define _ABCC_ERROR( hCtrl, eSeverity, iErrorCode, lAddInfo ) ErrorReporter( hCtrl, eSeverity, iErrorCode, lAddInfo )
#else
#define _ABCC_ERROR( hCtrl, eSeverity, iErrorCode, lAddInfo )
#endif

/*
** DEBUG Level macros.
*/
#if( ABCC_CFG_DEBUG_EVENT_ENABLED )
#define DEBUG_EVENT( args ) ABCC_PORT_DebugPrint( args )
#else
#define DEBUG_EVENT( args )
#endif

#if( ABCC_CFG_DEBUG_ERR_ENABLED )
#define DEBUG_ERR( args )   ABCC_PORT_DebugPrint( args )
#else
#define DEBUG_ERR( args )
#endif


#define ABCC_ASSERT( hCtrl, x ) if ( !( x ) ){                                        \
        DEBUG_ERR((FileLine));                                                   \
        _ABCC_ERROR( hCtrl, ABCC_SEV_FATAL , ABCC_EC_INTERNAL_ERROR, 0 );}

#define ABCC_ASSERT_ERR(hCtrl, x, eSeverity, iErrorCode, lAddInfo ) if (!( x )){      \
        DEBUG_ERR((FileLine));                                                   \
        _ABCC_ERROR( hCtrl, eSeverity , iErrorCode, lAddInfo );}

#define ABCC_ERROR( hCtrl, eSeverity, iErrorCode, lAddInfo )                         \
         DEBUG_ERR((FileLine));                                                  \
        _ABCC_ERROR( hCtrl, eSeverity , iErrorCode, lAddInfo );                        \
        ABCC_SetMainStateError(hCtrl)


#if ABCC_CFG_DEBUG_MESSAGING
/*------------------------------------------------------------------------------
** Prints ABCC message content using ABCC_PORT_DebugPrint().
** Prints: Message buffer address, message header and message data
**------------------------------------------------------------------------------
** Arguments:
**    pcInfo - General information about the debug print.
**    psMsg  - ABCC message
** Returns:
**    None
**------------------------------------------------------------------------------
*/
void ABCC_DebugPrintMsg( char* pcInfo, ABP_MsgType* psMsg );

/*------------------------------------------------------------------------------
** Prints buffer address and source id for an ABCC message.
**------------------------------------------------------------------------------
** Arguments:
**    pcInfo - General information about the debug print.
**    psMsg  - ABCC message
** Returns:
**    None
**------------------------------------------------------------------------------
*/
void ABCC_DebugPrintMsgEvent( char* pcInfo, ABP_MsgType* psMsg );

#define ABCC_DEBUG_MSG_DATA( pcInfo, psMsg )  ABCC_DebugPrintMsg( ( pcInfo ), ( psMsg ) )
#define ABCC_DEBUG_MSG_EVENT( pcInfo, psMsg ) ABCC_DebugPrintMsgEvent( ( pcInfo ), ( psMsg ) )
#define ABCC_DEBUG_MSG_GENERAL( pcInfo )      ABCC_PORT_DebugPrint( pcInfo )
#else
#define ABCC_DEBUG_MSG_DATA( pcInfo, psMsg )
#define ABCC_DEBUG_MSG_EVENT( pcInfo, psMsg )
#define ABCC_DEBUG_MSG_GENERAL( pcInfo )
#endif

#endif

/*******************************************************************************
** End of abcc_common.h
********************************************************************************
*/
