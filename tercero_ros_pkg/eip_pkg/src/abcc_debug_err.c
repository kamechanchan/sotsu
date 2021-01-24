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
** ABCC driver error and debug functions.
********************************************************************************
********************************************************************************
*/

#include "abcc_drv_cfg.h"
#include "abcc_sw_port.h"
#include "abcc_td.h"
#include "abp.h"
#include "abcc_debug_err.h"

/*******************************************************************************
** Constants
********************************************************************************
*/


/*******************************************************************************
** Typedefs
********************************************************************************
*/


/*******************************************************************************
** Public Globals
********************************************************************************
*/


/*******************************************************************************
** Private Globals
********************************************************************************
*/
#ifdef ABCC_SYS_16_BIT_CHAR
#define ABCC_GetMsgCmdField( psMsg )   ( ABCC_GetLowAddrOct( (psMsg)->sHeader.iCmdReserved ) )
#else
#define ABCC_GetMsgCmdField( psMsg )   ( (psMsg)->sHeader.bCmd )
#endif

ABCC_ErrorReporter ErrorReporter;

/*******************************************************************************
** Private Services
********************************************************************************
*/


/*******************************************************************************
** Public Services
********************************************************************************
*/
#if ABCC_CFG_DEBUG_MESSAGING
void ABCC_DebugPrintMsg( char* pcInfo, ABP_MsgType* psMsg )
{
   UINT16 i;
   UINT16 iDataSize;
   UINT8 bData;

   iDataSize = ABCC_GetMsgDataSize( psMsg );

   ABCC_PORT_DebugPrint( ( "\n%s:\n", pcInfo ) );
   ABCC_PORT_DebugPrint( ( "[ MsgBuf:0x%08x Size:0x%04x SrcId  :0x%02x DestObj:0x%02x\n  Inst  :0x%04x     Cmd :0x%02x   CmdExt0:0x%02x CmdExt1:0x%02x ]\n",
                         (UINT32)psMsg,
                          ABCC_GetMsgDataSize( psMsg ),
                          ABCC_GetMsgSourceId( psMsg ),
                          ABCC_GetMsgDestObj( psMsg ),
                          ABCC_GetMsgInstance( psMsg ),
                          ABCC_GetMsgCmdField( psMsg ),
                          ABCC_GetMsgCmdExt0( psMsg ),
                          ABCC_GetMsgCmdExt1( psMsg ) ) );

   ABCC_PORT_DebugPrint( ("[ ") );
   for( i = 0; i < iDataSize; i++ )
   {
      if( ( i % 16 ) == 15 )
      {
         ABCC_PORT_DebugPrint( ("\n  ") );
      }

      ABCC_GetMsgData8( psMsg, &bData, i );
      ABCC_PORT_DebugPrint( ("0x%02x ", bData ) );
   }

   ABCC_PORT_DebugPrint( ( "]\n\n") );
}

void ABCC_DebugPrintMsgEvent( char* pcInfo, ABP_MsgType* psMsg )
{
   ABCC_PORT_DebugPrint( ( "%s: MsgBuf:0x%08x SrcId:0x%02x\n",
                          pcInfo,(UINT32)psMsg,
                          ABCC_GetMsgSourceId( psMsg ) ) );
}
#endif


