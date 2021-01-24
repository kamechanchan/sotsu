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
** Implementation of remap function
********************************************************************************
********************************************************************************
*/


#include "abcc_drv_cfg.h"

#if( ABCC_CFG_REMAP_SUPPORT_ENABLED )

#include "abcc_td.h"
#include "abp.h"
#include "abcc.h"
#include "abcc_handler.h"
#include "abcc_link.h"

#include "IDLTp.h"

/*******************************************************************************
** Public Globals
********************************************************************************
*/

/*******************************************************************************
** Private Globals
********************************************************************************
*/


static UINT16   abcc_iNewPdReadSize[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]    = {{0}};
static UINT16   abcc_iNewPdWriteSize[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]   = {{0}};


static void abcc_RemapRespMsgSent( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   ABCC_SetPdSize( hCtrl,
       abcc_iNewPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex],
       abcc_iNewPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );

   ABCC_CbfRemapDone(hCtrl);
}

ABCC_ErrorCodeType ABCC_SendRemapRespMsg( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsgResp, UINT16 iNewReadPdSize, const UINT16 iNewWritePdSize )
{
   ABCC_ErrorCodeType eResult;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   abcc_iNewPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = iNewReadPdSize;
   abcc_iNewPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = iNewWritePdSize;

   /*
    ** When ack is sent abcc_RemapRespMsgSent will be called.
    */
   eResult = ABCC_LinkWrMsgWithNotification(  hCtrl, psMsgResp, abcc_RemapRespMsgSent );

   return( eResult );
}

#endif

