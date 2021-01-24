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
** Implementation of abcc setup state machine
********************************************************************************
********************************************************************************
*/

#include "abcc_drv_cfg.h"
#include "abcc_td.h"
#include "abcc_drv_if.h"
#include "abp.h"
#include "abcc.h"
#include "abcc_handler.h"
#include "abcc_link.h"
#include "abcc_mem.h"
#include "abcc_sys_adapt.h"
#include "abcc_debug_err.h"
#include "abcc_timer.h"

#include "IDLTp.h"


/*******************************************************************************
** Constants
********************************************************************************
*/

/*
** Invalid ADI index.
*/
#define AD_INVALID_ADI_INDEX           ( 0xffff )

/*******************************************************************************
** Typedefs
********************************************************************************
*/

typedef enum CmdStateType
{
   /*
   ** Order dependent states.
   */
   MSG_GET_DATA_FORMAT,
   MSG_GET_NW_PARAM_SUPPORT,
   MSG_GET_MODULE_TYPE,
   MSG_GET_NETWORK_TYPE,
   MSG_MAP_PD_READ_WRITE,
   MSG_USER_INIT_START,
   MSG_USER_INIT,
   MSG_READ_RDPD_SIZE,
   MSG_READ_WRPD_SIZE,
   MSG_SETUP_COMPLETE,
   MSG_DONE
}
CmdStateType;

/*******************************************************************************
** Private Globals
********************************************************************************
*/

/*------------------------------------------------------------------------------
** iModuleType       - ABCC module type (read out during SETUP state)
** iNetworkType      - ABCC network type (read out during SETUP state)
** abcc_iPdReadSize  - Read process data size
** abcc_iPdWriteSize - Write process data size
** eParameterSupport - Parameter support (read out during SETUP state)
**------------------------------------------------------------------------------
*/
static UINT16   iModuleType[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];
static UINT16   iNetworkType[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];
static NetFormatType     eNetFormat[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];
static ParameterSupportType eParameterSupport[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT];

static CmdStateType      eCmdState[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]  = {{MSG_GET_DATA_FORMAT}};
/*
** Help varibales for ADI mapping servcie
*/
static AD_AdiEntryType*    psAdiEntry[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]     = {{NULL}};
static AD_DefaultMapType*  psDefaultMap[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]   = {{NULL}};
static UINT16              abcc_iNumAdi[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]  = {{0}};
static UINT16              iMappingIndex[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]  = {{0}};

/*
** Currently used processdata sizes
*/
static UINT16   abcc_iPdReadSize[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]    = {{0}};
static UINT16   abcc_iPdWriteSize[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]   = {{0}};

/*
** Current wrpd sizes devided in octet and bits
*/
static UINT16   abcc_iPdWriteBitSize[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]   = {{0}};

/*
** Current rdpd sizes devided in octet and bits
*/
static UINT16   abcc_iPdReadBitSize[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT]   = {{0}};



/*------------------------------------------------------------------------------
** SetupCommands()
**------------------------------------------------------------------------------
*/
void ABCC_SetupInit( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]  = MSG_GET_DATA_FORMAT;

   iModuleType[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = 0xFFFF;
   iNetworkType[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = 0xFFFF;
   eNetFormat[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = NET_UNKNOWN;
   eParameterSupport[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = PARAMETER_UNKNOWN;

   psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]     = NULL;
   psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]   = NULL;
   abcc_iNumAdi[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = 0;
   iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]  = 0;
   abcc_iPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]    = 0;
   abcc_iPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]   = 0;
   abcc_iPdWriteBitSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]  = 0;
   abcc_iPdReadBitSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]   = 0;
}


/*------------------------------------------------------------------------------
** Find ADI entry table index for the specified instance number.
**------------------------------------------------------------------------------
** Arguments:
**    iInstance         -  Instance number.
**
** Returns:
**    AD_INVALID_ADI_INDEX      - Instance was not found.
**------------------------------------------------------------------------------
*/
static UINT16 GetAdiIndex( IDL_CTRL_HDL hCtrl, UINT16 iInstance )
{
   UINT16 i;
   UINT16  iIndex;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   iIndex = AD_INVALID_ADI_INDEX;

   for( i = 0; i < abcc_iNumAdi[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]; i++ )
   {
      if( psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ i ].iInstance == iInstance )
      {
         iIndex = i;
         break;
      }
   }

   return( iIndex );
}

static UINT16 abcc_GetAdiMapSizeInBits(IDL_CTRL_HDL hCtrl, const AD_AdiEntryType* psAdiEntry, UINT8 bNumElem, UINT8 bElemStartIndex )
{
   UINT16 iSize;

#if( ABCC_CFG_STRUCT_DATA_TYPE )
   UINT16 i;
   if( psAdiEntry->psStruct == NULL )
   {
      iSize = ABCC_GetDataTypeSizeInBits( psAdiEntry->bDataType ) * bNumElem ;
   }
   else
   {
      iSize = 0;
      for( i = bElemStartIndex; i < ( bNumElem + bElemStartIndex ) ; i++ )
      {
         iSize += ABCC_GetDataTypeSizeInBits( psAdiEntry->psStruct[ i ].bDataType );
      }
   }
#else
      (void)bElemStartIndex;
      iSize = ABCC_GetDataTypeSizeInBits( psAdiEntry->bDataType ) * bNumElem ;
#endif

   return iSize;
}

static void abcc_FillMapExtCommand( ABP_MsgType16* psMsg16, UINT16 iAdi, UINT8 bAdiTotNumElem, UINT8 bElemStartIndex, UINT8 bNumElem, UINT8 bDataType )
{
   psMsg16->aiData[ 0 ] = iTOiLe( iAdi );                               /* ADI Instance number. */
   ABCC_SetLowAddrOct( psMsg16->aiData[ 1 ], bAdiTotNumElem);           /* Total number of elements in ADI. */
   ABCC_SetHighAddrOct( psMsg16->aiData[ 1 ], bElemStartIndex );
   ABCC_SetLowAddrOct( psMsg16->aiData[ 2 ], bNumElem);
   ABCC_SetHighAddrOct( psMsg16->aiData[ 2 ], 1 );                      /* Number of type descriptors. */
   ABCC_SetLowAddrOct( psMsg16->aiData[ 3 ], bDataType );               /* ADI element data type. */
   psMsg16->sHeader.iDataSize = iTOiLe( 7 );                            /* The number of used octets in aiData. (The bytes written below). */
}

BOOL ABCC_SetupCommands( IDL_CTRL_HDL hCtrl )
{
   UINT16 iLocalMapIndex;
   UINT16 iLocalSize;
   BOOL fRunSM = TRUE;
   IDL_CTRL_HANDLE hCtrlHandle;
   ABCC_MsgType pMsgSendBuffer;

   hCtrlHandle.hCtrlHandle = hCtrl;

   while( fRunSM )
   {
      fRunSM = FALSE;
      switch( eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] )
      {
         case MSG_GET_DATA_FORMAT:
            pMsgSendBuffer.psMsg = ABCC_GetCmdMsgBuffer(hCtrl);
            ABCC_GetAttribute( pMsgSendBuffer.psMsg, ABP_OBJ_NUM_NW, 1 ,
                               ABP_NW_IA_DATA_FORMAT, ABCC_GetNewSourceId(hCtrl) );
            ABCC_LinkWriteMessage(hCtrl, pMsgSendBuffer.psMsg);
            break;

         case MSG_GET_NW_PARAM_SUPPORT:
            pMsgSendBuffer.psMsg = ABCC_GetCmdMsgBuffer(hCtrl);
            ABCC_GetAttribute( pMsgSendBuffer.psMsg, ABP_OBJ_NUM_NW, 1 ,
                               ABP_NW_IA_PARAM_SUPPORT, ABCC_GetNewSourceId(hCtrl) );
            ABCC_LinkWriteMessage( hCtrl, pMsgSendBuffer.psMsg );
            break;

         case MSG_GET_MODULE_TYPE:
            pMsgSendBuffer.psMsg = ABCC_GetCmdMsgBuffer(hCtrl);
            ABCC_GetAttribute( pMsgSendBuffer.psMsg, ABP_OBJ_NUM_ANB, 1 ,
                               ABP_ANB_IA_MODULE_TYPE, ABCC_GetNewSourceId(hCtrl) );
            ABCC_LinkWriteMessage( hCtrl, pMsgSendBuffer.psMsg);
            break;

         case MSG_GET_NETWORK_TYPE:
            pMsgSendBuffer.psMsg = ABCC_GetCmdMsgBuffer(hCtrl);
            ABCC_GetAttribute( pMsgSendBuffer.psMsg, ABP_OBJ_NUM_NW, 1 ,
                               ABP_NW_IA_NW_TYPE, ABCC_GetNewSourceId(hCtrl) );
            ABCC_LinkWriteMessage(hCtrl, pMsgSendBuffer.psMsg);
            break;

         case MSG_MAP_PD_READ_WRITE:

            if( psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] && psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] && ( psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].eDir != PD_END_MAP ) )
            {
               if( psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].iInstance != AD_MAP_PAD_ADI )
               {
                  iLocalMapIndex = GetAdiIndex( hCtrl, psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].iInstance );

                  if( iLocalMapIndex == AD_INVALID_ADI_INDEX )
                  {
                     ABCC_ERROR(hCtrl, ABCC_SEV_WARNING, ABCC_EC_DEFAULT_MAP_ERR,
                                 (UINT32)psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].iInstance );
                     ABCC_SetMainStateError(hCtrl);
                     break;
                  }
               }
            }
            else
            {
               eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_USER_INIT_START;
               fRunSM = TRUE;
               break;
            }

            if( ABCC_ReadModuleId(hCtrl) == ABP_MODULE_ID_ACTIVE_ABCC40 ) /* If an ABCC40 is attached. */
            {
               UINT8 bNumElemToMap;
               UINT8 bElemMapStartIndex;

               /*
               ** Implement mapping according to the extended command for ABCC40.
               */
               pMsgSendBuffer.psMsg = ABCC_GetCmdMsgBuffer(hCtrl);
               ABCC_SetHighAddrOct( pMsgSendBuffer.psMsg16->sHeader.iSourceIdDestObj, ABP_OBJ_NUM_NW );
               pMsgSendBuffer.psMsg16->sHeader.iInstance            = iTOiLe( 1 );
               ABCC_SetLowAddrOct( pMsgSendBuffer.psMsg16->sHeader.iCmdExt0CmdExt1, 1 ); /* Number of mapping items to add. */
               ABCC_SetHighAddrOct( pMsgSendBuffer.psMsg16->sHeader.iCmdExt0CmdExt1, 0 ); /* Reserved. */

               if ( psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].iInstance != AD_MAP_PAD_ADI )
               {
                  if( psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].bNumElem == AD_DEFAULT_MAP_ALL_ELEM )
                  {
                     bNumElemToMap = psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].bNumOfElements;
                     bElemMapStartIndex = 0;
                  }
                  else
                  {
                     bNumElemToMap = psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].bNumElem;
                     bElemMapStartIndex = psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].bElemStartIndex;
                  }

                  abcc_FillMapExtCommand( pMsgSendBuffer.psMsg16,
                                           psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].iInstance ,          /* Adi */
                                           psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].bNumOfElements ,     /* Adi total num elements */
                                           bElemMapStartIndex,                               /* Mapping  start index */
                                           bNumElemToMap,                                    /* Num elements to map */
                                           psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].bDataType  );        /* Data type */
                  iLocalSize = abcc_GetAdiMapSizeInBits( hCtrl, &psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ],
                                                         bNumElemToMap, bElemMapStartIndex );

#if( ABCC_CFG_STRUCT_DATA_TYPE )
                  if ( psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].psStruct != NULL )
                  {
                     UINT16 iDescOffset;
                     iDescOffset = 0;
                     ABCC_SetHighAddrOct( pMsgSendBuffer.psMsg16->aiData[ 2 ], bNumElemToMap );

                     while ( iDescOffset < bNumElemToMap  )
                     {
                        ABCC_SetLowAddrOct( pMsgSendBuffer.psMsg16->aiData[ ( iDescOffset >> 1) + 3 ], psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].psStruct[ iDescOffset + bElemMapStartIndex].bDataType );
                        iDescOffset++;
                        if( iDescOffset < bNumElemToMap )
                        {
                           ABCC_SetHighAddrOct( pMsgSendBuffer.psMsg16->aiData[ ( iDescOffset >> 1) + 3 ], psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].psStruct[ iDescOffset + bElemMapStartIndex].bDataType );
                           iDescOffset++;
                        }
                     }
                     pMsgSendBuffer.psMsg16->sHeader.iDataSize            = iTOiLe( 6 + iDescOffset );
                  }
#endif
               }
               else
               {

                   abcc_FillMapExtCommand( pMsgSendBuffer.psMsg16,
                                           0 ,                                         /* Adi */
                                           psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].bNumElem ,    /* Adi total num elements */
                                           0,                                          /* Mapping  start index */
                                           psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].bNumElem ,    /* Num elements to map */
                                           ABP_PAD1 );                                 /* Data type */
                  iLocalSize = psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].bNumElem;

               }

               if( psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].eDir == PD_READ )
               {
                  ABCC_SetLowAddrOct( pMsgSendBuffer.psMsg16->sHeader.iCmdReserved, ABP_MSG_HEADER_C_BIT | ABP_NW_CMD_MAP_ADI_READ_EXT_AREA );
                  abcc_iPdReadBitSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]             += iLocalSize;
                  abcc_iPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]             = ( abcc_iPdReadBitSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] + 7 )/8;
               }
               else
               {
                  ABCC_SetLowAddrOct( pMsgSendBuffer.psMsg16->sHeader.iCmdReserved, ABP_MSG_HEADER_C_BIT | ABP_NW_CMD_MAP_ADI_WRITE_EXT_AREA );
                  abcc_iPdWriteBitSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] += iLocalSize;
                  abcc_iPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ( abcc_iPdWriteBitSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] + 7 )/8;
               }
               iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]++;

               ABCC_LinkWriteMessage( hCtrl, pMsgSendBuffer.psMsg);
               break;
            }
            else /* If an ABCC30 is attached. */
            {
               iLocalSize = ABCC_GetDataTypeSize( psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].bDataType ) * psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].bNumOfElements;

               pMsgSendBuffer.psMsg = ABCC_GetCmdMsgBuffer(hCtrl);
               ABCC_SetHighAddrOct( pMsgSendBuffer.psMsg16->sHeader.iSourceIdDestObj, ABP_OBJ_NUM_NW );
               pMsgSendBuffer.psMsg16->sHeader.iInstance            = iTOiLe( 1 );
               pMsgSendBuffer.psMsg16->sHeader.iDataSize            = iTOiLe( 4 );
               pMsgSendBuffer.psMsg16->sHeader.iCmdExt0CmdExt1     = iTOiLe( psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].iInstance );       /* ADI Instance number. */

               ABCC_SetLowAddrOct( pMsgSendBuffer.psMsg16->aiData[ 0 ], psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].bDataType );            /* ADI data type. */
               ABCC_SetHighAddrOct( pMsgSendBuffer.psMsg16->aiData[ 0 ], psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iLocalMapIndex ].bNumOfElements );      /* Number of elements in ADI. */
               pMsgSendBuffer.psMsg16->aiData[ 1 ]       = iTOiLe( iLocalMapIndex + 1 );                                     /* ADI order number. */

               if( psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex][ iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ].eDir == PD_READ )
               {
                  ABCC_SetLowAddrOct( pMsgSendBuffer.psMsg16->sHeader.iCmdReserved, ABP_MSG_HEADER_C_BIT | ABP_NW_CMD_MAP_ADI_READ_AREA );
                  abcc_iPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]             += iLocalSize;
               }
               else
               {
                  ABCC_SetLowAddrOct( pMsgSendBuffer.psMsg16->sHeader.iCmdReserved, ABP_MSG_HEADER_C_BIT | ABP_NW_CMD_MAP_ADI_WRITE_AREA );
                  abcc_iPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]            += iLocalSize;
               }
               iMappingIndex[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]++;

               ABCC_LinkWriteMessage(hCtrl, pMsgSendBuffer.psMsg);
               break;
            }

         case MSG_USER_INIT_START:
            eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_USER_INIT;
            ABCC_CbfUserInitReq(hCtrl);

            break;
         case MSG_USER_INIT:
            /* Do nothing */
            break;

         case MSG_READ_RDPD_SIZE:
            /* Do nothing */
            break;

         case MSG_READ_WRPD_SIZE:
            pMsgSendBuffer.psMsg = ABCC_GetCmdMsgBuffer(hCtrl);
            ABCC_GetAttribute( pMsgSendBuffer.psMsg, ABP_OBJ_NUM_NW, 1 ,
                               ABP_NW_IA_WRITE_PD_SIZE, ABCC_GetNewSourceId(hCtrl) );
            ABCC_LinkWriteMessage(hCtrl, pMsgSendBuffer.psMsg);
             break;

         case MSG_SETUP_COMPLETE:

            pMsgSendBuffer.psMsg = ABCC_GetCmdMsgBuffer(hCtrl);

            ABCC_SetByteAttribute(  pMsgSendBuffer.psMsg, ABP_OBJ_NUM_ANB, 1,
                                    ABP_ANB_IA_SETUP_COMPLETE, TRUE,
                                    ABCC_GetNewSourceId(hCtrl) );

            ABCC_LinkWriteMessage(hCtrl, pMsgSendBuffer.psMsg);

            break;

         case MSG_DONE:
            return TRUE;
      } /* end switch( command state ) */
   }

   return FALSE;
}


/*------------------------------------------------------------------------------
** SetupResponses()
**------------------------------------------------------------------------------
*/
void ABCC_SetupResponses( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsgReciveBuffer )
{
   ABCC_MsgType pMsgReciveBuffer;
   ABCC_MsgHandlerFuncType pnMsgHandler = 0;
   IDL_CTRL_HANDLE hCtrlHandle;

   hCtrlHandle.hCtrlHandle = hCtrl;
   pMsgReciveBuffer.psMsg = psMsgReciveBuffer;

   switch( eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] )
   {
      case MSG_GET_DATA_FORMAT:
         ABCC_ASSERT_ERR( hCtrl, ABCC_VerifyMessage( pMsgReciveBuffer.psMsg ) == ABCC_EC_NO_ERROR, ABCC_SEV_WARNING, ABCC_EC_RESP_MSG_E_BIT_SET, (UINT32)ABCC_GetLowAddrOct(pMsgReciveBuffer.psMsg16->aiData[ 0 ]) );
         eNetFormat[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ( NetFormatType )ABCC_GetLowAddrOct( pMsgReciveBuffer.psMsg16->aiData[ 0 ] );
         ABCC_ASSERT(  hCtrl, eNetFormat[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] < 2 );
         DEBUG_EVENT(("RSP MSG_DATA_FORMAT: %d\n", eNetFormat[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ));

         eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_GET_NW_PARAM_SUPPORT;
         break;

      case MSG_GET_NW_PARAM_SUPPORT:
         ABCC_ASSERT_ERR( hCtrl, ABCC_VerifyMessage( pMsgReciveBuffer.psMsg ) == ABCC_EC_NO_ERROR, ABCC_SEV_WARNING, ABCC_EC_RESP_MSG_E_BIT_SET, (UINT32)ABCC_GetLowAddrOct( pMsgReciveBuffer.psMsg16->aiData[ 0 ] )  );
         eParameterSupport[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ( ParameterSupportType )ABCC_GetLowAddrOct ( pMsgReciveBuffer.psMsg16->aiData[ 0 ] );
         DEBUG_EVENT(("RSP MSG_GET_PARAM_SUPPORT: %d\n", eParameterSupport[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]));
         eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_GET_MODULE_TYPE;
         break;

      case MSG_GET_MODULE_TYPE:
         ABCC_ASSERT_ERR( hCtrl, ABCC_VerifyMessage( pMsgReciveBuffer.psMsg ) == ABCC_EC_NO_ERROR, ABCC_SEV_WARNING, ABCC_EC_RESP_MSG_E_BIT_SET, (UINT32) pMsgReciveBuffer.psMsg16->aiData[ 0 ]);
         iModuleType[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = iLeTOi( pMsgReciveBuffer.psMsg16->aiData[ 0 ] );
         DEBUG_EVENT(("RSP MSG_GET_MODULE_ID: 0x%x\n",iModuleType[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]));
         eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_GET_NETWORK_TYPE;
         break;

      case MSG_GET_NETWORK_TYPE:
         ABCC_ASSERT_ERR( hCtrl, ABCC_VerifyMessage( pMsgReciveBuffer.psMsg ) == ABCC_EC_NO_ERROR, ABCC_SEV_WARNING, ABCC_EC_RESP_MSG_E_BIT_SET, (UINT32) pMsgReciveBuffer.psMsg16->aiData[ 0 ] );
         iNetworkType[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = iLeTOi( pMsgReciveBuffer.psMsg16->aiData[ 0 ] );
         DEBUG_EVENT(("RSP MSG_GET_NETWORK_ID :0x%x\n", iNetworkType[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]));

         /*
         ** We now have all information needed to initialize the ADIs and the mapping.
         */
         eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_MAP_PD_READ_WRITE;
         abcc_iNumAdi[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = ABCC_CbfAdiMappingReq (hCtrl, (const AD_AdiEntryType** )&psAdiEntry[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex],
                                                (const AD_DefaultMapType** )&psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
         break;

      case MSG_USER_INIT:
         DEBUG_EVENT( ( "RSP MSG_ABCC_USER_INIT\n" ) );
         pnMsgHandler = ABCC_LinkGetMsgHandler(hCtrl, ABCC_GetLowAddrOct( pMsgReciveBuffer.psMsg16->sHeader.iSourceIdDestObj ) );
         if( pnMsgHandler )
         {
            pnMsgHandler( hCtrl, pMsgReciveBuffer.psMsg );
         }
         else
         {
            ABCC_ERROR( hCtrl, ABCC_SEV_WARNING, ABCC_EC_INVALID_RESP_SOURCE_ID,(UINT32) pMsgReciveBuffer.psMsg );
         }
         break;

      case MSG_MAP_PD_READ_WRITE:
         DEBUG_EVENT(("RSP MSG_MAP_IO_****\n"));
         ABCC_ASSERT_ERR( hCtrl, ABCC_VerifyMessage( pMsgReciveBuffer.psMsg ) == ABCC_EC_NO_ERROR, ABCC_SEV_WARNING, ABCC_EC_RESP_MSG_E_BIT_SET, (UINT32) pMsgReciveBuffer.psMsg);
         break;

      case MSG_READ_RDPD_SIZE:
         ABCC_ASSERT_ERR( hCtrl, ABCC_VerifyMessage( pMsgReciveBuffer.psMsg ) == ABCC_EC_NO_ERROR, ABCC_SEV_WARNING, ABCC_EC_RESP_MSG_E_BIT_SET, (UINT32) pMsgReciveBuffer.psMsg);

         if( psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == NULL )
         {
            /* Use received read size */
            abcc_iPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = iLeTOi( pMsgReciveBuffer.psMsg16->aiData[ 0 ] );
         }
         else
         {
            /* Verify that ABCC and driver has the same view */
            ABCC_ASSERT( hCtrl, abcc_iPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ==  iLeTOi( pMsgReciveBuffer.psMsg16->aiData[ 0 ] ) );
         }
         eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_READ_WRPD_SIZE;
         break;

      case MSG_READ_WRPD_SIZE:
         ABCC_ASSERT_ERR( hCtrl, ABCC_VerifyMessage( pMsgReciveBuffer.psMsg ) == ABCC_EC_NO_ERROR, ABCC_SEV_WARNING, ABCC_EC_RESP_MSG_E_BIT_SET, (UINT32) pMsgReciveBuffer.psMsg);

         if( psDefaultMap[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] == NULL )
         {
            /* Use received write size */
            abcc_iPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = iLeTOi( pMsgReciveBuffer.psMsg16->aiData[ 0 ] );
         }
         else
         {
            /* Verify that ABCC and driver has the same view */
            ABCC_ASSERT(hCtrl, abcc_iPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] ==  iLeTOi( pMsgReciveBuffer.psMsg16->aiData[ 0 ] ) );
         }

         eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_SETUP_COMPLETE;
         break;

      case MSG_SETUP_COMPLETE:
         DEBUG_EVENT(("RSP MSG_SETUP_COMPLETE\n"));
         ABCC_ASSERT_ERR( hCtrl, ABCC_VerifyMessage( pMsgReciveBuffer.psMsg ) == ABCC_EC_NO_ERROR, ABCC_SEV_WARNING, ABCC_EC_RESP_MSG_E_BIT_SET, (UINT32) pMsgReciveBuffer.psMsg );

         DEBUG_EVENT(("Mapped PD size, RdPd %d WrPd: %d\n", abcc_iPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex], abcc_iPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]));
         pnABCC_DrvSetPdSize(hCtrl, abcc_iPdReadSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex], abcc_iPdWriteSize[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] );
         eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_DONE;
         break;

      default:
         ABCC_ASSERT( hCtrl, FALSE );
         break;
   } /* end switch */
}


void ABCC_UserInitComplete( IDL_CTRL_HDL hCtrl )
{
   ABP_MsgType* pMsgSendBuffer;
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;

   eCmdState[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex] = MSG_READ_RDPD_SIZE;

   pMsgSendBuffer = ABCC_MemAlloc( hCtrl );
   ABCC_GetAttribute( pMsgSendBuffer, ABP_OBJ_NUM_NW, 1 ,
                      ABP_NW_IA_READ_PD_SIZE, ABCC_GetNewSourceId(hCtrl) );
   ABCC_LinkWriteMessage( hCtrl, pMsgSendBuffer );
}

UINT16 ABCC_NetworkType( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   return iNetworkType[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];
}

UINT16 ABCC_ModuleType( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   return iModuleType[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];
}

NetFormatType ABCC_NetFormatType( IDL_CTRL_HDL hCtrl )
{
    IDL_CTRL_HANDLE hCtrlHandle;
    hCtrlHandle.hCtrlHandle = hCtrl;
    return eNetFormat[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];
}

ParameterSupportType ABCC_ParameterSupport( IDL_CTRL_HDL hCtrl )
{
   IDL_CTRL_HANDLE hCtrlHandle;
   hCtrlHandle.hCtrlHandle = hCtrl;
   return eParameterSupport[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];
}
















