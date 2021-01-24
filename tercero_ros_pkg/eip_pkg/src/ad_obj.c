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
** Implementation of the AD object.
********************************************************************************
********************************************************************************
*/
#include "abcc_td.h"
#include "abcc_sw_port.h"
#include "abcc_drv_cfg.h"
#include "abcc_obj_cfg.h"
#include "abp.h"
#include "abcc_ad_if.h"
#include "abcc.h"
#include "appl_abcc_handler.h"
#include "ad_obj.h"

/*******************************************************************************
** Constants
********************************************************************************
*/

#if( ABCC_CFG_REMAP_SUPPORT_ENABLED )
#if( AD_MAX_NUM_WRITE_ADI_TO_MAP > AD_MAX_NUM_READ_ADI_TO_MAP )
#define AD_MAX_OF_READ_WRITE_TO_MAP AD_MAX_NUM_WRITE_ADI_TO_MAP
#else
#define AD_MAX_OF_READ_WRITE_TO_MAP AD_MAX_NUM_READ_ADI_TO_MAP
#endif
#endif

/*
** Value used in ad_MapType as ADI index when ADI 0 is mapped.
*/
#define AD_MAP_PAD_INDEX                     ( 0xfffe )

/*
** Invalid ADI index.
*/
#define AD_INVALID_ADI_INDEX                 ( 0xffff )

/*
** Default number of sub-elements
*/
#define AD_DEFAULT_NUM_SUB_ELEMS             ( 1 )

/*******************************************************************************
** Typedefs
********************************************************************************
*/

/*------------------------------------------------------------------------------
** Type with mapping information for a single ADI
**------------------------------------------------------------------------------
** iAdiIndex      - Index to ADI entry table.
** bNumElements   - Number of mapped elements.
** bStartIndex    - Element start index for the mapping
**------------------------------------------------------------------------------
*/
typedef struct ad_Map
{
  UINT16           iAdiIndex;
  UINT8            bNumElements;
  UINT8            bStartIndex;
}
ad_MapType;

/*------------------------------------------------------------------------------
** Type with mapping information for a specific direction (read/write).
**------------------------------------------------------------------------------
** paiMappedAdiList    - Pointer to list of all mapped items.
** iNumMappedAdi       - Number of mapped ADI:s
** iMaxNumMappedAdi    - Maximum number of mapped ADI:s.
** iPdSize             - Current process data size in octets.
**------------------------------------------------------------------------------
*/
typedef struct ad_MapInfo
{
   ad_MapType*   paiMappedAdiList;
   UINT16        iNumMappedAdi;
   UINT16        iMaxNumMappedAdi;
   UINT16        iPdSize;
}
ad_MapInfoType;

/*******************************************************************************
** Private Globals
********************************************************************************
*/
static BOOL ad_fDoNetworkEndianSwap = FALSE;
static const AD_DefaultMapType* ad_asDefaultMap = NULL;
static const AD_AdiEntryType* ad_asADIEntryList = NULL;
static UINT16  ad_iNumOfADIs;
static UINT16  ad_iHighestInstanceNumber;
static ad_MapType ad_PdReadMapping[ AD_MAX_NUM_READ_ADI_TO_MAP ];
static ad_MapType ad_PdWriteMapping[ AD_MAX_NUM_WRITE_ADI_TO_MAP ];
static ad_MapInfoType ad_ReadMapInfo;
static ad_MapInfoType ad_WriteMapInfo;

/*******************************************************************************
** Private Services
********************************************************************************
*/

/*------------------------------------------------------------------------------
** Converts number of octet offset to byte offset.
**------------------------------------------------------------------------------
*/
#ifdef ABCC_SYS_16_BIT_CHAR
#define OctetToByteOffset( x )  ( ( x ) >> 1 )
#else
#define OctetToByteOffset( x )  ( x )
#endif

/*------------------------------------------------------------------------------
** Checks if a ABP data type is either a bit or pad type.
**------------------------------------------------------------------------------
*/
#define Is_BITx_Or_PADx( type ) ( ABP_Is_PADx( type ) || ABP_Is_BITx( type ) )

/*------------------------------------------------------------------------------
** The bit variable is converted to whole octets and added to the octet variable.
** The remaining bits are saved to the bit variable.
** For a 16 bit char system the number of octets are kept even.
** Note that both input variables are updated.
**------------------------------------------------------------------------------
*/
#ifdef ABCC_SYS_16_BIT_CHAR
#define AddBitsToOctetSize( octet, bits ) \
do                                        \
{                                         \
   (octet) += (bits >> 4 ) << 1;          \
   (bits) %= 16;                          \
}                                         \
while( 0 )
#else
#define AddBitsToOctetSize( octet, bits ) \
do                                        \
{                                         \
   (octet) += (bits) >> 3;                \
   (bits) %= 8;                           \
}                                         \
while( 0 )
#endif

/*------------------------------------------------------------------------------
** Calculates the bit offset to the startindex element in the ADI.
**------------------------------------------------------------------------------
*/
#define CalcStartindexBitOffset( bDataType, iStartIndex )      \
   ABCC_GetDataTypeSizeInBits( bDataType ) * ( iStartIndex )


/*------------------------------------------------------------------------------
** Add octet variable and bit variable and round up to nearest octet.
**------------------------------------------------------------------------------
*/
#define SizeInOctets( octet, bits ) ( (octet) + ( (bits) + 7 ) / 8 )

/*------------------------------------------------------------------------------
** Convert bit offset to octet offset.
**------------------------------------------------------------------------------
*/
#define BitToOctetOffset( bitOffset ) ( (bitOffset) >> 3 )

/*------------------------------------------------------------------------------
** Copies a 16 bit values from a source to a destination. Each value will be
** endian swapped. The function support octet alignment.
**------------------------------------------------------------------------------
** Arguments:
**    pxDest            - Base pointer to the destination.
**    iDestOctetOffset  - Octet offset to the destination where the copy will
**                        begin.
**    pxSrc             - Bsse pointer to source data.
**    iSrcOctetOffset   - Octet offset to the source where the copy will begin.
**    iNumElem          - Number of 16 bit values to copy.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void Copy16WithEndianSwap( void* pxDest, UINT16 iDestOctetOffset,
                                  void* pxSrc,  UINT16 iSrcOctetOffset,
                                  UINT16 iNumElem )
{
   UINT16 i;
   UINT16 iConv;

   for( i = 0; i < iNumElem; i++ )
   {
      ABCC_PORT_Copy16( &iConv, 0, pxSrc, iSrcOctetOffset + ( i << 1 ) );
      iConv = ABCC_iEndianSwap( iConv );
      ABCC_PORT_Copy16( pxDest, iDestOctetOffset + ( i << 1 ), &iConv, 0 );
   }
}

/*------------------------------------------------------------------------------
** Copies a 32 bit values from a source to a destination. Each value will be
** endian swapped. The function support octet alignment.
**------------------------------------------------------------------------------
** Arguments:
**    pxDest            - Base pointer to the destination.
**    iDestOctetOffset  - Octet offset to the destination where the copy will
**                        begin.
**    pxSrc             - Bsse pointer to source data.
**    iSrcOctetOffset   - Octet offset to the source where the copy will begin.
**    iNumElem          - Number of 32 bit values to copy.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void Copy32WithEndianSwap( void* pxDest, UINT16 iDestOctetOffset,
                                  void* pxSrc,  UINT16 iSrcOctetOffset,
                                  UINT16 iNumElem )
{
   UINT16 i;
   UINT32 lConv;

   for( i = 0; i < iNumElem; i++ )
   {
      ABCC_PORT_Copy32( &lConv, 0, pxSrc, iSrcOctetOffset + ( i << 2 ) );
      lConv = ABCC_lEndianSwap( lConv );
      ABCC_PORT_Copy32( pxDest, iDestOctetOffset + ( i << 2 ), &lConv, 0 );
   }
}


/*------------------------------------------------------------------------------
** Copies a 64 bit values from a source to a destination. Each value will be
** endian swapped. The function support octet alignment.
**------------------------------------------------------------------------------
** Arguments:
**    pxDest            - Base pointer to the destination.
**    iDestOctetOffset  - Octet offset to the destination where the copy will
**                        begin.
**    pxSrc             - Bsse pointer to source data.
**    iSrcOctetOffset   - Octet offset to the source where the copy will begin.
**    iNumElem          - Number of 64 bit values to copy.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
#if( ABCC_CFG_64BIT_ADI_SUPPORT )
static void Copy64WithEndianSwap( void* pxDest, UINT16 iDestOctetOffset,
                                  void* pxSrc,  UINT16 iSrcOctetOffset,
                                  UINT16 iNumElem )
{
   UINT16 i;
   UINT64 lConv;

   for( i = 0; i < iNumElem; i++ )
   {
      ABCC_PORT_Copy64( &lConv, 0, pxSrc, iSrcOctetOffset + ( i << 3 ) );
      lConv = ABCC_l64EndianSwap( lConv );
      ABCC_PORT_Copy64( pxDest, iDestOctetOffset + ( i << 3 ), &lConv, 0 );
   }
}
#endif

/*------------------------------------------------------------------------------
** Calculates size of ADI rounded up to nearest octet.
**------------------------------------------------------------------------------
** Arguments:
**    psAdiEntry  -  Pointer to ADI entry.
**
** Returns:
**    Size in octets.
**------------------------------------------------------------------------------
*/
static UINT16 GetAdiSizeInOctets( const AD_AdiEntryType* psAdiEntry )
{
   UINT16 iSize;
#if( ABCC_CFG_STRUCT_DATA_TYPE )
   UINT16 i;

   if( psAdiEntry->psStruct != NULL )
   {
      iSize = 0;
      for( i = 0; i < psAdiEntry->bNumOfElements; i++ )
      {
         iSize +=
            ABCC_GetDataTypeSizeInBits( psAdiEntry->psStruct[ i ].bDataType );
      }
   }
   else
#endif
   {
      iSize = ABCC_GetDataTypeSizeInBits( psAdiEntry->bDataType ) *
                                          psAdiEntry->bNumOfElements ;
   }
   iSize = ( iSize + 7 ) / 8;
   return( iSize );
}

/*------------------------------------------------------------------------------
** Calculates map size of of an ADI mapping item, in bits.
**------------------------------------------------------------------------------
** Arguments:
**    psAdiEntry         -  Pointer to ADI entry.
**    bNumElem           -  Number of elements
**    bElemStartIndex    -  First element index.
**
** Returns:
**    Size in bits.
**------------------------------------------------------------------------------
*/
static UINT16 GetAdiMapSizeInBits( const AD_AdiEntryType* psAdiEntry,
                                   UINT8 bNumElem,
                                   UINT8 bElemStartIndex )
{
   UINT16 iSize;
#if( ABCC_CFG_STRUCT_DATA_TYPE )
   UINT16 i;
   if( psAdiEntry->psStruct != NULL )
   {
      iSize = 0;
      for( i = bElemStartIndex; i < ( bNumElem + bElemStartIndex ) ; i++ )
      {
         iSize +=
            ABCC_GetDataTypeSizeInBits( psAdiEntry->psStruct[ i ].bDataType );
      }
   }
   else
   {
      iSize = ABCC_GetDataTypeSizeInBits( psAdiEntry->bDataType ) * bNumElem ;
   }
#else
      (void)bElemStartIndex;
      iSize = ABCC_GetDataTypeSizeInBits( psAdiEntry->bDataType ) * bNumElem ;
#endif

   return( iSize );
}

/*------------------------------------------------------------------------------
** Calculates total map size in octets.
**------------------------------------------------------------------------------
** Arguments:
**    psMap         -  Pointer to mapping information. The size is
**                     calculated based on the on the mapping list provided
**                     in the structure. The iPdSize member is updated with
**                     the new size.
**
** Returns:
**    None.
**------------------------------------------------------------------------------
*/
static void UpdateMapSize( ad_MapInfoType* psMap )
{
   UINT16 iMapIndex;
   UINT16 iAdiIndex;

   psMap->iPdSize = 0;
   for( iMapIndex = 0; iMapIndex < psMap->iNumMappedAdi; iMapIndex++ )
   {
      iAdiIndex = psMap->paiMappedAdiList[ iMapIndex ].iAdiIndex;
      if( iAdiIndex != AD_MAP_PAD_INDEX )
      {
         psMap->iPdSize +=
            GetAdiMapSizeInBits( &ad_asADIEntryList[ iAdiIndex ],
                                 psMap->paiMappedAdiList[ iMapIndex ].bNumElements,
                                 psMap->paiMappedAdiList[ iMapIndex ].bStartIndex );
      }
      else
      {
         psMap->iPdSize += psMap->paiMappedAdiList[ iMapIndex ].bNumElements;
      }
   }
   psMap->iPdSize = SizeInOctets( 0, psMap->iPdSize );
}

/*------------------------------------------------------------------------------
** Find ADI entry table index for the specified instance number.
**------------------------------------------------------------------------------
** Arguments:
**    iInstance         -  Instance number.
**
** Returns:
**    0 - 0xfffe                - Index in ADI entry table.
**    AD_INVALID_ADI_INDEX      - Instance was not found.
**------------------------------------------------------------------------------
*/
static UINT16 GetAdiIndex( UINT16 iInstance )
{
   UINT16 i;
   UINT16  iIndex;

   iIndex = AD_INVALID_ADI_INDEX;

   if( iInstance == 0 )
   {
      return( AD_MAP_PAD_INDEX );
   }

   for( i = 0; i < ad_iNumOfADIs; i++ )
   {
      if( ad_asADIEntryList[ i ].iInstance == iInstance )
      {
         iIndex = i;
         break;
      }
   }

   return( iIndex );
}

#if( ABCC_CFG_REMAP_SUPPORT_ENABLED )
/*------------------------------------------------------------------------------
** Process of remap command.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl               - Handle to the hardware.
**    ABP_MsgType         - Pointer to remap command.
**    psCurrMap           - Pointer to current mapping. Will hold the new
**                          mapping when processing is done.
**
** Returns:
**    None.
**------------------------------------------------------------------------------
*/
static void RemapProcessDataCommand( IDL_CTRL_HDL hCtrl,
                                     ABP_MsgType* psMsg,
                                     ad_MapInfoType* psCurrMap )
{

   UINT16 iAdi;
   UINT16 iMsgIndex;
   UINT16 iAddItemIndex;
   UINT16 iMapIndex;
   UINT16 bStartOfRemap;
   UINT8  bErrCode;
   UINT16 iItemsToRemove;
   UINT16 iItemsToAdd;
   UINT16 iDataSize;
   ad_MapType sMap;

   iDataSize = 1;
   bStartOfRemap = ABCC_GetMsgCmdExt( psMsg );

   ABCC_GetMsgData16( psMsg, &iItemsToRemove, 0);
   ABCC_GetMsgData16( psMsg, &iItemsToAdd, 2);

   /*
   ** A lot of sanity checks first since all actions of the command shall
   ** either be carried out or rejected.
   */
   if( ABCC_GetMsgDataSize( psMsg ) < 4 )
   {
      /*
      ** Not enough data provided
      */
      bErrCode = ABP_ERR_NOT_ENOUGH_DATA;
   }
   else if( bStartOfRemap > psCurrMap->iNumMappedAdi )
   {
      /*
      ** Not an allowed mapping number
      */
      bErrCode = ABP_ERR_INV_CMD_EXT_0;
   }
   else if( ( bStartOfRemap + iItemsToRemove ) > psCurrMap->iNumMappedAdi )
   {
      /*
      ** Cannot remove more than currently is mapped
      */
      bErrCode = ABP_ERR_OUT_OF_RANGE;
   }
   else if( ( psCurrMap->iNumMappedAdi + iItemsToAdd - iItemsToRemove )  >
              psCurrMap->iMaxNumMappedAdi )
   {
      /*
      ** This will result in more maps than we can handle
      */
      bErrCode = ABP_ERR_NO_RESOURCES;
   }
   else if( ABCC_GetMsgDataSize( psMsg ) < 4 + ( iItemsToAdd * 4 ) )
   {
      bErrCode = ABP_ERR_NOT_ENOUGH_DATA;
   }
   else if( ABCC_GetMsgDataSize( psMsg ) > 4 + ( iItemsToAdd * 4 ) )
   {
      bErrCode = ABP_ERR_TOO_MUCH_DATA;
   }
   else
   {
      bErrCode = ABP_ERR_NO_ERROR;
   }

   /*
   ** Check New ADI:s
   */
   if( bErrCode == ABP_ERR_NO_ERROR )
   {
      iAddItemIndex = 0;
      iMsgIndex = 4;
      while( ( bErrCode == ABP_ERR_NO_ERROR ) && ( iAddItemIndex < iItemsToAdd ) )
      {
         ABCC_GetMsgData16( psMsg, &iAdi, iMsgIndex );
         sMap.iAdiIndex = GetAdiIndex( iAdi );
         iMsgIndex += 2;
         ABCC_GetMsgData8( psMsg, &sMap.bStartIndex, iMsgIndex++ );
         ABCC_GetMsgData8( psMsg, &sMap.bNumElements, iMsgIndex++ );

         if( sMap.iAdiIndex != AD_MAP_PAD_INDEX )
         {
            if( ( sMap.bStartIndex + sMap.bNumElements ) >
                   ad_asADIEntryList[ sMap.iAdiIndex ].bNumOfElements )
            {
               bErrCode = ABP_ERR_OBJ_SPECIFIC;
               /*
               ** ABP_APPD_ERR_INVALID_NUM_ELEMENTS
               */
               ABCC_SetMsgData8( psMsg, 0x02, 1 );
               iDataSize = 2;
            }
         }
         iAddItemIndex++;
      }
   }

   if( bErrCode == ABP_ERR_NO_ERROR )
   {
      /*
      ** Move ADI if required
      */
      if( ( iItemsToRemove != iItemsToAdd ) &&
          ( iItemsToRemove  <  ( psCurrMap->iNumMappedAdi - bStartOfRemap  ) ) )
      {
         INT16  iItemsToMove;
         UINT16 iMoveFrom;
         UINT16 iMoveTo;
         UINT16 iIndex;

         /*
         ** Data needs to be moved
         */
         iMoveFrom = bStartOfRemap + iItemsToRemove;
         iMoveTo = bStartOfRemap  + iItemsToAdd;
         iItemsToMove = psCurrMap->iNumMappedAdi - bStartOfRemap - iItemsToRemove;

         if( iItemsToRemove > iItemsToAdd )
         {
            for( iIndex = 0; iIndex < iItemsToMove; iIndex++ )
            {
               psCurrMap->paiMappedAdiList[ iMoveTo + iIndex ] =
                  psCurrMap->paiMappedAdiList[ iMoveFrom + iIndex ];
            }
         }
         else
         {
            for( iIndex = iItemsToMove ; iIndex > 0 ; iIndex-- )
            {
               psCurrMap->paiMappedAdiList[ iMoveTo + iIndex - 1 ] =
                  psCurrMap->paiMappedAdiList[ iMoveFrom + iIndex - 1 ];
            }
         }
      }

      iMapIndex = bStartOfRemap;
      psCurrMap->iNumMappedAdi -= iItemsToRemove;
      psCurrMap->iNumMappedAdi += iItemsToAdd;

      iMsgIndex = 4;
      for( iAddItemIndex = 0; iAddItemIndex < iItemsToAdd; iAddItemIndex++ )
      {
         ABCC_GetMsgData16( psMsg, &iAdi, iMsgIndex );
         psCurrMap->paiMappedAdiList[ iMapIndex ].iAdiIndex = GetAdiIndex( iAdi );
         iMsgIndex += 2;
         ABCC_GetMsgData8( psMsg, &psCurrMap->paiMappedAdiList[ iMapIndex ].bStartIndex, iMsgIndex++ );
         ABCC_GetMsgData8( psMsg, &psCurrMap->paiMappedAdiList[ iMapIndex ].bNumElements, iMsgIndex++ );
         iMapIndex++;
      }

      UpdateMapSize( psCurrMap );

      ABCC_SetMsgData16(psMsg, psCurrMap->iPdSize, 0);
      ABP_SetMsgResponse( psMsg, 2 );
      ABCC_SendRemapRespMsg( hCtrl, psMsg, ad_ReadMapInfo.iPdSize,
                             ad_WriteMapInfo.iPdSize);
   }
   else
   {
      ABP_SetMsgErrorResponse( psMsg, iDataSize, bErrCode );
      ABCC_SendRespMsg(hCtrl, psMsg );
   }
}
#endif


/*------------------------------------------------------------------------------
** Copy bit data. Any alignment is allowed.
**------------------------------------------------------------------------------
** Arguments:
**    pxDest            - Destination base pointer.
**    iDestBitOffset    - Bit offset relative destination pointer.
**    pxSrc             - Source base pointer.
**    iSrcBitOffset     - Bit offset relative source pointer.
**    bDataType         - Data type according to ABP_<X> types in abp.h
**    iNumElem          - Number of elements to copy.
**
** Returns:
**    Size of copied data in bits.
**------------------------------------------------------------------------------
*/
static UINT16 CopyBitData( void* pxDest,
                           UINT16 iDestBitOffset,
                           void* pxSrc,
                           UINT16 iSrcBitOffset,
                           UINT8 bDataType,
                           UINT16 iNumElem )
{
   UINT8  bCopySize;
   UINT16 i;
   UINT16 iSetBitSize = 0;
   UINT32 lBitMask;
   UINT32 lSrc;
   UINT32 lDest;
   UINT16 iSrcOctetOffset;
   UINT16 iDestOctetOffset;

   iSrcOctetOffset = 0;
   iDestOctetOffset = 0;

   if( ABP_Is_PADx( bDataType ) )
   {
      /*
      ** This is only a pad. No copy is done.
      */
      iSetBitSize += bDataType - ABP_PAD0;
   }
   else
   {
      /*
      ** Separate offsets into octets and bits.
      */
      AddBitsToOctetSize( iSrcOctetOffset, iSrcBitOffset );
      AddBitsToOctetSize( iDestOctetOffset, iDestBitOffset );

      /*
      ** Calculate number of bits to be set.
      */
      iSetBitSize += ( ( bDataType - ABP_BIT1 ) + 1 );

      for( i = 0; i < iNumElem; i++ )
      {
         /*
         ** Calculate the number of octets that has to be copied
         ** to include both destination bit offset and bit size.
         */
         bCopySize = ( iSetBitSize + iDestBitOffset + 7 ) / 8;

         /*
         ** Copy parts to be manipulated into local 32 bit variables to
         ** guarantee correct alignment.
         */
         ABCC_PORT_CopyOctets( &lSrc, 0, pxSrc, iSrcOctetOffset ,
                               ABP_UINT32_SIZEOF );
         ABCC_PORT_CopyOctets( &lDest, 0, pxDest, iDestOctetOffset, bCopySize );

         /*
         ** Bit data types crossing octet boundaries are always little endian.
         */
         lSrc = lLeTOl( lSrc );
         lDest = lLeTOl( lDest );

         /*
         ** Calculate bit mask and align it with destination bit offset.
         */
         lBitMask = ( (UINT32)1 << iSetBitSize ) - 1;
         lBitMask <<= iDestBitOffset;

         /*
         ** Align source bits with destination bits
         */
         if( iSrcBitOffset <  iDestBitOffset )
         {
            lSrc <<= iDestBitOffset - iSrcBitOffset;
         }
         else
         {
            lSrc >>= iSrcBitOffset - iDestBitOffset;
         }

         /*
         ** Clear destinations bits and mask source bits an insert source bits
         ** into destination bit position.
         */
         lDest &=  ~lBitMask;
         lSrc &=  lBitMask;
         lDest |= lSrc;

         /*
         ** Restore endian.
         */
         lDest = lTOlLe( lDest );

         /*
         ** Copy local updated data into final destination.
         */
         ABCC_PORT_CopyOctets( pxDest, iDestOctetOffset, &lDest, 0, bCopySize );

         /*
         ** Update bit offsets to next bit field.
         */
         iSrcBitOffset += iSetBitSize;
         AddBitsToOctetSize( iSrcOctetOffset, iSrcBitOffset );
         iDestBitOffset += iSetBitSize;
         AddBitsToOctetSize( iDestOctetOffset, iDestBitOffset );
      }
      iSetBitSize *= iNumElem;
   }
   return( iSetBitSize );
}

/*------------------------------------------------------------------------------
**  Copy value (single element or parts of an array) of a specific type
**  from a specified source to a destination. If the host platform endian
**  differs from network endian a swap will be done.
**  NOTE!! For all non-bit data types the source and destination must be octet
**  aligned.
**------------------------------------------------------------------------------
** Arguments:
**    pxDst             - Destination base pointer.
**    iDestBitOffset    - Bit offset relative destination pointer.
**    pxSrc             - Source base pointer.
**    iSrcBitOffset     - Bit offset relative source pointer.
**    bDataType         - Data type according to ABP_<X> types in abp.h
**    iNumElem          - Number of elements to copy.
**
** Returns:
**    Size of copied data in bits.
**------------------------------------------------------------------------------
*/
static UINT16 CopyValue( void* pxDst,
                         UINT16 iDestBitOffset,
                         void* pxSrc ,
                         UINT16 iSrcBitOffset,
                         UINT8 bDataType,
                         UINT16 iNumElem )
{
   UINT8 bDataTypeSizeInOctets;
   UINT16 iBitSetSize;

   if( Is_BITx_Or_PADx( bDataType ) )
   {
      iBitSetSize = CopyBitData( pxDst,
                                 iDestBitOffset,
                                 pxSrc,
                                 iSrcBitOffset,
                                 bDataType,
                                 iNumElem );
   }
   else
   {
      bDataTypeSizeInOctets = ABCC_GetDataTypeSize( bDataType );

      if( ad_fDoNetworkEndianSwap )
      {
         switch( bDataTypeSizeInOctets )
         {
         case 1:
            ABCC_PORT_CopyOctets( pxDst, BitToOctetOffset( iDestBitOffset ),
                                  pxSrc, BitToOctetOffset( iSrcBitOffset ),
                                  iNumElem );
            break;

         case 2:
            Copy16WithEndianSwap( pxDst, BitToOctetOffset( iDestBitOffset ),
                                  pxSrc, BitToOctetOffset( iSrcBitOffset ),
                                  iNumElem );
            break;

         case 4:
            Copy32WithEndianSwap( pxDst, BitToOctetOffset( iDestBitOffset ),
                                  pxSrc, BitToOctetOffset( iSrcBitOffset ),
                                  iNumElem );
            break;

   #if( ABCC_CFG_64BIT_ADI_SUPPORT )
         case 8:
            Copy64WithEndianSwap( pxDst, BitToOctetOffset( iDestBitOffset ),
                                  pxSrc, BitToOctetOffset( iSrcBitOffset ),
                                  iNumElem );
            break;
   #endif
         default:
            break;
         }
      }
      else
      {
         ABCC_PORT_CopyOctets( pxDst, BitToOctetOffset( iDestBitOffset ),
                               pxSrc, BitToOctetOffset( iSrcBitOffset ),
                               bDataTypeSizeInOctets * iNumElem );
      }

      iBitSetSize = ( iNumElem * bDataTypeSizeInOctets ) << 3;
   }

   return( iBitSetSize );
}



/*------------------------------------------------------------------------------
**  Get min, max or default value(s) for an ADI. If the ADI is a structured ADI
**  all default values will have the same format as the structure describes.
**  The result is in network endian format.
**------------------------------------------------------------------------------
** Arguments:
**    psAdiEntry        - Pointer to ADI entry
**    pxDest            - Destination pointer
**    eMinMaxDefault    - Get min, max or default value described by
**                        AD_MinMaxDefaultIndexType
**    piOctetSize       - Size in octets of the written default values
**
** Returns:
**    ABP error code.
**------------------------------------------------------------------------------
*/
#if( AD_IA_MIN_MAX_DEFAULT_ENABLE )
static UINT8 GetMinMaxDefault( const AD_AdiEntryType* psAdiEntry,
                               void* pxDest,
                               AD_MinMaxDefaultIndexType eMinMaxDefault,
                               UINT16* piBitSize)
{
   UINT16 iSrcBitOffset;

   *piBitSize = 0;

#if( ABCC_CFG_STRUCT_DATA_TYPE )
   if( psAdiEntry->psStruct != NULL )
   {
      UINT16 i;

      for(i = 0; i < psAdiEntry->bNumOfElements; i++ )
      {
         if( psAdiEntry->psStruct[ i ].uData.sVOID.pxValueProps != NULL )
         {
            iSrcBitOffset = CalcStartindexBitOffset( psAdiEntry->psStruct[ i ].bDataType, (UINT16)eMinMaxDefault );

            *piBitSize += CopyValue( pxDest,
                                     *piBitSize,
                                     psAdiEntry->psStruct[ i ].uData.sVOID.pxValueProps,
                                     iSrcBitOffset,
                                     psAdiEntry->psStruct[ i ].bDataType,
                                     1 );
         }
         else
         {
            *piBitSize = 0;
            return( ABP_ERR_INV_CMD_EXT_0 );
         }
      }
   }
   else
#endif
   {
      if( psAdiEntry->uData.sVOID.pxValueProps != NULL )
      {
         iSrcBitOffset = CalcStartindexBitOffset( psAdiEntry->bDataType,
                                                  (UINT16)eMinMaxDefault );

         *piBitSize += CopyValue( pxDest,
                                  *piBitSize,
                                  psAdiEntry->uData.sVOID.pxValueProps,
                                  iSrcBitOffset,
                                  psAdiEntry->bDataType,
                                  1 );
      }
      else
      {
         *piBitSize = 0;
         return( ABP_ERR_INV_CMD_EXT_0 );
      }
   }
   return( ABP_ERR_NO_ERROR );
}
#endif

/*------------------------------------------------------------------------------
**  Get ADI of any data type. The read ADI will have network endian format.
**------------------------------------------------------------------------------
** Arguments:
**    psAdiEntry        - Pointer to ADI entry
**    pxDest            - Destination base pointer.
**    bNumElements      - Number of elements to read.
**    bStartIndex       - Index to first element to read.
**    piDestBitOffset   - Pointer to destination bit offset.
**                        This offset will be incremented according to the size
**                        of the read.
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void GetAdiValue( const AD_AdiEntryType* psAdiEntry,
                         void* pxDest,
                         UINT8 bNumElements,
                         UINT8 bStartIndex,
                         UINT16* piDestBitOffset )
{
   UINT16 iSrcBitOffset;

#if( ABCC_CFG_ADI_GET_SET_CALLBACK )
   /*
   ** If a get callback is registered the user is notified that the ADI shall be
   ** updated.
   */
   if( psAdiEntry->pnGetAdiValue != NULL )
   {
      psAdiEntry->pnGetAdiValue( psAdiEntry,
                                 bNumElements,
                                 bStartIndex );
   }
#endif

#if( ABCC_CFG_STRUCT_DATA_TYPE )
   if( psAdiEntry->psStruct != NULL )
   {
      UINT16 i;

      i = 0;

      /*
      ** For structures each element is handled separately.
      */
      for( i = bStartIndex; i < bNumElements + bStartIndex; i++ )
      {
         iSrcBitOffset = psAdiEntry->psStruct[ i ].bBitOffset;

         *piDestBitOffset += CopyValue( pxDest,
                                        *piDestBitOffset,
                                        psAdiEntry->psStruct[ i ].uData.sVOID.pxValuePtr,
                                        iSrcBitOffset,
                                        psAdiEntry->psStruct[ i ].bDataType,
                                        psAdiEntry->psStruct[ i ].iNumSubElem );
      }
   }
   else
#endif
   {
      iSrcBitOffset = CalcStartindexBitOffset( psAdiEntry->bDataType, bStartIndex );
      *piDestBitOffset += CopyValue( pxDest,
                                     *piDestBitOffset,
                                     psAdiEntry->uData.sVOID.pxValuePtr,
                                     iSrcBitOffset,
                                     psAdiEntry->bDataType,
                                     bNumElements );
   }
}

/*------------------------------------------------------------------------------
**  Set ADI of any data type. The provided data must have network endian format.
**------------------------------------------------------------------------------
** Arguments:
**    psAdiEntry        - Pointer to ADI entry.
**    pxData            - Source base pointer.
**    bNumElements      - Number of elements to write.
**    bStartIndex       - Index to first element to write.
**    piSrcBitOffset    - Pointer to source bit offset.
**                        This offset will be incremented according to the size
**                        written.
** Returns:
**    None
**------------------------------------------------------------------------------
*/
static void SetAdiValue( const AD_AdiEntryType* psAdiEntry,
                         void* pxData,
                         UINT8 bNumElements,
                         UINT8 bStartIndex,
                         UINT16* piSrcBitOffset )
{
   UINT16 iDestBitOffset;

#if( ABCC_CFG_STRUCT_DATA_TYPE )
   if( psAdiEntry->psStruct != NULL )
   {
      UINT16 i;

      /*
      ** For structures each element is handled separately.
      */
      for( i = bStartIndex; i < bNumElements + bStartIndex; i++ )
      {
         iDestBitOffset = psAdiEntry->psStruct[ i ].bBitOffset;
         *piSrcBitOffset += CopyValue( psAdiEntry->psStruct[ i ].uData.sVOID.pxValuePtr,
                                       iDestBitOffset,
                                       pxData,
                                       *piSrcBitOffset,
                                       psAdiEntry->psStruct[ i ].bDataType,
                                       psAdiEntry->psStruct[ i ].iNumSubElem );
      }
   }
   else
#endif
   {
      iDestBitOffset = CalcStartindexBitOffset( psAdiEntry->bDataType, bStartIndex );
      *piSrcBitOffset += CopyValue( psAdiEntry->uData.sVOID.pxValuePtr,
                                    iDestBitOffset,
                                    pxData,
                                    *piSrcBitOffset,
                                    psAdiEntry->bDataType,
                                    bNumElements );
   }
#if( ABCC_CFG_ADI_GET_SET_CALLBACK )
   if( psAdiEntry->pnSetAdiValue != NULL )
   {
      /*
      ** If a set callback is registered the user is notified that the ADI is
      ** updated.
      */
      psAdiEntry->pnSetAdiValue( psAdiEntry,
                                 bNumElements,
                                 bStartIndex );
   }
#endif
}

/*******************************************************************************
** Public Services
********************************************************************************
*/
EXTFUNC AD_ErrType AD_Init( const AD_AdiEntryType* psAdiEntry,
                            UINT16 iNumAdi,
                            const AD_DefaultMapType* psDefaultMap )
{
   UINT16 iMapIndex = 0;
   UINT16 iAdiIndex = 0;
   UINT8 bNumElem;
   UINT8 bElemStartIndex;
   /*
   ** In this context we should initialize the AD object to be prepared for
   ** startup.
   */
   ad_asADIEntryList = psAdiEntry;
   ad_asDefaultMap = psDefaultMap;

   ad_iNumOfADIs =  iNumAdi;
   ad_iHighestInstanceNumber = 0;

   ad_ReadMapInfo.paiMappedAdiList = ad_PdReadMapping;
   ad_ReadMapInfo.iPdSize = 0;
   ad_ReadMapInfo.iNumMappedAdi = 0;
   ad_ReadMapInfo.iMaxNumMappedAdi = AD_MAX_NUM_READ_ADI_TO_MAP;

   ad_WriteMapInfo.paiMappedAdiList = ad_PdWriteMapping;
   ad_WriteMapInfo.iPdSize = 0;
   ad_WriteMapInfo.iNumMappedAdi = 0;
   ad_WriteMapInfo.iMaxNumMappedAdi = AD_MAX_NUM_WRITE_ADI_TO_MAP;

   if( ad_asDefaultMap != NULL )
   {
      while( ad_asDefaultMap[ iMapIndex ].eDir != PD_END_MAP )
      {
         iAdiIndex = GetAdiIndex( ad_asDefaultMap[ iMapIndex ].iInstance );

         if( iAdiIndex == AD_INVALID_ADI_INDEX )
         {
            return AD_UNKNOWN_ADI;
         }

         bNumElem = ad_asDefaultMap[ iMapIndex ].bNumElem;
         bElemStartIndex = ad_asDefaultMap[ iMapIndex ].bElemStartIndex;

         if( iAdiIndex != AD_MAP_PAD_INDEX )
         {
            if( ad_asDefaultMap[ iMapIndex ].bNumElem == AD_DEFAULT_MAP_ALL_ELEM )
            {
               bNumElem = ad_asADIEntryList[ iAdiIndex ].bNumOfElements;
               bElemStartIndex = 0;
            }
         }

         if( ad_asDefaultMap[ iMapIndex ].eDir == PD_READ )
         {
            if( ad_ReadMapInfo.iNumMappedAdi >= ad_ReadMapInfo.iMaxNumMappedAdi )
            {
               return AD_TOO_MANY_READ_MAPPINGS;
            }

            ad_ReadMapInfo.paiMappedAdiList[ ad_ReadMapInfo.iNumMappedAdi ].bNumElements = bNumElem;
            ad_ReadMapInfo.paiMappedAdiList[ ad_ReadMapInfo.iNumMappedAdi ].bStartIndex = bElemStartIndex;
            ad_ReadMapInfo.paiMappedAdiList[ ad_ReadMapInfo.iNumMappedAdi ].iAdiIndex = iAdiIndex;
            ad_ReadMapInfo.iNumMappedAdi++;
         }
         else
         {
            if( ad_WriteMapInfo.iNumMappedAdi >= ad_WriteMapInfo.iMaxNumMappedAdi )
            {
               return AD_TOO_MANY_WRITE_MAPPINGS;
            }

            ad_WriteMapInfo.paiMappedAdiList[ ad_WriteMapInfo.iNumMappedAdi ].bNumElements = bNumElem;
            ad_WriteMapInfo.paiMappedAdiList[ ad_WriteMapInfo.iNumMappedAdi ].bStartIndex = bElemStartIndex;
            ad_WriteMapInfo.paiMappedAdiList[ ad_WriteMapInfo.iNumMappedAdi ].iAdiIndex = iAdiIndex;
            ad_WriteMapInfo.iNumMappedAdi++;
         }
         iMapIndex++;
      }
   }

   UpdateMapSize( &ad_WriteMapInfo );
   UpdateMapSize( &ad_ReadMapInfo );

   if( ad_ReadMapInfo.iPdSize > ABCC_CFG_MAX_PROCESS_DATA_SIZE )
   {
      return AD_PD_READ_SIZE_ERR;
   }

   if( ad_WriteMapInfo.iPdSize > ABCC_CFG_MAX_PROCESS_DATA_SIZE )
   {
      return AD_PD_WRITE_SIZE_ERR;
   }

   for( iAdiIndex = 0; iAdiIndex < ad_iNumOfADIs; iAdiIndex++ )
   {
      if( ad_asADIEntryList[ iAdiIndex ].iInstance > ad_iHighestInstanceNumber )
      {
         ad_iHighestInstanceNumber = ad_asADIEntryList[ iAdiIndex ].iInstance;
      }
   }

   return AD_NO_ERROR;
}

const AD_AdiEntryType* AD_GetAdiInstEntry( UINT16 iInstance )
{
   UINT16 i;
   const AD_AdiEntryType* psEntry = NULL;

   i = GetAdiIndex( iInstance );
   if( i != 0xffff )
   {
      psEntry = &ad_asADIEntryList[ i ];
   }
   return( psEntry );
}

void AD_ProcObjectRequest( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsgBuffer )
{
   const AD_AdiEntryType* psAdiEntry;
   UINT16 iMsgBitOffset;
   UINT16 iItemSize;
   UINT16 iTemp;
   UINT16 iDataSize;
   UINT8  bErrCode;

   iMsgBitOffset = 0;
   iDataSize = 0;
   bErrCode = ABP_ERR_NO_ERROR;

   if( iLeTOi( psMsgBuffer->sHeader.iInstance ) == ABP_INST_OBJ )
   {
      /*
      ** A request to the object instance.
      */
      switch( ABCC_GetMsgCmdBits( psMsgBuffer ) )
      {
      case ABP_CMD_GET_ATTR:
         switch( ABCC_GetMsgCmdExt0( psMsgBuffer ) )
         {
         case ABP_OA_NAME:
            ABCC_SetMsgString( psMsgBuffer, "Application data",16 ,0 );
            iDataSize = 16;
            break;

         case ABP_OA_REV:
            ABCC_SetMsgData8( psMsgBuffer, 1, 0 );
            iDataSize = ABP_OA_REV_DS;
            break;

         case ABP_OA_NUM_INST:
            ABCC_SetMsgData16( psMsgBuffer, ad_iNumOfADIs, 0 );
            iDataSize = ABP_OA_NUM_INST_DS;
            break;

         case ABP_OA_HIGHEST_INST:
            ABCC_SetMsgData16( psMsgBuffer, ad_iHighestInstanceNumber, 0 );
            iDataSize = ABP_OA_HIGHEST_INST_DS;
            break;

         case ABP_APPD_OA_NR_READ_PD_MAPPABLE_INSTANCES:
            {
               UINT16 iIndex;
               UINT16 iCnt = 0;

               for( iIndex=0; iIndex < ad_iNumOfADIs; iIndex++ )
               {
                  if( ad_asADIEntryList[iIndex ].bDesc & ABP_APPD_DESCR_MAPPABLE_READ_PD )
                  {
                     iCnt++;
                  }
               }
               ABCC_SetMsgData16( psMsgBuffer, iCnt, 0 );
               iDataSize = ABP_UINT16_SIZEOF;
            }
            break;

         case ABP_APPD_OA_NR_WRITE_PD_MAPPABLE_INSTANCES:
            {
               UINT16 iIndex;
               UINT16 iCnt = 0;

               for( iIndex=0; iIndex < ad_iNumOfADIs; iIndex++ )
               {
                  if( ad_asADIEntryList[ iIndex ].bDesc & ABP_APPD_DESCR_MAPPABLE_WRITE_PD )
                  {
                     iCnt++;
                  }
               }
               ABCC_SetMsgData16( psMsgBuffer, iCnt, 0 );
               iDataSize = ABP_UINT16_SIZEOF;
            }
            break;

         default:
            /*
            ** Unsupported attribute.
            */
            bErrCode = ABP_ERR_INV_CMD_EXT_0;
            break;
         }
         break;

      case ABP_APPD_CMD_GET_INST_BY_ORDER:
         iTemp = ABCC_GetMsgCmdExt( psMsgBuffer );

         if( ( iTemp == 0 ) ||
             ( iTemp > ad_iNumOfADIs ) )
         {
            /*
            ** Requested order number does not exist.
            */
            bErrCode = ABP_ERR_INV_CMD_EXT_0;
         }
         else
         {
            ABCC_SetMsgData16( psMsgBuffer,
                               ad_asADIEntryList[ iTemp - 1 ].iInstance,
                               0 );
            iDataSize = ABP_UINT16_SIZEOF;
         }
         break;

#if( ABCC_CFG_REMAP_SUPPORT_ENABLED )
      case ABP_APPD_REMAP_ADI_WRITE_AREA:
         RemapProcessDataCommand( hCtrl, psMsgBuffer, &ad_WriteMapInfo );
         psMsgBuffer = NULL;
         break;

      case ABP_APPD_REMAP_ADI_READ_AREA:
         RemapProcessDataCommand( hCtrl, psMsgBuffer, &ad_ReadMapInfo );
         psMsgBuffer = NULL;
         break;
#endif
      case ABP_APPD_GET_INSTANCE_NUMBERS:

         if( ABCC_GetMsgCmdExt0( psMsgBuffer ) != 0 )
         {
            bErrCode = ABP_ERR_INV_CMD_EXT_0;
         }
         else
         {
            UINT16 iStartingOrder;
            UINT16 iReqInstances;
            UINT16 i;
            UINT16 ii;

            ABCC_GetMsgData16( psMsgBuffer, &iStartingOrder, 0 );
            ABCC_GetMsgData16( psMsgBuffer, &iReqInstances, 2 );

            switch( ABCC_GetMsgCmdExt1( psMsgBuffer ) )
            {
            case ABP_APPD_LIST_TYPE_ALL:
               iDataSize = 0;
               for ( i = 0; ( i < iReqInstances ) && ( iStartingOrder + i <= ad_iNumOfADIs ); i++ )
               {
                  ABCC_SetMsgData16( psMsgBuffer,
                                     ad_asADIEntryList[ iStartingOrder + i - 1 ].iInstance,
                                     iDataSize );
                  iDataSize += ABP_UINT16_SIZEOF;
               }
               break;

            case ABP_APPD_LIST_TYPE_RD_PD_MAPPABLE:
               iDataSize = 0;
               for ( i = 0, ii = 1; ( ii < ( iStartingOrder + iReqInstances ) ) && ( i < ad_iNumOfADIs ); i++ )
               {
                  if( ad_asADIEntryList[ i ].bDesc & ABP_APPD_DESCR_MAPPABLE_READ_PD )
                  {
                     if( ii >= iStartingOrder )
                     {
                        ABCC_SetMsgData16( psMsgBuffer,
                                           ad_asADIEntryList[ i ].iInstance,
                                           iDataSize );
                        iDataSize += ABP_UINT16_SIZEOF;
                     }
                     ii++;
                  }
               }
               break;

            case ABP_APPD_LIST_TYPE_WR_PD_MAPPABLE:
               iDataSize = 0;
               for ( i = 0, ii = 1; ( ii < ( iStartingOrder + iReqInstances ) ) && ( i < ad_iNumOfADIs ); i++ )
               {
                  if( ad_asADIEntryList[ i ].bDesc & ABP_APPD_DESCR_MAPPABLE_WRITE_PD )
                  {
                     if( ii >= iStartingOrder )
                     {
                        ABCC_SetMsgData16( psMsgBuffer, ad_asADIEntryList[ i ].iInstance, iDataSize );
                        iDataSize += ABP_UINT16_SIZEOF;
                     }
                     ii++;
                  }
               }
               break;

            default:
               bErrCode = ABP_ERR_INV_CMD_EXT_1;
               break;
            }
         }
         break;

      default:
         bErrCode = ABP_ERR_UNSUP_CMD;
         break;
      }
   }
   else if( ( psAdiEntry = AD_GetAdiInstEntry( ABCC_GetMsgInstance( psMsgBuffer ) ) ) != NULL )
   {
      /*
      ** The ADI instance was found. Now switch on command.
      */
      switch( ABCC_GetMsgCmdBits( psMsgBuffer ) )
      {
      case ABP_CMD_GET_ATTR:
         /*
         ** Switch on attribute.
         */
         switch( ABCC_GetMsgCmdExt0( psMsgBuffer ) )
         {
         case ABP_APPD_IA_NAME:
            if( psAdiEntry->pacName )
            {
               iDataSize = (UINT16)strlen( psAdiEntry->pacName );
               ABCC_SetMsgString( psMsgBuffer,
                                  psAdiEntry->pacName, iDataSize, 0 );
            }
            else
            {
               iDataSize = 0;
            }
            break;

         case ABP_APPD_IA_DATA_TYPE:
#if( ABCC_CFG_STRUCT_DATA_TYPE )
            if( psAdiEntry->psStruct != NULL )
            {
               UINT16 i;

               iDataSize = ABP_APPD_IA_DATA_TYPE_DS*psAdiEntry->bNumOfElements;
               for ( i = 0; i < psAdiEntry->bNumOfElements; i++ )
               {
                  ABCC_SetMsgData8( psMsgBuffer,
                                    psAdiEntry->psStruct[i].bDataType, i );
               }
            }
            else
#endif
            {
               ABCC_SetMsgData8( psMsgBuffer, psAdiEntry->bDataType, 0 );
               iDataSize = ABP_APPD_IA_DATA_TYPE_DS;
            }
            break;

         case ABP_APPD_IA_NUM_ELEM:
            ABCC_SetMsgData8( psMsgBuffer, psAdiEntry->bNumOfElements, 0 );
            iDataSize = ABP_APPD_IA_NUM_ELEM_DS;
            break;

         case ABP_APPD_IA_DESCRIPTOR:
#if( ABCC_CFG_STRUCT_DATA_TYPE )
            if( psAdiEntry->psStruct != NULL )
            {
               UINT16 i;

               iDataSize = ABP_APPD_IA_DESCRIPTOR_DS * psAdiEntry->bNumOfElements;
               for ( i = 0; i < psAdiEntry->bNumOfElements; i++ )
               {
                  ABCC_SetMsgData8( psMsgBuffer,
                                    psAdiEntry->psStruct[i].bDesc, i );
               }
            }
            else
#endif
            {
               ABCC_SetMsgData8( psMsgBuffer, psAdiEntry->bDesc, 0 );
               iDataSize = ABP_APPD_IA_DESCRIPTOR_DS;
            }
            break;

         case ABP_APPD_IA_VALUE: /* Value. */
            if( !( psAdiEntry->bDesc & ABP_APPD_DESCR_GET_ACCESS ) )
            {
               bErrCode = ABP_ERR_ATTR_NOT_GETABLE;
               break;
            }
            ABCC_GetMsgDataPtr( psMsgBuffer )[ 0 ] = 0;

            GetAdiValue( psAdiEntry, ABCC_GetMsgDataPtr( psMsgBuffer ),
                            psAdiEntry->bNumOfElements, 0,
                            &iMsgBitOffset);
            iDataSize = SizeInOctets( 0, iMsgBitOffset );
            break;
#if( AD_IA_MIN_MAX_DEFAULT_ENABLE )
         case ABP_APPD_IA_MAX_VALUE:
            bErrCode = GetMinMaxDefault( psAdiEntry,
                                         ABCC_GetMsgDataPtr( psMsgBuffer ),
                                         AD_MAX_VALUE_INDEX,
                                         &iDataSize );
            iDataSize = SizeInOctets( 0, iDataSize );
            break;
         case ABP_APPD_IA_MIN_VALUE:
            bErrCode = GetMinMaxDefault( psAdiEntry,
                                         ABCC_GetMsgDataPtr( psMsgBuffer ),
                                         AD_MIN_VALUE_INDEX,
                                         &iDataSize );
            iDataSize = SizeInOctets( 0, iDataSize );
            break;
         case ABP_APPD_IA_DFLT_VALUE:
            bErrCode = GetMinMaxDefault( psAdiEntry,
                                         ABCC_GetMsgDataPtr( psMsgBuffer ),
                                         AD_DEFAULT_VALUE_INDEX,
                                         &iDataSize );
            iDataSize = SizeInOctets( 0, iDataSize );
            break;
#endif
         case ABP_APPD_IA_NUM_SUB_ELEM:
#if( ABCC_CFG_STRUCT_DATA_TYPE )
            if( psAdiEntry->psStruct != NULL )
            {
               UINT16 i;

               iDataSize = ABP_APPD_IA_NUM_SUB_ELEM_DS * psAdiEntry->bNumOfElements;
               for ( i = 0; i < psAdiEntry->bNumOfElements; i++ )
               {
                  ABCC_SetMsgData16( psMsgBuffer,
                                     psAdiEntry->psStruct[i].iNumSubElem,
                                     ( i * ABP_APPD_IA_NUM_SUB_ELEM_DS ) );
               }
            }
            else
#endif
            {
               // ABCC_SetMsgData16( psMsgBuffer, AD_DEFAULT_NUM_SUB_ELEMS, 0 );
               // iDataSize = ABP_APPD_IA_NUM_SUB_ELEM_DS;
               bErrCode = ABP_ERR_INV_CMD_EXT_0;
            }
            break;
         default:
            /*
            ** Unsupported attribute.
            */
            bErrCode = ABP_ERR_INV_CMD_EXT_0;
            break;
         }
         break;
      case ABP_CMD_SET_ATTR:
         switch( ABCC_GetMsgCmdExt0( psMsgBuffer ) )
         {
         case ABP_APPD_IA_NAME:
         case ABP_APPD_IA_DATA_TYPE:
         case ABP_APPD_IA_NUM_ELEM:
         case ABP_APPD_IA_DESCRIPTOR:
            /*
            ** Attributes are not settable.
            */
            bErrCode = ABP_ERR_ATTR_NOT_SETABLE;
            break;

         case ABP_APPD_IA_VALUE:
            if( !( psAdiEntry->bDesc & ABP_APPD_DESCR_SET_ACCESS ) )
            {
               bErrCode = ABP_ERR_ATTR_NOT_SETABLE;
               break;
            }

            /*
            ** Check the length of each array.
            */
            iItemSize = GetAdiSizeInOctets( psAdiEntry );
            if( iLeTOi( psMsgBuffer->sHeader.iDataSize ) > iItemSize )
            {
               bErrCode = ABP_ERR_TOO_MUCH_DATA;
               break;
            }
            else if( iLeTOi( psMsgBuffer->sHeader.iDataSize ) < iItemSize )
            {
               bErrCode = ABP_ERR_NOT_ENOUGH_DATA;
               break;
            }
            /*
            ** TODO Range check
            */
            if(  FALSE )
            {
               bErrCode = ABP_ERR_OUT_OF_RANGE;
            }
            else
            {
               SetAdiValue( psAdiEntry, ABCC_GetMsgDataPtr( psMsgBuffer ),
                            psAdiEntry->bNumOfElements, 0,
                            &iMsgBitOffset);
               /*
               ** Success.
               */
               iDataSize = 0;
            }
            break;

         default:
            /*
            ** Unsupported attribute.
            */
            bErrCode = ABP_ERR_INV_CMD_EXT_0;
            break;
         }
         break;

      case ABP_CMD_GET_ENUM_STR:
         switch( ABCC_GetMsgCmdExt0( psMsgBuffer ) )
         {
         case ABP_APPD_IA_VALUE:
            if( psAdiEntry->bDataType == ABP_ENUM )
            {
               if( ( psAdiEntry->uData.sENUM.psValueProps == NULL ) ||
                   ( psAdiEntry->uData.sENUM.psValueProps->pasEnumStrings == NULL ) )
               {
                  bErrCode = ABP_ERR_UNSUP_CMD;
               }
               else
               {
                  UINT8 b = 0;

                  for( b = 0; b < psAdiEntry->uData.sENUM.psValueProps->bNumOfEnumStrings; b++ )
                  {
                     if( psAdiEntry->uData.sENUM.psValueProps->pasEnumStrings[ b ].eValue ==
                         ABCC_GetMsgCmdExt1( psMsgBuffer ) )
                     {
                        break;
                     }
                  }

                  if( b < psAdiEntry->uData.sENUM.psValueProps->bNumOfEnumStrings )
                  {
                     iDataSize = (UINT16)strlen( psAdiEntry->uData.sENUM.psValueProps->pasEnumStrings[ b ].acEnumStr );
                     ABCC_SetMsgString( psMsgBuffer,
                                        psAdiEntry->uData.sENUM.psValueProps->pasEnumStrings[ b ].acEnumStr,
                                        iDataSize, 0 );
                  }
                  else
                  {
                     /*
                     ** The enum value was not found in the string lookup.
                     */
                     bErrCode = ABP_ERR_OUT_OF_RANGE;
                  }
               }
            }
            else
            {
               bErrCode = ABP_ERR_UNSUP_CMD;
            }
            break;

         default:
            bErrCode = ABP_ERR_UNSUP_CMD;
            break;
         }
         break;
      case ABP_CMD_GET_INDEXED_ATTR:
         switch( ABCC_GetMsgCmdExt0( psMsgBuffer ) )
         {
            case ABP_APPD_IA_VALUE:
               if( !( psAdiEntry->bDesc & ABP_APPD_DESCR_GET_ACCESS ) )
               {
                  bErrCode = ABP_ERR_ATTR_NOT_GETABLE;
                  break;
               }
               else
               {
                  GetAdiValue( psAdiEntry, ABCC_GetMsgDataPtr( psMsgBuffer ),
                                  1, ABCC_GetMsgCmdExt1( psMsgBuffer ),
                                  &iMsgBitOffset );

                  iDataSize = SizeInOctets( 0, iMsgBitOffset );

                  if( iDataSize == 0 )
                  {
                     bErrCode = ABP_ERR_OUT_OF_RANGE;
                  }
               }
            break;

         default:
            bErrCode = ABP_ERR_UNSUP_CMD;
            break;
         }
         break;
      case ABP_CMD_SET_INDEXED_ATTR:
         switch( ABCC_GetMsgCmdExt0( psMsgBuffer ) )
         {
            case ABP_APPD_IA_VALUE:
               if( !( psAdiEntry->bDesc & ABP_APPD_DESCR_SET_ACCESS ) )
               {
                  bErrCode = ABP_ERR_ATTR_NOT_SETABLE;
                  break;
               }
               else
               {
                  if( FALSE )
                  {
                     bErrCode = ABP_ERR_OUT_OF_RANGE;
                  }
                  else
                  {
                     SetAdiValue( psAdiEntry,
                                  ABCC_GetMsgDataPtr( psMsgBuffer ),
                                  1, ABCC_GetMsgCmdExt1( psMsgBuffer ),
                                  &iMsgBitOffset);
                     /*
                     ** Success.
                     */
                     iDataSize = 0;
                  }
               }
               break;

         default:
            bErrCode =  ABP_ERR_UNSUP_CMD;
            break;
         }
         break;

      default:
         /*
         ** Unsupported command.
         */
         bErrCode = ABP_ERR_UNSUP_CMD;
         break;
      }
   }
   else
   {
      /*
      ** The instance was not found.
      */
      bErrCode = ABP_ERR_UNSUP_INST;
   }

   /*
   ** Special handling. The remap response is already handled.
   */
   if( psMsgBuffer != NULL )
   {
      if( bErrCode == ABP_ERR_NO_ERROR )
      {
         ABP_SetMsgResponse( psMsgBuffer, iDataSize );
      }
      else
      {
         ABP_SetMsgErrorResponse( psMsgBuffer, 1, bErrCode );
      }

      ABCC_SendRespMsg( hCtrl, psMsgBuffer );
   }
}


void AD_UpdatePdReadData( void* pxPdDataBuf )
{
   UINT16 i;
   UINT16 iRdPdBitOffset;
   const ad_MapType* AD_paiPdReadMap = ad_ReadMapInfo.paiMappedAdiList;

   iRdPdBitOffset = 0;

   if( AD_paiPdReadMap )
   {
      for ( i = 0; i < ad_ReadMapInfo.iNumMappedAdi; i++ )
      {
         if( AD_paiPdReadMap->iAdiIndex != AD_MAP_PAD_INDEX )
         {
            SetAdiValue( &ad_asADIEntryList[ AD_paiPdReadMap->iAdiIndex ],
                            pxPdDataBuf,
                            AD_paiPdReadMap->bNumElements,
                            AD_paiPdReadMap->bStartIndex,
                            &iRdPdBitOffset );
         }
         else
         {
            iRdPdBitOffset += AD_paiPdReadMap->bNumElements;
         }

         AD_paiPdReadMap++;
      }
   }
}

BOOL AD_UpdatePdWriteData( void* pxPdDataBuf )
{
   UINT16 i;
   UINT16 iWrPdBitOffset;
   const ad_MapType* paiPdWriteMap = ad_WriteMapInfo.paiMappedAdiList;

   if( paiPdWriteMap )
   {
      iWrPdBitOffset = 0;

      for( i = 0; i < ad_WriteMapInfo.iNumMappedAdi ; i++)
      {
         if( paiPdWriteMap->iAdiIndex != AD_MAP_PAD_INDEX )
         {
            GetAdiValue( &ad_asADIEntryList[ paiPdWriteMap->iAdiIndex ],
                         pxPdDataBuf,
                         paiPdWriteMap->bNumElements,
                         paiPdWriteMap->bStartIndex ,
                         &iWrPdBitOffset);
         }
         else
         {
            iWrPdBitOffset += paiPdWriteMap->bNumElements;
         }
         paiPdWriteMap++;
      }
      return( TRUE );
   }
   return( FALSE );
}

UINT16 AD_AdiMappingReq( IDL_CTRL_HDL hCtrl,
                         const AD_AdiEntryType** ppsAdiEntry,
                         const AD_DefaultMapType** ppsDefaultMap )
{
   NetFormatType eNetFormat;
   eNetFormat = ABCC_NetFormatType(hCtrl);
#ifdef ABCC_SYS_BIG_ENDIAN
   ad_fDoNetworkEndianSwap = ( eNetFormat == NET_LITTLEENDIAN ) ? TRUE : FALSE;
#else
   ad_fDoNetworkEndianSwap = ( eNetFormat == NET_LITTLEENDIAN ) ? FALSE : TRUE;
#endif

   *ppsAdiEntry = ad_asADIEntryList;
   *ppsDefaultMap = ad_asDefaultMap;

   return ad_iNumOfADIs;
}

void AD_RemapDone( IDL_CTRL_HDL hCtrl )
{
   /*
   ** This Write Process Data update is to ensure that the write process data
   ** is updated with the right content.
   */
   ABCC_TriggerWrPdUpdate(hCtrl);
}
