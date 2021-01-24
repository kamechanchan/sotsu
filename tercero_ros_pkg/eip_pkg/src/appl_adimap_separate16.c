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
** Example of an ADI setup with 32 ADIs each one holding a 16 bit value.
**
********************************************************************************
********************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>
#include "appl_adi_config.h"


#if ( APPL_ACTIVE_ADI_SETUP == APPL_ADI_SETUP_SEPARATE_16 )

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
/*------------------------------------------------------------------------------
** Forward declarations
**------------------------------------------------------------------------------
*/
static void SetAdi10Value( const struct AD_AdiEntry* psAdiEntry, UINT8 bNumElements, UINT8 bStartIndex );
static void GetAdi11Value( const struct AD_AdiEntry* psAdiEntry, UINT8 bNumElements, UINT8 bStartIndex );

/*------------------------------------------------------------------------------
** Data holder for the ADI instances
**------------------------------------------------------------------------------
*/
static UINT16  appl_aiUint16_10[ 250 ];
static UINT16  appl_aiUint16_11[ 250 ];
static UINT16  appl_Uint16_12 = 0;
/*******************************************************************************
** Public Globals
********************************************************************************
*/

/*------------------------------------------------------------------------------
** 32 16-bit values individually
** See abcc_ad_if.h for a more detailed description.
**------------------------------------------------------------------------------
*/
/*-----------------------------------------------------------------------------------------------------------------------
** iInstance | pabName | bDataType | bNumOfElements | bDesc | pxValuePtr | pxValuePropPtr| pnGetAdiValue | pnSetAdiValue
**-----------------------------------------------------------------------------------------------------------------------
*/
const AD_AdiEntryType APPL_asAdiEntryList[] =
{
   { 10,   "ABP_UINT16_SET",        ABP_UINT16,   250, APPL_READ_MAP_WRITE_ACCESS_DESC, { { appl_aiUint16_10 ,NULL } }, NULL, NULL, SetAdi10Value },
   { 11,   "ABP_UINT16_GET",        ABP_UINT16,   250, APPL_WRITE_MAP_READ_ACCESS_DESC, { { appl_aiUint16_11 ,NULL } }, NULL, GetAdi11Value, NULL },
   { 12,   "ABP_UINT16_COUNTER",    ABP_UINT16,   1,  APPL_NOT_MAP_WRITE_ACCESS_DESC,  { { &appl_Uint16_12 ,NULL  } }, NULL, NULL, NULL         },
};

/*
**------------------------------------------------------------------------------
** Map all adi:s in both directions
** See abcc_ad_if.h for a more detailed description.
**------------------------------------------------------------------------------
** 1. AD instance | 2. Direction | 3. Num elements | 4. Start index |
**------------------------------------------------------------------------------
*/
const AD_DefaultMapType APPL_asAdObjDefaultMap[] =
{
   { 10,  PD_READ,   AD_DEFAULT_MAP_ALL_ELEM, 0 },
   { 11,  PD_WRITE,  AD_DEFAULT_MAP_ALL_ELEM, 0 },
   { AD_DEFAULT_MAP_END_ENTRY }
};
/*******************************************************************************
** Private Services
********************************************************************************
*/
/*------------------------------------------------------------------------------
** Callback of type ABCC_GetAdiValueFuncType. The function will be called when
** the network reads ADI #11. It will increment the value of ADI#12 every time
** this is done.
**
** ABCC_GetAdiValueFuncType is declared in abcc_ad_if.h
**------------------------------------------------------------------------------
*/
static void GetAdi11Value( const struct AD_AdiEntry* psAdiEntry, UINT8 bNumElements, UINT8 bStartIndex )
{
   appl_Uint16_12++;
}
/*------------------------------------------------------------------------------
** Callback of type ABCC_SetAdiValueFuncType. The function will be called when
** the network writes to ADI#10. It copies the changed values from ADI#10 to ADI#11.
**
** ABCC_SetAdiValueFuncType is declared in abcc_ad_if.h
**------------------------------------------------------------------------------
*/
static void SetAdi10Value( const struct AD_AdiEntry* psAdiEntry, UINT8 bNumElements, UINT8 bStartIndex )
{
   // UINT8 index;
   // for( index = bStartIndex ; index < bStartIndex+bNumElements ; index++ )
   // {
   //    printf("%d", appl_aiUint16_11[ index ]);
   // }
   // printf("\n");
}
/*******************************************************************************
** Public Services
********************************************************************************
*/
UINT16 APPL_GetNumAdi( void )
{
   return( sizeof( APPL_asAdiEntryList ) / sizeof( AD_AdiEntryType ) );
}
/*******************************************************************************
** Tasks
********************************************************************************
*/
#endif

