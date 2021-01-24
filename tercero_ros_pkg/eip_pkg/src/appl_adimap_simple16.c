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
** Example of an ADI setup with an array of 16 bit values
**
********************************************************************************
********************************************************************************
*/

#include "appl_adi_config.h"

#if ( APPL_ACTIVE_ADI_SETUP == APPL_ADI_SETUP_SIMPLE_16 )

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
** Data holder for the ADI instances
**------------------------------------------------------------------------------
*/
static UINT16 appl_aiUint16[ 32 ];

/*------------------------------------------------------------------------------
** Min, max and default value for appl_aiUint16
**------------------------------------------------------------------------------
*/
static AD_UINT16Type appl_sUint16Prop = { { 0, 0xFFFF, 0 } };

/*******************************************************************************
** Public Globals
********************************************************************************
*/

/*------------------------------------------------------------------------------
** 32 16-bit values as an array
**------------------------------------------------------------------------------
*/

/*-------------------------------------------------------------------------------------------------------------
** 1. iInstance | 2. pabName | 3. bDataType | 4. bNumOfElements | 5. bDesc | 6. pxValuePtr | 7. pxValuePropPtr
**--------------------------------------------------------------------------------------------------------------
*/
const AD_AdiEntryType APPL_asAdiEntryList[] =
{
   {  0x1,  "ABP_UINT16_WRITE",   ABP_UINT16,   32, APPL_WRITE_MAP_READ_ACCESS_DESC, { { appl_aiUint16, &appl_sUint16Prop } }, NULL, NULL, NULL },
   {  0x2,  "ABP_UINT16_READ",    ABP_UINT16,   32, APPL_READ_MAP_WRITE_ACCESS_DESC,  { { appl_aiUint16, &appl_sUint16Prop } }, NULL, NULL, NULL }
};

/*------------------------------------------------------------------------------
** Map all adi:s in both directions
**------------------------------------------------------------------------------
** 1. AD instance | 2. Direction | 3. Num elements | 4. Start index |
**------------------------------------------------------------------------------
*/
const AD_DefaultMapType APPL_asAdObjDefaultMap[] =
{
   { 1, PD_WRITE, AD_DEFAULT_MAP_ALL_ELEM, 0 },
   { 2, PD_READ,  AD_DEFAULT_MAP_ALL_ELEM, 0 },
   { AD_DEFAULT_MAP_END_ENTRY }
};

/*******************************************************************************
** Private Services
********************************************************************************
*/

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
