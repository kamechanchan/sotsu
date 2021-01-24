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
** Application Data Object 0xFE (ADI) - Public interfaces
********************************************************************************
********************************************************************************
** Services :
** AD_Init()                  - Initiate AD object.
** AD_ProcObjectRequest()     - Object request handling.
** AD_RemapDone()             - Remap finished.
** AD_UpdatePdReadData()      - Update of read process data.
** AD_UpdatePdWriteData()     - Update of write process data.
********************************************************************************
********************************************************************************
*/
#ifndef AD_OBJ_H
#define AD_OBJ_H

#include "abcc_td.h"
#include "abp.h"
#include "abcc_ad_if.h"

/*******************************************************************************
** Typedefs
********************************************************************************
*/
/*------------------------------------------------------------------------------
**  AD_NO_ERROR                - All OK
**  AD_PD_REDA_SIZE_ERR        - Total process data read size too large.
**                               Check ABCC_CFG_MAX_PROCESS_DATA_SIZE.
**  AD_PD_WRITE_SIZE_ERR       - Total process data write too large.
**                               Check ABCC_CFG_MAX_PROCESS_DATA_SIZE.
**  AD_TOO_MANY_READ_MAPPINGS  -  Read process data map has too many entries.
**                               Check AD_MAX_OF_READ_WRITE_TO_MAP.
**  AD_TOO_MANY_WRITE_MAPPINGS -  Write process data map has too many entries.
**                               Check AD_MAX_OF_WRITE_WRITE_TO_MAP.
**  AD_UNKNOWN_ADI             - Requested ADI could not be found.
**------------------------------------------------------------------------------
*/
typedef enum AD_Err
{
   AD_NO_ERROR = 0,
   AD_PD_READ_SIZE_ERR,
   AD_PD_WRITE_SIZE_ERR,
   AD_TOO_MANY_READ_MAPPINGS,
   AD_TOO_MANY_WRITE_MAPPINGS,
   AD_UNKNOWN_ADI
}
AD_ErrType;

/*******************************************************************************
** Public Services
********************************************************************************
*/
/*------------------------------------------------------------------------------
**  Initiates the AD object.
**------------------------------------------------------------------------------
** Arguments:
**    psAdiEntry   - Pointer to used ADI entry table.
**    iNumAdi      - Number of ADI:s in ADI entry table.
**    psDefaultMap - Pointer to default map. Set to NULL if no default map
**                   shall be used.
**
** Returns:
**    AD_ErrType
**------------------------------------------------------------------------------
*/
EXTFUNC AD_ErrType AD_Init( const AD_AdiEntryType* psAdiEntry,
                            UINT16 iNumAdi,
                            const AD_DefaultMapType* psDefaultMap );

/*------------------------------------------------------------------------------
**  Retrieve default mapping information from AD object. The AD objects also
**  prepares for network endian conversion if needed. The function may only be
**  called when requested by driver through the callback ABCC_CbfAdiMappingReq()
**------------------------------------------------------------------------------
** Arguments:
**     ppsAdiEntry    - Pointer to retrieve ADI entry table.
**                      NULL is returned if no default map is available.
**     ppsDefaultMap  - Pointer to retrieve default map information.
**                      NULL is returned if no default map is available.
**
** Returns:
**    Number of ADI:s in psAdiEntry table.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT16 AD_AdiMappingReq( IDL_CTRL_HDL hCtrl, 
                                 const AD_AdiEntryType** ppsAdiEntry,
                                 const AD_DefaultMapType** ppsDefaultMap );

/*------------------------------------------------------------------------------
** Indicate to AD object that the remap is finished
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
#if( ABCC_CFG_REMAP_SUPPORT_ENABLED )
EXTFUNC void AD_RemapDone( IDL_CTRL_HDL hCtrl );
#endif

/*------------------------------------------------------------------------------
** Process an object request against the Application Data Object.
**------------------------------------------------------------------------------
** Arguments:
**    psMsgBuffer      - Pointer to ABCC command message.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void AD_ProcObjectRequest( IDL_CTRL_HDL hCtrl, ABP_MsgType* psMsgBuffer );

/*------------------------------------------------------------------------------
** Update AD object with new read process data received from the ABCC.
**------------------------------------------------------------------------------
** Arguments:
**    pxPdDataBuf      - Pointer read process data buffer.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void AD_UpdatePdReadData( void* pxPdDataBuf );

/*------------------------------------------------------------------------------
** Fetch write process data from AD object.
**------------------------------------------------------------------------------
** Arguments:
**    pxPdDataBuf       - Pointer write process data buffer.
**
** Returns:
**    TRUE  -  Write process data buffer is updated.
**    FALSE -  No update was made.
**
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL AD_UpdatePdWriteData( void* pxPdDataBuf );
#endif  /* inclusion lock */
