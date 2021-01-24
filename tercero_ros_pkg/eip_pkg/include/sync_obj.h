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
** Sync Object 0xEE - Public interfaces
********************************************************************************
********************************************************************************
** Services:
**    SYNC_GetCycleTime()            - Returns the set cycle time
**    SYNC_GetInputCaptureTime()     - Returns the set input capture time
**    SYNC_GetMode()                 - Returns the set sync mode
**    SYNC_GetOutputValidTime()      - Returns the set output valid time
**    SYNC_ProcessCmdMsg()           - Processes a message addressed to this
**                                     object
**    SYNC_SetInputProcessingTime()  - Set the input processing time
**    SYNC_SetMinCycleTime()         - Set the minimum allowed cycle time
**    SYNC_SetOutputProcessingTime() - Set the output processing time
********************************************************************************
********************************************************************************
*/
#ifndef SYNC_OBJ_H
#define SYNC_OBJ_H

#include "abcc_td.h"
#include "abp.h"
#include "abcc_ad_if.h"


/*******************************************************************************
** Typedefs
********************************************************************************
*/

/*------------------------------------------------------------------------------
** Structure describing the sync modes of attribute 7, 'Sync mode'.
**------------------------------------------------------------------------------
*/
typedef enum SYNC_SyncMode
{
   SYNC_MODE_NONSYNCHRONOUS = 0,
   SYNC_MODE_SYNCHRONOUS = 1
}
SYNC_SyncModeType;

/*******************************************************************************
** Public Services
********************************************************************************
*/

/*------------------------------------------------------------------------------
** Get the currently configured cycle time
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    Cycle time in nanoseconds
**------------------------------------------------------------------------------
*/
EXTFUNC UINT32 SYNC_GetCycleTime( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Get the currently configured input capture time
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    Input capture time in nanoseconds
**------------------------------------------------------------------------------
*/
EXTFUNC UINT32 SYNC_GetInputCaptureTime( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Get the currently configured sync mode
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    Sync mode of type SYNC_SyncModeType.
**------------------------------------------------------------------------------
*/
EXTFUNC SYNC_SyncModeType SYNC_GetMode( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Get the currently configured output valid time
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    Output valid time in nanoseconds
**------------------------------------------------------------------------------
*/
EXTFUNC UINT32 SYNC_GetOutputValidTime( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Process an object request against the SYNC Object.
**------------------------------------------------------------------------------
** Arguments:
**    psMsgBuffer      - Pointer to ABCC command message.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void SYNC_ProcessCmdMsg( IDL_CTRL_HDL hCtrl,
                                 ABP_MsgType* psNewMessage );

/*------------------------------------------------------------------------------
** Updates the input processing time reported to the ABCC upon request.
** This value may be changed in runtime to reflect the timing required for the
** current process data map.
**------------------------------------------------------------------------------
** Arguments:
**    lInputProcTimeNs - Input processing time in nanoseconds
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void SYNC_SetInputProcessingTime( IDL_CTRL_HDL hCtrl,
                                          UINT32 lInputProcTimeNs );

/*------------------------------------------------------------------------------
** Updates the minimum cycle time reported to the ABCC upon request.
** This value may be changed in runtime to reflect the timing required for the
** current process data map.
**------------------------------------------------------------------------------
** Arguments:
**    lMinCycleTimeNs - Minimum cycle time in nanoseconds
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void SYNC_SetMinCycleTime( IDL_CTRL_HDL hCtrl,
                                   UINT32 lMinCycleTimeNs );

/*------------------------------------------------------------------------------
** Updates the output processing time reported to the ABCC upon request.
** This value may change in runtime to reflect the timing required for the
** current process data map.
**------------------------------------------------------------------------------
** Arguments:
**    lOutputProcTimeNs - Output processing time in nanoseconds
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void SYNC_SetOutputProcessingTime( IDL_CTRL_HDL hCtrl, 
                                           UINT32 lOutputProcTimeNs );

#endif
