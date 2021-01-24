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
*/

#ifndef __IDL_H__
#define __IDL_H__

#include "IDL_hwpara.h"

/*******************************************************************************
** Public Services
********************************************************************************
*/

/*------------------------------------------------------------------------------
** ABCC_OpenController()
** This function will open the connection to the hardware.
**------------------------------------------------------------------------------
** Arguments:
**       dwHwIndex         - Opens the hardware with the given index
**       pstcHwPara        - Hardware specific Parameters. If NULL default
**                           settings will be used.
**       hCtrl             - Handle to the hardware.
** Note! If the hardware is used by another application this call will fail.
** Returns:
**       TRUE if hardware was succesfully opened otherwise FALSE
**------------------------------------------------------------------------------
*/
EXTFUNC IDL_DLLEXPORT BOOL ABCC_OpenController(UINT32 dwHwIndex,
                                               IDL_HW_PARA *pstcHwPara,
                                               IDL_CTRL_HDL* hCtrl);

/*------------------------------------------------------------------------------
** ABCC_CloseController()
** This function will close the connection to the hardware and releases the
** handle.
** Note! After calling ABCC_CloseController the handle should be
** IDL_INVALID_HANDLE.
**------------------------------------------------------------------------------
** Arguments:
**       hCtrl             - Handle which should be released
**------------------------------------------------------------------------------
*/
EXTFUNC IDL_DLLEXPORT void ABCC_CloseController(IDL_CTRL_HDL* hCtrl);

/*------------------------------------------------------------------------------
** IDL_UseCriticalImpl()
**------------------------------------------------------------------------------
** Arguments:
**       hCtrl             - Handle to the hardware.
**------------------------------------------------------------------------------
*/
EXTFUNC IDL_DLLEXPORT void IDL_APICALL IDL_UseCriticalImpl( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** IDL_EnterCriticalImpl()
**------------------------------------------------------------------------------
** Arguments:
**       hCtrl             - Handle to the hardware.
**------------------------------------------------------------------------------
*/
EXTFUNC IDL_DLLEXPORT void IDL_APICALL IDL_EnterCriticalImpl( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** IDL_ExitCriticalImpl()
**------------------------------------------------------------------------------
** Arguments:
**       hCtrl             - Handle to the hardware.
**------------------------------------------------------------------------------
*/
EXTFUNC IDL_DLLEXPORT void IDL_APICALL IDL_ExitCriticalImpl( IDL_CTRL_HDL hCtrl );

#endif //__IDL_H__
