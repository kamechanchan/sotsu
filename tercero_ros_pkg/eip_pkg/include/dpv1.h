/*******************************************************************************
********************************************************************************
**                                                                            **
** ABCC Starter Kit version 3.03.02 (2017-03-28)                              **
**                                                                            **
** Delivered with:                                                            **
**    ABP            7.39.01 (2017-03-22)                                     **
**    ABCC Driver    5.03.02 (2017-03-28)                                     **
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
** This is the public header file for the Profibus DPV1 IO object.
********************************************************************************
********************************************************************************
** Services:
**    DPV1_ProcessCmdMsg()    - Processes commands sent to the DPV1 object.
**
********************************************************************************
********************************************************************************
*/

#ifndef DPV1_H
#define DPV1_H

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
** Public Services
********************************************************************************
*/

/*------------------------------------------------------------------------------
** Processes commands sent to the Profibus object.
**------------------------------------------------------------------------------
** Arguments:
**    hCtrl             - Handle to the hardware.
**    psNewMessage      - Pointer to a ABP_MsgType message.
**
** Returns:
**    None.
**------------------------------------------------------------------------------
*/
void DPV1_ProcessCmdMsg( IDL_CTRL_HDL hCtrl, ABP_MsgType* psNewMessage );

#endif  /* inclusion lock */
