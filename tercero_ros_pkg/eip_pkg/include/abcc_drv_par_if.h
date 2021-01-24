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
** permission. When used together with a product from HMS, this code can be   **
** modified, reproduced and distributed in binary form without any            **
** restrictions.                                                              **
**                                                                            **
** THE CODE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND. HMS DOES NOT    **
** WARRANT THAT THE FUNCTIONS OF THE CODE WILL MEET YOUR REQUIREMENTS, OR     **
** THAT THE OPERATION OF THE CODE WILL BE UNINTERRUPTED OR ERROR-FREE, OR     **
** THAT DEFECTS IN IT CAN BE CORRECTED.                                       **
********************************************************************************
********************************************************************************
** Defines the parallel driver interface.
********************************************************************************
********************************************************************************
** Driver services:
**       ABCC_DrvParInit()	                    - Initialize driver privates and states to default values.
**       ABCC_DrvParISR()                       - Calls in the interrupt context to acknowledge received interrupts.
**       ABCC_DrvParExecute()                   - Drives the internal send/receive process if applicable
**       ABCC_DrvParWriteMessage()              - Writes a message.
**       ABCC_DrvParWriteProcessData()          - Writes current process data.
**       ABCC_DrvParIsReadyForWriteMessage()    - Checks if the driver is ready to send a new write message.
**       ABCC_DrvParIsReadyForCmd()             - Checks if the Anybus is ready to receive a new command message.
**       ABCC_DrvParSetNbrOfCmds()              - Sets the number of simultaneous commands that is supported by the application.
**       ABCC_DrvParSetAppStatus()              - Sets the current application status.
**       ABCC_DrvParSetPdSize()                 - Sets the current process data size.
**       ABCC_DrvParSetMsgReceiverBuffer()      - Sets the message receiver buffer.
**       ABCC_DrvParSetIntMask()                - Set interrupt mask
**       ABCC_DrvParGetWrPdBuffer()             - Get wrpd buffer
**       ABCC_DrvParGetModCap()                 - Read module capability
**       ABCC_DrvParGetLedStat()                - Read led status
**       ABCC_DrvParParGetIntStatus()              - Get current interrupt status
**       ABCC_DrvParGetAnybusState()            - Get current Anybus state
**       ABCC_DrvParReadProcessData()           - Get read process data if any.
**       ABCC_DrvParReadMessage()               - Get read message if any.
**       ABCC_DrvParRegisterWd()                - Function to call when Wd timeOuts
**       ABCC_DrvParIsSuperviced()              - Is the network supervised
********************************************************************************
********************************************************************************
*/

#ifndef PHY_DRV_PAR_IF_H_
#define PHY_DRV_PAR_IF_H_
#include "abcc_drv_cfg.h"
#include "abcc_td.h"
#include "abp.h"

#if( ABCC_CFG_DRV_PARALLEL )

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
** Initializes the driver to default values.
** Must be called before the driver is used.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_DrvParInit( IDL_CTRL_HDL hCtrl, UINT8 bOpmode );


/*------------------------------------------------------------------------------
** Calls in the interrupt context to acknowledge received interrupts.
**
** Remarks:
**       The ISR routine will clear all pending interrupts.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       Acknowledged interrupts.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT16  ABCC_DrvParISR( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Drives the internal send process.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_DrvParRunDriverTx( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Drives the internal receive process.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       Pointer to successfully sent write message.
**------------------------------------------------------------------------------
*/
EXTFUNC ABP_MsgType* ABCC_DrvParRunDriverRx( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Writes a message to the driver.
**------------------------------------------------------------------------------
** Arguments:
**       psWriteMsg:    Pointer to message.
**
** Returns:
**       True:          Message was successfully written and can be deallocated
**                      immediately.
**       False:         Message was not yet written and cannot be deallocated.
**                      The psWriteMsg pointer is owned by the driver until the
**                      message is written and the pointer is returned in the
**                      driver execution response.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ABCC_DrvParWriteMessage( IDL_CTRL_HDL hCtrl, 
                                      ABP_MsgType* psWriteMsg );


/*------------------------------------------------------------------------------
** Writes current process data.
** The data is copied before returning from the method.
**------------------------------------------------------------------------------
** Arguments:
**       pxProcessData: Pointer to process data to be sent.
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_DrvParWriteProcessData( IDL_CTRL_HDL hCtrl, 
                                          void* pxProcessData );

/*------------------------------------------------------------------------------
** Checks if the driver is in the correct state for writing process data to the anybus
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       True:          Driver is in correct state to send WrPd
**       False:         Driver is not in correct state to send Wrpd
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ABCC_DrvParIsReadyForWrPd( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Checks if the driver is ready to send a new write message.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       True:          Driver is ready to send a new write message.
**       False:         Driver is not ready to send a new write message.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ABCC_DrvParIsReadyForWriteMessage( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** The host application checks if the Anybus is ready to receive a new command
** message.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       True:          OK to send new command.
**       False:         NOK to send new command.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ABCC_DrvParIsReadyForCmd( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Sets the number of simultaneous commands that is supported by the application.
**------------------------------------------------------------------------------
** Arguments:
**       bNbrOfCmds:    Number of commands that the application is ready to receive.
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_DrvParSetNbrOfCmds( IDL_CTRL_HDL hCtrl, UINT8 bNbrOfCmds );


/*------------------------------------------------------------------------------
**  Sets the current application status.
**  Note! This information is not supported by all protocols.
**------------------------------------------------------------------------------
** Arguments:
**       eAppStatus:    Current application status.
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_DrvParSetAppStatus( IDL_CTRL_HDL hCtrl, 
                                      ABP_AppStatusType eAppStatus );


/*------------------------------------------------------------------------------
** Sets the current process data size.
**------------------------------------------------------------------------------
** Arguments:
**       iReadPdSize:   Size of read process data (bytes)
**       iWritePdSize:  Size of write process data (bytes)
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_DrvParSetPdSize( IDL_CTRL_HDL hCtrl, const UINT16 iReadPdSize, 
                                   const UINT16 iWritePdSize );


/*------------------------------------------------------------------------------
** Sets the receiver buffer, to be used for the next read message.
**------------------------------------------------------------------------------
** Arguments:
**       psReadMsg:     Pointer where next read message will be put.
**                      psReadMsg is not allowed to contain a NULL value.
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_DrvParSetMsgReceiverBuffer( IDL_CTRL_HDL hCtrl, 
                                              ABP_MsgType* const psReadMsg );


/*------------------------------------------------------------------------------
** Sets Interrupt mask according to h_aci.h.
**------------------------------------------------------------------------------
** Arguments:
**       iIntMask:      Interrupt mask set according to h_aci.h.
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_DrvParSetIntMask( IDL_CTRL_HDL hCtrl, const UINT16 iIntMask );


/*------------------------------------------------------------------------------
** Get WrpdBuffer for the user to update.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       Pointer to WrPd buffer.
**------------------------------------------------------------------------------
*/
EXTFUNC void* ABCC_DrvParGetWrPdBuffer( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Read module capabillity
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       Module capability.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT16 ABCC_DrvParGetModCap( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Read module capability
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       Module capability.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT16 ABCC_DrvParGetLedStatus( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Gets the Anybus interrupt status.
**
** Remarks:
**       The ABCC_DrvISR() function will clear all pending interrupts. This
**       function must be called before ABCC_DrvISR() or it will always return 0.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       The Anybus interrupt status.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT16 ABCC_DrvParGetIntStatus( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Gets the Anybus state.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       The Anybus state.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT8 ABCC_DrvParGetAnybusState( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Reads the read process data.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       A pointer to the read process data; or NULL if no process data to read
**       was available.
**------------------------------------------------------------------------------
*/
EXTFUNC void* ABCC_DrvParReadProcessData( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Reads the read message.
**------------------------------------------------------------------------------
** Arguments:
**       None.
**
** Returns:
**       A pointer to the read message; or NULL if no message is available.
**       The pointer, if not NULL, will point to the buffer previously set by
**       calling ABCC_DrvSetMsgReceiverBuffer().
**------------------------------------------------------------------------------
*/
EXTFUNC ABP_MsgType* ABCC_DrvParReadMessage( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
**  Returns supervision bit in status register.
**------------------------------------------------------------------------------
** Arguments:
**          -
**
** Returns:
**          TRUE: The device is supervised by another network device.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ABCC_DrvParIsSupervised( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
**  Returns anybus status register.
**------------------------------------------------------------------------------
** Arguments:
**          -
**
** Returns:
**          Anybus status register
**------------------------------------------------------------------------------
*/
EXTFUNC UINT8 ABCC_DrvParGetAnbStatus ( IDL_CTRL_HDL hCtrl );

#endif  /* ABCC_CFG_DRV_PARALLEL */

#endif  /* inclusion lock */

/*******************************************************************************
** End of drv_if.h
********************************************************************************
*/
