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
** Defines the generic driver interface implemented by each specific driver.
********************************************************************************
********************************************************************************
** Driver services:
**    pnABCC_DrvInit()	                    - Initialize driver privates and
**                                            states to default values.
**    pnABCC_DrvISR()                       - Calls in the interrupt context to
**                                            acknowledge received
**                                            interrupts.
**    pnABCC_DrvRunDriverTx()               - Drives the internal send process
**                                            if applicable
**    pnABCC_DrvRunDriverRx()               - Drives the internal receive
**                                            process if applicable
**    pnABCC_DrvWriteMessage()              - Writes a message.
**    pnABCC_DrvWriteProcessData()          - Writes current process data.
**    pnABCC_DrvISReadyForWriteMessage()    - Checks if the driver is ready
**                                            to send a new write message.
**    pnABCC_DrvISReadyForCmd()             - Checks if the Anybus is ready to
**                                            receive a new command message.
**    pnABCC_DrvSetNbrOfCmds()              - Sets the number of simultaneous
**                                            commands that is supported by
**                                            the application.
**    pnABCC_DrvSetAppStatus()              - Sets the current application
**                                            status.
**    pnABCC_DrvSetPdSize()                 - Sets the current process data
**                                            size.
**    pnABCC_DrvSetMsgReceiverBuffer()      - Sets the message receiver buffer.
**    pnABCC_DrvSetIntMask()                - Set interrupt mask
**    pnABCC_DrvGetWrPdBuffer()             - Get wrpd buffer
**    pnABCC_DrvGetModCap()                 - Read module capability
**    pnABCC_DrvGetLedStatus()              - Read led status
**    pnABCC_DrvGetIntStatus()              - Get current interrupt status
**    pnABCC_DrvGetAnybusState()            - Get current Anybus state
**    pnABCC_DrvReadProcessData()           - Get read process data if any.
**    pnABCC_DrvReadMessage()               - Get read message if any.
**    pnABCC_DrvIsSuperviced()              - Is the network supervised
********************************************************************************
********************************************************************************
*/

#ifndef PHY_DRV_IF_H_
#define PHY_DRV_IF_H_

#include "abcc_drv_cfg.h"
#include "abcc_td.h"
#include "abcc_debug_err.h"
#include "abp.h"

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
** Reads an amount of bytes from the ABCC memory.
** This function/macro will be used by the driver when reading process data or
** message data from the ABCC memory.
** See also the ABCC_SYS_READ_RDPD macro description below.
**------------------------------------------------------------------------------
** Arguments:
**    iMemOffset  - Memory offset to start writing to.
**                  8 bit char platforms  : iMemOffset in octets
**                  16 bit char platforms : iMemOffset in 16 bit words
**    pxData      - Pointer to the data to be written.
**    iLength     - The amount of data to write in octets.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
#define ABCC_DrvParallelRead( hCtrl, iMemOffset, pxData, iLength )                    \
        ABCC_SYS_ParallelRead( hCtrl, iMemOffset, pxData, iLength )

/*------------------------------------------------------------------------------
** Reads a byte from the ABCC memory.
**------------------------------------------------------------------------------
** Arguments:
**    iMemOffset - Offset from ABCC base address.
**                 8 bit char platforms  : iMemOffset in octets
**                 16 bit char platforms : iMemOffset in 16 bit words
**
** Returns:
**    Read UINT8
**------------------------------------------------------------------------------
*/
#define ABCC_DrvRead8( hCtrl, iMemOffset )  ABCC_SYS_ParallelRead8( hCtrl, iMemOffset )
#define ABCC_DrvRead16( hCtrl, iMemOffset ) ABCC_SYS_ParallelRead16( hCtrl, iMemOffset )

/*------------------------------------------------------------------------------
** Writes a word to the ABCC memory.
** In case of a memory mapped system this function does not need not be
** implemented. See ABCC_SYS_WRITE16 macro description below.
**------------------------------------------------------------------------------
** Arguments:
**    iMemOffset - Offset from ABCC base address.
**                 8 bit char platforms  : iMemOffset in octets
**                 16 bit char platforms : iMemOffset in 16 bit words
**    iData      - Data to be written to ABCC
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
#define ABCC_DrvParallelWrite( hCtrl, iMemOffset, pxData, iLength )                   \
        ABCC_SYS_ParallelWrite( hCtrl, iMemOffset, pxData, iLength )

/*------------------------------------------------------------------------------
** The driver will use the ABCC_SYS_WRITE8 and ABCC_SYS_WRITE16 macros to
** access the ABCC registers. In case of a memory mapped system,
** MEMORY_MAPPED_ACCESS is defined an direct memory access will be done
** using ABCC_CFG_PARALLEL_BASE_ADR to calculate the absolute address. In case
** of a non memory mapped system the ABCC_SYS_ParallelWrite8/16 will be called.
**------------------------------------------------------------------------------
** Arguments:
**    iMemOffset - Offset from ABCC base address.
**                 8 bit char platforms  : iMemOffset in octets
**                 16 bit char platforms : iMemOffset in 16 bit words
**    pbData     - Data to be written to ABCC
**
** Returns:
**    None
**
**------------------------------------------------------------------------------
*/
#define ABCC_DrvWrite8( hCtrl, iMemOffset, pbData )                                   \
        ABCC_SYS_ParallelWrite8( hCtrl, iMemOffset, pbData )
#define ABCC_DrvWrite16( hCtrl, iMemOffset, pbData )                                   \
        ABCC_SYS_ParallelWrite16( hCtrl, iMemOffset, pbData )

/*------------------------------------------------------------------------------
** Get the address to the received read process data.
** For a non memory mapped system the system adaption layer need to provide a
** buffer where the read process data can be stored.
** No implementation is needed for a memory mapped system since the macro
** provides the information.
**------------------------------------------------------------------------------
** Argument:
**    None
**
** Returns:
**    Address to RdPdBuffer.
**
**------------------------------------------------------------------------------
*/
#define ABCC_DrvParallelGetRdPdBuffer( hCtrl )                                        \
        ABCC_SYS_ParallelGetRdPdBuffer( hCtrl )

/*------------------------------------------------------------------------------
** This macro will be used by the driver to read process data. In case of a
** memory mapped system the macro will be empty since the buffer already
** contains the received data. ( See  ABCC_SYS_ParallelGetRdPdBuffer ).
** For a non memory mapped system ABCC_SYS_ParallelRead will be called with the
** buffer received from ABCC_SYS_ParallelGetRdPdBuffer()
** The buffer will be valid until the next read process data read is done.
** For ABCC30 ( ABCC_CFG_DRV_PARALLEL_30 defined ) the legacy address offset is
** used.
**------------------------------------------------------------------------------
** Arguments:
**          buffer: Pointer where to store received process data
**          size: Size of read process data in octets.
** Returns:
**------------------------------------------------------------------------------
*/
#define ABCC_DrvReadRdPd( hCtrl, buffer, size )                                       \
        ABCC_SYS_ParallelRead( hCtrl, ABP_RDPD_ADR_OFFSET, buffer, size )

/*------------------------------------------------------------------------------
** Get the address to store the write process data.
** For a non memory mapped system the system adaption layer need to provide a
** buffer where the write process data can be stored.
** No implementation is needed for a memory mapped system since the macro
** provides the information.
**------------------------------------------------------------------------------
** Argument:
**    None
**
** Returns:
**    Address to WrPdBuffer
**
**------------------------------------------------------------------------------
*/
#define ABCC_DrvParallelGetWrPdBuffer( hCtrl )   ABCC_SYS_ParallelGetWrPdBuffer( hCtrl )

/*------------------------------------------------------------------------------
** This macro will be used by the driver to write process data. In case of a
** memory mapped system the macro will be empty since the buffer already
** contains the written data. ( See  ABCC_SYS_ParallelGetWrPdBuffer ).
** For a non memory mapped system ABCC_SYS_ParallelWrite will be called with the
** buffer received from ABCC_SYS_ParallelGetWrPdBuffer().
** When function has returned new process data can be written to the buffer.
** For ABCC30 ( ABCC_CFG_DRV_PARALLEL_30 defined ) the legacy address offset is
** used.
**------------------------------------------------------------------------------
** Arguments:
**    buffer - Pointer to write process data
**    size   - Size of write process data in octets.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
#define ABCC_DrvWriteWrPd( hCtrl, pxBuffer, iSize )                                   \
        ABCC_SYS_ParallelWrite( hCtrl, ABP_WRPD_ADR_OFFSET, pxBuffer, iSize )

/*------------------------------------------------------------------------------
** Initializes the driver to default values.
** Must be called before the driver is used.
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ( *pnABCC_DrvInit )( IDL_CTRL_HDL hCtrl, UINT8 bOpmode );

/*------------------------------------------------------------------------------
** Calls in the interrupt context to acknowledge received interrupts.The ISR
** routine will clear all pending interrupts.
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    Acknowledged interrupts.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT16 ( *pnABCC_DrvISR )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Drives the internal send process.
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ( *pnABCC_DrvRunDriverTx )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Drives the internal receive process.
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    Pointer to successfully sent write message.
**------------------------------------------------------------------------------
*/
EXTFUNC ABP_MsgType* ( *pnABCC_DrvRunDriverRx )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Writes a message to the driver.
**------------------------------------------------------------------------------
** Arguments:
**    psWriteMsg   - Pointer to message.
**
** Returns:
**    True         - Message was successfully written and can be deallocated
**                   immediately.
**    False        - Message was not yet written and cannot be deallocated.
**                   The psWriteMsg pointer is owned by the driver until the
**                   message is written and the pointer is returned in the
**                   driver execution response.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ( *pnABCC_DrvWriteMessage) ( IDL_CTRL_HDL hCtrl, ABP_MsgType* psWriteMsg );

/*------------------------------------------------------------------------------
** Writes current process data.
** The data is copied before returning from the method.
**------------------------------------------------------------------------------
** Arguments:
**    pbProcessData - Pointer to process data to be sent.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void ( *pnABCC_DrvWriteProcessData )( IDL_CTRL_HDL hCtrl, void* pbProcessData );

/*------------------------------------------------------------------------------
** Checks if the driver is in the correct state for writing process data to the
** anybus
**------------------------------------------------------------------------------
** Arguments:
**       None
**
** Returns:
**       True        - Driver is in correct state to send WrPd
**       False:      - Driver is not in correct state to send Wrpd
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ( *pnABCC_DrvISReadyForWrPd )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Checks if the driver is ready to send a new write message.
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    True        - Driver is ready to send a new write message.
**    False       - Driver is not ready to send a new write message.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ( *pnABCC_DrvISReadyForWriteMessage )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** The host application checks if the Anybus is ready to receive a new command
** message.
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    True         - OK to send new command.
**    False        - NOK to send new command.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ( *pnABCC_DrvISReadyForCmd )( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Sets the number of simultaneous commands that is supported by the
** application.
**------------------------------------------------------------------------------
** Arguments:
**    bNbrOfCmds  - Number of commands that the application is ready to receive.
**
** Returns:
**       None
**------------------------------------------------------------------------------
*/
EXTFUNC void ( *pnABCC_DrvSetNbrOfCmds )( IDL_CTRL_HDL hCtrl, UINT8 bNbrOfCmds );

/*------------------------------------------------------------------------------
**  Sets the current application status.
**  Note! This information is not supported by all protocols.
**------------------------------------------------------------------------------
** Arguments:
**    eAppStatus   - Current application status.
**
** Returns:
**    None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ( *pnABCC_DrvSetAppStatus )( IDL_CTRL_HDL hCtrl, ABP_AppStatusType eAppStatus );

/*------------------------------------------------------------------------------
** Sets the current process data size.
**------------------------------------------------------------------------------
** Arguments:
**    iReadPdSize  - Size of read process data (bytes)
**    iWritePdSize - Size of write process data (bytes)
**
** Returns:
**       None.
**------------------------------------------------------------------------------
*/
EXTFUNC void ( *pnABCC_DrvSetPdSize )( IDL_CTRL_HDL hCtrl, const UINT16 iReadPdSize,
                                       const UINT16 iWritePdSize );

/*------------------------------------------------------------------------------
** Sets Interrupt mask according to h_aci.h.
**------------------------------------------------------------------------------
** Arguments:
**    iIntMask     - Interrupt mask set according to h_aci.h.
**
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void ( *pnABCC_DrvSetIntMask )( IDL_CTRL_HDL hCtrl, const UINT16 iIntMask );

/*------------------------------------------------------------------------------
** Get WrpdBuffer for the user to update.
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    Pointer to WrPd buffer.
**------------------------------------------------------------------------------
*/
EXTFUNC void* ( *pnABCC_DrvGetWrPdBuffer )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Read module capabillity
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    Module capability.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT16 ( *pnABCC_DrvGetModCap )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Read module capability
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    Module capability.
**------------------------------------------------------------------------------
*/
EXTFUNC UINT16 ( *pnABCC_DrvGetLedStatus )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Gets the Anybus interrupt status. The pnABCC_DrvISR() function will clear all
** pending interrupts. This function must be called before pnABCC_DrvISR() or it
** will always return 0.
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    The Anybus interrupt status.
**------------------------------------------------------------------------------
*/
/*EXTFUNC UINT16 ( *pnABCC_DrvGetIntStatus )( IDL_CTRL_HDL hCtrl );*/

/*------------------------------------------------------------------------------
** Gets the Anybus state.
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    The Anybus state
**------------------------------------------------------------------------------
*/
EXTFUNC UINT8 ( *pnABCC_DrvGetAnybusState )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Reads the read process data.
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    A pointer to the read process data; or NULL if no process data to read
**    was available.
**------------------------------------------------------------------------------
*/
EXTFUNC void* ( *pnABCC_DrvReadProcessData )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Reads the read message.
**------------------------------------------------------------------------------
** Arguments:
**    None.
**
** Returns:
**    A pointer to the read message; or NULL if no message is available.
**    The pointer, if not NULL, will point to the buffer previously set by
**    calling pnABCC_DrvSetMsgReceiverBuffer().
**------------------------------------------------------------------------------
*/
EXTFUNC ABP_MsgType* ( *pnABCC_DrvReadMessage )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
** Returns supervision bit in status register.
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    TRUE: The device is supervised by another network device.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ( *pnABCC_DrvIsSupervised )( IDL_CTRL_HDL hCtrl );

/*------------------------------------------------------------------------------
**  Returns anybus status register.
**------------------------------------------------------------------------------
** Arguments:
**    None
**
** Returns:
**    Anybus status register
**------------------------------------------------------------------------------
*/
EXTFUNC UINT8 ( *pnABCC_DrvGetAnbStatus )( IDL_CTRL_HDL hCtrl );

#endif  /* inclusion lock */

/*******************************************************************************
** End of drv_if.h
********************************************************************************
*/
