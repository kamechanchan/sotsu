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
** COPYRIGHT NOTIFICATION (c) 2015 HMS Industrial Networks AB                 **
**                                                                            **
** This program is the property of HMS Industrial Networks AB.                **
** It may not be reproduced, distributed, or used without permission          **
** of an authorized company official.                                         **
********************************************************************************
********************************************************************************
** Configuration parameters of the driver.
**
** The following definitions are used to control and configure the driver.
** These defines shall be specified by the driver in abcc_drv_cfg.h. Therefore
** it is recommended not to change these settings.
********************************************************************************
********************************************************************************
** Services:
**   None
********************************************************************************
********************************************************************************
*/

#ifndef ABCC_CFG_H_
#define ABCC_CFG_H_

/*******************************************************************************
** Constants
********************************************************************************
*/

/*------------------------------------------------------------------------------
** #define ABCC_SYS_BIG_ENDIAN
**
** Defined in abcc_td.h.
**
** Define if an big endian system is used as host. If not defined little endian
** is assumed.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_SYS_16_BIT_CHAR
**
** Defined in abcc_td.h.
**
** Define if a 16 bit char system is used as host. If not defined 8 bit char
** system is assumed.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_DRV_SPI         (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for SPI operating mode.
** Note! Support for multiple operating modes is possible.
** Depending on module id different operating modes can be activated (see
** ABCC_CFG_ABCC_OP_MODE_30 and ABCC_CFG_ABCC_OP_MODE_40 configuration further
** down). Each supported mode increases memory usage.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_DRV_PARALLEL    (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for parallel event operating mode for both 8
** and 16 bit mode.
** Note! Support for multiple operating modes is possible.
** Depending on module id different operating modes can be activated (see
** ABCC_CFG_ABCC_OP_MODE_30 and ABCC_CFG_ABCC_OP_MODE_40 configuration further
** down). Each supported mode increases memory usage.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_DRV_PARALLEL_30 (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for parallel 8-bit ping-pong operating mode.
** Note! Support for multiple operating modes is possible.
** Depending on module id different operating modes can be activated (see
** ABCC_CFG_ABCC_OP_MODE_30 and ABCC_CFG_ABCC_OP_MODE_40 configuration further
** down). Each supported mode increases memory usage.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_DRV_SERIAL      (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for serial ping/pong operating mode.
** Note! Support for multiple operating modes is possible.
** Depending on module id different operating modes can be activated (see
** ABCC_CFG_ABCC_OP_MODE_30 and ABCC_CFG_ABCC_OP_MODE_40 configuration further
** down). Each supported mode increases memory usage.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_OP_MODE_GETTABLE         (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver to retrieve the operating mode from external hardware.
** If TRUE the ABCC_SYS_GetOpmode() function must be implemented in the system
** adaption layer.
**
** If this is not TRUE ABCC_CFG_ABCC_OP_MODE_X described above must be defined.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_OP_MODE_SETTABLE         (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver to control the operating mode set to the ABCC host
** connector. Else it is assumed the operating mode signals of the host
** connector is fixed or controlled by external hardware.
** If TRUE the ABCC_SYS_SetOpmode() function must be implemented in the system
** adaption layer.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_ABCC_OP_MODE_30
** #define ABCC_CFG_ABCC_OP_MODE_40
**
** Defined in abcc_drv_cfg.h.
**
** These definitions shall be defined if only one operating mode per module type
** is used. It is only necessary to define the operating mode for the module
** types to be used. If defined it shall be set to any ABP_OP_MODE_X definition
** from abp.h.
** If an operating mode for each module type is set the ABCC_SYS_SetOpmode()
** has to be implemented.
** If none of these definitions are set the ABCC_SYS_GetOpmode() must be
** implemented to retrieve the operating mode from external hardware
** (e.g. DIP switch) and ABCC_SYS_SetOpmode() must be implemented to set the
** operating mode to the host connector.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_SPI_MSG_FRAG_LEN                   ( 32 )
**
** Defined in abcc_drv_cfg.h.
**
** Length of SPI message fragment in bytes per SPI transaction.
** If the message fragment length is shorter than the largest message to be
** transmitted the sending or receiving of a message may be fragmented and
** take several SPI transactions to be completed. Each SPI transaction will have
** a message field of this length regardless if a message is present or not.
** If messages are important the fragment length should be set to the largest
** message to avoid fragmentation. If IO data are important the message fragment
** length should be set to a smaller value to speed up the SPI transaction.
** For high message performance a fragment length up to 1524 octets is
** supported. The message header is 12 octets, so 16 or 32 octets will be enough
** to support small messages without fragmentation.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_MEMORY_MAPPED_ACCESS        (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for memory mapped ABCC interface. If memory
** direct access is chosen the user will have access directly to the ABCC
** process data memory i.e. no internal copy is required.
** If FALSE the following functions must be implemented in the system adaption
** layer:
**    ABCC_SYS_ParallelRead()
**    ABCC_SYS_ParallelRead8()
**    ABCC_SYS_ParallelRead16()
**    ABCC_SYS_ParallelWrite()
**    ABCC_SYS_ParallelWrite8()
**    ABCC_SYS_ParallelWrite16()
**    ABCC_SYS_ParallelGetRdPdBuffer()
**    ABCC_SYS_ParallelGetWrPdBuffer()
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_PARALLEL_BASE_ADR             ( 0x00000000 )
**
** Defined in abcc_drv_cfg.h.
**
** Define the base address of the ABCC if a memory mapped interface is used.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_ABCC_MODULE_ID
**
** Defined in abcc_drv_cfg.h.
**
** If the ABCC module detection pins on the host connector are not
** connected ABCC_CFG_ABCC_MODULE_ID must be defined to the correct
** ABCC module id that corresponds to the module ID of the used device.
** If defined it shall be set to any ABP_MODULE_ID_X definition from abp.h.
**-----------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_MOD_DETECT_PINS_CONN     (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Set to TRUE if the module detect pins on the ABCC host connector are
** in use. If TRUE the ABCC_SYS_ModuleDetect() function in the system adaption
** layer must be implemented.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_MAX_NUM_APPL_CMDS     ( 2 )
**
** Defined in abcc_drv_cfg.h.
**
** Number of commands that could be sent without receiving a response.
** At least 2 buffers are required by the driver.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_MAX_NUM_ABCC_CMDS     ( 2 )
**
** Defined in abcc_drv_cfg.h.
**
** Number of commands that could be received without sending a response.
** At least 2 buffers are required by the driver.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_MAX_MSG_SIZE                       ( 255 )
**
** Defined in abcc_drv_cfg.h.
**
** Size of largest message in bytes that will be used.
**
** Note! ABCC30 supports 255 byte messages and ABCC40 supports 1524 byte
** messages. ABCC_CFG_MAX_MSG_SIZE should be set to largest size that will be
** sent or received. If this size is not known it recommended to set the maximum
** supported size.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_MAX_PROCESS_DATA_SIZE              ( 512 )
**
** Defined in abcc_drv_cfg.h.
**
** Size of max process data in bytes that will be used in either direction.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_SYNC_ENABLE (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for sync.
** If TRUE the ABCC_CbfSyncIsr() must be implemented by the application.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_USE_ABCC_SYNC_SIGNAL (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support to enable and disable sync interrupt using
** the sync signal from the ABCC.
** If TRUE ABCC_SYS_SyncInterruptEnable() and ABCC_SYS_SyncInterruptDisable()
** must be implemented by the application and ABCC_CbfSyncIsr() must be called
** from the sync interrupt handler.
** If FALSE the ABCC interrupt sync event will be used as sync source and
** ABCC_CbfSyncIsr() will be called by the driver.
** The define is only valid if ABCC_CFG_SYNC_ENABLE is TRUE.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_POLL_ABCC_IRQ_PIN (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support to read the status of the interrupt pin.
** This function will enable the driver to use the interrupt pin to check if an
** event has occurred even if the interrupt is disabled. For example the ABCC
** power up event could be detected this way.
** If TRUE the user will be forced to implement the
** ABCC_SYS_IsAbccInterruptActive() function in the system adaption layer.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_INT_ENABLED     (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for ABCC interrupt (IRQ_N pin on the host
** connector). If TRUE the user will be forced to implement the following
** functions in the system adaption layer:
**
**    ABCC_SYS_AbccInterruptEnable()
**    ABCC_SYS_AbccInterruptDisable()
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_INT_ENABLE_MASK_PAR                 ( ABP_INTMASK_RDPDIEN )
**
** Defined in abcc_drv_cfg.h.
**
** Defines what ABCC interrupts shall be enabled in parallel operating mode.
** This is a bit mask built up by the ABP_INTMASK_X definitions in abp.h.
** If an event is not notified via the ABCC interrupt it must be polled by
** ABCC_RunDriver(). If not defined in abcc_drv_cfg.h the default mask is 0.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_INT_ENABLE_MASK_SPI              ( 0 )
**
** Defined in abcc_drv_cfg.h.
**
** Defines what ABCC interrupts shall be enabled in SPI operating mode.
** The mask is composed of ABP_INTMASK_X definitions in abp.h.
** If an event is not notified via the ABCC interrupt it must be polled using
** ABCC_RunDriver().
** If not defined in abcc_drv_cfg.h the default mask is 0.
**
** Note! There are currently no support in the driver to handle interrupt
** driven SPI based on ABCC events.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_HANDLE_INT_IN_ISR_MASK  ( ABP_INTMASK_RDPDIEN )
**
** Defined in abcc_drv_cfg.h.
**
** Defines what interrupt events from the ABCC that should be handled in
** interrupt context. The mask is composed of ABP_INTMASK_X bits.
** Events that are enabled in the interrupt enable mask (ABCC_CFG_INT_ENABLE_MASK_X)
** but not configured to be handled by the ISR will be translated to a bit field
** of ABCC_ISR_EVENT_X definitions and forwarded to the user via the
** ABCC_CbfEvent() callback.
** If not defined in abcc_drv_cfg.h the default value will be:
** Parallel 16/8: 0 (No events handled by the ISR)
** Other operating modes N/A
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_WD_TIMEOUT_MS                      ( 1000 )
**
** Defined in abcc_drv_cfg.h.
**
** Timeout for ABCC communication watchdog.
** Note! Currently the watchdog functionality is only supported by SPI-,
** serial- and paralell30 (ping/pong) operating modes.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_REMAP_SUPPORT_ENABLED    (BOOL - TRUE/FALSE)
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver and AD object support for the re-map command.
** If TRUE the ABCC_CbfRemapDone() needs to be implemented by the application.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_STRUCT_DATA_TYPE   (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver and AD object support for structured data data types.
** This define will affect the AD_AdiEntryType (abcc_ad_if.h) used for defining
** the user ADI:s.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_ADI_GET_SET_CALLBACK   (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for triggering of callback notifications each
** time an ADI is read or written. This define will affect the AD_AdiEntryType
** (abcc_ad_if.h) used for defining the user ADI:s.
** If an ADI is read by the network the callback is invoked before the action.
** If an ADI is written by the network the callback is invoked after the action.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_64BIT_ADI_SUPPORT
**
** Defined in abcc_drv_cfg.h.
**
** Define if 64 bit data type in the Application Data object shall be supported.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_ERR_REPORTING_ENABLED          (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable the error reporting callback function (ABCC_CbfDriverError())
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_DEBUG_EVENT_ENABLED         (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for print out of debug events within the
** driver. ABCC_PORT_DebugPrint() (abcc_sw_port.h) will be used to print debug
** information.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_DEBUG_ERR_ENABLED           (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable printout of debug information when ABCC_CbfDriverError() is
** called such as file name and line number.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_DEBUG_MESSAGING             (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable printout of received and sent messages. Related events such as
** buffer allocation and qeueing information is also printed.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_STARTUP_TIME_MS           ( 1500 )
**
** If the ABCC interrupt pin is connected this define will be used as a timeout
** while waiting for the ABCC to become ready for communication. If the
** interrupt pin is not available the define will serve as time to wait before
** starting to communicate with ABCC. If not defined default will be 1500 ms.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_SYNC_MEASUREMENT_IP      (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for measurement of input processing time (used
** for SYNC). This define is used during development by activating it and
** compiling special test versions of the product.
** When ABCC_CFG_SYNC_MEASUREMENT_IP is TRUE ABCC_SYS_GpioReset() is
** called at the WRPD interrupt. If running in SPI operating mode it is
** instead called when ABCC_SpiRunDriver() has finished sending data to the
** Anybus.
** When ABCC_CFG_SYNC_MEASUREMENT_IP is TRUE ABCC_GpioSet() needs to be
** called at the Input Capture Point.
**------------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------------
** #define ABCC_CFG_SYNC_MEASUREMENT_OP   (BOOL - TRUE/FALSE )
**
** Defined in abcc_drv_cfg.h.
**
** Enable/disable driver support for measurement of output processing time (used
** for SYNC). This define is used during development by activating it and
** compiling special test versions of the product.
** When ABCC_CFG_SYNC_MEASUREMENT_OP is TRUE ABCC_SYS_GpioSet() is called
** from the RDPDI interrupt.
** When ABCC_CFG_SYNC_MEASUREMENT_OP is TRUE ABCC_GpioReset() needs to be
** called at the Output Valid Point.
**------------------------------------------------------------------------------
*/

#endif  /* inclusion lock */
