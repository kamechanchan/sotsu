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
** This program is the property of HMS Industrial Networks AB.                **
** It may not be reproduced, distributed, or used without permission          **
** of an authorized company official.                                         **
********************************************************************************
********************************************************************************
** Driver configuration of the ABCC driver. The configuration parameters are
** documented in the driver's public interface abcc_cfg.h.
********************************************************************************
********************************************************************************
** Services:
********************************************************************************
********************************************************************************
*/

#ifndef ABCC_DRV_CFG_H_
#define ABCC_DRV_CFG_H_

#include "abcc_td.h"
#include "abp.h"

/*******************************************************************************
** Constants
********************************************************************************
*/

/*------------------------------------------------------------------------------
** All operating modes are supported
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_DRV_PARALLEL                      ( TRUE )

/*------------------------------------------------------------------------------
** The software can both get the operating mode from external resource and set
** the selected operating mode to the ABCC host connector.
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_OP_MODE_GETTABLE                  ( TRUE )
#define ABCC_CFG_OP_MODE_SETTABLE                  ( TRUE )

/*------------------------------------------------------------------------------
** Module id is read from host connector. No softconfiguration is done.
**------------------------------------------------------------------------------
*/
/* #define ABCC_CFG_ABCC_MODULE_ID        ABP_MODULE_ID_ACTIVE_ABCC40 */

/*------------------------------------------------------------------------------
** Module detection configuration
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_MOD_DETECT_PINS_CONN              ( TRUE )

/*------------------------------------------------------------------------------
** Configuration of message handling
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_MAX_NUM_APPL_CMDS                 ( 32 )
#define ABCC_CFG_MAX_NUM_ABCC_CMDS                 ( 32 )
#define ABCC_CFG_MAX_MSG_SIZE                      ( 1524 )
#define ABCC_CFG_MAX_PROCESS_DATA_SIZE             ( 4096 )

/*------------------------------------------------------------------------------
** Interrupt configuration
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_INT_ENABLED                       ( TRUE )
#if( ABCC_CFG_INT_ENABLED )
   #define ABCC_CFG_INT_ENABLE_MASK_PAR            ( ABP_INTMASK_RDPDIEN | ABP_INTMASK_STATUSIEN | ABP_INTMASK_RDMSGIEN | ABP_INTMASK_WRMSGIEN | ABP_INTMASK_ANBRIEN  )
   #define ABCC_CFG_INT_ENABLE_MASK_SPI            ( 0 )
   #define ABCC_CFG_HANDLE_INT_IN_ISR_MASK         ( ABP_INTMASK_RDPDIEN )
#endif

/*------------------------------------------------------------------------------
** Watchdog configuration
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_WD_TIMEOUT_MS                     ( 1000 )

/*------------------------------------------------------------------------------
** Remap support configuration
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_REMAP_SUPPORT_ENABLED             ( TRUE )

/*------------------------------------------------------------------------------
** The Application data object ADI support configuration
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_STRUCT_DATA_TYPE                  ( TRUE )
#define ABCC_CFG_ADI_GET_SET_CALLBACK              ( TRUE )
#define ABCC_CFG_64BIT_ADI_SUPPORT                 ( TRUE )

/*------------------------------------------------------------------------------
** Debug event print functions
**------------------------------------------------------------------------------
*/
#define ABCC_CFG_ERR_REPORTING_ENABLED             ( TRUE )
#define ABCC_CFG_DEBUG_EVENT_ENABLED               ( FALSE )
#define ABCC_CFG_DEBUG_ERR_ENABLED                 ( FALSE )
#define ABCC_CFG_DEBUG_MESSAGING                   ( FALSE )

#endif  /* inclusion lock */
