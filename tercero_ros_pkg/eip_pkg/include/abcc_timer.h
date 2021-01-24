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
** This program is the property of HMS Industrial Networks AB.                **
** It may not be reproduced, distributed, or used without permission          **
** of an authorized company official.                                         **
********************************************************************************
********************************************************************************
** Implements abcc timer service.
********************************************************************************
********************************************************************************
** Services:
** ABCC_TimerInit()         - Init internal
** ABCC_TimerCreate()       - Start Timer
** ABCC_TimerStart()        - Start Timer
** ABCC_TimerStop()         - Stop timer
** ABCC_TimerTick()         - Tick
** ABCC_TimerDisable()      - Disable tick action
**
********************************************************************************
********************************************************************************
*/

#ifndef ABCC_TIMER_H_
#define ABCC_TIMER_H_

#include "abcc_drv_cfg.h"
#include "abcc_td.h"
#include "abcc_sys_adapt.h"

/*******************************************************************************
** Constants
********************************************************************************
*/

#define ABCC_TIMER_NO_HANDLE ( 0xff )

/*******************************************************************************
** Typedefs
********************************************************************************
*/

/*
 ** Timeout callback function type.
 */
typedef void (*ABCC_TimerTimeoutCallbackType)( IDL_CTRL_HDL hCtrl );


/*
 ** Type for identifying timer
 */
typedef UINT8 ABCC_TimerHandle;


/*******************************************************************************
** Public Globals
********************************************************************************
*/

/*******************************************************************************
** Public Services
********************************************************************************
*/


/*------------------------------------------------------------------------------
** void ABCC_TimerInit( IDL_CTRL_HDL hCtrl );
** Need to called before the timer is used
**------------------------------------------------------------------------------
** Arguments:
**    None
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_TimerInit( IDL_CTRL_HDL hCtrl );


/*------------------------------------------------------------------------------
** Allocates a timer resource and returns a handle.
**------------------------------------------------------------------------------
** Arguments:
**    pnHandleTimeout: Function to call if timeout.
** Returns:
**    ABCC_TimerHandle ( Used as identifier when using timer functions. )
**                       TIMER_NO_HANDLE is returned if no timer was available
**------------------------------------------------------------------------------
*/
EXTFUNC ABCC_TimerHandle ABCC_TimerCreate( IDL_CTRL_HDL hCtrl, ABCC_TimerTimeoutCallbackType pnHandleTimeout );


/*------------------------------------------------------------------------------
** Start timer.
** When the timeout is reached, the registered callback function is called.
** Note!! This function is dependent on that ABCC_TimerTick() is called
** on regular basis.
**------------------------------------------------------------------------------
** Arguments:
**    xHandle:     Identifier of timer to be started.
**    lTimeoutMs:  Timeout in ms.
**
** Returns:
**    TRUE if the timer had expired before re-start.
**    FALSE Timer had not expired before re-start.
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ABCC_TimerStart( IDL_CTRL_HDL hCtrl, ABCC_TimerHandle xHandle,
                              UINT32 lTimeoutMs );


/*------------------------------------------------------------------------------
** Stop timer.
**------------------------------------------------------------------------------
** Arguments:
**    xHandle: Identifier of timer to be stopped.
** Returns:
**    FALSE If timeout is already reached
**    TRUE: Timer stopped OK
**------------------------------------------------------------------------------
*/
EXTFUNC BOOL ABCC_TimerStop( IDL_CTRL_HDL hCtrl, ABCC_TimerHandle xHandle );



/*------------------------------------------------------------------------------
** ABCC_TimerTick(). Provides delta time since last timer tick call.
** Typically called from timer interrupt.
**------------------------------------------------------------------------------
** Arguments:
**    iDeltaTimeMs: Time in ms since last timerTick call
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_TimerTick( IDL_CTRL_HDL hCtrl, const INT16 iDeltaTimeMs );


/*------------------------------------------------------------------------------
** ABCC_TimerDisable(). Disable tick action
**------------------------------------------------------------------------------
** Arguments:
**    None
** Returns:
**    None
**------------------------------------------------------------------------------
*/
EXTFUNC void ABCC_TimerDisable( IDL_CTRL_HDL hCtrl );


#endif  /* inclusion lock */
