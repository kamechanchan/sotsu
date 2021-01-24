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
** File Description:
** Definition of types, structs and unions for thread initialization.
********************************************************************************
********************************************************************************
** Services:
********************************************************************************
********************************************************************************
*/
#ifndef __IDL_THREAD_H__
#define __IDL_THREAD_H__

#include "abcc_td.h"

/*******************************************************************************
** Defines
********************************************************************************
*/

#define IDL_CPU_ALL    0xFFFFFFFF
#define IDL_MAX_CPU    (sizeof(DWORD) * 8)

#define IDL_CPU(x)     (1 << x)

/*******************************************************************************
** Typedefs
********************************************************************************
*/

/**
  Scheduler types

  @ingroup SchedulerTypes
*/
typedef enum
{
  IDL_SCHED_NOCHANGE    = 0x00,       ///< keep actual scheduler
  IDL_SCHED_FIFO        = 0x01,       ///< fifo scheduling
  IDL_SCHED_RR          = 0x02,       ///< round robin scheduling
  IDL_SCHED_OTHER       = 0x03        ///< other scheduling
} e_SCHEDPOLICY;

/*------------------------------------------------------------------------------
** IDL_SCHEDULER_SETTINGS.
**------------------------------------------------------------------------------
** 1. dwSchedulerPolicy       - Scheduler policy class @see e_SCHEDPOLICY
** 2. dwSchedulerPriority     - Scheduler thread priority
** 3. dwCpuAffinity           - CPU affinity of the thread @see e_CPUAFFINITY
*/
typedef struct _IDL_SCHEDULER_SETTINGS
{
    UINT32 dwSchedulerPolicy;
    UINT32 dwSchedulerPriority;
    UINT32 dwCpuAffinity;
} IDL_SCHEDULER_SETTINGS,*PIDL_SCHEDULER_SETTINGS;

#endif // __IDL_THREAD_H__
