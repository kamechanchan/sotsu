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
** file_description
** Definition of ABCC types
********************************************************************************
********************************************************************************
** Services:
.
********************************************************************************
********************************************************************************
*/

#ifndef ABCC_TD_H
#define ABCC_TD_H

#include <stddef.h> // for offsetof
#include <stdlib.h> // for malloc / free

/*******************************************************************************
**
** Constants
**
********************************************************************************
*/

/*---------------------------------------------------------------------------
**
** BOOL8
**
** INT8
** INT16
** INT32
**
** UINT8
** UINT16
** UINT32
**
** FLOAT32
**
** The standard boolean data type, 8-bit.
** The standard signed 8, 16, and 32 bit data types (respectively).
** The standard unsigned 8, 16, and 32 bit data types (respectively).
**---------------------------------------------------------------------------
*/
typedef __int32_t       BOOL;
typedef __uint8_t       BOOL8;
typedef __uint8_t       UINT8;
typedef __int8_t        INT8;
typedef __uint16_t      UINT16;
typedef __int16_t       INT16;
typedef __uint32_t      UINT32;
typedef __int32_t       INT32;
typedef __uint64_t      UINT64;
typedef __int64_t       INT64;
typedef float           FLOAT32;





/*------------------------------------------------------------------------------
** LeINT16
** LeINT32
** LeUINT16
** LeUINT32
**
** Little endian data types for words and longwords.
**------------------------------------------------------------------------------
*/
typedef INT16     LeINT16;
typedef INT32     LeINT32;
typedef UINT16    LeUINT16;
typedef UINT32    LeUINT32;
typedef UINT64    LeUINT64;


/*------------------------------------------------------------------------------
** BeINT16
** BeINT32
** BeUINT16
** BeUINT32
**
** Big endian data types for words and longwords.
**------------------------------------------------------------------------------
*/
typedef INT16     BeINT16;
typedef INT32     BeINT32;
typedef UINT16    BeUINT16;
typedef UINT32    BeUINT32;
typedef UINT64    BeUINT64;

/*---------------------------------------------------------------------------
**
** FALSE
** TRUE
**
** These are the symbolic constants for true and false used in boolean
** data type comparisons.
**
**---------------------------------------------------------------------------
*/
#ifndef FALSE
   #define FALSE     0
#endif

#ifndef TRUE
   #define TRUE      ( !FALSE )
#endif



/*---------------------------------------------------------------------------
**
** NULL
**
** Default value for invalid pointers.
**
**---------------------------------------------------------------------------
*/
#ifndef NULL
#define NULL 0
#endif

/*---------------------------------------------------------------------------
**
** OSIDL_Memcpy
**
** OS independent memcpy.
**
**---------------------------------------------------------------------------
*/
#ifndef OSIDL_Memcpy
#define OSIDL_Memcpy memcpy
#endif

/*---------------------------------------------------------------------------
**
** OSIDL_Printf
**
** OS independent printf.
**
**---------------------------------------------------------------------------
*/
#ifndef OSIDL_Printf
#define OSIDL_Printf printf
#endif

#ifdef __cplusplus
   #define CPLUSPLUS
#endif

#ifdef CPLUSPLUS
   #define EXTFUNC extern "C"
#else
   #define EXTFUNC extern
#endif


/** DLL EXPORT definition */
#ifdef __DLL_EXPORT__
  #define IDL_DLLEXPORT __attribute__ ((visibility("default")))
#else
  #define IDL_DLLEXPORT
#endif

/** Calling convention */
#if defined(__i386__)
  #define IDL_APICALL __attribute__ ((stdcall))
#elif defined(__arm__)
  #define IDL_APICALL  
#elif defined(__aarch64__)
  #define IDL_APICALL  
#elif defined(__amd64__)
  #define IDL_APICALL
#endif

#endif  /* inclusion lock */

