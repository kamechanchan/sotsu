///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  OS dependent function hiding

  @file OsIdl.h
*/

#ifndef __OSIDL_H__
#define __OSIDL_H__

//////////////////////////////////////////////////////////////////////////
// include files
#include <stdio.h>
#include <memory.h> // for memset (softer than including string.h)
#include <errno.h>  // for ENOMEM
#include <poll.h>   // for poll
#include <stddef.h> // for offsetof
#include <stdlib.h> // for malloc / free

#include <IDL_error.h>  // for error definition
#include <IDL_thread.h>

#include <abcc_td.h>


//////////////////////////////////////////////////////////////////////////
// constants and macros

#define OS_uSleep                   OSIDL_uSleep
#define OS_Sleep                    OSIDL_Sleep
#define OS_InterlockedExchange      OSIDL_InterlockedExchange
#define OS_InterlockedExchangeAdd   OSIDL_InterlockedExchangeAdd
#define OS_InterlockedOr            OSIDL_InterlockedOr
#define OS_InterlockedIncrement     OSIDL_InterlockedIncrement
#define OS_InterlockedDecrement     OSIDL_InterlockedDecrement
#define OS_GetTimeInUs              OSIDL_GetTimeInUs
#define OS_GetTimeInMs              OSIDL_GetTimeInMs
#define OS_CreateEvent              OSIDL_CreateEvent
#define OS_SetEvent                 OSIDL_SetEvent
#define OS_WaitForSingleObject      OSIDL_WaitForSingleObject
#define OS_CreateMutex              OSIDL_CreateMutex
#define OS_CreateRecursiveMutex     OSIDL_CreateRecursiveMutex
#define OS_LockMutex                OSIDL_LockMutex
#define OS_UnlockMutex              OSIDL_UnlockMutex
#define OS_CloseHandle              OSIDL_CloseHandle
#define OS_GetLastError             OSIDL_GetLastError
#define OS_CreateThread             OSIDL_CreateThread
#define OS_JoinThread               OSIDL_JoinThread
#define OS_CancelThread             OSIDL_CancelThread

typedef void          VOID;         ///< Define used datatype to easier porting the IDL to another OS
typedef char          CHAR;         ///< Define used datatype to easier porting the IDL to another OS
typedef __uint8_t     BYTE;         ///< Define used datatype to easier porting the IDL to another OS
typedef __uint16_t    WORD;         ///< Define used datatype to easier porting the IDL to another OS
typedef __uint32_t    DWORD;        ///< Define used datatype to easier porting the IDL to another OS
typedef __uint64_t    QWORD;        ///< Define used datatype to easier porting the IDL to another OS
typedef long          LONG;         ///< Define used datatype to easier porting the IDL to another OS
typedef unsigned long ULONG;        ///< Define used datatype to easier porting the IDL to another OS
typedef __uint32_t    HRESULT;      ///< Define used datatype to easier porting the IDL to another OS
typedef float         FLOAT;        ///< Define used datatype to easier porting the IDL to another OS

typedef VOID          *PVOID;       ///< Define used datatype to easier porting the IDL to another OS
typedef CHAR          *PCHAR;       ///< Define used datatype to easier porting the IDL to another OS
typedef BYTE          *PBYTE;       ///< Define used datatype to easier porting the IDL to another OS
typedef BOOL          *PBOOL;       ///< Define used datatype to easier porting the IDL to another OS
typedef WORD          *PWORD;       ///< Define used datatype to easier porting the IDL to another OS
typedef DWORD         *PDWORD;      ///< Define used datatype to easier porting the IDL to another OS
typedef LONG          *PLONG;       ///< Define used datatype to easier porting the IDL to another OS
typedef FLOAT         *PFLOAT;      ///< Define used datatype to easier porting the IDL to another OS
typedef void          *IDL_HANDLE;  ///< Define used datatype to easier porting the IDL to another OS
typedef IDL_HANDLE    *PIDL_HANDLE; ///< Define used datatype to easier porting the IDL to another OS


#define LOBYTE(wVal)  ((BYTE) wVal)           ///< Macro for accessing low byte @ingroup OsIdl
#define HIBYTE(wVal)  ((BYTE) ( wVal >> 8))   ///< Macro for accessing high byte @ingroup OsIdl
#define LOWORD(dwVal) ((WORD) dwVal)          ///< Macro for accessing low word @ingroup OsIdl
#define HIWORD(dwVal) ((WORD) ( dwVal >> 16)) ///< Macro for accessing high word @ingroup OsIdl

#define OSIDL_WAIT_FOREVER  ((DWORD)0xFFFFFFFF)  ///< Blocking function call @ingroup OsIdl

// return values of OSIDL_WaitForSingleObject
#define OSIDL_WAIT_OBJECT_0 0UL                  ///< Wait succeeded @ingroup OsIdl
#define OS_WAIT_OBJECT_0    OSIDL_WAIT_OBJECT_0
#define OSIDL_WAIT_TIMEOUT  1UL                  ///< Wait timed out @ingroup OsIdl
#define OS_WAIT_TIMEOUT     OSIDL_WAIT_TIMEOUT
#define OSIDL_WAIT_FAILED   ((DWORD)0xFFFFFFFF)  ///< Wait failed @ingroup OsIdl
#define OS_WAIT_FAILED      OSIDL_WAIT_FAILED

// Definition of TRUE and FALSE
#ifndef FALSE
  #define FALSE 0 ///< FALSE @ingroup OsIdl
#endif
#ifndef TRUE
  #define TRUE 1  ///< TRUE @ingroup OsIdl
#endif

/** Macro to find max value */
#ifndef max
  #define max(a,b)   (((a) > (b)) ? (a) : (b))
#endif

/** Macro to find min value */
#ifndef min
  #define min(a,b)   (((a) < (b)) ? (a) : (b))
#endif

/** Macro to get element count of an array  @ingroup OsIdl */
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))

/** Macro to swap the bytes of a WORD (ab -> ba)  @ingroup OsIdl */
#define SWAP16(w)  ( (WORD)  ((LOBYTE(w) << 8) | HIBYTE (w)))

/** Macro to swap the bytes of a DWORD (abcd -> dcba)  @ingroup OsIdl */
#define SWAP32(dw) ( (DWORD) (((SWAP16 (LOWORD (dw))) << 16) | (SWAP16 (HIWORD (dw)))))

/** Macro for printf redirection  @ingroup OsIdl */
#define OSIDL_Printf printf
#define OS_Printf    OSIDL_Printf

/** Macro for fflush redirection  @ingroup OsIdl */
#define OSIDL_Fflush(stream) fflush(stream)
#define OS_Fflush(stream)    OSIDL_Fflush(stream)

/** Macro for printf redirection  @ingroup OsIdl */
#define OSIDL_Printf printf
#define OS_Printf    OSIDL_Printf

///////////////////////////////////////////////////////////////////////////////
/**
  This macro retrieve a reference to the host object.

  @param t
    Type of the host object
  @param m
    Member variable which represents the inner object

  @return
   Reference to the host object.

  @note
    Set -Wno-invalid-offsetof flag in compiler to omit warning messages
    generated by this macro
*/
#ifndef HOSTOBJECT
  #define HOSTOBJECT(t,m) ((t&) *((char*) this - offsetof(t,m)))
#endif


//////////////////////////////////////////////////////////////////////////
// exported functions

//*** C-API
#ifdef __cplusplus
extern "C"
{
#endif


///////////////////////////////////////////////////////////////////////////////
/**
  Suspends the current thread for the specified time.

  @param dwMicroseconds
    Number of microseconds [us] to suspend thread

  @ingroup OsIdl
*/
IDL_DLLEXPORT void  IDL_APICALL OSIDL_uSleep ( DWORD dwMicroseconds );


///////////////////////////////////////////////////////////////////////////////
/**
  Suspends the current thread for the specified time.

  @param dwMilliseconds
    Number of milliseconds [ms] to suspend thread

  @ingroup OsIdl
*/
IDL_DLLEXPORT void IDL_APICALL OSIDL_Sleep ( DWORD dwMilliseconds );


///////////////////////////////////////////////////////////////////////////////
/**
  Sets a LONG variable to the specified value as an atomic operation.

  @param plDest
    A pointer to the value to be exchanged. The function sets this variable
    to Value, and returns its prior value.
  @param lValue
    The value to be exchanged with the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsIdl
*/
IDL_DLLEXPORT LONG IDL_APICALL OSIDL_InterlockedExchange ( PLONG plDest,
                                                        LONG  lValue );


///////////////////////////////////////////////////////////////////////////////
/**
  Performs an atomic addition of two LONG values.

  @param plDest
    A pointer to the variable. The value of this variable will be replaced
    with the result of the operation
  @param lValue
    The value to be added to the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsIdl
*/
IDL_DLLEXPORT LONG IDL_APICALL OSIDL_InterlockedExchangeAdd( PLONG plDest,
                                                          LONG  lValue );


///////////////////////////////////////////////////////////////////////////////
/**
  Performs an atomic OR operation on the specified LONG values. The function 
  prevents more than one thread from using the same variable simultaneously.

  @param plDest
    A pointer to the first operand. This value will be ORed with lValue.
  @param lValue
    The value to be ORed to the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsIdl
*/
IDL_DLLEXPORT LONG IDL_APICALL OSIDL_InterlockedOr( PLONG plDest,
                                                 LONG  lValue );

                                                 
///////////////////////////////////////////////////////////////////////////////
/**
  Increments the value of the specified LONG variable as an atomic operation

  @param plDest
    A pointer to the variable to be incremented.

  @retval LONG
    The function returns the resulting incremented value.

  @ingroup OsIdl
*/
IDL_DLLEXPORT LONG IDL_APICALL OSIDL_InterlockedIncrement( PLONG plDest );


///////////////////////////////////////////////////////////////////////////////
/**
  Decrements the value of the specified LONG variable as an atomic operation

  @param plDest
    A pointer to the variable to be decremented.

  @retval LONG
    The function returns the resulting decremented value.

  @ingroup OsIdl
*/
IDL_DLLEXPORT LONG IDL_APICALL OSIDL_InterlockedDecrement( PLONG plDest );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current time in microseconds [us].

  @retval DWORD
    Current time in microseconds [us].

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_GetTimeInUs ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current time in milliseconds [ms].

  @retval DWORD
    Current time in milliseconds [ms].

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_GetTimeInMs ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Creates a new unnamed event. The event is initially not signaled and auto 
  reset.

  @retval IDL_HANDLE
    If the function succeeds, the return value is a handle to the event
    object. If the function fails, the return value is NULL.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT IDL_HANDLE IDL_APICALL OSIDL_CreateEvent ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Set the state of the specified event object to signaled

  @param hEvent
    Event handle to set

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_SetEvent ( IDL_HANDLE hEvent );


///////////////////////////////////////////////////////////////////////////////
/**
  Wait until either the specified object is in the signaled state, or until
  the time-out interval elapses.

  @param hEvent
    Handle to wait for get signaled
  @param dwTimeout
    Specifies the time-out interval, in milliseconds [ms]. If the interval
    elapses, the function returns, even if the object's state is non signaled.

  @retval DWORD
    @ref OSIDL_WAIT_OBJECT_0
      Success. The specified object�s state is signaled. @n
    @ref OSIDL_WAIT_TIMEOUT
      Failure. The time-out interval elapsed, and the object's state is
      non signaled. @n
    @ref OSIDL_WAIT_FAILED
      Failure. Waiting failed maybe due to an invalid handle. To get extended
      error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_WaitForSingleObject ( IDL_HANDLE hEvent,
                                                         DWORD  dwTimeout );


///////////////////////////////////////////////////////////////////////////////
/**
  Creates a new unnamed mutex which can be uses for mutual exclusion

  @retval IDL_HANDLE
    If the function succeeds, the return value is a handle to the event
    object. If the function fails, the return value is NULL.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT IDL_HANDLE IDL_APICALL OSIDL_CreateMutex ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Creates a new recursive mutex which can be uses for mutual exclusion

  @retval IDL_HANDLE
    If the function succeeds, the return value is a handle to the event
    object. If the function fails, the return value is NULL.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT IDL_HANDLE IDL_APICALL OSIDL_CreateRecursiveMutex ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Acquires a mutual lock

  @param hMutex
    Mutex to acquire

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSIDL_GetLastError.

  @note
    This mutex is not re-entrant. Trying to acquire the mutex from the
    same thread a second time will lead to a dead-lock situation!

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_LockMutex ( IDL_HANDLE hMutex );


///////////////////////////////////////////////////////////////////////////////
/**
  Release a mutual lock.

  @param hMutex
    Mutex to acquire

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_UnlockMutex ( IDL_HANDLE hMutex );

///////////////////////////////////////////////////////////////////////////////
/**
  Release a mutual lock.

  @param hMutex
    Mutex to acquire

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_IsLockedMutex ( IDL_HANDLE hMutex );

///////////////////////////////////////////////////////////////////////////////
/**
  Closes a handle and deletes the object.

  @param hHandle
    Handle to close

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_CloseHandle ( IDL_HANDLE hHandle );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current thread's last exception code.

  @retval DWORD
    The calling thread�s last-error code value.

  @ingroup OsIdl
*/
IDL_DLLEXPORT DWORD IDL_APICALL OSIDL_GetLastError ( void );

///////////////////////////////////////////////////////////////////////////////
/**
  Creates a new thread

  @param start_routine
    Callback function of the new thread.

  @param pArg
    Argument of the callback function.

  @param pSchedulersettings
    Passes the scheduler settings for the created thread. If this value is NULL
    the standard scheduler settings will be used.

  @retval IDL_HANDLE
    If the function succeeds, the return value is a handle to the event
    object. If the function fails, the return value is NULL.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT IDL_HANDLE IDL_APICALL OSIDL_CreateThread ( VOID *(*start_routine) (VOID *),
                                                          VOID *                  pArg,
                                                          IDL_SCHEDULER_SETTINGS* pSchedulersettings);

///////////////////////////////////////////////////////////////////////////////
/**
  Joins a thread

  @retval hHandle
    Handle to the thread which should be joined.

  @retval HRESULT
    If the function succeeds, the return value is IDL_OK.
    If the function fails, the return value is unequal to IDL_OK.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT HRESULT IDL_APICALL OSIDL_JoinThread ( IDL_HANDLE hHandle, VOID **thread_return );

///////////////////////////////////////////////////////////////////////////////
/**
  Cancels a thread

  @param hHandle
    Handle to the thread which should be joined.

  @retval HRESULT
    If the function succeeds, the return value is IDL_OK.
    If the function fails, the return value is unequal to IDL_OK.
    To get extended error information, call @ref OSIDL_GetLastError.

  @ingroup OsIdl
*/
IDL_DLLEXPORT HRESULT IDL_APICALL OSIDL_CancelThread ( IDL_HANDLE hHandle );

#ifdef __cplusplus
}
#endif // __cplusplus


#endif //__OSIDL_H__
