///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Definition of error codes used by the IDL.

  @file IDL_error.h
*/

#ifndef __IDL_ERROR_H__
#define __IDL_ERROR_H__

//////////////////////////////////////////////////////////////////////////
// include files
#include <OsIdl.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros

/** Maximal length of an error string */
#define TEST_MAX_ERRSTR       256

/** Facility code for the IDL error codes */
#define FACILITY_IXXAT_IDL    0x00FE0000

/** Severity code success */
#define SEV_SUCCESS           0x00000000
/** Severity code info */
#define SEV_INFO              0x40000000
/** Severity code warning */
#define SEV_WARN              0x80000000
/** Severity code error */
#define SEV_ERROR             0xC0000000

/** Customer flag */
#define CUSTOMER_FLAG         0x20000000
/** Reserved flag */
#define RESERVED_FLAG         0x10000000

/** IXXAT defined IDL info code */
#define SEV_IDL_INFO          (SEV_INFO  | CUSTOMER_FLAG | FACILITY_IXXAT_IDL)
/** IXXAT defined IDL warning code */
#define SEV_IDL_WARN          (SEV_WARN  | CUSTOMER_FLAG | FACILITY_IXXAT_IDL)
/** IXXAT defined IDL error code */
#define SEV_IDL_ERROR         (SEV_ERROR | CUSTOMER_FLAG | FACILITY_IXXAT_IDL)

/** Mask to determine the facility code */
#define FACILITY_MASK         0x0FFF0000
/** Mask to determine the status resp. error code */
#define STATUS_MASK           0x0000FFFF

/** Data type for IDL_RESULT */
#define IDL_RESULT              HRESULT

//////////////////////////////////////////////////////////////////////////
// data types

/**
  List of IDL error codes.
*/
typedef enum
{
  IDL_OK = (0),                                  ///< Operation finished successfully.
  IDL_ERR_FIRST = (SEV_IDL_ERROR | 0x0000),
  IDL_ERR_INVALIDARG,                            ///< One or more arguments are invalid.
  IDL_ERR_FAILED,                                ///< Failed with unknown error.
  IDL_ERR_NOTIMPL,                               ///< Function is not implemented.
  IDL_ERR_NOT_SUPPORTED,                         ///< Type or parameter is not supported.
  IDL_ERR_ACCESS_DENIED,                         ///< Access denied.
  IDL_ERR_RESOURCE_BUSY,                         ///< The device or resource is busy.
  IDL_ERROR_OUTOFMEMORY,                         ///< Ran out of memory
  IDL_ERR_INVALID_HANDLE,                        ///< The handle is invalid.
  IDL_ERR_INVALID_POINTER,                       ///< The pointer is invalid.
  IDL_ERR_INVALID_DATATYPE,                      ///< The data type is invalid.
  IDL_ERR_INVALID_DATA,                          ///< The data is invalid.
  IDL_ERR_TIMEOUT,                               ///< This operation returned because the timeout period expired.
  IDL_ERR_INSUFFICIENT_RESOURCES,                ///< The available resources are insufficient to perform this request.
  IDL_ERR_RESOURCE_NOT_FOUND,                    ///< The device or resource could not be found.
  IDL_ERR_REQUEST_FAILED,                        ///< The request failed.
  IDL_ERR_INVALID_RESPONSE,                      ///< The received response is invalid.
  IDL_ERR_UNKNOWN_RESPONSE,                      ///< The received response is unknown.
  IDL_ERR_BAD_COMMAND,                           ///< The device does not recognize the command.
  IDL_ERR_NO_MORE_DATA,                          ///< No more data is available.
  IDL_ERR_NO_MORE_SPACE_LEFT,                    ///< No more space left to perform this request.
  IDL_ERR_UNSUPPORTED_VERSION,                   ///< The available or requested version is unsupported.
  //IDL specific
  IDL_ERR_INVALID_CTRLHANDLE,                    ///< The specified IDL board or controller handle is invalid.
  IDL_ERR_WRONG_STATE,                           ///< IDL is in wrong state to perform this request.
  IDL_ERR_CREATE_FAILED,                         ///< IDL interface could not be instantiated correctly.
  IDL_ERR_IRQTEST_FAILED,                        ///< IRQ handler could not be installed correctly.
  IDL_ERR_FWDOWNLOAD,                            ///< An error occurred while downloading or verifying firmware
  IDL_ERR_DPRAM_LOCK_VIOLATION,                  ///< A lock violation occurred while communication via DPRAM interface.
  IDL_ERR_DRPAM_IO_ERROR,                        ///< An I/O error occurred while reading from or writing to DRPAM interface
  IDL_ERR_UNSUPPORTED_FWVERSION,                 ///< The boot manager or firmware version is unsupported.
  IDL_ERR_USB_INCOMPLETE_DESCRIPTOR,             ///< The USB device descriptor is invalid.
  IDL_ERR_FLASH_WRITE,                           ///< The system cannot write to the specified flash memory.
  IDL_ERR_FLASH_READ,                            ///< The system cannot read from the specified flash memory.
  IDL_ERR_FLASH_ERASE,                           ///< The requested delete operation could not be performed on the flash memory.
  IDL_ERR_FLASH_VERIFY,                          ///< The requested verify operation failed.
  IDL_ERROR_SUCCESS_REBOOT_REQUIRED,             ///< The requested operation finishes successfully. Changes will not be effective until the system is rebooted.
  IDL_ERR_LAST
} e_IDLERROR;

#endif
