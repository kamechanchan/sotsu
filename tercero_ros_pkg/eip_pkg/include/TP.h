//******************************************************************************
// COPYRIGHT NOTIFICATION (c) 2003 HMS Industrial Networks AB
//
// This program is the property of HMS Industrial Networks AB.
// It may not be reproduced, distributed, or used without permission
// of an authorised company official.
//******************************************************************************

#ifndef TP_H
#define TP_H

#include <stddef.h> // for offsetof
#include <stdlib.h> // for malloc / free
#include <abcc_td.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define TP_DLL_VERSION 0x0202 // 2.2


// TP_ERROR_STRINGS creates an array of char* into which the error messages of
// the Transport providers can be used as index to get a description of the error
//#define TP_ERROR_STRINGS


typedef void* TP_Path; // used to clarify the interfaces


#ifdef __BORLANDC__
#if sizeof( UINT8 ) != 1
   #error UINT8 is not 8 bits
#endif
#if sizeof( UINT16 ) != 2
   #error UINT16 is not 16 bits
#endif
#if sizeof( UINT32 ) != 4
   #error UINT32 is not 32 bits
#endif
#endif


// Enumeration of interface types
typedef enum TP_InterfaceType
{
   TP_SERIAL = 1,
   TP_PARALLEL = 2,
   TP_SPI = 4,
//   TP_PCIe = 0x10000000,        // ToDo MaG 26.02.15: PCIe Interface
   TP_ANY = 0xFFFFFFFF
} TP_InterfaceType;


// Enumeration of provider specific commands. Please note that this list is valid for ALL providers, in case the command is supported
typedef enum TP_MessageCommandType
{
   TP_CMD_RESET                = 0x01,
   TP_CMD_WAIT_EVENT           = 0x02,
   TP_CMD_SET_LED              = 0x03,
   TP_CMD_GET_SWITCH           = 0x04,
   TP_CMD_FUNCTION_REQUEST     = 0x05,
   TP_CMD_USB2_SPECIFIC        = 0x06,
   TP_CMD_GET_PORT             = 0x07,
   TP_CMD_SET_PORT             = 0x08,
   TP_CMD_SET_PORT_CONFIG      = 0x09,
                                       // ToDo MaG 26.02.15: PCIe Interface
   TP_CMD_SET_OP_MODE          = 0x80, // setting operating mode (only 16bit parallel Mode supported) 
   TP_CMD_GET_OP_MODE          = 0x81, // get current operating mode
   TP_CMD_GET_MODULE_ID        = 0x82, // get module identification
   TP_CMD_GET_MODULE_DETECTION = 0x83, // get module detection (NP30/NP40)
   TP_CMD_SET_PD_SIZE          = 0x84  // set the mapped process data sizes (INpact)
} TP_MessageCommandType;

// Enumeration of PCIe provider specific commands. Please note that this list is only valid for the PCIe provider.
typedef enum TP_PCIeSpecificCommandType
{
  TP_SPECIFIC_CMD_NONE = 0,
}TP_PCIeSpecificCommandType;


// Enumeration of responses when using the message interface
typedef enum TP_MessageResponseType
{
   // First all of the responses which can be considered as generic, which could be used by a lot of commands
   TP_CMD_ERR_NONE = 0,
   TP_CMD_ERR_UNKNOWN_CMD = 1,
   TP_CMD_ERR_DATASIZE = 2,
   TP_CMD_ERR_TIMEOUT = 3,
   TP_CMD_ERR_PARAMETER = 4,

   // Thereafter a reserved area for responses specific to that command and/or that transport provider...
   // in order to resolve these the customer needs to report what transport provider and which command was used...
   TP_CMD_ERR_SPECIFIC = 0x8000000,
} TP_MessageResponseType;


// Structure of the parity for serial communication
typedef enum TP_SerialParityType
{
   TP_PARITY_NONE = 0,
   TP_PARITY_ODD = 1,
   TP_PARITY_EVEN = 2,
   TP_MARK = 3,
   TP_SPACE = 4,
} TP_SerialParityType;


// Structure of the stop bits for serial communication
typedef enum TP_SerialStopBitType
{
   TP_STOPBIT_ONE = 0,
   TP_STOPBIT_ONEANDHALF = 1,
   TP_STOPBIT_TWO = 2,
} TP_SerialStopBitType;


// Structure of the wire mode for SPI communication
typedef enum TP_SpiWireModeType
{
   TP_SPI_4WIRE = 0,
   TP_SPI_3WIRE = 1,
} TP_SpiWireModeType;


// Structure of the provider specific messages which can be sent to the transport providers.
typedef union TP_MessageType
{
   struct
   {
      TP_MessageCommandType eCommand;
      UINT8 bDataSize;

      UINT8 abData[ 255 ];
   } sReq;

   struct
   {
      TP_MessageResponseType eResponse;
      UINT8 bDataSize;

      UINT8 abData[ 255 ];
   } sRsp;
} TP_MessageType;


// Enumeration of different status codes returned from the transport functions
typedef enum TP_StatusType
{
   TP_ERR_NONE = 0,
   TP_ERR_OTHER = 1,
   TP_ERR_WIN_FAIL = 2,
   TP_ERR_MEM_ALLOC = 3,
   TP_ERR_CONFIG = 4,
   TP_ERR_ABORTED = 5,
   TP_ERR_MEM_SIZE = 6,
   TP_ERR_OPEN = 7,
   TP_ERR_NO_HW = 8,
   TP_ERR_VERIFY = 9,
   TP_ERR_NOT_OPEN = 10,
   TP_ERR_INVALID_PATH_ID = 11,
   TP_ERR_NULL_FUNCTION = 12,
   TP_ERR_NOT_SUPPORTED = 13,
} TP_StatusType;


#ifdef TP_ERROR_STRINGS
const char* TP_ErrorStrings[] =
{
   "No Error",
   "Unspecified error condition",
   "Windows function failed",
   "Memory allocation failure",

   "Configuration error",
   "Action aborted by user",
   "Could not open parallel memory window of the desired size",
   "Failed to open transport path",

   "No hardware of that type found", // "Could not find associated hardware",
   "Verification error",
   "Transport Path not open",

   "Invalid transport path id",
   "Attempt to call null-function",

   "Configuration is not supported",
};
#endif // TP_ERROR_STRINGS


#ifdef __BORLANDC__

// According to ANSI-C a enum should always be an integer.
// On Borland, please set the "Treat enums as ints"-option
#if sizeof( TP_StatusType ) != sizeof( int )
   #error TP_StatusType is not the size of an int!
#endif

#endif // __BORLANDC__


#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus


#endif // not TP_H

//******************************************************************************
// End of TP.h
//******************************************************************************
