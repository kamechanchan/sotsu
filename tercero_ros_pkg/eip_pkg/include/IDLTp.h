///////////////////////////////////////////////////////////////////////////////
// IXXAT Automation GmbH
///////////////////////////////////////////////////////////////////////////////
/**
  @todo brief description

  @file IDLTp.h
*/
#ifndef __IDLTP_H__
#define __IDLTP_H__

//////////////////////////////////////////////////////////////////////////
// include files
#include <TP.h>
#include <OsIdl.h>
#include <IDL_hwpara.h>
#include <abcc_td.h>
#include <IDL.h>

#define ABCC_EXEC_CMD        0x00
#define ABCC_EXEC_SIMULATE   0x01

#define ABCC_READ   0x01
#define ABCC_WRITE  0x02

/// IDL controller handle translation helper
typedef union _tagIDL_CTRL_HANDLE
{
  IDL_CTRL_HDL hCtrlHandle;   ///< IDL Controller handle

  struct
  {
    UINT32 dwBoardIndex:8;     ///< Board / Hardware Index
    UINT32 dwCtrlIndex:8;      ///< Controller Index
    UINT32 dwReserved1:8;      ///< reserved
    UINT32 dwReserved2:7;      ///< reserved
    UINT32 dwValid:1;          ///< Validation flag
  }Bits;
}IDL_CTRL_HANDLE, *PIDL_CTRL_HANDLE;

//////////////////////////////////////////////////////////////////////////
// constants and macros

//*** C-API
#ifdef __cplusplus
extern "C"
{
#endif

//////////////////////////////////////////////////////////////////////////
/**
  This function will create a path that contains the configurations for this
  Transport Provider.

  @param aReturnPath
    The in-parameter must be a NULL-pointer that don�t have any data allocated.
    The CreatePath-function will then allocate enough data to contain the transport
    provider configuration.
    This path will then be used in every other functions as in-parameter.

  @return
    This function can fail in two ways. If there isn�t enough memory in Windows to
    allocate the path, it will result in the error code TP_ERR_MEM_ALLOC.
    The function will also fail with error code: TP_ERR_OTHER if the D2XX drivers are not
    installed.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType CreatePath              ( TP_Path* aReturnPath );

//////////////////////////////////////////////////////////////////////////
/**
  The DestroyPath-function must be called when the path created with
  CreatePath-function is not needed anymore. This function will clear up the
  memory and invalidate the path.

  @param aPath

  @return
    Even though this function have a return value, it will not return any error code.
    The function will always return the error code: TP_ERR_NONE.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType DestroyPath             ( TP_Path aPath );

//////////////////////////////////////////////////////////////////////////
/**
  The current configuration is copied byte-by-byte to the parameter aConfigtPtr.

  @param aPath
  @param aReturnConfigPtr

  @return
    If the function succeeds it returns TP_ERR_NONE, otherwise a value other
    than TP_ERR_NONE.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType GetConfig               ( IDL_CTRL_HDL hCtrl, void** aReturnConfigPtr );

//////////////////////////////////////////////////////////////////////////
/**
  The data set in the parameter aConfig is copied byte-by-byte to the current
  configuration.

  @param aPath
  @param aConfig

  @return
    If the function succeeds it returns TP_ERR_NONE, otherwise a value other
    than TP_ERR_NONE.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType SetConfig               ( IDL_CTRL_HDL hCtrl, void* aConfig );

//////////////////////////////////////////////////////////////////////////
/**
  This function will create a graphical user interface to change the selected
  device

  @param aPath
  @param anInterface

  @return
    If the function succeeds it returns TP_ERR_NONE, otherwise a value other
    than TP_ERR_NONE.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType UserConfig              ( IDL_CTRL_HDL hCtrl, TP_InterfaceType anInterface );

//////////////////////////////////////////////////////////////////////////
/**
  This function contains commands that are specific for this special device.

  @param aPath
  @param aMessage

  @return
    If the function succeeds it returns VCI_OK, otherwise a value other
    than VCI_OK.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType ProviderSpecificCommand ( IDL_CTRL_HDL hCtrl, TP_MessageType* aMessage );

IDL_DLLEXPORT TP_StatusType ParallelOpenTest        ( IDL_CTRL_HDL hCtrl, void* pFirmware,UINT32 dwSize );

//////////////////////////////////////////////////////////////////////////
/**
  function to open the device connection specified through the TP_Path structure

  @param aPath
  @param aSize

  @return
    If the function succeeds it returns VCI_OK, otherwise a value other
    than VCI_OK.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType ParallelOpen            ( TP_Path aPath, UINT16 aSize, UINT32 dwHwIndex,
                                                      UINT32 dwCtrlIndex, IDL_HW_PARA* pstcHwPara,
                                                      IDL_CTRL_HDL* hCtrl);

//////////////////////////////////////////////////////////////////////////
/**
  When calling the ParallelClose function, the DUT will be set to off and
  the LED for the DUT will be lit red.

  @param aPath

  @return
    If the function succeeds it returns VCI_OK, otherwise a value other
    than VCI_OK.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType ParallelClose           ( IDL_CTRL_HDL hCtrl );

//////////////////////////////////////////////////////////////////////////
/**
  ParallelRead will read a specific  amount of data from the device.

  @param aPath
  @param anOffset
    The parameter anOffset defines the memory address
  @param someData
    The readen data is stored within the someData parameter
  @param anAmount
    The anAmount parameter defines how much data to read.

  @return
    If the function succeeds it returns VCI_OK, otherwise a value other
    than VCI_OK.

  @note
    Access:    public
    When using ParallelRead in 16-bits mode, it will handle reading data on
    odd addresses and/or odd data lengths. In those cases, the function can
    send the MCU_READ_DATA command up to three times. The table 10 below explains
    the cases. The table will explain if there should be a start byte and/or a
    end byte. If there should be a start byte, then the first byte of the data
    will be sent with a MCU_READ_DATA command and then the rest of the data will
    be sent with a new MCU_READ_DATA command. So the data will be split into
    two parts. The same goes if there should be a end byte, but instead the last
    byte is sent as a separate command.
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType ParallelRead            ( IDL_CTRL_HDL hCtrl, UINT16 anOffset, UINT8* someData, UINT16 anAmount );

//////////////////////////////////////////////////////////////////////////
/**
  The ParallelVerifyRead function uses recursion to divide the data into chunks
  of 50 bytes. Each chunk is read separately from the device.

  @param aPath
  @param anOffset
  @param someData
  @param anAmount
  @param aMaxTries

  @return
    If the function succeeds it returns VCI_OK, otherwise a value other
    than VCI_OK.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType ParallelVerifyRead      ( IDL_CTRL_HDL hCtrl, UINT16 anOffset, UINT8* someData, UINT16 anAmount, UINT16 aMaxTries );

//////////////////////////////////////////////////////////////////////////
/**
  The function will write the data specified in the someData parameter to
  the device.

  @param aPath
  @param anOffset
    The anOffset parameter will specify the memory address to store the data
    on the device.
  @param someData
    The someData parameter points on the data to write
  @param anAmount
    The parameter anAmount specifies the size of the data that will be written

  @return
    If the function succeeds it returns VCI_OK, otherwise a value other
    than VCI_OK.

  @note
    Access:    public
    When using ParallelWrite in 16-bits mode, it will handle writing data on odd
    addresses and/or odd data lengths. In those cases, the function can send the
    MCU_WRITE_DATA command up to three times. The table 8 below explains the cases.
    The table will explain if there should be a start byte and/or a end byte. If
    there should be a start byte, then the first byte of the data will be sent with
    a MCU_WRITE_DATA command and then the rest of the data will be sent with a new
    MCU_WRITE_DATA command. So the data will be split into two parts. The same goes
    if there should be a end byte, but instead the last byte is sent as a separate
    command.
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType ParallelWrite           ( IDL_CTRL_HDL hCtrl, UINT16 anOffset, const UINT8* someData, UINT16 anAmount );

//////////////////////////////////////////////////////////////////////////
/**
  This function will make several tries to write a byte of the device !

  @param aPath
  @param anOffset
  @param someData
  @param anAmount
  @param aMaxTries

  @return
    If the function succeeds it returns VCI_OK, otherwise a value other
    than VCI_OK.

  @note
    Access:    public
*/
//////////////////////////////////////////////////////////////////////////
IDL_DLLEXPORT TP_StatusType ParallelVerifyWrite     ( IDL_CTRL_HDL hCtrl,
                                                      UINT16 anOffset,
                                                      const UINT8* someData,
                                                      UINT16 anAmount,
                                                      UINT16 aMaxTries );

IDL_DLLEXPORT UINT32 IdlTPGetVersion ( UINT32* pdwMajorVersion, UINT32* pdwMinorVersion );

IDL_DLLEXPORT void IDL_APICALL IDL_InterruptEnable( IDL_CTRL_HDL hCtrl );

IDL_DLLEXPORT void IDL_APICALL IDL_InterruptDisable( IDL_CTRL_HDL hCtrl );

IDL_DLLEXPORT TP_Path IDL_APICALL IDL_GetPath( IDL_CTRL_HDL hCtrl );

//*** C-API
#ifdef __cplusplus
}
#endif

//////////////////////////////////////////////////////////////////////////
// data types

#endif //__IDL_H__
