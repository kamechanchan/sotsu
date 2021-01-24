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
** Definition of types, structs and unions for hardware initialization and
** communication.
********************************************************************************
********************************************************************************
*/
#ifndef __IDL_HWPARA_H__
#define __IDL_HWPARA_H__


//////////////////////////////////////////////////////////////////////////
// include files
#include "abcc_td.h"

#include "IDL_thread.h"

//////////////////////////////////////////////////////////////////////////
// constants and macros

/** Maximum supported boards by the IDL API. @ingroup HwTypes */
#define IDL_MAXBOARDCOUNT    32
/** Maximum supported controllers per board. @ingroup CtrlTypes */
#define IDL_MAXCTRLCOUNT     8

/** Definition of an invalid handle. */
#define IDL_INVALID_HANDLE   0


//////////////////////////////////////////////////////////////////////////
// data types
/**
  IDL controller handle

  @ingroup CtrlTypes
*/
typedef UINT32 IDL_CTRL_HDL;


/**
  IDL structure versions
*/
typedef enum
{
  IDL_STRUCT_VERSION_V0 = 0,      ///< Version 0
  IDL_STRUCT_VERSION_V1,          ///< Version 1
  IDL_STRUCT_VERSION_V2,          ///< Version 2
  IDL_STRUCT_VERSION_V3           ///< Version 3
} e_IDL_STRUCT_VERSION;


/**
  Hardware classes

  @ingroup HwTypes
*/
typedef enum
{
  IDL_HW_UNDEFINED  = 0x00,        ///< undefined
  IDL_HW_PCI        = 0x01,        ///< PCI/PCIe hardware
  IDL_HW_ISA        = 0x02,        ///< ISA hardware
  IDL_HW_USB        = 0x03         ///< USB hardware
} e_HWCLASS;

/*------------------------------------------------------------------------------
** IDL_PCI_SETTINGS.
**------------------------------------------------------------------------------
** 1. dwVer            - Version of valid union struct
** 2. u                - Hardware configuration container
** 3. V0               - Version 0
** 4. V1               - Version 1
** 5. dwReserved       - Reserved
** 5. sSchedSettings   - Scheduler settings
*/
typedef struct
{
  UINT32 dwVer;                 ///< Version of valid union struct

  union
  {
    struct
    {
      UINT32 dwReserved;       ///< Reserved
    } V0;                      ///< Version 0
    struct
    {
      IDL_SCHEDULER_SETTINGS sSchedSettings;  ///< Scheduler settings
    } V1;                          ///< Version 1
  } u;              ///< Version controlled structs container
} IDL_PCI_SETTINGS;


/*------------------------------------------------------------------------------
** IDL_ISA_SETTINGS.
**------------------------------------------------------------------------------
** 1. dwVer            - Version of valid union struct
** 2. u                - Hardware configuration container
** 3. V0               - Version 0
** 4. dwAddress        - Address of the ISA hardware
** 5. bInterrupt       - Interrupt of the ISA hardware
*/
typedef struct
{
  UINT32 dwVer;                 ///< Version of valid union struct

  union
  {
    struct
    {
      UINT32 dwAddress;         ///< Address of the ISA hardware
      UINT8  bInterrupt;        ///< Interrupt of the ISA hardware
    } V0;           ///< Version 0
  } u;                             ///< Version controlled structs container
} IDL_ISA_SETTINGS;


/*------------------------------------------------------------------------------
** IDL_USB_SETTINGS.
**------------------------------------------------------------------------------
** 1. dwVer            - Version of valid union struct
** 2. u                - Hardware configuration container
** 3. V0               - Version 0
** 4. dwReserved       - Reserved for future use
*/
typedef struct
{
  UINT32 dwVer;                 ///< Version of valid union struct

  union
  {
    struct
    {
      UINT32 dwReserved;        ///< Reserved for future use
    } V0;                       ///< Version 0
  } u;                          ///< Version controlled structs container
} IDL_USB_SETTINGS;


/*------------------------------------------------------------------------------
** IDL_HW_PARA.
**------------------------------------------------------------------------------
** 1. wHardwareClass         - Hardware class @see e_HWCLASS
** 2. dwFlags                - Flag field for special purpose 
**                             @see e_SETTINGS_FLAGS
** 3. u                      - Hardware configuration container
** 4. sPciSettings           - PCI hardware settings @see IDL_PCI_SETTINGS
** 5. sIsaSettings           - ISA hardware settings @see IDL_ISA_SETTINGS
** 6. sUsbSettings           - USB hardware settings @see IDL_USB_SETTINGS
*/
typedef struct
{
  UINT16 wHardwareClass;             ///< Hardware class @see e_HWCLASS
  UINT32 dwFlags;                    ///< Flag field for special purpose @see e_SETTINGS_FLAGS

  union
  {
    IDL_PCI_SETTINGS sPciSettings;  ///< PCI hardware settings @see IDL_PCI_SETTINGS
    IDL_ISA_SETTINGS sIsaSettings;  ///< ISA hardware settings @see IDL_ISA_SETTINGS
    IDL_USB_SETTINGS sUsbSettings;  ///< USB hardware settings @see IDL_USB_SETTINGS
  } u;                              ///< Hardware configuration container
} IDL_HW_PARA;

#endif //__IDL_HWPARA_H__
