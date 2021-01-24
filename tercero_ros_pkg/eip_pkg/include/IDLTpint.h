//******************************************************************************
// COPYRIGHT NOTIFICATION (c) 2003 HMS Industrial Networks AB
//
// This program is the property of HMS Industrial Networks AB.
// It may not be reproduced, distributed, or used without permission
// of an authorised company official.
//******************************************************************************

#ifndef __IDLTPINT_H__
#define __IDLTPINT_H__

#include "TP.h"
#include <Driver.hpp>
#include <HwController.hpp>

#include <XATpshpack1.h>

enum UsbModeType
{
  USB_8BIT,
  USB_16BIT,
  USB_SPI,
};

// TP_PciConfigType
//
// Structure containing all information necessary for the user to supply to
// configure a path to a PCI board.
typedef struct TP_PciConfigType
{
   // Every configuration structure in ALL transport providers SHALL contain
   // the size of the configuration structure in the first two bytes.
   // We'd better obey that as well.
   UINT16 mySize;
   TP_InterfaceType myConfigType; // used to keep track of the type of config
   UsbModeType myMode;

   // And now for the configuration settings
   IDL_HANDLE hDevHandle;   // This Id is unique for a complete session
   UINT8 myIsrEnableFlag;
} TP_PciConfigType;


// TP_PciPathType
//
// Structure containing all information regarding a path to a PCI board
// The user will be given a pointer to a structure of this kind when he
// asks for a identifier for the path (upon creating it).
typedef struct TP_PciPathType
{
   // We need to know our configuration!
   TP_PciConfigType myConfig;

   BOOL myOpenFlag; // Keep track of the open/closed status
   CDriver* pDriver;
   CHwController* pHwController;
   CHwAdapter* pHwAdapter;
}
TP_PciPathType;

#include <XATpoppack.h>

#endif // not __IDLTPINT_H__


//******************************************************************************
// End of pci.h
//******************************************************************************
