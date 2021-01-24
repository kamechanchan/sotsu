/*******************************************************************************
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
*/

/***************************************************************************************************
** Include files
***************************************************************************************************/

/* Enable debugging */
#define DBG_PRINTF      FALSE
#define DBG_PCIE_IF     FALSE       // Debugging PCIe command communication

#include "IDLTp.h"
#include <abcc_td.h>
#include "abcc_debug_err.h"

#include <abcc_sw_port.h>

#include "abcc_sys_adapt.h"
#include "abcc_sys_adapt_par.h"	// MaG: only 16bit interface supported
#include "abp.h"

EXTFUNC void ( *ABCC_ISR )( IDL_CTRL_HDL hCtrl );
void* ISR( void *pMyID );

void ABCC_SYS_UseCriticalImpl( IDL_CTRL_HDL hCtrl );
void ABCC_SYS_EnterCriticalImpl( IDL_CTRL_HDL hCtrl );
void ABCC_SYS_ExitCriticalImpl( IDL_CTRL_HDL hCtrl );

/***************************************************************************************************
** global variables
***************************************************************************************************/
static IDL_SCHEDULER_SETTINGS* apIsrSchedulerSettings[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT] = {{0}};

/***************************************************************************************************
** static constants, file types, macros, variables
***************************************************************************************************/
/* The ACI external memory map is 16 kB. */
#define ACI_MEMORY_MAP_SIZE 16384

static UINT8    sys_abReadProcessData[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT][ ABCC_CFG_MAX_PROCESS_DATA_SIZE ];  /* Process data byte array. */
static UINT8    sys_abWriteProcessData[IDL_MAXBOARDCOUNT][IDL_MAXCTRLCOUNT][ ABCC_CFG_MAX_PROCESS_DATA_SIZE ]; /* Process data byte array. */

static UINT8 sys_bOpmode = ABP_OP_MODE_16_BIT_PARALLEL;       // default 16bit parallel mode

/***************************************************************************************************
** static function-prototypes
***************************************************************************************************/
static void TP_ReadMemory(IDL_CTRL_HDL hCtrl, UINT16 iMemOffset, UINT8* pbData, UINT16 iLength);
static void TP_WriteMemory(IDL_CTRL_HDL hCtrl, UINT16 iMemOffset, UINT8* pbData, UINT16 iLength);

/***************************************************************************************************
** global functions
***************************************************************************************************/
BOOL ABCC_SYS_HwInit( IDL_CTRL_HDL hCtrl )
{
   /*
   ** Implement according to abcc_sys_adapt.h
   */
   return 1;
}

BOOL ABCC_SYS_Init( IDL_CTRL_HDL hCtrl )
{
   return 1;
}

void ABCC_SYS_Close( IDL_CTRL_HDL hCtrl )
{
}

#define ABCC_MAXMEMSIZE 0x1000
void ABCC_SYS_ParallelRead( IDL_CTRL_HDL hCtrl, UINT16 iMemOffset, void* pxData, UINT16 iLength )
{
  UINT16 wCount = ABCC_MAXMEMSIZE;
  UINT16 wOffset = 0;

  while (wOffset < iLength)
  {
    if (iLength < ABCC_MAXMEMSIZE)
      wCount = iLength;

    TP_ReadMemory(hCtrl, iMemOffset + wOffset, ((UINT8*)pxData) + wOffset, wCount);
    wOffset += wCount;
  }
}

UINT8 ABCC_SYS_ParallelRead8( IDL_CTRL_HDL hCtrl, UINT16 iMemOffset )
{
   UINT8 bData;
   ABCC_SYS_ParallelRead( hCtrl, iMemOffset, &bData, sizeof( UINT8 ) );
   return bData;
}

UINT16 ABCC_SYS_ParallelRead16( IDL_CTRL_HDL hCtrl, UINT16 iMemOffset )
{
   UINT16 iData;
   ABCC_SYS_ParallelRead( hCtrl, iMemOffset, (UINT8*)&iData, sizeof( UINT16 ) );
   return iData;
}

void ABCC_SYS_ParallelWrite( IDL_CTRL_HDL hCtrl, UINT16 iMemOffset, void* pxData, UINT16 iLength )
{
  UINT16 wCount = ABCC_MAXMEMSIZE;
  UINT16 wOffset = 0;

  while (wOffset < iLength)
  {
    if (iLength < ABCC_MAXMEMSIZE)
      wCount = iLength;

    TP_WriteMemory(hCtrl, iMemOffset + wOffset, ((UINT8*)pxData) + wOffset, wCount);
    wOffset += wCount;
  }
}

void ABCC_SYS_ParallelWrite8( IDL_CTRL_HDL hCtrl, UINT16 iMemOffset, UINT8 pbData )
{
   ABCC_SYS_ParallelWrite( hCtrl, iMemOffset, (UINT8*)&pbData, sizeof( UINT8 ) );
}

void ABCC_SYS_ParallelWrite16( IDL_CTRL_HDL hCtrl, UINT16 iMemOffset, UINT16 pbData )
{
   ABCC_SYS_ParallelWrite( hCtrl, iMemOffset, (UINT8*)&pbData, sizeof( UINT16 ) );
}

/*** OPERATING MODE HANDLING **********************************************************************
 -> 0x00     reserved
 -> 0x01     SPI                             (not supported)
 -> 0x02     Stand-alone shift register      (not supported)
 -> 0x03     reserved
 -> 0x04     reserved
 -> 0x05     reserved
 -> 0x06     reserved
 -> 0x07     16-bit parallel                 (DEFAULT)
 -> 0x08      8-bit parallel                 (not supported)
 -> 0x09     serial  19.2 kbps               (not supported)
 -> 0x0A     serial  57.6 kbps               (not supported)
 -> 0x0B     serial 115.2 kbps               (not supported)
 -> 0x0C     serial 625.0 kbps               (not supported)
 -> 0x0D     reserved
 -> 0x0E     reserved
 -> 0x0F     Service Mode                    (not supported)
 *************************************************************************************************/
void ABCC_SYS_SetOpmode( IDL_CTRL_HDL hCtrl, UINT8 bOpmode )
{
  TP_StatusType eStatus;
  TP_MessageType sMsg;
  sMsg.sReq.eCommand = TP_CMD_SET_OP_MODE;
  sMsg.sReq.bDataSize = 1;
  sMsg.sReq.abData[0] = bOpmode;

  ABCC_PORT_EnterCritical(hCtrl);
  eStatus = ProviderSpecificCommand(hCtrl, &sMsg);
  ABCC_PORT_ExitCritical(hCtrl);
  if (eStatus != TP_ERR_NONE)
  {
    DEBUG_EVENT(("Transport provider error %d\n", eStatus));
  }
  sys_bOpmode = bOpmode;
}

UINT8 ABCC_SYS_GetOpmode( IDL_CTRL_HDL hCtrl )
{
  TP_StatusType eStatus;
  TP_MessageType sMsg;
  sMsg.sReq.eCommand = TP_CMD_GET_OP_MODE;
  sMsg.sReq.bDataSize = 0;

  ABCC_PORT_EnterCritical(hCtrl);
  eStatus = ProviderSpecificCommand(hCtrl, &sMsg);
  ABCC_PORT_ExitCritical(hCtrl);
  if (eStatus != TP_ERR_NONE)
  {
    DEBUG_EVENT(("Transport provider error %d\n", eStatus));
  }
  return sMsg.sRsp.abData[0];
}

/*** RESET HANDLING *******************************************************************************
 Reset handling valid values for eCommand[0]
   -> 0     set reset line to "0"
   -> 1     set reset line to "1"
   -> 2     activate reset impulse
 *************************************************************************************************/
void ABCC_SYS_HWReset( IDL_CTRL_HDL hCtrl )
{
  TP_StatusType eStatus;
  TP_MessageType sMsg;
  sMsg.sReq.eCommand = TP_CMD_RESET;
  sMsg.sReq.bDataSize = 1;
  sMsg.sReq.abData[0] = 0;

  ABCC_PORT_EnterCritical(hCtrl);
  eStatus = ProviderSpecificCommand(hCtrl, &sMsg);
  ABCC_PORT_ExitCritical(hCtrl);
  if (eStatus != TP_ERR_NONE)
  {
    DEBUG_EVENT(("Transport provider error %d\n", eStatus));
  }
}

void ABCC_SYS_HWReleaseReset( IDL_CTRL_HDL hCtrl )
{
  TP_StatusType eStatus;
  TP_MessageType sMsg;
  sMsg.sReq.eCommand = TP_CMD_RESET;
  sMsg.sReq.bDataSize = 1;
  sMsg.sReq.abData[0] = 1;

  ABCC_PORT_EnterCritical(hCtrl);
  eStatus = ProviderSpecificCommand(hCtrl, &sMsg);
  ABCC_PORT_ExitCritical(hCtrl);
  if (eStatus != TP_ERR_NONE)
  {
    DEBUG_EVENT(("Transport provider error %d\n", eStatus));
  }
}

void ABCC_SYS_HWResetImpulse(IDL_CTRL_HDL hCtrl) // not used yet
{
  TP_StatusType eStatus;
  TP_MessageType sMsg;
  sMsg.sReq.eCommand = TP_CMD_RESET;
  sMsg.sReq.bDataSize = 1;
  sMsg.sReq.abData[0] = 2;

  ABCC_PORT_EnterCritical(hCtrl);
  eStatus = ProviderSpecificCommand(hCtrl, &sMsg);
  ABCC_PORT_ExitCritical(hCtrl);
  if (eStatus != TP_ERR_NONE)
  {
    DEBUG_EVENT(("Transport provider error %d\n", eStatus));
  }
}

/*** MODULE IDENTIFICATION ************************************************************************
  return values
  -> 0x00     Active  Anybus CompactCom 30 (invalid value)
  -> 0x01     Passive Anybus CompactCom    (invalid value)
  -> 0x02     Active  Anybus CompactCom 40
  -> 0x03     Customer specific
 *************************************************************************************************/
UINT8 ABCC_SYS_ReadModuleId( IDL_CTRL_HDL hCtrl )
{
  TP_StatusType eStatus;
  TP_MessageType sMsg;
  sMsg.sReq.eCommand = TP_CMD_GET_MODULE_ID;
  sMsg.sReq.bDataSize = 0;

  ABCC_PORT_EnterCritical(hCtrl);
  eStatus = ProviderSpecificCommand(hCtrl, &sMsg);
  ABCC_PORT_ExitCritical(hCtrl);
  if (eStatus != TP_ERR_NONE)
  {
    DEBUG_EVENT(("Transport provider error %d\n", eStatus));
  }

  return sMsg.sRsp.abData[0];
}

/*** SET THE PROCESSDATA SIZE *********************************************************************
  return values
  -> != 0     successful
  -> 0        failed or not supported
 *************************************************************************************************/
BOOL ABCC_SYS_SetPDSize ( IDL_CTRL_HDL hCtrl, const UINT16 iReadPdSize, const UINT16 iWritePdSize  )
{
  TP_StatusType  eStatus;
  TP_MessageType sMsg;

  sMsg.sReq.eCommand = TP_CMD_SET_PD_SIZE;
  sMsg.sReq.bDataSize = 4;
  *((UINT16 *)&sMsg.sReq.abData[0]) = iReadPdSize;
  *((UINT16 *)&sMsg.sReq.abData[2]) = iWritePdSize;

  ABCC_PORT_EnterCritical(hCtrl);
  eStatus = ProviderSpecificCommand(hCtrl, &sMsg);
  ABCC_PORT_ExitCritical(hCtrl);
  if (eStatus != TP_ERR_NONE)
  {
    DEBUG_EVENT(("Transport provider error %d\n", eStatus));
    while (1);
  }

  return sMsg.sRsp.abData[0];
}

/*** MODULE DETECTION *****************************************************************************
  return values ATTENTION: results inverted!
  -> 0x00     Module present
  -> 0x01     Module not present
  -> 0x02     Module not present
  -> 0x03     Module not present
 *************************************************************************************************/
BOOL ABCC_SYS_ModuleDetect( IDL_CTRL_HDL hCtrl )
{
  TP_StatusType eStatus;
  TP_MessageType sMsg;
  sMsg.sReq.eCommand = TP_CMD_GET_MODULE_DETECTION;
  sMsg.sReq.bDataSize = 0;

  ABCC_PORT_EnterCritical(hCtrl);
  eStatus = ProviderSpecificCommand(hCtrl, &sMsg);
  ABCC_PORT_ExitCritical(hCtrl);
  if (eStatus != TP_ERR_NONE)
  {
    DEBUG_EVENT(("Transport provider error %d\n", eStatus));
    return FALSE;
  }

  return sMsg.sRsp.abData[0];
}

/*** LOCAL PROCESS DATA BUFFER HANDLING ***********************************************************
 return local process data buffer
 *************************************************************************************************/
void* ABCC_SYS_ParallelGetRdPdBuffer( IDL_CTRL_HDL hCtrl )
{
  IDL_CTRL_HANDLE hCtrlHandle;
  hCtrlHandle.hCtrlHandle = hCtrl;
  return (void*)sys_abReadProcessData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];
}

void* ABCC_SYS_ParallelGetWrPdBuffer( IDL_CTRL_HDL hCtrl )
{
  IDL_CTRL_HANDLE hCtrlHandle;
  hCtrlHandle.hCtrlHandle = hCtrl;
  return (void*)sys_abWriteProcessData[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex];
}

/*** INTERRUPT / EVENT HANDLING *******************************************************************
 Interrupt handling
 There are two possibilites to handle the interrupts:
 - polling the interrupt flag
 - interpreting the events from the PCIe transport provider
*************************************************************************************************/
void ABCC_SYS_AbccInterruptEnable( IDL_CTRL_HDL hCtrl )
{
   IDL_InterruptEnable(hCtrl);
}

void ABCC_SYS_AbccInterruptDisable( IDL_CTRL_HDL hCtrl )
{
   IDL_InterruptDisable(hCtrl);
}

/*** LOCAL PROCESS DATA BUFFER HANDLING ***********************************************************
return local process data buffer
*************************************************************************************************/
void ABCC_SYS_UseCriticalImpl( IDL_CTRL_HDL hCtrl )
{
  IDL_UseCriticalImpl(hCtrl);
}

void ABCC_SYS_EnterCriticalImpl( IDL_CTRL_HDL hCtrl )
{
  IDL_EnterCriticalImpl(hCtrl);
}

void ABCC_SYS_ExitCriticalImpl( IDL_CTRL_HDL hCtrl )
{
  IDL_ExitCriticalImpl(hCtrl);
}

/*
 ** This function will start the transport provider connection
 ** Note! This function is called by the application before
 ** the driver is accessed.
 */
BOOL ABCC_OpenController( UINT32 dwHwIndex, IDL_HW_PARA *pstcHwPara, IDL_CTRL_HDL *hCtrl )
{
  BOOL oRet = FALSE;
  TP_StatusType eStatType;
  TP_Path xPathHandle;

  if (TP_ERR_NONE == CreatePath(&xPathHandle))
  {
    eStatType = UserConfig(*hCtrl, TP_PARALLEL);
    if (TP_ERR_NONE == eStatType)
    {
      eStatType = ParallelOpen  (xPathHandle, 0, dwHwIndex, 0, pstcHwPara, hCtrl);
      if (TP_ERR_NONE == eStatType)
      {
        oRet = TRUE;
        if(pstcHwPara){
          switch (pstcHwPara->wHardwareClass) {
          case IDL_HW_PCI:
            if(IDL_STRUCT_VERSION_V1 == pstcHwPara->u.sPciSettings.dwVer){
              apIsrSchedulerSettings[dwHwIndex][0] = (IDL_SCHEDULER_SETTINGS*)malloc(sizeof(IDL_SCHEDULER_SETTINGS));
              memcpy(apIsrSchedulerSettings[dwHwIndex][0],
                  &pstcHwPara->u.sPciSettings.u.V1.sSchedSettings,
                  sizeof(IDL_SCHEDULER_SETTINGS));
            }
            break;
          case IDL_HW_USB:
            if(IDL_STRUCT_VERSION_V1 == pstcHwPara->u.sPciSettings.dwVer){
              oRet = FALSE;
            }
            break;
          case IDL_HW_ISA:
            if(IDL_STRUCT_VERSION_V1 == pstcHwPara->u.sPciSettings.dwVer){
              oRet = FALSE;
            }
            break;
            default:
              apIsrSchedulerSettings[dwHwIndex][0] = NULL;
              break;
          }
        }
      }
    }

    if (TP_ERR_NONE != eStatType)
    {
      DEBUG_EVENT(("Failed to open transport provider\n"));
      DestroyPath(xPathHandle);
      oRet = FALSE;
    }
  }

  return oRet;
}


/*
 ** This function will close the transport provider connection.
 ** This function is called by the application at system shutdown.
 ** to release tranport provider recources
 */
void ABCC_CloseController( IDL_CTRL_HDL* hCtrl )
{
  IDL_CTRL_HANDLE hCtrlHandle;

  if(NULL == hCtrl)
    return;
  if(*hCtrl == IDL_INVALID_HANDLE)
    return;

  hCtrlHandle.hCtrlHandle = *hCtrl;

  if(TP_ERR_NONE == ParallelClose(*hCtrl))
    *hCtrl = IDL_INVALID_HANDLE;

  if(apIsrSchedulerSettings[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex])
    free(apIsrSchedulerSettings[hCtrlHandle.Bits.dwBoardIndex][hCtrlHandle.Bits.dwCtrlIndex]);
}

/***************************************************************************************************
** static functions
***************************************************************************************************/
static void TP_ReadMemory(IDL_CTRL_HDL hCtrl, UINT16 iMemOffset, UINT8* pbData, UINT16 iLength)
{
  TP_StatusType eStatType;

  eStatType = ParallelRead(hCtrl, iMemOffset, pbData, iLength);
  if (eStatType != TP_ERR_NONE)
  {
     DEBUG_EVENT(("TP_ReadMemory(): ERROR occured (return code: %d)\n", eStatType));
  }
}

static void TP_WriteMemory(IDL_CTRL_HDL hCtrl, UINT16 iMemOffset, UINT8* pbData, UINT16 iLength)
{
  TP_StatusType eStatType;

  eStatType = ParallelWrite(hCtrl, iMemOffset, pbData, iLength);
  if (eStatType != TP_ERR_NONE)
  {
    DEBUG_EVENT(("TP_WriteMemory(): ERROR occured (return code: %d)\n", eStatType));
  }
}


/*******************************************************************************
** End of abcc_sys_adapt.c
********************************************************************************
*/
