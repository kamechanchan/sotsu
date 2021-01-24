/*******************************************************************************
********************************************************************************
**                                                                            **
** ABCC Starter Kit version 1.01.01 (2015-07-07)                              **
**                                                                            **
** Delivered with:                                                            **
**    ABCC Driver 3.01.01 (2015-07-07)                                        **
**    ABP         7.12.01 (2015-06-25)                                        **
**                                                                            */
/*******************************************************************************
********************************************************************************
** COPYRIGHT NOTIFICATION (c) 2015 HMS Industrial Networks AB                 **
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
** Example of an ADI setup with 5 ADIs
**
********************************************************************************
********************************************************************************
*/
#include <stdio.h>
#ifdef __LINUX__
#include <termios.h>
#endif
#ifndef __WINCE__
#include <unistd.h>
#endif
#include <fcntl.h>

#include "abcc.h"
#include "IDL_hwpara.h"

#include "appl_adi_config.h"
#include "appl_abcc_handler.h"


#if ( APPL_ACTIVE_ADI_SETUP == APPL_ADI_SETUP_SPEED )

/*******************************************************************************
** Constants
********************************************************************************
*/
#define ADI_MINIMUM_SPEED        -500
#define ADI_MAXIMUM_SPEED         500
#define ADI_MINIMUM_TEMPERATURE   -40
#define ADI_MAXIMUM_TEMPERATURE   125

/*******************************************************************************
** Typedefs
********************************************************************************
*/

/*******************************************************************************
** Private Globals
********************************************************************************
*/
extern IDL_CTRL_HDL hCtrl;

/*------------------------------------------------------------------------------
** Data holder for the ADI instances
**------------------------------------------------------------------------------
*/
UINT16   iADI_Speed;
INT16    iADI_Temperature;
UINT16   iADI_RefSpeed;
BOOL8    fADI_DirectionIsForward;
UINT8    bADI_TripCurrent;

/*------------------------------------------------------------------------------
** Min, max and default value for appl_aiUint16
**------------------------------------------------------------------------------
*/
static AD_SINT16Type  sADI_SpeedProps              = {{ ADI_MINIMUM_SPEED, ADI_MAXIMUM_SPEED, 10 }};
static AD_SINT16Type  sADI_TemperatureProps        = {{ ADI_MINIMUM_TEMPERATURE, ADI_MAXIMUM_TEMPERATURE, 25 }};
static AD_UINT16Type  sADI_RefSpeedProps           = {{ 0, 500, 10 }};
static AD_BOOL8Type   sADI_DirectionIsForwardProps = {{ FALSE, TRUE, TRUE }};
static AD_UINT8Type   sADI_TripCurrentProps        = {{ 0, 6, 4 }};

/*******************************************************************************
** Public Globals
********************************************************************************
*/
static void SetSpeedValue( const struct AD_AdiEntry* psAdiEntry, UINT8 bNumElements, UINT8 bStartIndex );
extern int kbhit(void);

/*------------------------------------------------------------------------------
** 32 16-bit values as an array
**------------------------------------------------------------------------------
*/
const AD_AdiEntryType APPL_asAdiEntryList[] =
{
   /* Index: 1 */ {  1,  "Speed",                 ABP_SINT16,   1, APPL_WRITE_MAP_READ_ACCESS_DESC, { { &iADI_Speed,              &sADI_SpeedProps              } }, NULL, NULL, NULL },
   /* Index: 2 */ {  2,  "Ref speed",             ABP_UINT16,   1, APPL_READ_MAP_WRITE_ACCESS_DESC, { { &iADI_RefSpeed,           &sADI_RefSpeedProps           } }, NULL, NULL, SetSpeedValue },
   /* Index: 3 */ {  3,  "Direction = Forward",   ABP_BOOL,     1, APPL_READ_MAP_WRITE_ACCESS_DESC, { { &fADI_DirectionIsForward, &sADI_DirectionIsForwardProps } }, NULL, NULL, NULL },
   /* Index: 4 */ {  4,  "Temperature",           ABP_SINT16,   1, APPL_WRITE_MAP_READ_ACCESS_DESC, { { &iADI_Temperature,        &sADI_TemperatureProps        } }, NULL, NULL, NULL },
   /* Index: 5 */ {  5,  "Trip Current",          ABP_UINT8,    1, APPL_READ_MAP_WRITE_ACCESS_DESC, { { &bADI_TripCurrent,        &sADI_TripCurrentProps        } }, NULL, NULL, NULL }
};

/*------------------------------------------------------------------------------
** Map all adi:s in both directions
**------------------------------------------------------------------------------
** 1. AD instance | 2. Direction | 3. Num elements | 4. Start index |
**------------------------------------------------------------------------------
*/
const AD_DefaultMapType APPL_asAdObjDefaultMap[] =
{
    { 1, PD_WRITE },
    { 2, PD_READ },
    { 3, PD_READ },
    { 4, PD_WRITE },
    { 5, PD_READ },
    { AD_DEFAULT_MAP_END_ENTRY }
};

/*******************************************************************************
** Private Services
********************************************************************************
*/

BOOL8 RunUi(IDL_CTRL_HDL hCtrl)
{
  const  char   abRotor[] = "|/-\\";
  static char   abUserInput[] = { "\0\0\0\0" };
  static UINT32 iMilliSeconds = 0;
  static INT8   iAngle = 0;
  static UINT8  bInputIndex = 0;
  INT32         lNewTemperature;
  BOOL8         fKbInput = FALSE;

  if (kbhit())
  {
    abUserInput[bInputIndex++] = getchar();
    fKbInput = TRUE;
  }

  if ((ABCC_AnbState(hCtrl) == ABP_ANB_STATE_PROCESS_ACTIVE))
  {
    if (fKbInput)
    {
      if ((abUserInput[bInputIndex - 1] == '\r') ||
        (abUserInput[bInputIndex - 1] == '\n') ||
        (bInputIndex >= sizeof(abUserInput)))
      {
        abUserInput[bInputIndex - 1] = 0;

        if (bInputIndex > 1)
        {
          sscanf(abUserInput, "%d", &lNewTemperature);

          if (lNewTemperature > ADI_MAXIMUM_TEMPERATURE)
          {
            iADI_Temperature = ADI_MAXIMUM_TEMPERATURE;
          }
          else if (lNewTemperature < ADI_MINIMUM_TEMPERATURE)
          {
            iADI_Temperature = ADI_MINIMUM_TEMPERATURE;
          }
          else
          {
            iADI_Temperature = (INT16)lNewTemperature;
          }
        }
        bInputIndex = 0;
        abUserInput[0] = '\0';
      }
      printf("\n\nMotor: %c  ", abRotor[iAngle]);
      printf("\tSpeed: %4hd rpm, ", iADI_Speed);
      printf("\tPoles: %4d, ", bADI_TripCurrent);
      printf("\tTemperature: %4d ", iADI_Temperature);
    }
    if (fADI_DirectionIsForward)
    {
      if (iADI_Speed != iADI_RefSpeed)
      {
        /*
        ** Speed has changed.
        */
        iADI_Speed = iADI_RefSpeed;
        ABCC_TriggerWrPdUpdate(hCtrl);
      }
    }
    else
    {
      if (iADI_Speed != -iADI_RefSpeed)
      {
        /*
        ** Speed has changed.
        */
        iADI_Speed = -iADI_RefSpeed;
        ABCC_TriggerWrPdUpdate(hCtrl);
      }
    }

    if ((iADI_RefSpeed == 0) ||
      (iMilliSeconds >= (UINT32)(60000 / (iADI_RefSpeed * 8))))
    {
      printf("\rMotor: %c  ", abRotor[iAngle]);
      printf("\tSpeed: %4hd rpm, ", iADI_Speed);
      printf("\tPoles: %4d, ", bADI_TripCurrent);
      printf("\tTemperature: %4d ", iADI_Temperature);

      if (iADI_RefSpeed != 0)
      {
        iMilliSeconds = 0;

        if (fADI_DirectionIsForward)
        {
          iAngle = (iAngle + 1) & 0x3;
        }
        else
        {
          iAngle = (iAngle - 1) & 0x3;
        }
      }
    }
    else
    {
      iMilliSeconds += 50;
    }
  }

  return (FALSE);

} /* End of RunUi() */

static void SetSpeedValue( const struct AD_AdiEntry* psAdiEntry, UINT8 bNumElements, UINT8 bStartIndex )
{
   if(RunUi(hCtrl)){
       APPL_Shutdown(hCtrl);
   }
}

/*******************************************************************************
** Public Services
********************************************************************************
*/

UINT16 APPL_GetNumAdi( void )
{
   return( sizeof( APPL_asAdiEntryList ) / sizeof( AD_AdiEntryType ) );
}

/*******************************************************************************
** Tasks
********************************************************************************
*/

#endif
