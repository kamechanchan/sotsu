/*******************************************************************************
********************************************************************************
**                                                                            **
** ABCC Starter Kit version 2.01.01 (2015-12-14)                              **
**                                                                            **
** Delivered with:                                                            **
**    ABCC Driver 4.01.01 (2015-12-14)                                        **
**    ABP         7.16.01 (2015-10-14)                                        **
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
** Main program
********************************************************************************
********************************************************************************
*/

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

#include "abcc_td.h"
#include "abcc.h"
#include "ad_obj.h"
#include "appl_abcc_handler.h"
#include "appl_adi_config.h"

// #ifdef __cplusplus
// }
// #endif


#include "IDL.h"
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>

IDL_CTRL_HDL hCtrl = 0;

#define APPL_TIMER_MS         1
#define USE_TIMER_INTERRUPT   0

/*------------------------------------------------------------------------------
** This is a posix implementation of the keyboard hit function of the
** MSVC compiler
**------------------------------------------------------------------------------
** Arguments:
**    -
**
** Returns:
**    Returns 1 if an input was made otherwise 0.
**------------------------------------------------------------------------------
*/
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

/*------------------------------------------------------------------------------
** Checks if the user typed 'q' or 'Q'
**------------------------------------------------------------------------------
** Arguments:
**    -
**
** Returns:
**    Returns TRUE if 'q' or 'Q' was typed.
**------------------------------------------------------------------------------
*/
BOOL APPL_MustQuit(){
   BOOL fRes = FALSE;
   char bUserKey;
   if (kbhit())
   {
      bUserKey = getchar();

      if ((bUserKey == 'q') ||
            (bUserKey == 'Q'))
      {
         /*
          ** Q is for quit.
          */
         fRes = TRUE;
      }else {
         putc(bUserKey, stdin);
         fRes = FALSE;
      }
   }
   return fRes;
}

#if( USE_TIMER_INTERRUPT )
static void TimerIsr( void )
{
   ABCC_RunTimerSystem( hCtrl, APPL_TIMER_MS );
}

static void SetupTimerInterrupt( void )
{
}
#else
static void DelayMs( UINT32 lDelayMs )
{
  usleep(1000 * lDelayMs);
}
#endif

static void Reset( void )
{
}

void setElements(const std_msgs::UInt16MultiArray::ConstPtr& order)
{
   UINT16 *pc_to_plc_data;
   pc_to_plc_data = (UINT16 *)(APPL_asAdiEntryList[1].uData.sVOID.pxValuePtr);

   for (int i=0; i < order->data.size(); i++)
   {
      pc_to_plc_data[i] = (UINT16)order->data[i];
   }
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "eip_start");
   ros::NodeHandle nh;
   ros::Publisher pub = nh.advertise<std_msgs::UInt16MultiArray>("hand_state", 1);
   ros::Subscriber sub = nh.subscribe("hand_order", 1, setElements);
   ros::Rate loop_rate(10);

   /*
   ** Make sure the ABCC reset signal is kept low by the platform specific
   ** initialization to keep the ABCC module in reset until the driver releases
   ** it.
   */
   APPL_AbccHandlerStatusType eAbccHandlerStatus = APPL_MODULE_NO_ERROR;

   UINT16 *plc_to_pc_data;

   if(!ABCC_OpenController(0, NULL, &hCtrl)){
      printf("ABCC_OpenController failed\n");
      return 0;
   }
   if( ABCC_HwInit(hCtrl) != ABCC_EC_NO_ERROR )
   {
      return( 0 );
   }

#if( USE_TIMER_INTERRUPT )
   SetupTimerInterrupt();
#endif
   while( ros::ok() )
   {
      if ( eAbccHandlerStatus == APPL_MODULE_NO_ERROR )
      {
         eAbccHandlerStatus = APPL_HandleAbcc(hCtrl);
         if(APPL_MustQuit())
            APPL_Shutdown(hCtrl);

         #if( !USE_TIMER_INTERRUPT )
               ABCC_RunTimerSystem( hCtrl, APPL_TIMER_MS );
               DelayMs( APPL_TIMER_MS );
         #endif
         switch( eAbccHandlerStatus )
         {
         case APPL_MODULE_RESET:
            Reset();
            break;
         default:
            break;
         }

         plc_to_pc_data = (UINT16 *)(APPL_asAdiEntryList[0].uData.sVOID.pxValuePtr);

         std_msgs::UInt16MultiArray input;
         for (int i=0; i < (UINT16)(APPL_asAdiEntryList[0].bNumOfElements); i++)
         {
            input.data.push_back(plc_to_pc_data[i]);
         }
         pub.publish(input);
         ros::spinOnce();
      }
   }

   ABCC_CloseController(&hCtrl);

   return( 0 );
}
