#include <oneshot_calibration_system/ar_center_broadcaster.h>

using ar_center_broadcaster::ARCenterBroadcaster;
using ar_center_broadcaster::BroadcasterParam;

const int BroadcasterParam::LOOP_RATE_TIME = 10;

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_center_broadcatser");
  ros::NodeHandle nh;

  ARCenterBroadcaster broadcaster(nh);

  ros::Rate loop_rate(BroadcasterParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    broadcaster.broadcast();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
