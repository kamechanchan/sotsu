#include <oneshot_calibration_system/true_ar_center_broadcaster.h>

using true_ar_center_broadcaster::TrueARCenterBroadcaster;
using true_ar_center_broadcaster::BroadcasterParam;

const int BroadcasterParam::LOOP_RATE_TIME = 10;

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "true_ar_center_broadcaster");
  ros::NodeHandle nh;

  TrueARCenterBroadcaster publisher(nh);

  ros::Rate loop_rate(BroadcasterParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    publisher.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
