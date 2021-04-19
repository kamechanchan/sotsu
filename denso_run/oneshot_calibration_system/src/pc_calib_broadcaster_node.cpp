#include <oneshot_calibration_system/pc_calib_broadcaster.h>

using pc_calib_broadcaster::PCCalibBroadcaster;
using pc_calib_broadcaster::CalibParam;

const int CalibParam::LOOP_RATE_TIME = 10;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pc_calib_broadcaster");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  PCCalibBroadcaster broadcaster(nh, n);
  ros::Rate loop_rate(CalibParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    broadcaster.broadcastTF();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
