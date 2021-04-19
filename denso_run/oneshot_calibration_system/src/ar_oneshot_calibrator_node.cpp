#include <oneshot_calibration_system/ar_oneshot_calibrator.h>

using ar_oneshot_calibrator::AROneshotCalibrator;
using ar_oneshot_calibrator::ARCalibParam;

const int ARCalibParam::LOOP_RATE_TIME = 10;

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "oneshot_calibration");
  ros::NodeHandle nh;

  AROneshotCalibrator calibrator(nh);

  ros::Rate loop_rate(ARCalibParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    calibrator.broadcast();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
