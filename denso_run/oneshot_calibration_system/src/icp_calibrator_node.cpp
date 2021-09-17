#include <oneshot_calibration_system/icp_calibrator.h>

using icp_calibrator::ICPcalibrator;
using icp_calibrator::ICPcalibParam;

const int ICPcalibParam::LOOP_RATE_TIME = 10;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp_calibrator");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  ICPcalibrator calibrator(nh, n);
  ros::Rate loop_rate(ICPcalibParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    if (calibrator.flag)
    {
      calibrator.publishICPResult();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
