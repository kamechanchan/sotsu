#include <oneshot_calibration_system/hand_ar_oneshot_calibrator.h>

using hand_ar_oneshot_calibrator::HandAROneshotCalibrator;
using hand_ar_oneshot_calibrator::CalibParam;

const int CalibParam::LOOP_RATE_TIME = 10;

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_oneshot_calibration");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  HandAROneshotCalibrator calibrator(nh, n);

  ros::Rate loop_rate(CalibParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    calibrator.broadcastTF();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
