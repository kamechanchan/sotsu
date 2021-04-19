#include <denso_state_behavior/measurement/measurement_behavior.h>

using measurement_behavior::MeasurementBehavior;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "measurement_behavior_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate loop_rate(1);

  MeasurementBehavior measurement_behavior(nh);

  while(ros::ok())
  {
    measurement_behavior.broadcastWorkPoint();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
