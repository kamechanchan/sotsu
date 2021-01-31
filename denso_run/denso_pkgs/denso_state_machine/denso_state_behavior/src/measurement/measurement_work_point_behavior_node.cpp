#include <denso_state_behavior/measurement/measurement_work_point_behavior.h>

using measurement_work_point_behavior::MeasurementWorkPointBehavior;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "measurement_work_point_behavior_node");
  ros::NodeHandle nh;

  MeasurementWorkPointBehavior measurement_work_point_behavior(nh);

  while(ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}