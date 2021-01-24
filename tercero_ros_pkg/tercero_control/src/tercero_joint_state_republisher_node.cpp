#include <tercero_control/tercero_joint_state_republisher.h>

using tercero_joint_state_republisher::TerceroJointStateRePublisher;
using tercero_joint_state_republisher::HandStateParam;
using tercero_joint_state_republisher::TerceroParam;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tercero_joint_state_republisher");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  TerceroJointStateRePublisher tercero_joint_state_republisher(nh);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  spinner.stop();

  return 0;
}