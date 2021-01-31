#include <mhand_control/mhand_joint_state_republisher.h>

using mhand_joint_state_republisher::MHandJointStateRePublisher;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mhand_joint_state_republisher");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  MHandJointStateRePublisher mhand_joint_state_republisher(nh);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  spinner.stop();

  return 0;
}