#include <iai_gripper_control/iai_gripper_joint_state_republisher.h>

using iai_gripper_joint_state_republisher::IAIGripperJointStateRePublisher;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "iai_gripper_joint_state_republisher");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  IAIGripperJointStateRePublisher iai_gripper_joint_state_republisher(nh);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  spinner.stop();

  return 0;
}