#ifndef IAI_GRIPPER_JOINT_STATE_REPUBLISHER_H
#define IAI_GRIPPER_JOINT_STATE_REPUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
#include <cmath>

namespace iai_gripper_joint_state_republisher
{
class IAIGripperJointStateRePublisher
{
public:
  IAIGripperJointStateRePublisher(ros::NodeHandle& nh);

private:
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr& js);

private:
  ros::NodeHandle nh_;
  ros::Subscriber js_sub_;
  ros::Publisher js_pub_;
};
}  // namespace iai_gripper_joint_state_republisher

#endif  // IAI_GRIPPER_JOINT_STATE_REPUBLISHER_H
