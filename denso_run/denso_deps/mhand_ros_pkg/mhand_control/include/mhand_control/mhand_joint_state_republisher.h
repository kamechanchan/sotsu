#ifndef MHAND_JOINT_STATE_REPUBLISHER_H
#define MHAND_JOINT_STATE_REPUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
#include <cmath>

namespace mhand_joint_state_republisher
{
class MHandJointStateRePublisher
{
public:
  MHandJointStateRePublisher(ros::NodeHandle& nh);

private:
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr& js);

private:
  ros::NodeHandle nh_;
  ros::Subscriber js_sub_;
  ros::Publisher js_pub_;
};
}  // namespace mhand_joint_state_republisher

#endif  // MHAND_JOINT_STATE_REPUBLISHER_H
