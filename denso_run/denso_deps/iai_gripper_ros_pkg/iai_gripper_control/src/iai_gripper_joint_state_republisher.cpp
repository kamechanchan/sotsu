#include <iai_gripper_control/iai_gripper_joint_state_republisher.h>

using iai_gripper_joint_state_republisher::IAIGripperJointStateRePublisher;

IAIGripperJointStateRePublisher::IAIGripperJointStateRePublisher(ros::NodeHandle& nh) : nh_(nh)
{
  js_pub_ = nh.advertise<sensor_msgs::JointState>(
      nh.param<std::string>("joint_state_republish_topic_name", "/joint_states_republish"), 1);
  js_sub_ =
      nh.subscribe<sensor_msgs::JointState>(nh.param<std::string>("joint_state_subscribe_topic_name", "/joint_states"),
                                            10, &IAIGripperJointStateRePublisher::jointstateCallback, this);
}

void IAIGripperJointStateRePublisher::jointstateCallback(const sensor_msgs::JointState::ConstPtr& js)
{
  sensor_msgs::JointState joint_state;
  joint_state.header = js->header;
  joint_state.name = js->name;
  joint_state.position = js->position;
  joint_state.velocity = js->velocity;
  joint_state.effort = js->effort;

  js_pub_.publish(joint_state);
}
