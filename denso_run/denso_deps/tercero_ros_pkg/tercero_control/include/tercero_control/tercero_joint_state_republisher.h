#ifndef TERCERO_JOINT_STATE_REPUBLISHER_H
#define TERCERO_JOINT_STATE_REPUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt16MultiArray.h>
#include <iostream>
#include <vector>
#include <cmath>

namespace tercero_joint_state_republisher
{
class TerceroJointStateRePublisher
{
public:
  TerceroJointStateRePublisher(ros::NodeHandle& nh);

private:
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr& js);
  void handstateCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg);
  double getChackJointPosition();
  double getPusherJointPosition();

private:
  ros::NodeHandle nh_;
  ros::Subscriber js_sub_;
  ros::Subscriber hand_state_sub_;
  ros::Publisher js_pub_;
  std::vector<uint16_t> hand_state_;
};

struct HandStateParam
{
  static const int STATE_DATA_SIZE;
  static const int STATE_CHACK_READY_CH;
  static const int STATE_CHACK_MOVE_CH;
  static const int STATE_CHACK_POSITION_CH;
  static const int STATE_PUSHER_READY_CH;
  static const int STATE_PUSHER_MOVE_CH;
  static const int STATE_PUSHER_POSITION_CH;

  static const int STATE_READY_ERROR;
  static const int STATE_MOVE_OPEN;
  static const int STATE_MOVE_CLOSE;
  static const int STATE_MOVE_UP;
  static const int STATE_MOVE_DOWN;
};

struct TerceroParam
{
  static const double MIN_CHACK_POSITION;
  static const double MIN_PUSHER_POSITION;

  static const double MM_COEFFICIENT;
  static const double MM2M;
};
}  // namespace tercero_joint_state_repubisher

#endif  // TERCERO_JOINT_STATE_REPUBLISHER_H
