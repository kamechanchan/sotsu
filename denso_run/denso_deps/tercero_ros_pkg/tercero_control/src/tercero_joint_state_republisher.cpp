#include <tercero_control/tercero_joint_state_republisher.h>

using tercero_joint_state_republisher::TerceroJointStateRePublisher;
using tercero_joint_state_republisher::HandStateParam;
using tercero_joint_state_republisher::TerceroParam;

const int HandStateParam::STATE_DATA_SIZE = 24;
const int HandStateParam::STATE_CHACK_READY_CH = 0;
const int HandStateParam::STATE_CHACK_MOVE_CH = 1;
const int HandStateParam::STATE_CHACK_POSITION_CH = 2;
const int HandStateParam::STATE_PUSHER_READY_CH = 16;
const int HandStateParam::STATE_PUSHER_MOVE_CH = 17;
const int HandStateParam::STATE_PUSHER_POSITION_CH = 18;

const int HandStateParam::STATE_READY_ERROR = 0;
const int HandStateParam::STATE_MOVE_OPEN = 1;
const int HandStateParam::STATE_MOVE_CLOSE = 2;
const int HandStateParam::STATE_MOVE_UP = 1;
const int HandStateParam::STATE_MOVE_DOWN = 2;

const double TerceroParam::MIN_CHACK_POSITION = 0.0;
const double TerceroParam::MIN_PUSHER_POSITION = 0.0;

const double TerceroParam::MM_COEFFICIENT = 1 / 10.0;
const double TerceroParam::MM2M = 1 / 1000.0;

TerceroJointStateRePublisher::TerceroJointStateRePublisher(ros::NodeHandle& nh) : nh_(nh)
{
  js_pub_ = nh.advertise<sensor_msgs::JointState>(
      nh.param<std::string>("joint_state_republish_topic_name", "/joint_states_republish"), 1);
  js_sub_ =
      nh.subscribe<sensor_msgs::JointState>(nh.param<std::string>("joint_state_subscribe_topic_name", "/joint_states"),
                                            10, &TerceroJointStateRePublisher::jointstateCallback, this);
  hand_state_sub_ = nh.subscribe<std_msgs::UInt16MultiArray>("/hand_state", 1,
                                                             &TerceroJointStateRePublisher::handstateCallback, this);
  hand_state_.clear();
}

double TerceroJointStateRePublisher::getChackJointPosition()
{
  if (hand_state_.size() < HandStateParam::STATE_DATA_SIZE)
  {
    return TerceroParam::MIN_CHACK_POSITION;
  }
  else if (hand_state_[HandStateParam::STATE_CHACK_READY_CH] == HandStateParam::STATE_READY_ERROR)
  {
    return TerceroParam::MIN_CHACK_POSITION;
  }
  else if (!(hand_state_[HandStateParam::STATE_CHACK_MOVE_CH] == HandStateParam::STATE_MOVE_OPEN ||
             hand_state_[HandStateParam::STATE_CHACK_MOVE_CH] == HandStateParam::STATE_MOVE_CLOSE))
  {
    return TerceroParam::MIN_CHACK_POSITION;
  }
  else
  {
    return (static_cast<double>(hand_state_[HandStateParam::STATE_CHACK_POSITION_CH]) * TerceroParam::MM_COEFFICIENT) *
           TerceroParam::MM2M;
  }
}

double TerceroJointStateRePublisher::getPusherJointPosition()
{
  if (hand_state_.size() < HandStateParam::STATE_DATA_SIZE)
  {
    return TerceroParam::MIN_PUSHER_POSITION;
  }
  else if (hand_state_[HandStateParam::STATE_PUSHER_READY_CH] == HandStateParam::STATE_READY_ERROR)
  {
    return TerceroParam::MIN_PUSHER_POSITION;
  }
  else if (!(hand_state_[HandStateParam::STATE_PUSHER_MOVE_CH] == HandStateParam::STATE_MOVE_UP ||
             hand_state_[HandStateParam::STATE_PUSHER_MOVE_CH] == HandStateParam::STATE_MOVE_DOWN))
  {
    return TerceroParam::MIN_PUSHER_POSITION;
  }
  else
  {
    return (static_cast<double>(hand_state_[HandStateParam::STATE_PUSHER_POSITION_CH]) * TerceroParam::MM_COEFFICIENT) *
           TerceroParam::MM2M;
  }
}

void TerceroJointStateRePublisher::handstateCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
  hand_state_ = msg->data;
}

void TerceroJointStateRePublisher::jointstateCallback(const sensor_msgs::JointState::ConstPtr& js)
{
  sensor_msgs::JointState joint_state;
  joint_state.header = js->header;
  joint_state.name = js->name;
  joint_state.position = js->position;
  joint_state.velocity = js->velocity;
  joint_state.effort = js->effort;

  for (int i = 0; i < joint_state.name.size(); i++)
  {
    if (joint_state.name[i] == "finger_3rd_joint")
    {
      joint_state.position[i] = getPusherJointPosition();
    }
    else if (joint_state.name[i] == "finger_R_joint" || joint_state.name[i] == "finger_L_joint")
    {
      joint_state.position[i] = getChackJointPosition();
    }
    else
    {
      continue;
    }

    joint_state.velocity[i] = 0.0;
    joint_state.effort[i] = 0.0;
  }

  js_pub_.publish(joint_state);
}
