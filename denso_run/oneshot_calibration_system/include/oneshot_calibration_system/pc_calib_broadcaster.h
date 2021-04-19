#ifndef PC_CALIB_BROADCASTER_H
#define PC_CALIB_BROADCASTER_H

#include <cstdlib>

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

namespace pc_calib_broadcaster
{
class PCCalibBroadcaster
{
public:
  PCCalibBroadcaster(ros::NodeHandle& nh, ros::NodeHandle& n);
  void broadcastTF(void);

private:
  void sendSetting(const geometry_msgs::Transform trans);

private:
  ros::NodeHandle nh_;
  std::string target_frame_;
  std::string source_frame_;
  std::string node_name_;
  ros::Subscriber trans_msgs_sub_;
  tf::TransformBroadcaster br_;
  tf::Transform output_tf_;
  bool flag_;
};
struct CalibParam
{
  static const int SUBSCRIBER_QUEUE_SIZE;
  static const int LOOP_RATE_TIME;
};
}  // PCCalibBroadcaster

#endif  // PC_CALIB_BROADCASTER_H
