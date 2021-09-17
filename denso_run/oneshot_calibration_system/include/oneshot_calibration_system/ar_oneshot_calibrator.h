#ifndef AR_ONESHOT_CALIBRATOR_H
#define AR_ONESHOT_CALIBRATOR_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace ar_oneshot_calibrator
{
class AROneshotCalibrator
{
public:
  AROneshotCalibrator(ros::NodeHandle& nh);

  void broadcast(void);

private:
  void lookupTransform(geometry_msgs::TransformStamped& transform);
  void updatePhotoneoTransform(const sensor_msgs::Image::ConstPtr& img);
  void getEulerRPY(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw);
  void getQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q);

private:
  ros::NodeHandle nh_;
  ros::Subscriber texture_sub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster br_;

  geometry_msgs::TransformStamped ar_to_phoxi_;
  geometry_msgs::TransformStamped base_link_to_true_ar_;
  geometry_msgs::TransformStamped fixed_photoneo_center_;
};
struct ARCalibParam
{
  static const int LOOP_RATE_TIME;
  static const int SUBSCRIBER_QUEUE_SIZE;
};
}  // namespace oneshot_calibrator

#endif  // AR_ONESHOT_CALIBRATOR_H
