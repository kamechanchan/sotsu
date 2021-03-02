#ifndef HAND_AR_ONESHOT_CALIBRATOR_H
#define HAND_AR_ONESHOT_CALIBRATOR_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>


namespace hand_ar_oneshot_calibrator
{
class HandAROneshotCalibrator
{
public:
  HandAROneshotCalibrator(ros::NodeHandle& nh, ros::NodeHandle& n);
  void lookupTransform(geometry_msgs::TransformStamped& transform);
  void calibrateSensor(const sensor_msgs::Image::ConstPtr& img);
  void getEulerRPY(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw);
  void getQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q);
  void broadcastTF(void);
  cv::Mat generateRotationMatrix(double roll, double pitch, double yaw);


private:
  ros::NodeHandle nh_;
  std::string target_frame_;
  std::string source_frame_;
  std::string ar_frame_;
  std::string base_frame_;
  bool flag_;
  ros::Subscriber texture_sub_;
  geometry_msgs::TransformStamped fixed_photoneo_center_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped ar_to_phoxi_;
  geometry_msgs::TransformStamped base_link_to_true_ar_;
};
struct CalibParam
{
  static const float DURATION_TIME;
  static const int START_TIME;
  static const int PUBLISHER_QUEUE_SIZE;
  static const int SUBSCRIBER_QUEUE_SIZE;
  static const int LOOP_RATE_TIME;
};
}  // namespace hand_ar_oneshot_calibrator

#endif
