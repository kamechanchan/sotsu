#ifndef ICP_CALIBRATOR_H
#define ICP_CALIBRATOR_H

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

namespace icp_calibrator
{
class ICPcalibrator
{
public:
  ICPcalibrator(ros::NodeHandle& nh, ros::NodeHandle& n);
  void publishICPResult(void);
  bool flag;

private:
  void convertData(const std_msgs::Float64MultiArray icp_data);
  void multiArrayToMat4d(const std_msgs::Float64MultiArray& icp_array, Eigen::Matrix4d& mat4d);
  void lookupTransform(tf::StampedTransform& transform);

private:
  ros::NodeHandle nh_;
  ros::Subscriber icp_result_sub_;
  ros::Publisher icp_calib_pub_;
  std::string target_frame_;
  std::string source_frame_;
  tf2_ros::Buffer tfBuffer_;
  tf::TransformListener tf_;
  geometry_msgs::Transform base_link_to_photoneo_;
};
struct ICPcalibParam
{
  static const int PUBLISHER_QUEUE_SIZE;
  static const int SUBSCRIBER_QUEUE_SIZE;
  static const int LOOP_RATE_TIME;
};
}  // icp_calibrator

#endif  // ICP_CALIBRATOR_H
