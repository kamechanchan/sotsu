#ifndef TRUE_AR_CENTER_BROADCASTER_H
#define TRUE_AR_CENTER_BROADCASTER_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen3/Eigen/Eigenvalues>

namespace true_ar_center_broadcaster
{
class TrueARCenterBroadcaster
{
public:
  TrueARCenterBroadcaster(ros::NodeHandle& nh);

  void publish(void);
  void broadcast(void);

private:
  bool getTransformInfoFromYAML(std::string marker, geometry_msgs::TransformStamped& trans);
  void getTransformFromYAML(std::string marker, geometry_msgs::TransformStamped& trans);
  void updateTrueARTransformArray(void);
  void updateCenterTransform(void);

private:
  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster br_;

  std::vector<geometry_msgs::TransformStamped> true_ar_transform_vector_;
  geometry_msgs::TransformStamped true_ar_center_transform_;
};
struct BroadcasterParam
{
  static const int LOOP_RATE_TIME;
  static const int MATRIX_SIZE;
};
}  // namespace true_ar_center_broadcaster

#endif  // TRUE_AR_CENTER_BROADCASTER_H
