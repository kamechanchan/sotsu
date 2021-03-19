#ifndef AR_CENTER_BROADCASTER_H
#define AR_CENTER_BROADCASTER_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace ar_center_broadcaster
{
class ARCenterBroadcaster
{
public:
  ARCenterBroadcaster(ros::NodeHandle& nh);

  void broadcast(void);

private:
  void updateARCenterTransform(void);
  void updateARTransformVector(void);
  bool getTransformInfoFromYAML(std::string marker, geometry_msgs::TransformStamped& transform);
  bool lookupTransform(geometry_msgs::TransformStamped& transform, const ros::Time& time);

private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster br_;

  std::vector<geometry_msgs::TransformStamped> ar_transform_vector_;
  geometry_msgs::TransformStamped ar_center_transform_;
};
struct BroadcasterParam
{
  static const int LOOP_RATE_TIME;
  static const int MATRIX_SIZE;
  static const int START_TIME;
  static const float DURATION_TIME;
};
}  // namespace ar_center_broadcaster

#endif  // AR_CENTER_BROADCASTER_H
