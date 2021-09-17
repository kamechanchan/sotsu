#ifndef ICP_REGISTRATOR_H
#define ICP_REGISTRATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

namespace bayes_pose
{
class ICPRegistrator
{
public:
  ICPRegistrator(ros::NodeHandle nh, const std::string& compared_pc_topic_name,
                 const std::string& reference_pc_topic_name, const std::string& registrated_pc_topic_name);
  void referenceCB(const sensor_msgs::PointCloud2ConstPtr& reference_pc);
  void registrate(const sensor_msgs::PointCloud2ConstPtr& compared_pc);
  void publish();
  void broadcast_tf();
  void lookupInitTransform();

protected:
  ros::NodeHandle nh_;
  bool is_init_;
  bool is_ready_;
  sensor_msgs::PointCloud2 reference_pc_;
  sensor_msgs::PointCloud2 registrated_pc_;
  ros::Publisher registrated_pc_pub_;
  ros::Subscriber reference_pc_sub_;
  ros::Subscriber compared_pc_sub_;
  std::string source_frame_name_;
  std::string reference_frame_name_;
  Eigen::Affine3d output_affine3d_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster br_;
};
}  // namespace bayes_pose

#endif  // ICP_REGISTRATOR_H
