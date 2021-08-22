#ifndef BAYES_REGISTRATOR_H
#define BAYES_REGISTRATOR_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <bayesopt/bayesopt.hpp>
#include <bayesopt/parameters.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/math/constants/constants.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

typedef boost::numeric::ublas::vector<double> vectord;

namespace bayes_pose
{
class PositionOptimizer : public bayesopt::ContinuousModel
{
public:
  PositionOptimizer(bayesopt::Parameters params);
  void setInputClouds(const sensor_msgs::PointCloud2& compared_pc, const sensor_msgs::PointCloud2& reference_pc);
  void setInputClouds(const pcl::PointCloud<pcl::PointXYZ>& compared_pc,
                      const pcl::PointCloud<pcl::PointXYZ>& reference_pc);
  double evaluateSample(const vectord& xin);
  bool checkReachability(const vectord& query);
  void applyTranslation(const double x, const double y, const double z);
  void applyTranslation(const double x, const double y, const double z, Eigen::Affine3d& matrix);
  double calcC2CDistance();
  pcl::PointCloud<pcl::PointXYZ> getOptimumRefCloud(const vectord& opt_param);

public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr compared_pc_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_pcl_;
};

class OrientationOptimizer : public bayesopt::ContinuousModel
{
public:
  OrientationOptimizer(bayesopt::Parameters params);
  void setInputClouds(const sensor_msgs::PointCloud2& compared_pc, const sensor_msgs::PointCloud2& reference_pc);
  void setInputClouds(const pcl::PointCloud<pcl::PointXYZ>& compared_pc,
                      const pcl::PointCloud<pcl::PointXYZ>& reference_pc);
  double evaluateSample(const vectord& xin);
  bool checkReachability(const vectord& query);
  void applyRotation(const double roll, const double pitch, const double yaw);
  void applyRotation(const double roll, const double pitch, const double yaw, Eigen::Affine3d& matrix);
  double calcC2CDistance();
  pcl::PointCloud<pcl::PointXYZ> getOptimumRefCloud(const vectord& opt_param);

public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr compared_pc_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_pcl_;
};

class PostureOptimizer : public bayesopt::ContinuousModel
{
public:
  PostureOptimizer(bayesopt::Parameters params);
  void setInputClouds(const sensor_msgs::PointCloud2& compared_pc, const sensor_msgs::PointCloud2& reference_pc);
  void setInputClouds(const pcl::PointCloud<pcl::PointXYZ>& compared_pc,
                      const pcl::PointCloud<pcl::PointXYZ>& reference_pc);
  double evaluateSample(const vectord& xin);
  bool checkReachability(const vectord& query);
  void applyTransform(const double x, const double y, const double z, const double roll, const double pitch,
                      const double yaw);
  double calcC2CDistance();

public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr compared_pc_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_pcl_;
};

class BayesRegistrator
{
public:
  BayesRegistrator(ros::NodeHandle nh, const std::string& compared_pc_topic_name,
                   const std::string& reference_pc_topic_name, const std::string& registrated_pc_topic_name,
                   bayesopt::Parameters pos_params, bayesopt::Parameters ori_params,
                   bayesopt::Parameters posture_params);
  void referenceCB(const sensor_msgs::PointCloud2ConstPtr& reference_pc);
  void registrate(const sensor_msgs::PointCloud2ConstPtr& compared_pc);
  void applyTranslation(const double x, const double y, const double z,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_pc);
  void applyRotation(const double roll, const double pitch, const double yaw,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_pc);
  void applyTransform(const double x, const double y, const double z, const double roll, const double pitch,
                      const double yaw, const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_pc);
  void publish();
  void broadcast_tf();
  void lookupInitTransform();

protected:
  ros::NodeHandle nh_;
  bool is_init_;
  bool is_tf_init_;
  bool is_ready_;
  bool stop_estimate_trigger_;
  sensor_msgs::PointCloud2 reference_pc_;
  sensor_msgs::PointCloud2 compared_pc_;
  sensor_msgs::PointCloud2 registrated_pc_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registrated_pc_pcl_;
  ros::Publisher registrated_pc_pub_;
  ros::Subscriber reference_pc_sub_;
  ros::Subscriber compared_pc_sub_;
  PositionOptimizer pos_opt_;
  OrientationOptimizer ori_opt_;
  PostureOptimizer posture_opt_;
  std::string source_frame_name_;
  std::string reference_frame_name_;
  Eigen::Affine3d output_affine3d_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster br_;
};

}  // namespace bayes_pose

#endif  // BAYES_REGISTRATOR_H
