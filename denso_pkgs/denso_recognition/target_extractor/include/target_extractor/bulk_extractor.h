#ifndef BULK_EXTRACTOR_H
#define BULK_EXTRACTOR_H

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>

namespace bulk_extractor
{
class BulkExtractor
{
public:
  BulkExtractor(ros::NodeHandle& nh, ros::NodeHandle& n);
  void publish(void);
  void updateBaseCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void loadDiffCloud(void);

private:
  void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
                       const double leaf_size);                             // ダウンサンプリング
  void segmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);  // 平面除去
  void removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);   // 外れ値除去
  void clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);      // 一番大きいクラスタを抽出
  void extractDifference(pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud);
  void getCropCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
  void cropBulk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

private:
  ros::NodeHandle nh_;
  ros::Publisher extracted_cloud_pub_;
  ros::Subscriber base_cloud_sub_;
  ros::Subscriber diff_cloud_sub_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_;       // 被差分点群
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud_;       // 差分点群
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud_;  // 出力点群

  bool base_flag_;
  bool diff_flag_;
  bool crop_flag_;
  double octree_resolution_;
  double leaf_size_;
  double cluster_tolerance_;
  double cluster_min_size_;
  double cluster_max_size_;
  Eigen::Vector3f crop_translation_;
  Eigen::Vector3f crop_rotation_;
  Eigen::Vector4f min_point_;
  Eigen::Vector4f max_point_;

  std::string diff_filename_;
};
}  // namespace bulk_extractor

#endif  // BULK_EXTRACTOR_H
