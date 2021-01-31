#ifndef TOP_PART_EXTRACTOR_H
#define TOP_PART_EXTRACTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

namespace top_part_extractor
{
class TopPartExtractor
{
public:
  TopPartExtractor(ros::NodeHandle& nh);
  void publish(void);

private:
  void updatePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
                       const double leaf_size);  // ダウンサンプリング
  geometry_msgs::Vector3 getHighestPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractCloud(const geometry_msgs::Vector3 point, const int radius);

private:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_;
  tf::StampedTransform transform_;
  tf::TransformListener listener_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bulk_cloud_;  // 入力点群
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr part_cloud_;  // 出力点群

  bool flag_;
  double leaf_size_;
};
}  // namespace top_part_extractor

#endif  // TOP_PART_EXTRACTOR_H
