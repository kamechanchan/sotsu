#ifndef MESHCLOUD_PUBLISHER_H
#define MESHCLOUD_PUBLISHER_H

#include <meshcloud_publisher/meshcloud_sampler.h>

#include <pcl_ros/impl/transforms.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace meshcloud_publisher
{
class MeshCloudPublisher
{
public:
  MeshCloudPublisher(ros::NodeHandle& nh);
  ~MeshCloudPublisher()
  {
  }

  void getMesh(const std::string dir_path);
  void transformMesh();
  void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
  void publishCloud();

private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  sensor_msgs::PointCloud2 mesh_pointcloud_ros_;
  ros::Publisher mesh_pointcloud_publisher_;

  pcl::PointCloud<pcl::PointXYZ> mesh_pointcloud_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_mesh_pointcloud_ptr_pcl_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>> parts_clouds_;
  std::vector<pcl::PolygonMesh> meshes_;
  std::vector<std::string> link_names_;
  std::vector<std::string> frame_names_;
};
struct MeshCloudParam
{
  static const int RETRY_COUNT_LIMIT;
  static const float LEAF_SIZE;
  static const float DURATION_TIME;
};
}  // namespace meshcloud_publisher

#endif  // MESHCLOUD_PUBLISHER_H
