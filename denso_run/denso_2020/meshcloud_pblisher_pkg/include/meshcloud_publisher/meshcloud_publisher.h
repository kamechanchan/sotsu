#ifndef MESHCLOUD_PUBLISHER_H
#define MESHCLOUD_PUBLISHER_H

#include <meshcloud_publisher/meshcloud_sampler.h>

#include <pcl_ros/impl/transforms.hpp>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

namespace meshcloud_publisher
{
class MeshCloudPublisher
{
public:
  MeshCloudPublisher(ros::NodeHandle& nh);
  ~MeshCloudPublisher();

  void getMesh(const std::string dir_path);
  void transformMesh();
  void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
  void publishCloud();
  void broadcastTF();
  bool loadMeshInfoFromYAML(const std::string& file_path);
  bool setMeshCloud();

private:
  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster br_;
  std::vector<geometry_msgs::TransformStamped> ts_;
  sensor_msgs::PointCloud2 mesh_pointcloud_ros_;
  sensor_msgs::PointCloud2 merged_pointcloud_ros_;
  ros::Publisher mesh_pointcloud_publisher_;

  pcl::PointCloud<pcl::PointXYZ> mesh_pointcloud_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_mesh_pointcloud_ptr_pcl_;

  std::vector<pcl::PolygonMesh> meshes_;
  std::vector<std::string> model_names_;
  std::vector<std::string> base_frames_;
  std::vector<std::string> mesh_pkgs_;
  std::vector<std::string> to_mesh_paths_;
  std::vector<std::string> mesh_names_;
  std::vector<double> meshes_x_;
  std::vector<double> meshes_y_;
  std::vector<double> meshes_z_;
  std::vector<double> meshes_qx_;
  std::vector<double> meshes_qy_;
  std::vector<double> meshes_qz_;
  std::vector<double> meshes_qw_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> parts_clouds_;

  double leaf_size_;
  int sampling_points_;
  int model_number_;
  std::string node_id_;
  std::string yaml_file_;
  std::string yaml_path_;
  std::string mesh_pc_parent_frame_;
};
}  // namespace meshcloud_publisher

#endif  // MESHCLOUD_PUBLISHER_H
