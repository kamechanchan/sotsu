#include "gen_dataset/dnn_input.hpp"

#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/impl/transforms.hpp>


using dnn_input::DnnInput;

DnnInput::DnnInput(ros::NodeHandle nh, ros::NodeHandle n) : nh_(nh), rate_(n.param("loop_rate", 10)), voxel_n_(n.param("division_number", 50)), frame_id_(n.param<std::string>("crop_frame_id", "world")), sensor_frame_id_(n.param<std::string>("sensor_frame_id", "photoneo_center_optical_frame"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/filtered_pointcloud"), 1, &DnnInput::cropCallback, this);
  voxel_pub_ = nh_.advertise<pose_estimator_msgs::InputData>("/input_data", 1);
}

void DnnInput::cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  sensor_msgs::PointCloud2 trans_pc = *source_pc;
  pcl::PointCloud<pcl::PointXYZ> pcl_source;

  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<pcl::PointXYZ>(pcl_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr normarized_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  normarized_ptr->resize(pcl_source_ptr->points.size());

  getVoxelSize(pcl_source_ptr);

  for (size_t i = 0; i < pcl_source_ptr->points.size(); ++i)
  {
    normarized_ptr->points[i].x = (pcl_source_ptr->points[i].x - voxel_min_.x) / diff_max_;
    normarized_ptr->points[i].y = (pcl_source_ptr->points[i].y - voxel_min_.y) / diff_max_;
    normarized_ptr->points[i].z = (pcl_source_ptr->points[i].z - voxel_min_.z) / diff_max_;
  }

  voxel_data_.voxel_array.data = { 0 };
  voxel_data_.voxel_array.data.resize(voxel_n_ * voxel_n_ * voxel_n_);

  float leaf = float(1.0 / voxel_n_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  downsampleCloud(normarized_ptr, output, leaf);
  //  //octree構造のcloudからvoxelの重心を求める（失敗）
  //  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree(leaf);
  //  octree.setInputCloud(normarized_ptr);
  //  octree.defineBoundingBox();
  //  octree.addPointsFromInputCloud();
  //  pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector centroids;
  //  octree.getVoxelCentroids(centroids);
  //
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new
  //  pcl::PointCloud<pcl::PointXYZ>);
  //  output->points.assign(centroids.begin(), centroids.end());
  //  output->width = uint32_t(centroids.size());
  //  output->height = 1;
  //  output->is_dense = true;

  std::cout << "downsampled cloud size : " << output->points.size() << std::endl;

  voxel_data_.pose.position.x = voxel_min_.x;
  voxel_data_.pose.position.y = voxel_min_.y;
  voxel_data_.pose.position.z = voxel_min_.z;
  voxel_data_.pose.orientation.x = diff_max_;
  input_pub_data_.input_voxel.voxels.push_back(voxel_data_);

  std::cout << "piblish input data" << std::endl;
  voxel_pub_.publish(input_pub_data_);

}

void DnnInput::getVoxelSize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::getMinMax3D(*cloud, voxel_min_, voxel_max_);
  double voxel_offset_x = 0.01;
  double voxel_offset_y = 0.01;
  double voxel_offset_z = 0.01;
  voxel_min_.x = voxel_min_.x - voxel_offset_x;
  voxel_min_.y = voxel_min_.y - voxel_offset_y;
  voxel_min_.z = voxel_min_.z - voxel_offset_z;
  voxel_max_.x = voxel_max_.x + voxel_offset_x;
  voxel_max_.y = voxel_max_.y + voxel_offset_y;
  voxel_max_.z = voxel_max_.z + voxel_offset_z;
  float diff_x = voxel_max_.x - voxel_min_.x;
  float diff_y = voxel_max_.y - voxel_min_.y;
  float diff_z = voxel_max_.z - voxel_min_.z;

  std::cout << "original number of pointcloud : " << cloud->points.size() << std::endl;

  diff_max_ = diff_x;
  if (diff_max_ < diff_y)
  {
    diff_max_ = diff_y;
  }
  if (diff_max_ < diff_z)
  {
    diff_max_ = diff_z;
  }
}

void DnnInput::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output, float leaf)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf, leaf, leaf);
  //sor.setLeafSize(leaf * 2, leaf * 2, leaf * 2);
  sor.filter(*output);
  makeVoxel(output, leaf);
}



void DnnInput::makeVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf)
{
  int point_n = cloud->points.size();
  for (int i = 0; i < point_n; i++)
  {
    int voxel_x = cloud->points[i].x/leaf;
    int voxel_y = cloud->points[i].y/leaf;
    int voxel_z = cloud->points[i].z/leaf;
    voxel_data_.voxel_array.data[(voxel_n_ * voxel_n_ * voxel_z) + (voxel_n_ * voxel_y) + voxel_x] = 1;
  }

}
