#include <pointcloud_processing/downsampling.h>

using downsampling::Downsampling;

Downsampling::Downsampling(ros::NodeHandle& nh, ros::NodeHandle& n)
  : nh_(nh)
  , downsample_pc_src_(n.param<std::string>("downsample_pc_src", "/cropped_cloud"))
  , downsample_pc_dst_(n.param<std::string>("downsample_pc_dst", "/downsampled_cloud"))
  , downsample_frame_id_(n.param<std::string>("downsample_frame_id", "world"))
  , downsample_leafsize_x_(n.param<float>("downsample_leafsize_x", 0.02))
  , downsample_leafsize_y_(n.param<float>("downsample_leafsize_y", 0.02))
  , downsample_leafsize_z_(n.param<float>("downsample_leafsize_z", 0.02))
{
  pcl_sub_ = nh_.subscribe(downsample_pc_src_, 10, &Downsampling::cloudCB, this);
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(downsample_pc_dst_, 1);
  std::cout << "[downsampling] init done" << std::endl;
}

void Downsampling::cloudCB(const sensor_msgs::PointCloud2& input)
{
  std::cout << "[downsampling] receive point cloud" << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
  sensor_msgs::PointCloud2 output;

  pcl::fromROSMsg(input, cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
  voxelSampler.setInputCloud(cloud.makeShared());
  voxelSampler.setLeafSize(downsample_leafsize_x_, downsample_leafsize_y_, downsample_leafsize_z_);
  voxelSampler.filter(cloud_downsampled);

  pcl::toROSMsg(cloud_downsampled, output);
  output.header.frame_id = downsample_frame_id_;
  pcl_pub_.publish(output);
}
