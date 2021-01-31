#include <target_extractor/set_initial_cloud.h>

using set_initial_cloud::SetInitialCloud;

// Class methods definitions
SetInitialCloud::SetInitialCloud(ros::NodeHandle& nh, ros::NodeHandle& n)
  : nh_(nh)
  , initial_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  , leaf_size_ (0.002)
  , cloud_filename_ (n.param<std::string>("cloud_filename", "diff_cloud.pcd"))
{
  cloud_sub_ = nh.subscribe(n.param<std::string>("initial_pc_topic_name", "/photoneo/pointcloud"), 1, &SetInitialCloud::saveCloud, this);
}

void SetInitialCloud::saveCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *initial_cloud_);
  std::string cloud_filepath = ros::package::getPath("target_extractor") + "/pcd/";
  pcl::io::savePCDFileASCII (cloud_filepath + cloud_filename_, *initial_cloud_);
  // std::cout << __LINE__ << std::endl;
}


