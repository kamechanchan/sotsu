#include <pointcloud_processing/filter.h>

using stat_outlier_removal::Filter;

Filter::Filter(ros::NodeHandle& nh, ros::NodeHandle& n)
  : nh_(nh)
  , filter_pc_src_(n.param<std::string>("filter_pc_src", "/downsampled_cloud"))
  , filter_pc_dst_(n.param<std::string>("filter_pc_dst", "/filtered_cloud"))
  , filter_frame_id_(n.param<std::string>("filter_frame_id", "world"))
  , filter_mean_k_(n.param<int>("filter_mean_k", 10))
  , filter_stddev_mul_thresh_(n.param<float>("filter_stddev_mul_thresh", 0.2))
{
  pcl_sub_ = nh.subscribe(filter_pc_src_, 10, &Filter::cloudCB, this);
  pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>(filter_pc_dst_, 1);
  std::cout << "[filter] init done" << std::endl;
}

void Filter::cloudCB(const sensor_msgs::PointCloud2& input)
{
  std::cout << "[filter] receive point cloud" << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  sensor_msgs::PointCloud2 output;

  pcl::fromROSMsg(input, cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
  statFilter.setInputCloud(cloud.makeShared());
  statFilter.setMeanK(filter_mean_k_);
  statFilter.setStddevMulThresh(filter_stddev_mul_thresh_);
  statFilter.filter(cloud_filtered);

  pcl::toROSMsg(cloud_filtered, output);
  output.header.frame_id = filter_frame_id_;
  pcl_pub_.publish(output);
}
