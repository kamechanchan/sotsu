#ifndef POINTCLOUD_REGISTRATOR_H
#define POINTCLOUD_REGISTRATOR_H

#include <ros/ros.h>
//#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/registration/icp.h>

#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl_conversions/pcl_conversions.h>

namespace pointcloud_registrator
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
    MySyncPolicy;

class PointCloudRegistrator
{
public:
  PointCloudRegistrator(ros::NodeHandle& nh);
  ~PointCloudRegistrator();
  void runICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_mesh_pcl,
              const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sensor_pcl);
  void sensorAndMeshPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_mesh_ros,
                                       const sensor_msgs::PointCloud2::ConstPtr& pc_sensor_ros);

private:
  ros::NodeHandle nh_;
  ros::Publisher icp_data_pub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud1_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud2_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync_;
};
}  // pointcloud_registrator

#endif  // POINTCLOUD_REGISTRATOR_H
