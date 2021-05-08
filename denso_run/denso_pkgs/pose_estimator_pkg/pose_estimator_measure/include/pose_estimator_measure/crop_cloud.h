#ifndef CROP_CLOUD_H
#define CROP_CLOUD_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/io/io.h>


namespace crop_cloud
{
  class CropCloud
  {
    public:
      CropCloud(ros::NodeHandle nh, ros::NodeHandle n);
      void cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
      void cropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max);

    private:
      ros::NodeHandle nh_;
      ros::Rate rate_;
      std::string frame_id_;
      std::string sensor_frame_id_;
      ros::Publisher filtered_cloud_pub_;
      ros::Subscriber source_pc_sub_;
      tf::TransformListener tf_;
      pcl::PointXYZ crop_min_, crop_max_;
  };
}  // namespace crop_cloud

#endif // CROP_CLOUD_H
