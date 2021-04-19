#ifndef CROP_CLOUD_H
#define CROP_CLOUD_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/io/io.h>
#include <denso_recognition_msgs/BoundingBoxArray.h>
#include <denso_recognition_msgs/InputCNN.h>
#include <denso_recognition_msgs/InputCNNArray.h>


namespace crop_cloud
{
  class CropCloud
  {
    public:
      CropCloud(ros::NodeHandle nh, ros::NodeHandle n);
      void cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
      void getVoxelSize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output,
          float leaf);
      void cropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max);
      void makeVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float diff);

    private:
      ros::NodeHandle nh_;
      ros::Rate rate_;
      std::string frame_id_;
      std::string sensor_frame_id_;
      ros::Publisher filtered_cloud_pub_;
      ros::Publisher voxel_pub_;
      ros::Subscriber source_pc_sub_;
      tf::TransformListener tf_;

      pcl::PointXYZ crop_min_, crop_max_;
      pcl::PointXYZ voxel_min_, voxel_max_;
      denso_recognition_msgs::InputCNN input_data_;
      denso_recognition_msgs::InputCNNArray voxels_array_;  // clustering結果を格納配列

      int voxel_n_ = 50;
      float diff_max_ = 0;
  };
}  // namespace crop_cloud

#endif // CROP_CLOUD_H
