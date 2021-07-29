#include <pose_estimator_measure/crop_cloud.h>

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

#include <time.h>

using crop_cloud::CropCloud;

CropCloud::CropCloud(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh)
  , rate_(n.param("loop_rate", 10))
  , frame_id_(n.param<std::string>("crop_frame_id", "grand_truth"))
  , sensor_frame_id_(n.param<std::string>("sensor_frame_id", "photoneo_center_optical_frame"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/photoneo_center/pointcloud"), 1,
                                 &CropCloud::cropCallback, this);
  filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      n.param<std::string>("cropped_pc_topic_name", "/cropped_pointcloud"), 1);

  // voxel範囲指定用パラメータ
  n.param<float>("crop_x_min", crop_min_.x, 0.15);
  n.param<float>("crop_x_max", crop_max_.x, 1.5);
  n.param<float>("crop_y_min", crop_min_.y, -1.5);
  n.param<float>("crop_y_max", crop_max_.y, 1.5);
  n.param<float>("crop_z_min", crop_min_.z, 0.01);
  n.param<float>("crop_z_max", crop_max_.z, 0.5);
}


void CropCloud::cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  sensor_msgs::PointCloud2 trans_pc_0 = *source_pc;

  std::cout << source_pc->header.frame_id << std::endl;  // photoneo_test_optical_frame
  if (!(source_pc->header.frame_id == frame_id_))
  {
    try
    {
      pcl_ros::transformPointCloud(frame_id_, *source_pc, trans_pc_0, tf_);
    }
    catch (tf::ExtrapolationException e)
    {
      ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
    }
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_source_0;
  pcl::fromROSMsg(trans_pc_0, pcl_source_0);  // kinect用
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source_ptr_0(new pcl::PointCloud<pcl::PointXYZ>(pcl_source_0));

  cropBox(pcl_source_ptr_0, crop_min_, crop_max_);

  // 点群データをsensor座標系に変換
  sensor_msgs::PointCloud2 filtered_pc2;
  pcl::toROSMsg(*pcl_source_ptr_0, filtered_pc2);
  sensor_msgs::PointCloud2 trans_pc;
  try
  {
    pcl_ros::transformPointCloud(sensor_frame_id_, filtered_pc2, trans_pc, tf_);
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  clock_t start = clock();

  // 処理後の点群をpublish
  filtered_pc2.header.stamp = ros::Time::now();
  filtered_pc2.header.frame_id = sensor_frame_id_;
  filtered_cloud_pub_.publish(trans_pc);

  clock_t pcl_end = clock();
  const double time = static_cast<double>(pcl_end - start) / CLOCKS_PER_SEC * 1000;
  printf("time %lf[ms]\n\n", time);

}


void CropCloud::cropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max)
{
  Eigen::Vector4f minPoint;

  minPoint[0] = min.x;  // define minimum point x
  minPoint[1] = min.y;  // define minimum point y
  minPoint[2] = min.z;  // define minimum point z

  Eigen::Vector4f maxPoint;
  maxPoint[0] = max.x;  // define max point x
  maxPoint[1] = max.y;  // define max point y
  maxPoint[2] = max.z;  // define max point z

  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = 0;
  boxTranslatation[1] = 0;
  boxTranslatation[2] = 0;

  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0;  // rotation around x-axis
  boxRotation[1] = 0;  // rotation around y-axis
  boxRotation[2] = 0;  // in radians rotation around z-axis. this rotates your

  Eigen::Affine3f boxTransform;

  pcl::CropBox<pcl::PointXYZ> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);
  cropFilter.setRotation(boxRotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  cropFilter.filter(*cloud_filtered);
  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*cloud_filtered, *cloud);
}

