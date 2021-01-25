#include "cnn_pose_estimator/voxelization_from_cluster.hpp"

#include <iostream>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

// #include <euclidean_cluster.hpp>

using voxelize_cloud::VoxelizeCloud;

VoxelizeCloud::VoxelizeCloud(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh)
  , rate_(n.param("loop_rate", 10))
  , voxel_n_(50)
  , frame_id_(n.param<std::string>("clustering_id", "photoneo_test_optical_frame"))
// frame_id_(n.param<std::string>("crop_frame_id",
// "kinect_head_rgb_optical_frame"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/filtered_pointcloud"), 1,
                                 &VoxelizeCloud::cloudCallback, this);
  box_sub_ = nh_.subscribe(n.param<std::string>("box_name", "clustering_result"), 1, &VoxelizeCloud::boxCallback, this);
  filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      n.param<std::string>("filtered_pc_topic_name", "/filtered_pointcloud"), 1);
  voxel_pub_ = nh_.advertise<denso_recognition_msgs::InputCNNArray>("/input_data", 1);

  // voxel範囲指定用パラメータ
  n.param<float>("crop_x_min", crop_min_.x, 0.15);
  n.param<float>("crop_x_max", crop_max_.x, 1.5);
  n.param<float>("crop_y_min", crop_min_.y, -1.5);
  n.param<float>("crop_y_max", crop_max_.y, 1.5);
  n.param<float>("crop_z_min", crop_min_.z, 0.01);
  n.param<float>("crop_z_max", crop_max_.z, 0.5);
}

void VoxelizeCloud::boxCallback(const denso_recognition_msgs::BoundingBoxArray clustering_result)
{
  box_array_ = clustering_result;
  std::cout << "subscribed clustering results" << std::endl;
  std::cout << "the number of clustering box : " << box_array_.boxes.size() << std::endl;
}

void VoxelizeCloud::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  ros::Time t0 = ros::Time::now();

  std::cout << "transformation cloud to frame \"" << frame_id_ << "\" " << std::endl;
  //点群をsensor座標系からWorld座標系に変換
  //変換されたデータはtrans_pcに格納される．

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZRGB> pcl_source;
  pcl::fromROSMsg(*source_pc, pcl_source);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_source_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(pcl_source));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr normarized_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  normarized_cloud->resize(pcl_source_ptr->points.size());

  denso_recognition_msgs::InputCNNArray voxels_array_;  // clustering結果を格納する配列

  for (int j = 0; j < box_array_.boxes.size(); j++)
  {
    crop_min_.x = box_array_.boxes[j].pose.position.x - 0.02;
    crop_min_.y = box_array_.boxes[j].pose.position.y - 0.02;
    crop_min_.z = box_array_.boxes[j].pose.position.z - 0.02;
    crop_max_.x = box_array_.boxes[j].pose.position.x + box_array_.boxes[j].dimensions.x + 0.02;
    crop_max_.y = box_array_.boxes[j].pose.position.y + box_array_.boxes[j].dimensions.y + 0.02;
    crop_max_.z = box_array_.boxes[j].pose.position.z + box_array_.boxes[j].dimensions.z + 0.02;
    float diff_x = crop_max_.x - crop_min_.x;
    float diff_y = crop_max_.y - crop_min_.y;
    float diff_z = crop_max_.z - crop_min_.z;

    //    std::cout << crop_max_.x << " - " << crop_min_.x << " = " << diff_x
    //    << std::endl;
    //    std::cout << crop_max_.y << " - " << crop_min_.y << " = " << diff_y
    //    << std::endl;
    //    std::cout << crop_max_.z << " - " << crop_min_.z << " = " << diff_z
    //    << std::endl;

    std::cout << "original number of pointcloud : " << pcl_source_ptr->points.size() << std::endl;

    float diff_max = 0;
    diff_max = diff_x;
    if (diff_max < diff_y)
    {
      diff_max = diff_y;
    }
    if (diff_max < diff_z)
    {
      diff_max = diff_z;
    }

    for (size_t i = 0; i < pcl_source_ptr->points.size(); ++i)
    {
      normarized_cloud->points[i].x = (pcl_source_ptr->points[i].x - crop_min_.x) / diff_max;
      normarized_cloud->points[i].y = (pcl_source_ptr->points[i].y - crop_min_.y) / diff_max;
      normarized_cloud->points[i].z = (pcl_source_ptr->points[i].z - crop_min_.z) / diff_max;
    }

    input_data_.voxel_array.data = { 0 };
    input_data_.voxel_array.data.resize(voxel_n_ * voxel_n_ * voxel_n_);

    float leaf = float(1.0 / voxel_n_);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(normarized_cloud);
    sor.setLeafSize(leaf, leaf, leaf);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*output);
    makeVoxel(output, leaf);
    std::cout << "downsampled data size : " << output->points.size() << std::endl;

    input_data_.pose.position.x = crop_min_.x;
    input_data_.pose.position.y = crop_min_.y;
    input_data_.pose.position.z = crop_min_.z;
    input_data_.pose.orientation.x = diff_max;
    voxels_array_.voxels.push_back(input_data_);
  }
  std::cout << "piblish input data" << std::endl;
  voxel_pub_.publish(voxels_array_);

  ros::Time t1 = ros::Time::now();
  std::cout << "処理時間 : " << t1 - t0 << std::endl;
}

void VoxelizeCloud::cropBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB min, pcl::PointXYZRGB max)
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

  pcl::CropBox<pcl::PointXYZRGB> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);
  cropFilter.setRotation(boxRotation);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  cropFilter.filter(*cloud_filtered);
  pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*cloud_filtered, *cloud);
}

// void VoxelizeCloud::makeVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float
// diff)
//{
//  for (int i = 0; i < voxel_n_; i++){
//    float v_min_x = i*diff;
//    float v_max_x = (i+1)*diff;
//    for (int j = 0; j < voxel_n_; j++){
//      float v_min_y = j*diff;
//      float v_max_y = (j+1)*diff;
//      for (int k = 0; k < voxel_n_; k++){
//        float v_min_z = k*diff;
//        float v_max_z = (k+1)*diff;
//        for (int n = 0; n < cloud->points.size(); n++){
//          if ((cloud->points[n].x > v_min_x) && (cloud->points[n].x < v_max_x)
//          &&
//              (cloud->points[n].y > v_min_y) && (cloud->points[n].y < v_max_y)
//              &&
//              (cloud->points[n].z > v_min_z) && (cloud->points[n].z <
//              v_max_z)){
//            input_data_.voxel_array.data[(i*voxel_n_*voxel_n_) + (j*voxel_n_) +
//            k] = 1;
//            break;
//          }
//        }
//      }
//    }
//  }
//}

void VoxelizeCloud::makeVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float diff)
{
  for (int n = 0; n < cloud->points.size(); n++)
  {
    int point_x = cloud->points[n].x / diff;
    int point_y = cloud->points[n].y / diff;
    int point_z = cloud->points[n].z / diff;
    input_data_.voxel_array.data[(point_x * voxel_n_ * voxel_n_) + (point_y * voxel_n_) + point_z] = 1;
  }
}
