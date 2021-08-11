#include "cnn_pose_estimator/make_input.hpp"

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
  , voxel_n_(n.param("division_number", 50))
  // frame_id_(n.param<std::string>("crop_frame_id",
  // "kinect_head_rgb_optical_frame"))
  , frame_id_(n.param<std::string>("crop_frame_id", "world"))
  , sensor_frame_id_(n.param<std::string>("sensor_frame_id", "photoneo_test_optical_frame"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/filtered_pointcloud"), 1,
                                 &CropCloud::cropCallback, this);
  // ar_tf_sub_ = nh_.subscribe(n.param<std::string>("ar_tf_name", "/tf"), 1,
  // &CropCloud::cropCallback_tf, this);
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

// void CropCloud::cropCallback_tf(const geometry_msgs::TransformStamped pose)

void CropCloud::cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  clock_t start = clock();
  denso_recognition_msgs::InputCNNArray voxels_array_;
  //std::cout << source_pc->header.frame_id << std::endl;  // photoneo_test_optical_frame
  sensor_msgs::PointCloud2 trans_pc_0 = *source_pc;
  if (!(source_pc->header.frame_id == frame_id_))
  {
    //std::cout << "transformation cloud to frame \"" << frame_id_ << "\" " << std::endl;  // world
    //点群をsensor座標系からWorld座標系に変換
    //変換されたデータはtrans_pcに格納される．
    try
    {
      pcl_ros::transformPointCloud(frame_id_, *source_pc, trans_pc_0, tf_);
      //std::cout << "finish to transform cloud to " << frame_id_ << std::endl;
    }
    catch (tf::ExtrapolationException e)
    {
      ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
    }
  }

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZRGB> pcl_source_0;
  pcl::fromROSMsg(trans_pc_0, pcl_source_0);  // kinect用
  // pcl::fromROSMsg(*source_pc, pcl_source);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_source_ptr_0(new pcl::PointCloud<pcl::PointXYZRGB>(pcl_source_0));

  cropBox(pcl_source_ptr_0, crop_min_, crop_max_);
  std::cout << "finish to crop the cloud" << std::endl;

  // 点群データをsensor座標系に変換
  sensor_msgs::PointCloud2 filtered_pc2;
  pcl::toROSMsg(*pcl_source_ptr_0, filtered_pc2);
  sensor_msgs::PointCloud2 trans_pc;
  try
  {
    pcl_ros::transformPointCloud(sensor_frame_id_, filtered_pc2, trans_pc, tf_);
    //std::cout << "finish to transform cloud to " << sensor_frame_id_ << std::endl;
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  // 処理後の点群をpublish
  filtered_pc2.header.stamp = ros::Time::now();
  filtered_pc2.header.frame_id = "world";
  filtered_cloud_pub_.publish(filtered_pc2);

  pcl::PointCloud<pcl::PointXYZRGB> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);  //
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_source_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(pcl_source));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr normarized_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  normarized_cloud->resize(pcl_source_ptr->points.size());

  getVoxelSize(pcl_source_ptr);

  for (size_t i = 0; i < pcl_source_ptr->points.size(); ++i)
  {
    normarized_cloud->points[i].x = (pcl_source_ptr->points[i].x - voxel_min_.x) / diff_max_;
    normarized_cloud->points[i].y = (pcl_source_ptr->points[i].y - voxel_min_.y) / diff_max_;
    normarized_cloud->points[i].z = (pcl_source_ptr->points[i].z - voxel_min_.z) / diff_max_;
  }

  input_data_.voxel_array.data = { 0 };
  input_data_.voxel_array.data.resize(voxel_n_ * voxel_n_ * voxel_n_);
  //  makeVoxel(normarized_cloud, float(1.0/voxel_n_));

  float leaf = float(1.0 / voxel_n_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  downsampleCloud(normarized_cloud, output, leaf);
  //  //octree構造のcloudからvoxelの重心を求める（失敗）
  //  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB> octree(leaf);
  //  octree.setInputCloud(normarized_cloud);
  //  octree.defineBoundingBox();
  //  octree.addPointsFromInputCloud();
  //  pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::AlignedPointTVector centroids;
  //  octree.getVoxelCentroids(centroids);
  //
  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new
  //  pcl::PointCloud<pcl::PointXYZRGB>);
  //  output->points.assign(centroids.begin(), centroids.end());
  //  output->width = uint32_t(centroids.size());
  //  output->height = 1;
  //  output->is_dense = true;

  std::cout << "downsampled cloud size : " << output->points.size() << std::endl;

  //  int cnt = 0;
  //  for (int n = 0; n < voxel_n_*voxel_n_*voxel_n_; n++)
  //  {
  //    if (input_data_.voxel_array.data[n] == 1)
  //    {
  //      cnt += 1;
  //    }
  //  }
  //  std::cout << "count :" << cnt << std::endl;

  input_data_.pose.position.x = voxel_min_.x;
  input_data_.pose.position.y = voxel_min_.y;
  input_data_.pose.position.z = voxel_min_.z;
  input_data_.pose.orientation.x = diff_max_;
  voxels_array_.voxels.push_back(input_data_);
  //std::cout << input_data_.pose.position << std::endl;
  //std::cout << diff_max_ << std::endl;

  std::cout << "piblish input data (voxel)" << std::endl;
  voxel_pub_.publish(voxels_array_);

  clock_t end = clock();
  const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
  printf("time %lf[ms]\n\n", time);
}

void CropCloud::getVoxelSize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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

  //std::cout << voxel_max_.x << " - " << voxel_min_.x << " = " << diff_x << std::endl;
  //std::cout << voxel_max_.y << " - " << voxel_min_.y << " = " << diff_y << std::endl;
  //std::cout << voxel_max_.z << " - " << voxel_min_.z << " = " << diff_z << std::endl;

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

void CropCloud::downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, float leaf)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf, leaf, leaf);
  //sor.setLeafSize(leaf * 2, leaf * 2, leaf * 2);
  sor.filter(*output);
  makeVoxel(output, leaf);
}

void CropCloud::cropBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB min, pcl::PointXYZRGB max)
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

void CropCloud::makeVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float leaf)
{
  int point_n = cloud->points.size();
  for (int i = 0; i < point_n; i++)
  {
    int voxel_x = cloud->points[i].x/leaf;
    int voxel_y = cloud->points[i].y/leaf;
    int voxel_z = cloud->points[i].z/leaf;
    input_data_.voxel_array.data[(voxel_n_ * voxel_n_ * voxel_z) + (voxel_n_ * voxel_y) + voxel_x] = 1;
  }

//  for (int i = 0; i < voxel_n_; i++)
//  {
//    float v_min_z = i * leaf;
//    float v_max_z = (i + 1) * leaf;
//    for (int j = 0; j < voxel_n_; j++)
//    {
//      float v_min_y = j * leaf;
//      float v_max_y = (j + 1) * leaf;
//      for (int k = 0; k < voxel_n_; k++)
//      {
//        float v_min_x = k * leaf;
//        float v_max_x = (k + 1) * leaf;
//        for (int n = 0; n < cloud->points.size(); n++)
//        {
//          if ((cloud->points[n].x > v_min_x) && (cloud->points[n].x < v_max_x) && (cloud->points[n].y > v_min_y) &&
//              (cloud->points[n].y < v_max_y) && (cloud->points[n].z > v_min_z) && (cloud->points[n].z < v_max_z))
//          {
//            input_data_.voxel_array.data[(voxel_n_ * voxel_n_ * i) + (voxel_n_ * j) + k] =
//                1;  //なぜかこうしたらiとkを入れ替えたらうまく行く
//            break;
//          }
//        }
//      }
//    }
//  }
}
