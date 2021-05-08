#include <cnn_pose_estimator/make_input_server.h>

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

using make_input_server::MakeInputServer;

MakeInputServer::MakeInputServer(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh)
  , voxel_n_(n.param("division_number", 50))
  , diff_max_(0.0)
  , is_ok_(false)
  , sensor_frame_id_(n.param<std::string>("sensor_frame_id", "photoneo_test_optical_frame"))
{
  source_pc_sub_ = nh.subscribe(n.param<std::string>("source_pc_topic_name", "/filtered_pointcloud"), 1,
                                &MakeInputServer::cropCallback, this);
  voxel_pub_ = nh.advertise<denso_recognition_msgs::InputCNNArray>("/input_data", 1);
  get_voxel_array_server_ =
      nh.advertiseService("/cnn_pose_estimator/make_input", &MakeInputServer::getVoxelArray, this);
}

void MakeInputServer::cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  input_cloud_ = *source_pc;
}

void MakeInputServer::getVoxelSize(pcl::PointCloud<PointInT>::Ptr cloud)
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

void MakeInputServer::downsampleCloud(pcl::PointCloud<PointInT>::Ptr cloud,
                                      pcl::PointCloud<PointInT>::Ptr output, float leaf)
{
  pcl::VoxelGrid<PointInT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf, leaf, leaf);
  // sor.setLeafSize(leaf * 2, leaf * 2, leaf * 2);
  sor.filter(*output);
  makeVoxel(output, leaf);
}

void MakeInputServer::makeVoxel(pcl::PointCloud<PointInT>::Ptr cloud, float leaf)
{
  int point_n = cloud->points.size();
  for (int i = 0; i < point_n; i++)
  {
    int voxel_x = cloud->points[i].x / leaf;
    int voxel_y = cloud->points[i].y / leaf;
    int voxel_z = cloud->points[i].z / leaf;
    input_data_.voxel_array.data[(voxel_n_ * voxel_n_ * voxel_z) + (voxel_n_ * voxel_y) + voxel_x] = 1;
  }
}

bool MakeInputServer::getVoxelArray(denso_recognition_srvs::MakeInput::Request& req,
                                    denso_recognition_srvs::MakeInput::Response& res)
{
  is_ok_ = false;
  clock_t start = clock();

  sensor_msgs::PointCloud2 src_cloud;
  src_cloud = input_cloud_;

  if (src_cloud.data.size() == 0)
  {
    ROS_ERROR_STREAM("cloud size is empty !!");
    res.success = false;
    return res.success;
  }

  sensor_msgs::PointCloud2 trans_pc;
  try
  {
    pcl_ros::transformPointCloud(sensor_frame_id_, src_cloud, trans_pc, tf_);
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
    res.success = false;
    return res.success;
  }

  pcl::PointCloud<PointInT> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);  //
  pcl::PointCloud<PointInT>::Ptr pcl_source_ptr(new pcl::PointCloud<PointInT>(pcl_source));
  pcl::PointCloud<PointInT>::Ptr normarized_cloud(new pcl::PointCloud<PointInT>);
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

  float leaf = float(1.0 / voxel_n_);
  pcl::PointCloud<PointInT>::Ptr output(new pcl::PointCloud<PointInT>);
  downsampleCloud(normarized_cloud, output, leaf);

  std::cout << "downsampled cloud size : " << output->points.size() << std::endl;

  denso_recognition_msgs::InputCNNArray voxels_array;

  input_data_.pose.position.x = voxel_min_.x;
  input_data_.pose.position.y = voxel_min_.y;
  input_data_.pose.position.z = voxel_min_.z;
  input_data_.pose.orientation.x = diff_max_;
  voxels_array.voxels.push_back(input_data_);

  voxels_array_ = voxels_array;

  std::cout << "piblish input data (voxel)" << std::endl;

  clock_t end = clock();
  const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
  printf("time %lf[ms]\n\n", time);

  is_ok_ = true;
  res.success = true;
  return res.success;
}

void MakeInputServer::publishVoxelArray()
{
  if (is_ok_)
  {
    voxel_pub_.publish(voxels_array_);
  }
}
