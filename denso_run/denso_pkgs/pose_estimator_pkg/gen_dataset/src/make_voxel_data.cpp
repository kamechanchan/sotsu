#include <gen_dataset/make_voxel_data.h>

#include <ros/package.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using make_voxel_data::MakeVoxelData;

MakeVoxelData::MakeVoxelData(ros::NodeHandle& nh) : nh_(nh), data_number_(1), operation_time_(0.0), min_x_(0.0), min_y_(0.0), min_z_(0.0), max_x_(0.0), max_y_(0.0), max_z_(0.0)
{
  ros::param::param<int>("~max_data", max_data_, 100);
  ros::param::param<int>("~voxel_n", voxel_n_, 64);
  ros::param::param<std::string>("~package_name", package_name_, "cnn_pose_estimator");
  ros::param::param<std::string>("~object_name", object_name_, "HV8");
  ros::param::param<std::string>("~dataset_name", dataset_name_, "data1");
  voxel_.resize(voxel_n_ * voxel_n_ * voxel_n_);
}

void MakeVoxelData::doMinMax(const pcl::PointCloud<PointInT>::Ptr cloud)
{
  min_x_ = cloud->points[0].x;
  min_y_ = cloud->points[0].y;
  min_z_ = cloud->points[0].z;
  max_x_ = cloud->points[0].x;
  max_y_ = cloud->points[0].y;
  max_z_ = cloud->points[0].z;

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    if (min_x_ < cloud->points[i].x)
      min_x_ = min_x_;
    else
      min_x_ = cloud->points[i].x;

    if (min_y_ < cloud->points[i].y)
      min_y_ = min_y_;
    else
      min_y_ = cloud->points[i].y;

    if (min_z_ < cloud->points[i].z)
      min_z_ = min_z_;
    else
      min_z_ = cloud->points[i].z;

    if (max_x_ > cloud->points[i].x)
      max_x_ = max_x_;
    else
      max_x_ = cloud->points[i].x;

    if (max_y_ > cloud->points[i].y)
      max_y_ = max_y_;
    else
      max_y_ = cloud->points[i].y;

    if (max_z_ > cloud->points[i].z)
      max_z_ = max_z_;
    else
      max_z_ = cloud->points[i].z;
  }
}

void MakeVoxelData::makeVoxel(const pcl::PointCloud<PointInT>::Ptr cloud, const float& resol)
{
  for (int i = 0; i < voxel_.size(); i++)
  {
    voxel_[i] = 0;
  }
  for (int i = 0; i < cloud->points.size(); i++)
  {
    int point_x = cloud->points[i].x / resol;
    int point_y = cloud->points[i].y / resol;
    int point_z = cloud->points[i].z / resol;
    voxel_[(point_z * voxel_n_ * voxel_n_) + (point_y * voxel_n_) + point_x] = 1;
  }
}

bool MakeVoxelData::runMakeVoxelData()
{
  ros::Time t0 = ros::Time::now();

  std::string data_package_path;
  data_package_path = ros::package::getPath(package_name_);

  std::stringstream ss;
  ss << data_number_;

  std::string pcd_file_path;
  pcd_file_path = data_package_path + "/data/" + object_name_ + "/" + dataset_name_ + "/pcd/" + "cloud_" + ss.str() + ".pcd";

  pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT>);
  pcl::io::loadPCDFile(pcd_file_path, *cloud);

  doMinMax(cloud);

  float diff_x = max_x_ - min_x_;
  float diff_y = max_y_ - min_y_;
  float diff_z = max_z_ - min_z_;

  float diff_max = 0.0;
  diff_max = diff_x;
  if (diff_max < diff_y)
  {
    diff_max = diff_y;
  }
  if (diff_max < diff_z)
  {
    diff_max = diff_z;
  }

  std::string pose_file_path;
  pose_file_path = data_package_path + "/data/" + object_name_ + "/" + dataset_name_ + "/pose/" + "pose_" + ss.str() + ".csv";

  std::ifstream ifs(pose_file_path);
  std::vector<float> pose;
  pose.clear();
  std::string line;
  while (getline(ifs, line))
  {
    float data = stof(line);
    pose.push_back(data);
  }

  std::vector<float> trans;
  trans.resize(pose.size());

  trans[0] = (pose[0] - min_x_) / diff_max;
  trans[1] = (pose[1] - min_y_) / diff_max;
  trans[2] = (pose[2] - min_z_) / diff_max;
  trans[3] = pose[3];
  trans[4] = pose[4];
  trans[5] = pose[5];

  std::string trans_pose_path;
  trans_pose_path = data_package_path + "/data/" + object_name_ + "/" + dataset_name_ + "/learn_data/norm_pose/" + "pose_" + ss.str() + ".csv";

  std::ofstream trans_pose_file(trans_pose_path);
  for (int i = 0; i < trans.size(); i++)
  {
    trans_pose_file << trans[i] << std::endl;
  }
  trans_pose_file.close();

  pcl::PointCloud<PointInT>::Ptr normalized_cloud (new pcl::PointCloud<PointInT>);
  normalized_cloud->points.resize(cloud->points.size());

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    normalized_cloud->points[i].x = (cloud->points[i].x - min_x_) / diff_max;
    normalized_cloud->points[i].y = (cloud->points[i].y - min_y_) / diff_max;
    normalized_cloud->points[i].z = (cloud->points[i].z - min_z_) / diff_max;
  }

  float leaf = float(1.0/voxel_n_);
  pcl::VoxelGrid<PointInT> sor;
  sor.setInputCloud(normalized_cloud);
  sor.setLeafSize(leaf, leaf, leaf);
  pcl::PointCloud<PointInT>::Ptr output(new pcl::PointCloud<PointInT>);
  sor.filter(*output);

  makeVoxel(output, leaf);

  std::string voxel_file_path;
  voxel_file_path = data_package_path + "/data/" + object_name_ + "/" + dataset_name_ + "/learn_data/voxel/" + "voxel_" + ss.str() + ".csv";

  std::ofstream voxel_file(voxel_file_path);
  for (int i = 0; i < voxel_n_; i++)
  {
    for (int j = 0; j < voxel_n_; j++)
    {
      for (int k = 0; k < voxel_n_; k++)
      {
        voxel_file << voxel_[(i*voxel_n_*voxel_n_) + (j*voxel_n_) + k] << std::endl;
      }
    }
  }
  voxel_file.close();

  ros::Time t1 = ros::Time::now();
  data_number_ += 1;
  operation_time_ += (t1 - t0).toSec();
  std::cout << "処理時間 : " << operation_time_ / data_number_ << std::endl;

  std::cout << "Success to make voxel data " << ss.str() << std::endl;


  if (data_number_ > max_data_)
  {
    return true;
  }

  return false;

}
