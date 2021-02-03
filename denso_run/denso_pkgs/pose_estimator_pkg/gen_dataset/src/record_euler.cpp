#include <gen_dataset/record_euler.h>

#include <vector>

#include <ros/package.h>
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

using record_euler::RecordEuler;

typedef pcl::PointXYZ PointInT;

RecordEuler::RecordEuler(ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_), is_ok_(false), number_of_data_(0)
{
  ros::param::param<int>("~max_data", max_data_, 100);
  ros::param::param<int>("~lookup_timeout", timeout_, 20);
  ros::param::param<std::string>("~package_name", package_name_, "gen_dataset");
  ros::param::param<std::string>("~optical_sensor_frame", optical_sensor_frame_, "photoneo_center_v3_optical_frame");
  ros::param::param<std::string>("~object_name", object_name_, "HV8");
  ros::param::param<std::string>("~src_cloud_topic", src_cloud_topic_, "/photoneo_center/sensor/points");
  cloud_sub_ = nh.subscribe(src_cloud_topic_, 1, &RecordEuler::receiveCloudCallback, this);
}

void RecordEuler::receiveCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  receive_cloud_ = *cloud;
}

bool RecordEuler::savePose()
{
  geometry_msgs::TransformStamped transform;
  int count = 0;
  while (true)
  {
    try
    {
      transform = tf_buffer_.lookupTransform(optical_sensor_frame_, object_name_, ros::Time(0), ros::Duration(1.0));
      break;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      count += 1;
      if (count == timeout_)
      {
        return false;
      }
      continue;
    }
  }

  geometry_msgs::Quaternion quat_msg;
  quat_msg.x = transform.transform.rotation.x;
  quat_msg.y = transform.transform.rotation.y;
  quat_msg.z = transform.transform.rotation.z;
  quat_msg.w = transform.transform.rotation.w;
  double roll, pitch, yaw;
  tf::Quaternion quat(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  std::vector<double> object_pose;
  object_pose.clear();
  object_pose.push_back(transform.transform.translation.x);
  object_pose.push_back(transform.transform.translation.y);
  object_pose.push_back(transform.transform.translation.z);
  object_pose.push_back(roll);
  object_pose.push_back(pitch);
  object_pose.push_back(yaw);

  std::stringstream ss;
  ss << (number_of_data_ + 1);
  std::string pose_filename = "pose_" + ss.str() + ".csv";

  std::string pose_file_path;
  pose_file_path = ros::package::getPath(package_name_);

  std::string filename;
  filename = pose_file_path + "/data/" + object_name_ + "/" + data_name_ + "/pose/" + pose_filename;

  std::ofstream pose_file(filename);
  for(int i = 0; i < object_pose.size(); i++)
  {
    pose_file << object_pose[i] << std::endl;
  }

  pose_file.close();

  return true;
}

bool RecordEuler::savePCD(const sensor_msgs::PointCloud2& input_cloud)
{
  pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT>);
  pcl::fromROSMsg(input_cloud, *cloud);
  pcl::PointCloud<PointInT>::Ptr filter_cloud (new pcl::PointCloud<PointInT>);

  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud, *filter_cloud, mapping);

  if (filter_cloud->size() == 0)
  {
    return false;
  }

  std::stringstream ss;
  ss << (number_of_data_ + 1);
  std::string pcd_filename = "cloud_" + ss.str() + ".pcd";

  std::string pcd_file_path;
  pcd_file_path = ros::package::getPath(package_name_);

  std::string filename;
  filename = pcd_file_path + "/data/" + object_name_ + "/" + data_name_ + "/pcd/" + pcd_filename;

  pcl::io::savePCDFileASCII(filename, *filter_cloud);

  return true;

}

bool RecordEuler::savePCDandPose()
{
  ros::param::get("/receive_cloud/is_ok", is_ok_);
  if (is_ok_)
  {
    if (!savePose())
    {
      return false;
    }

    sensor_msgs::PointCloud2 cloud;
    cloud = receive_cloud_;
    if (cloud.data.size() == 0)
    {
      return false;
    }

    if (!savePCD(cloud))
    {
      return false;
    }

    std::cout << "Success to record data " << (number_of_data_ + 1) << " !!" << std::endl;

    number_of_data_ += 1;
  }
  ros::param::set("/receive_cloud/is_ok", false);
  ros::param::set("/record_cloud/is_ok", true);
  return true;
}

bool RecordEuler::runRecord()
{
  if (!savePCDandPose())
  {
    ROS_WARN("Failed to record !!");
  }

  if (number_of_data_ < max_data_)
  {
    return true;
  }
  else
  {
    return false;
  }
}
