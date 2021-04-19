#include <oneshot_calibration_system/ar_cube_pointcloud_extractor.h>

using ar_cube_pointcloud_extractor::ARCubePointCloudExtractor;
using ar_cube_pointcloud_extractor::ExtractorParam;

const float ExtractorParam::DURATION_TIME = 4.0;
const int ExtractorParam::START_TIME = 0;
const int ExtractorParam::PUBLISHER_QUEUE_SIZE = 10;
const int ExtractorParam::SUBSCRIBER_QUEUE_SIZE = 1;

ARCubePointCloudExtractor::ARCubePointCloudExtractor(ros::NodeHandle& nh, ros::NodeHandle& n)
  : nh_(nh)
{
  ROS_INFO("Constractor Start!!");
  extracted_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_pointcloud", ExtractorParam::PUBLISHER_QUEUE_SIZE);
  point_sub_ = nh_.subscribe("input_pointcloud", ExtractorParam::SUBSCRIBER_QUEUE_SIZE, &ARCubePointCloudExtractor::extractPointCloud, this);

  n.param<float>("cubemarker_size", cubemarker_size_, 0.05);
  n.param<float>("reserve_area", reserve_area_, 0.02);
  n.param<std::string>("target_frame", target_frame_, "base_link");
  n.param<std::string>("source_frame", source_frame_, "photoneo_center");
  n.getParam("ar_marker_indices/index_numbers", ar_indices_);
  for (int i = 0; i < ar_indices_.size(); i++)
    ar_link_name_vector_.push_back("/ar_marker_" + std::to_string(static_cast<int>(ar_indices_[i])));

  ROS_INFO("Constractor Finished!!");
}

void ARCubePointCloudExtractor::extractPointCloud(const sensor_msgs::PointCloud2::ConstPtr phoxi_points)
{
  ROS_INFO("extracting start");
  updateARTransform();
  tflistener(source_frame_, target_frame_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  tf::Transform tf;
  tf.setOrigin(transform_.getOrigin());
  tf.setRotation(transform_.getRotation());
  ROS_INFO("fromROSMsg.....");
  pcl::fromROSMsg(*phoxi_points, *cloud);
  ROS_INFO("Transform-pointcloud.....");
  pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
  extracted_points_.clear();
  ROS_INFO("Extraction_3Dmarker Start!!");
  std::for_each(trans_cloud->points.begin(), trans_cloud->points.end(), [this](pcl::PointXYZ pt) {
  for (unsigned int i = 0; i < ar_tf_vector_.size(); ++i)
  {
    if (discriminatePoint(pt, i))
    {
      pcl::PointXYZ buffer_point;
      buffer_point.x = pt.x;
      buffer_point.y = pt.y;
      buffer_point.z = pt.z;
      extracted_points_.push_back(buffer_point);
    }
  }
  });
  ROS_INFO("%zu", extracted_points_.size());
  ROS_INFO("Extraction_3Dmarker Finished!!");
}

bool ARCubePointCloudExtractor::discriminatePoint(pcl::PointXYZ target_point, int index)
{

  if (target_point.x > ar_tf_vector_[index].getOrigin().x() + (cubemarker_size_ + reserve_area_))
    return false;
  if (target_point.x < ar_tf_vector_[index].getOrigin().x() - (cubemarker_size_ + reserve_area_))
    return false;
  if (target_point.y > ar_tf_vector_[index].getOrigin().y() + (cubemarker_size_ + reserve_area_))
    return false;
  if (target_point.y < ar_tf_vector_[index].getOrigin().y() - (cubemarker_size_ + reserve_area_))
    return false;
  if (target_point.z > ar_tf_vector_[index].getOrigin().z())
    return false;
  if (target_point.z < ar_tf_vector_[index].getOrigin().z() - cubemarker_size_)
    return false;

  return true;
}

void ARCubePointCloudExtractor::publishPointCloud(void)
{
  auto msg = extracted_points_.makeShared();
  msg->header.frame_id = source_frame_;
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  extracted_pc_pub_.publish(msg);
}

void ARCubePointCloudExtractor::tflistener(std::string source_frame, std::string target_frame)
{
  ROS_INFO("tflistener start");
  ros::Time time = ros::Time(ExtractorParam::START_TIME);
  try
  {
    tf_listener__.waitForTransform(target_frame, source_frame, time, ros::Duration(ExtractorParam::DURATION_TIME));
    tf_listener__.lookupTransform(target_frame, source_frame, time, transform_);
    ROS_INFO("completed tflistener");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    ros::Duration(ExtractorParam::DURATION_TIME).sleep();
  }
}

void ARCubePointCloudExtractor::updateARTransform(void)
{
  while (ros::ok())
  {
    ar_tf_vector_.clear();
    try
    {
      std::for_each(ar_link_name_vector_.begin(), ar_link_name_vector_.end(), [this](std::string ar_name) {
      tf::StampedTransform ar_tf;
      tf_listener_.waitForTransform(target_frame_, ar_name, ros::Time(ExtractorParam::START_TIME), ros::Duration(ExtractorParam::DURATION_TIME));
      tf_listener_.lookupTransform(target_frame_, ar_name, ros::Time(ExtractorParam::START_TIME), ar_tf);
      ar_tf_vector_.push_back(ar_tf);
      });
      ROS_INFO("Completed updateARTransform");
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s Retry...", ex.what());
      ros::Duration(ExtractorParam::DURATION_TIME).sleep();
    }
  }
}
