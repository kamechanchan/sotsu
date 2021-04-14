#include <oneshot_calibration_system/workspace_pointcloud_extractor.h>

using workspace_pointcloud_extractor::WorkspacePointCloudExtractor;
using workspace_pointcloud_extractor::ExtractorParam;

const float ExtractorParam::DURATION_TIME = 4.0;
const int ExtractorParam::START_TIME = 0;
const int ExtractorParam::PUBLISHER_QUEUE_SIZE = 10;
const int ExtractorParam::SUBSCRIBER_QUEUE_SIZE = 1;

WorkspacePointCloudExtractor::WorkspacePointCloudExtractor(ros::NodeHandle& nh, ros::NodeHandle& n)
  : nh_(nh)
{
  ROS_INFO("Constractor Start!!");
  extracted_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("output_pointcloud", ExtractorParam::PUBLISHER_QUEUE_SIZE);
  point_sub_ = nh.subscribe("input_pointcloud", ExtractorParam::SUBSCRIBER_QUEUE_SIZE, &WorkspacePointCloudExtractor::extractPointCloud, this);
  n.param<float>("ar_height", ar_height_, 0.01);
  n.param<float>("reserve_area", reserve_area_, 0);
  n.param<std::string>("target_frame", target_frame_, "/base_link");
  n.param<std::string>("source_frame", source_frame_, "/photoneo_center_optical_frame");
  n.getParam("ar_marker_indices/index_numbers", ar_indices_);
  for (int i = 0; i < ar_indices_.size(); i++)
    ar_link_name_vector_.push_back("/ar_marker_" + std::to_string(static_cast<int>(ar_indices_[i])));

  ROS_INFO("Constractor Finished!!");
}

void WorkspacePointCloudExtractor::extractPointCloud(const sensor_msgs::PointCloud2::ConstPtr phoxi_points)
{
  updateARTransform();
  defineWorkSpace();
  tflistener("source_frame", "target_frame");
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
  ROS_INFO("Extraction_workspace_pointcloud Start!!");
  std::for_each(trans_cloud->points.begin(), trans_cloud->points.end(), [this](pcl::PointXYZ pt) {
  if (discriminatePoint(pt))
  {
    pcl::PointXYZ buffer_point;
    buffer_point.x = pt.x;
    buffer_point.y = pt.y;
    buffer_point.z = pt.z;
    extracted_points_.push_back(buffer_point);
  }
  });
  ROS_INFO("%zu", extracted_points_.size());
  ROS_INFO("Extraction_workspace_pointcloud finished!!");
}

bool WorkspacePointCloudExtractor::discriminatePoint(pcl::PointXYZ target_point)
{

  if (target_point.x > origin_vector_[0][0])
    return false;
  if (target_point.x < origin_vector_[1][0])
    return false;
  if (target_point.y > origin_vector_[0][1])
    return false;
  if (target_point.y < origin_vector_[1][1])
    return false;
  if (target_point.z > origin_vector_[0][2] + reserve_area_)
    return false;
  if (target_point.z < origin_vector_[0][2] - ar_height_)
    return false;

  return true;
}

void WorkspacePointCloudExtractor::publishPointCloud(void)
{
  auto msg = extracted_points_.makeShared();
  msg->header.frame_id = target_frame_;
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  extracted_pc_pub_.publish(msg);
}

void WorkspacePointCloudExtractor::defineWorkSpace()
{
  std::for_each(ar_tf_vector_.begin(), ar_tf_vector_.end(), [this](tf::StampedTransform ar_tf) {
  if (origin_vector_.empty())
  {
    std::vector<double> temp_origin_vector_max_, temp_origin_vector_min_;
    temp_origin_vector_max_.push_back(ar_tf.getOrigin().x());
    temp_origin_vector_max_.push_back(ar_tf.getOrigin().y());
    temp_origin_vector_max_.push_back(ar_tf.getOrigin().z());
    temp_origin_vector_min_.push_back(ar_tf.getOrigin().x());
    temp_origin_vector_min_.push_back(ar_tf.getOrigin().y());
    origin_vector_.push_back(temp_origin_vector_max_);
    origin_vector_.push_back(temp_origin_vector_min_);
  }
  else
  {
    if (origin_vector_[0][0] < ar_tf.getOrigin().x())
      origin_vector_[0][0] =  ar_tf.getOrigin().x();

    if (origin_vector_[0][1] < ar_tf.getOrigin().y())
      origin_vector_[0][1] = ar_tf.getOrigin().y();

    if (origin_vector_[0][2] < ar_tf.getOrigin().z())
      origin_vector_[0][2] = ar_tf.getOrigin().z();

    if (origin_vector_[1][0] > ar_tf.getOrigin().x())
      origin_vector_[1][0] = ar_tf.getOrigin().x();

    if (origin_vector_[1][1] > ar_tf.getOrigin().x())
      origin_vector_[1][1] = ar_tf.getOrigin().x();
  }
  });
}

void WorkspacePointCloudExtractor::tflistener(std::string source_frame, std::string target_frame)
{
  ROS_INFO("tflistener start");
  ros::Time time = ros::Time(ExtractorParam::START_TIME);
  try
  {
    tf_listener__.waitForTransform(target_frame_, source_frame_, time, ros::Duration(ExtractorParam::DURATION_TIME));
    tf_listener__.lookupTransform(target_frame_, source_frame_, time, transform_);
    ROS_INFO("completed tflistener");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    ros::Duration(ExtractorParam::DURATION_TIME).sleep();
  }
}

void WorkspacePointCloudExtractor::updateARTransform(void)
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
