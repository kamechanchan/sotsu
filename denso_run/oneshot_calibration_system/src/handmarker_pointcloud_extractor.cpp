#include <oneshot_calibration_system/handmarker_pointcloud_extractor.h>

using handmarker_pointcloud_extractor::HandMarkerPointCloudExtractor;
using handmarker_pointcloud_extractor::ExtractorParam;

const float ExtractorParam::DURATION_TIME = 4.0;
const int ExtractorParam::START_TIME = 0;
const int ExtractorParam::PUBLISHER_QUEUE_SIZE = 10;
const int ExtractorParam::SUBSCRIBER_QUEUE_SIZE = 1;

HandMarkerPointCloudExtractor::HandMarkerPointCloudExtractor(ros::NodeHandle& nh, ros::NodeHandle& n)
  : nh_(nh)
{
  ROS_INFO("Constractor Start!!");
  extracted_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_pointcloud", ExtractorParam::PUBLISHER_QUEUE_SIZE);
  point_sub_ = nh_.subscribe("input_pointcloud", ExtractorParam::SUBSCRIBER_QUEUE_SIZE, &HandMarkerPointCloudExtractor::extractPointCloud, this);

  n.param<float>("cubemarker_size", cubemarker_size_, 0.05);
  n.param<float>("reserve_area", reserve_area_, 0.02);
  n.param<std::string>("target_frame", target_frame_, "base_link");
  n.param<std::string>("source_frame", source_frame_, "photoneo_center");
  ROS_INFO("Constractor Finished!!");
}

void HandMarkerPointCloudExtractor::extractPointCloud(const sensor_msgs::PointCloud2::ConstPtr phoxi_points)
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
  ROS_INFO("Extraction_3Dmarker Finished!!");
}

bool HandMarkerPointCloudExtractor::discriminatePoint(pcl::PointXYZ target_point)
{

  if (target_point.x > 0.380393 + (cubemarker_size_ + reserve_area_))
    return false;
  if (target_point.x < 0.380393 - (cubemarker_size_ + reserve_area_))
    return false;
  if (target_point.y > 0.000625 + (cubemarker_size_ + reserve_area_))
    return false;
  if (target_point.y < 0.000625 - (cubemarker_size_ + reserve_area_))
    return false;
  if (target_point.z > 0.586193)
    return false;
  if (target_point.z < 0.586193 - cubemarker_size_ * 2)
    return false;

  return true;
}

void HandMarkerPointCloudExtractor::publishPointCloud(void)
{
  auto msg = extracted_points_.makeShared();
  msg->header.frame_id = target_frame_;
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  extracted_pc_pub_.publish(msg);
}

void HandMarkerPointCloudExtractor::tflistener(std::string source_frame, std::string target_frame)
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

void HandMarkerPointCloudExtractor::updateARTransform(void)
{
  while (ros::ok())
  {
    try
    {
      tf_listener_.waitForTransform("base_link", "tercero_cover_for_mesh_center", ros::Time(ExtractorParam::START_TIME), ros::Duration(ExtractorParam::DURATION_TIME));
      tf_listener_.lookupTransform("base_link", "tercero_cover_for_mesh_center", ros::Time(ExtractorParam::START_TIME), base_link_to_marker_);
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
