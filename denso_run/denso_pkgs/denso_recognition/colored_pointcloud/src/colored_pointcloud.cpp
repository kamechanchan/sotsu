#include <colored_pointcloud/colored_pointcloud.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl_ros/transforms.h>

#include <pcl/visualization/common/float_image_utils.h>

using colored_pointcloud::ColoredPointCloud;

ColoredPointCloud::ColoredPointCloud(ros::NodeHandle& nh)
  : nh_(nh)
  , it_(nh)
  , img_tran_sub_(it_, "/photoneo_center/sensor/image_color", 10)
  , caminfo_sub_(nh, "/photoneo_center/sensor/camera_info", 10)
  , pc_sub_(nh, "/cloud_without_segmented", 10)
  , sensor_sync_(sensor_sync_subs_(10), img_tran_sub_, caminfo_sub_, pc_sub_)
{
  ROS_INFO("Colored Pointcloud's constractor start");
  sensor_sync_.registerCallback(boost::bind(&ColoredPointCloud::callBack, this, _1, _2, _3));
  pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output_pointcloud", 10);
  pub_flag_ = false;
  tf_flag_ = false;
  if (!nh_.getParam("init_target", target_frame_))
  {
    ROS_ERROR("Error to get target frame_id");
  }
  if (!nh_.getParam("init_source", source_frame_))
  {
    ROS_ERROR("Error to get source frame_id");
  }

  if (!nh_.getParam("upper_filter_mode", upper_mode_))
  {
    ROS_ERROR("Error to get the flag of upper_color_filter_mode");
  }
  else if (!(nh_.getParam("upper_filter_val", upper_val_)) && upper_mode_)
  {
    ROS_ERROR("Error to get the upper filter value");
  }

  if (!nh_.getParam("lower_filter_mode", lower_mode_))
  {
    ROS_ERROR("Error to get the flag of lower_color_filter_mode");
  }
  else if (!(nh_.getParam("lower_filter_val", lower_val_)) && lower_mode_)
  {
    ROS_ERROR("Error to get the lower filter value");
  }

  ROS_INFO("Colored Pointcloud's constractor finished");
}

void ColoredPointCloud::callBack(const sensor_msgs::Image::ConstPtr& img,
                                 const sensor_msgs::CameraInfo::ConstPtr& cinfo,
                                 const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  ROS_INFO("Call_Back start");
  if (!tf_flag_)
    tflistener(target_frame_, source_frame_);
  colorConvert(img, cinfo, pc);
}

void ColoredPointCloud::tflistener(std::string target_frame, std::string source_frame)
{
  ROS_INFO("tflistener start");
  ros::Time time = ros::Time(0);
  try
  {
    tf_listener_.waitForTransform(target_frame, source_frame, time, ros::Duration(4.0));
    tf_listener_.lookupTransform(target_frame, source_frame, time, transform_);
    ROS_INFO("completed tflistener");
    tf_flag_ = true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    ros::Duration(4.0).sleep();
  }
}

void ColoredPointCloud::colorConvert(const sensor_msgs::Image::ConstPtr image,
                                     const sensor_msgs::CameraInfo::ConstPtr cinfo,
                                     const sensor_msgs::PointCloud2::ConstPtr points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  tf::Transform tf;
  tf.setOrigin(transform_.getOrigin());
  tf.setRotation(transform_.getRotation());
  pcl::fromROSMsg(*points, *cloud);
  ROS_INFO("Transform-pointcloud.....");
  pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
  ROS_INFO("CV_BRIDGE");
  std::cout << "[" << cinfo->K[0] << " " << cinfo->K[1] << " " << cinfo->K[2] << "]" << std::endl;
  std::cout << "[" << cinfo->K[3] << " " << cinfo->K[4] << " " << cinfo->K[5] << "]" << std::endl;
  std::cout << "[" << cinfo->K[6] << " " << cinfo->K[7] << " " << cinfo->K[8] << "]" << std::endl;
  cv_bridge::CvImageConstPtr cv_img_ptr;
  // cv::Mat buffer_image;
  try
  {
    cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    // buffer_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
    // cv_img_ptr = cv_bridge::toCvShare(image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  // cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
  // cv_image = cv_bridge::toCvShare(image)->image;
  cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
  cv_image = cv_img_ptr->image;
  cv::Mat rgb_image;
  // cv::cvtColor(buffer_image, rgb_image, CV_BGR2RGB);
  cv::cvtColor(cv_image, rgb_image, CV_BGR2RGB);
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cinfo);
  colored_points_.clear();
  int count = 0;
  double debug_x = 0;
  double debug_y = 0;
  ROS_INFO_STREAM("size: " << trans_cloud->points.size());
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = trans_cloud->points.begin(); pt < trans_cloud->points.end(); pt++)
  {
    count++;
    // if (count % 100 == 0)
    //   ROS_INFO_STREAM(count);
    if (pt->z < 0)
    {
      ROS_INFO("passed point");
      continue;
    }
    cv::Point3d pt_cv(pt->x, pt->y, pt->z);
    cv::Point2d uv;
    uv = cam_model.project3dToPixel(pt_cv);
    debug_x += uv.x;
    debug_y += uv.y;
    // ROS_INFO_STREAM("uv.x: " << uv.x << "  uv.y: " << uv.y);
    // ROS_INFO_STREAM("image_col / 2: " << rgb_image.cols / 2 << "   image_rows / 2: " << rgb_image.rows / 2);


    if (uv.x > (-rgb_image.cols / 2) && uv.x < (rgb_image.cols / 2) && uv.y > (-rgb_image.rows / 2) &&
        uv.y < (rgb_image.rows / 2))
    {
      // std::cout << "ttuuka!!" << std::endl;
      cv::Point2d converted_uv(uv.x + rgb_image.cols / 2, uv.y + rgb_image.rows / 2);
      cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(converted_uv.y, converted_uv.x);
      pcl::PointXYZRGB buffer_point;
      double rgb_ave = (rgb[0] + rgb[1] + rgb[2]) / 3.0;
      if (upper_mode_ && (rgb_ave < upper_val_))
        continue;

      if (lower_mode_ && (rgb_ave > lower_val_))
        continue;

      buffer_point.x = pt->x;
      buffer_point.y = pt->y;
      buffer_point.z = pt->z;
      buffer_point.r = rgb[0];
      buffer_point.g = rgb[1];
      buffer_point.b = rgb[2];
      colored_points_.push_back(buffer_point);
      ROS_INFO_STREAM("YES!!!!");
    }
    
    // std::cout << "tuuka??" << std::endl;
  }
  // std::cout << "average uv.x: " << debug_x / count << std::endl;
  // std::cout << "average uv.y: " << debug_y / count << std::endl;
  ROS_INFO_STREAM("tuuka");
  pub_flag_ = true;
  // std::cout << count << std::endl;
  if (tf_flag_ == true && pub_flag_)
  {
    //pcl::io::savePCDFile("/home/ericlabshinya/nigeru.pcd", colored_points_);
    ROS_INFO("%zu", colored_points_.size());
    ROS_INFO("convert_pointcloud_color finished!!");
  }
}

void ColoredPointCloud::publish(void)
{
  ROS_INFO_STREAM("pub_frag_: " << pub_flag_);
  if (pub_flag_)
  {
    auto msg = colored_points_.makeShared();
    msg->header.frame_id = target_frame_.c_str();
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    ROS_INFO_STREAM("ros_msgs size: " << msg->size());
    pcl::toROSMsg(*msg, ros_msg);
    pub_.publish(msg);
  }
}
