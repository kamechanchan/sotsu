#ifndef COLORED_POINTCLOUD_H
#define COLORED_POINTCLOUD_H

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl_ros/point_cloud.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace colored_pointcloud
{
class ColoredPointCloud
{
public:
  ColoredPointCloud(ros::NodeHandle& nh);
  void callBack(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& cinfo,
                const sensor_msgs::PointCloud2::ConstPtr& pc);
  void tflistener(std::string target_frame, std::string source_frame);
  void colorConvert(const sensor_msgs::Image::ConstPtr image, const sensor_msgs::CameraInfo::ConstPtr cinfo,
                    const sensor_msgs::PointCloud2::ConstPtr points);
  void publish(void);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                          sensor_msgs::PointCloud2>
      sensor_sync_subs_;
  message_filters::Synchronizer<sensor_sync_subs_> sensor_sync_;

public:
  pcl::PointCloud<pcl::PointXYZRGB> colored_points_;
  image_transport::ImageTransport it_;

private:
  ros::NodeHandle nh_;
  image_transport::SubscriberFilter img_tran_sub_;

 
  message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
  
  ros::Publisher pub_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform transform_;
  std::string target_frame_;
  std::string source_frame_;
  bool upper_mode_;
  bool lower_mode_;
  double upper_val_;
  double lower_val_;
  bool pub_flag_;
  bool tf_flag_;
  sensor_msgs::PointCloud2 ros_msg;
};
}  // colored_pointCloud

#endif  // COLORED_POINTCLOUD_H
