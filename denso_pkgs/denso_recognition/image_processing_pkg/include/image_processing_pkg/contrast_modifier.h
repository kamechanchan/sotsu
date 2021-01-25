#ifndef CONTRAST_MODIFIER_H
#define CONTRAST_MODIFIER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16MultiArray.h>
#include <cv_bridge/cv_bridge.h>

#include <image_processing_srvs/Contrast.h>

namespace contrast_modifier
{
class ContrastModifier
{
public:
  ContrastModifier(ros::NodeHandle& nh);
  bool filterImage(image_processing_srvs::Contrast::Request& req, image_processing_srvs::Contrast::Response& res);

private:
  void applyContrastModifier(cv::Mat& input, float scaling);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer contrast_srvs_;
};
}  // namespace contrast_modifier

#endif  // CONTRAST_MODIFIER_H