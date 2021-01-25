#ifndef GAMMA_MODIFIER_H
#define GAMMA_MODIFIER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16MultiArray.h>
#include <cv_bridge/cv_bridge.h>

#include <image_processing_srvs/Gamma.h>

namespace gamma_modifier
{
class GammaModifier
{
public:
  GammaModifier(ros::NodeHandle& nh);
  bool filterImage(image_processing_srvs::Gamma::Request& req, image_processing_srvs::Gamma::Response& res);

private:
  void applyGammaModifier(cv::Mat& input, float gamma);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer gamma_srvs_;
};
}  // namespace gamma_modifier

#endif  // GAMMA_MODIFIER_H