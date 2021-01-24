#ifndef HISTOGRAM_FLATTENING_H
#define HISTOGRAM_FLATTENING_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16MultiArray.h>
#include <cv_bridge/cv_bridge.h>

#include <image_processing_srvs/Histogram.h>

namespace histogram_flattening
{
class HistogramFlattening
{
public:
  HistogramFlattening(ros::NodeHandle& nh);
  bool filterImage(image_processing_srvs::Histogram::Request& req, image_processing_srvs::Histogram::Response& res);

private:
  void applyHistogramFlattening(cv::Mat& input);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer histogram_srvs_;
};
}  // namespace histogram_flattening

#endif  // HISTOGRAM_FLATTENING_H