#include <image_processing_pkg/gamma_modifier.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using gamma_modifier::GammaModifier;

GammaModifier::GammaModifier(ros::NodeHandle& nh) : nh_(nh)
{
  ROS_INFO("Ready to gamma processing server ...");

  gamma_srvs_ = nh_.advertiseService("gamma", &GammaModifier::filterImage, this);
}

void GammaModifier::applyGammaModifier(cv::Mat& input, float gamma)
{
  ROS_INFO_ONCE("processing %s ...", __FUNCTION__);

  cv::Mat fixed_img = input;

  // lookup table
  int i = 0;
  uchar lut[256];
  double gamma_coefficient = 1.0 / gamma;
  for (auto& elem : lut)
  {
    elem = pow(1.0 * i / 255.0, gamma_coefficient) * 255.0;
    i++;
  };

  // // 輝度値の置き換え処理
  cv::LUT(fixed_img, cv::Mat(cv::Size(256, 1), CV_8U, lut), input);
}

bool GammaModifier::filterImage(image_processing_srvs::Gamma::Request& req, image_processing_srvs::Gamma::Response& res)
{
  cv_bridge::CvImagePtr original_img = cv_bridge::toCvCopy(req.input, "bgr8");

  cv::Rect roi(cv::Point(req.roi_point.data[0], req.roi_point.data[1]),
               cv::Size(req.roi_size.data[0], req.roi_size.data[1]));
  cv::Mat croppedImg = original_img->image(roi);  // crop image

  float gamma = 25.0;
  applyGammaModifier(croppedImg, gamma);
  original_img->image.copyTo(croppedImg);  // paste cropped image

  sensor_msgs::ImagePtr result;
  result = cv_bridge::CvImage(std_msgs::Header(), "bgr8", original_img->image).toImageMsg();
  res.output = *result;
}
