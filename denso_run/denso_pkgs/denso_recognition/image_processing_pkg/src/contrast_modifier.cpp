#include <image_processing_pkg/contrast_modifier.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using contrast_modifier::ContrastModifier;

ContrastModifier::ContrastModifier(ros::NodeHandle& nh) : nh_(nh)
{
  ROS_INFO("Ready to contrast processing server ...");

  contrast_srvs_ = nh_.advertiseService("contrast", &ContrastModifier::filterImage, this);
}

void ContrastModifier::applyContrastModifier(cv::Mat& input, float scaling_coefficient)
{
  ROS_INFO_ONCE("processing %s ...", __FUNCTION__);

  cv::Mat fixed_img = input;

  // // lookup table
  int i = 0;
  uchar lut[256];
  for (auto& elem : lut)
  {
    elem = 255.0 / (1 + exp(-scaling_coefficient * (i - 128.0) / 255.0));
    i++;
  };
  // 輝度値の置き換え処理
  cv::LUT(fixed_img, cv::Mat(cv::Size(256, 1), CV_8U, lut), input);
}

bool ContrastModifier::filterImage(image_processing_srvs::Contrast::Request& req,
                                   image_processing_srvs::Contrast::Response& res)
{
  cv_bridge::CvImagePtr original_img = cv_bridge::toCvCopy(req.input, "bgr8");

  cv::Rect roi(cv::Point(req.roi_point.data[0], req.roi_point.data[1]),
               cv::Size(req.roi_size.data[0], req.roi_size.data[1]));  // set ROI
  cv::Mat croppedImg = original_img->image(roi);                       // crop image

  float scaling_coefficient = 5.0;
  applyContrastModifier(croppedImg, scaling_coefficient);

  original_img->image.copyTo(croppedImg);  // paste cropped image

  sensor_msgs::ImagePtr result;
  result = cv_bridge::CvImage(std_msgs::Header(), "bgr8", original_img->image).toImageMsg();
  res.output = *result;
}
