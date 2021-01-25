#include <image_processing_pkg/histogram_flattening.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using histogram_flattening::HistogramFlattening;

HistogramFlattening::HistogramFlattening(ros::NodeHandle& nh) : nh_(nh)
{
  ROS_INFO("Ready to histogram processing server ...");

  histogram_srvs_ = nh_.advertiseService("histogram", &HistogramFlattening::filterImage, this);
}

void HistogramFlattening::applyHistogramFlattening(cv::Mat& input)
{
  // ROS_INFO_ONCE("processing %s ...", __FUNCTION__);

  cv::Mat fixed_img = input;
  cv::Mat gray_img, equalized_gray_img;

  // BGR to GRAY
  cv::cvtColor(fixed_img, gray_img, CV_BGR2GRAY);

  //ヒストグラム平坦化
  equalizeHist(gray_img, equalized_gray_img);

  // output
  // GRAY to BGR
  cv::cvtColor(equalized_gray_img, input, CV_GRAY2BGR);
}

bool HistogramFlattening::filterImage(image_processing_srvs::Histogram::Request& req,
                                      image_processing_srvs::Histogram::Response& res)
{
  ROS_INFO_ONCE("processing %s of %s ...", __FUNCTION__, typeid(this).name());

  cv_bridge::CvImagePtr original_img = cv_bridge::toCvCopy(req.input, "bgr8");

  cv::Rect roi(cv::Point(req.roi_point.data[0], req.roi_point.data[1]),
               cv::Size(req.roi_size.data[0], req.roi_size.data[1]));  // set ROI
  cv::Mat croppedImg = original_img->image(roi);                       // crop image

  applyHistogramFlattening(croppedImg);

  original_img->image.copyTo(croppedImg);  // paste cropped image

  sensor_msgs::ImagePtr result;
  result = cv_bridge::CvImage(std_msgs::Header(), "bgr8", original_img->image).toImageMsg();
  res.output = *result;
}
