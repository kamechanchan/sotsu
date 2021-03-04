#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_processing_srvs/Contrast.h>

class ServiceTest
{
public:
  ServiceTest(ros::NodeHandle& nh, image_processing_srvs::Contrast& contrast_srv);

private:
  void getImage(const sensor_msgs::ImageConstPtr& img_msg);

private:
  ros::NodeHandle nh_;
  image_processing_srvs::Contrast contrast_srv_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceClient contrast_client_;
  sensor_msgs::Image image_;
};

ServiceTest::ServiceTest(ros::NodeHandle& nh, image_processing_srvs::Contrast& contrast_srv)
  : nh_(nh), contrast_srv_(contrast_srv)
{
  pub_ = nh_.advertise<sensor_msgs::Image>("contrast_modified_image", 1);
  sub_ = nh_.subscribe("photoneo_center/rgb_texture", 1, &ServiceTest::getImage, this);
  contrast_client_ = nh_.serviceClient<image_processing_srvs::Contrast>("contrast");
}

void ServiceTest::getImage(const sensor_msgs::ImageConstPtr& request_img)
{
  ROS_INFO("request image ...");
  contrast_srv_.request.input = *request_img;
  contrast_srv_.request.roi_point.data.clear();
  contrast_srv_.request.roi_point.data.push_back(request_img->width * 0);
  contrast_srv_.request.roi_point.data.push_back(request_img->height * 0);
  contrast_srv_.request.roi_size.data.clear();
  contrast_srv_.request.roi_size.data.push_back(request_img->width * 1);
  contrast_srv_.request.roi_size.data.push_back(request_img->height * 1);
  contrast_srv_.request.contrast.data = 10.0;

  if (contrast_client_.call(contrast_srv_))
  {
    ROS_INFO("success filtering !");
    image_ = contrast_srv_.response.output;
    image_.header.frame_id = request_img->header.frame_id;
    image_.header.seq = request_img->header.seq;
    image_.header.stamp = request_img->header.stamp;
    pub_.publish(image_);
  }
  else
  {
    ROS_ERROR("Faild to call contrast processing service");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "contrast_modifier_test");
  ros::NodeHandle nh;
  image_processing_srvs::Contrast contrast_srv;

  ServiceTest test(nh, contrast_srv);

  ros::spin();

  return 0;
}
