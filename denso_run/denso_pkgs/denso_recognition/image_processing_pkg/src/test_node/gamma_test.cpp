#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_processing_srvs/Gamma.h>

class ServiceTest
{
public:
  ServiceTest(ros::NodeHandle& nh, image_processing_srvs::Gamma& gamma_srv_);

private:
  void getImage(const sensor_msgs::ImageConstPtr& img_msg);

private:
  ros::NodeHandle nh_;
  image_processing_srvs::Gamma gamma_srv_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceClient gamma_client_;
  sensor_msgs::Image image_;
};

ServiceTest::ServiceTest(ros::NodeHandle& nh, image_processing_srvs::Gamma& gamma_srv) : nh_(nh), gamma_srv_(gamma_srv)
{
  pub_ = nh_.advertise<sensor_msgs::Image>("gamma_modified_image", 1);
  sub_ = nh_.subscribe("photoneo_center/rgb_texture", 1, &ServiceTest::getImage, this);
  gamma_client_ = nh_.serviceClient<image_processing_srvs::Gamma>("gamma");
}

void ServiceTest::getImage(const sensor_msgs::ImageConstPtr& request_img)
{
  ROS_INFO("request image ...");
  gamma_srv_.request.input = *request_img;
  gamma_srv_.request.roi_point.data.clear();
  gamma_srv_.request.roi_point.data.push_back(request_img->width * 0);
  gamma_srv_.request.roi_point.data.push_back(request_img->height * 0);
  gamma_srv_.request.roi_size.data.clear();
  gamma_srv_.request.roi_size.data.push_back(request_img->width * 1);
  gamma_srv_.request.roi_size.data.push_back(request_img->height * 1);
  gamma_srv_.request.gamma.data = 25.0;

  if (gamma_client_.call(gamma_srv_))
  {
    ROS_INFO("success filtering !");
    image_ = gamma_srv_.response.output;
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
  ros::init(argc, argv, "gamma_modifier_test");
  ros::NodeHandle nh;
  image_processing_srvs::Gamma gamma_srv;

  ServiceTest test(nh, gamma_srv);

  ros::spin();

  return 0;
}
