#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_processing_srvs/Histogram.h>

class ServiceTest
{
public:
  ServiceTest(ros::NodeHandle& nh, image_processing_srvs::Histogram& histogram_srv);

private:
  void getImage(const sensor_msgs::ImageConstPtr& img_msg);

private:
  ros::NodeHandle nh_;
  image_processing_srvs::Histogram histogram_srv_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceClient histogram_client_;
  sensor_msgs::Image image_;
};

ServiceTest::ServiceTest(ros::NodeHandle& nh, image_processing_srvs::Histogram& histogram_srv)
  : nh_(nh), histogram_srv_(histogram_srv)
{
  pub_ = nh_.advertise<sensor_msgs::Image>("histogram_modified_image", 1);
  sub_ = nh_.subscribe("photoneo_center/rgb_texture", 1, &ServiceTest::getImage, this);
  histogram_client_ = nh_.serviceClient<image_processing_srvs::Histogram>("histogram");
}

void ServiceTest::getImage(const sensor_msgs::ImageConstPtr& request_img)
{
  ROS_INFO("request image ...");
  histogram_srv_.request.input = *request_img;
  histogram_srv_.request.roi_point.data.clear();
  histogram_srv_.request.roi_point.data.push_back(request_img->width * 0);
  histogram_srv_.request.roi_point.data.push_back(request_img->height * 0);
  histogram_srv_.request.roi_size.data.clear();
  histogram_srv_.request.roi_size.data.push_back(request_img->width * 1);
  histogram_srv_.request.roi_size.data.push_back(request_img->height * 1);

  if (histogram_client_.call(histogram_srv_))
  {
    ROS_INFO("success filtering !");
    image_ = histogram_srv_.response.output;
    image_.header.frame_id = request_img->header.frame_id;
    image_.header.seq = request_img->header.seq;
    image_.header.stamp = request_img->header.stamp;
    pub_.publish(image_);
  }
  else
  {
    ROS_ERROR("Faild to call histogram processing service");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "histogram_flattening_test");
  ros::NodeHandle nh;
  image_processing_srvs::Histogram histogram_srv;

  ServiceTest test(nh, histogram_srv);

  ros::spin();

  return 0;
}