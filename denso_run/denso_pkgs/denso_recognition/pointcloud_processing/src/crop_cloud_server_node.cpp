#include <pointcloud_processing/crop_cloud_server.h>

using crop_cloud_server::CropCloudServer;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "crop_cloud_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::AsyncSpinner spinner(1);
  ros::Rate rate(1);
  spinner.start();

  CropCloudServer crop(nh, n);

  while (ros::ok())
  {
    ros::spinOnce();
    crop.publishCropCloud();
    rate.sleep();
  }

  return 0;
}
