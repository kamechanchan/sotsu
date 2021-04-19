#include <target_extractor/region_growing_extractor_server.h>

using region_growing_extractor_server::RegionGrowingExtractorServer;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "region_growing_extractor_server_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(1);

  RegionGrowingExtractorServer rg_server(nh);

  ROS_INFO("Ready to region growing extractor server");

  while (ros::ok())
  {
    ros::spinOnce();
    rg_server.publishExtractCloud();
    rate.sleep();
  }

  return 0;
}