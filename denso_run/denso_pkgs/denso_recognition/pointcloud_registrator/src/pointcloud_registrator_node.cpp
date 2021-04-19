#include <pointcloud_registrator/pointcloud_registrator.h>

using pointcloud_registrator::PointCloudRegistrator;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pointcloud_registrator");
  ros::NodeHandle nh;
  PointCloudRegistrator pcr(nh);

  ros::spin();

  return 0;
}
