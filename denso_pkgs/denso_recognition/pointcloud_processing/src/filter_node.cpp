#include <pointcloud_processing/filter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  stat_outlier_removal::Filter filter(nh, n);

  ros::spin();

  return 0;
}
