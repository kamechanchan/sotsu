#include <pointcloud_processing/downsampling.h>

using downsampling::Downsampling;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "downsampling");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  Downsampling downsampling(nh, n);
  ros::spin();
  return 0;
}
