#include <pointcloud_processing/crop_cloud.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "crop_box_node");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  crop_cloud::CropCloud crop(nh, n);
  crop.run();

  return 0;
}
