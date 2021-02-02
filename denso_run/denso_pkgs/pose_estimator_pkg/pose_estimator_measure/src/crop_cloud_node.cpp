#include "pose_estimator_measure/crop_cloud.h"

using crop_cloud::CropCloud;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "CropNode");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  CropCloud crop(nh, n);
  // crop.run();
  ros::spin();

  return 0;
}
