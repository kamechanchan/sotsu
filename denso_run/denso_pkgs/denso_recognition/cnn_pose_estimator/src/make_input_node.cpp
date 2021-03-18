//#include <crop_cloud.hpp>
#include "cnn_pose_estimator/make_input.hpp"

using crop_cloud::CropCloud;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "MakeInputNode");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  CropCloud crop(nh, n);
  // crop.run();
  ros::spin();

  return 0;
}
