#include "cnn_pose_estimator/voxelization_from_cluster.hpp"

using voxelize_cloud::VoxelizeCloud;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "MakeCNNInputNode");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  VoxelizeCloud voxelization(nh, n);
  // voxelization.run();
  ros::spin();

  return 0;
}
