#include "cnn_pose_estimator/euclidean_cluster.hpp"

using cluster_cloud::ClusterCloud;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "EuclideanClusterNode");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  ClusterCloud cluster(nh, n);
  // cluster.run();
  ros::spin();

  return 0;
}
