#include <cnn_pose_estimator/estimate_object_pose_server.h>

using estimate_object_pose_server::EstimateObjectPoseServer;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimate_pose_object_server_node");
  ros::NodeHandle nh;

  EstimateObjectPoseServer estimate_object_pose_server(nh);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}