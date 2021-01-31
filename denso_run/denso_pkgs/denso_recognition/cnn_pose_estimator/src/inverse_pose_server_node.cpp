#include <cnn_pose_estimator/inverse_pose_server.h>

using inverse_pose_server::InversePoseServer;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "inverse_pose_server_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(1);

  InversePoseServer inverse_pose_server(nh);

  while (ros::ok())
  {
    ros::spinOnce();
    inverse_pose_server.broadcastInverseTF();
    rate.sleep();
  }

  return 0;
}