#include <cnn_pose_estimator/make_input_server.h>

using make_input_server::MakeInputServer;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "make_input_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(1);

  MakeInputServer make_input_server(nh, n);

  while (ros::ok())
  {
    ros::spinOnce();
    make_input_server.publishVoxelArray();
    rate.sleep();
  }

  return 0;
}
