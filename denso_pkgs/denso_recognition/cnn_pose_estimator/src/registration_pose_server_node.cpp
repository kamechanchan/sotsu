#include <cnn_pose_estimator/registration_pose_server.h>

using registration_pose_server::RegistrationPoseServer;

// Node
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "RegistrationCloud");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RegistrationPoseServer registrator(nh, n);

  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    registrator.publishRegistratedCloudAndTF();
    rate.sleep();
  }

  return 0;
}
