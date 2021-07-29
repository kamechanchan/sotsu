#include<cnn_pose_estimator/registration_cloud.hpp>

using registration_cloud::RegistrationCloud;

// Node
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "RegistrationCloud");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  RegistrationCloud registrator(nh, n);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    registrator.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
