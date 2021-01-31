#include<cnn_pose_estimator/validation_registration.hpp>

using validation_registration::ValidationRegistration;

// Node
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ValidationRegistration");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  ValidationRegistration validator(nh, n);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    validator.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
