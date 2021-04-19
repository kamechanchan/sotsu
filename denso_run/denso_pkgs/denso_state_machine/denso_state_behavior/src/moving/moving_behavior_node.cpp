#include <denso_state_behavior/moving/moving_behavior.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moving_behavior");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moving_behavior::MovingBehavior moving(nh, private_nh);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
