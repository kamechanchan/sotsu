#include <target_extractor/set_initial_cloud.h>

using set_initial_cloud::SetInitialCloud;

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_initial_cloud");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  SetInitialCloud extractor(nh, n);

  ros::Rate loop_rate(10);
  ros::spin();

  return 0;
}
