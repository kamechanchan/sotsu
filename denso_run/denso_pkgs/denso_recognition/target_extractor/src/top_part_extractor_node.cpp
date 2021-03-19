#include <target_extractor/top_part_extractor.h>

using top_part_extractor::TopPartExtractor;

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "top_part_extractor");
  ros::NodeHandle nh;

  TopPartExtractor extractor(nh);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    extractor.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
