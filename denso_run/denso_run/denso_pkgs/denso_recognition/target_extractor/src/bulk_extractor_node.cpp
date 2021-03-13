#include <target_extractor/bulk_extractor.h>

using bulk_extractor::BulkExtractor;

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bulk_extractor");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  BulkExtractor extractor(nh, n);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    extractor.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
