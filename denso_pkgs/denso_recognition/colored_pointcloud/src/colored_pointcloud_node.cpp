#include <colored_pointcloud/colored_pointcloud.h>

using colored_pointcloud::ColoredPointCloud;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "colored_pointcloud");
  ros::NodeHandle nh("~");

  ColoredPointCloud colored_converter(nh);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    colored_converter.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
