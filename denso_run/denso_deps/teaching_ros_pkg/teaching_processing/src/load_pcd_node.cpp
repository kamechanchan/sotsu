#include <teaching_processing/load_pcd.h>

using load_pcd::LoadPCD;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "load_pcd_node");
  ros::NodeHandle nh;

  LoadPCD pcd_loader(nh);

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    pcd_loader.publishPCD();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}