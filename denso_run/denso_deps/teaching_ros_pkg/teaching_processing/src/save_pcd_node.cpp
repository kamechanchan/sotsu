#include <teaching_processing/save_pcd.h>

using save_pcd::SavePCD;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_pcd_node");
  ros::NodeHandle nh;

  SavePCD pcd_saver(nh);

  ros::spin();

  return 0;
}