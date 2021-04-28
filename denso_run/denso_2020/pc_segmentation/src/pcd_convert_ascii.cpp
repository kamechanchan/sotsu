#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>

const std::string pcd_name = "../pcd/table_scene_mug_stereo_textured";

int main(void)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string ori_pcd_name = pcd_name;
  ori_pcd_name += ".pcd";
  pcl::io::loadPCDFile(ori_pcd_name, *p_cloud);
  std::string new_pcd_name = pcd_name;
  new_pcd_name += "_ascii.pcd";
  pcl::io::savePCDFileASCII (new_pcd_name, *p_cloud);
  return 0;
}
