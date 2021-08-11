#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>

void makeHollowedPoints(const int index)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int part_num = 1; part_num < 22; part_num++)
  {
    if (index != part_num)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr buffer_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      std::string in_pcd_name = "../pcd/part_cloud/part";
      in_pcd_name += std::to_string(part_num);
      in_pcd_name += ".pcd";
      pcl::io::loadPCDFile(in_pcd_name, *buffer_cloud);
      for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = buffer_cloud->points.begin(); pt < buffer_cloud->points.end(); pt++)
      {
        pcl::PointXYZ buffer_point;
        buffer_point.x = pt->x;
        buffer_point.y = pt->y;
        buffer_point.z = pt->z;
        p_cloud->push_back(buffer_point);
      }
      std::cout << part_num << std::endl;
    }
  }
  std::string out_pcd_name = "../pcd/hollowed_part";
  out_pcd_name += std::to_string(index);
  out_pcd_name += "_cloud.pcd";
  pcl::io::savePCDFileASCII (out_pcd_name, *p_cloud);
}

int main()
{
  for (int i = 1; i < 22; i++) { makeHollowedPoints(i); }
  return 0;
}
