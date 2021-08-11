#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/common/time.h>
#include <min_cut.hpp>

const double FOREGROUND_POINT_X = 0.359723;
const double FOREGROUND_POINT_Y = -0.00037846;
const double FOREGROUND_POINT_Z = 0.0740576;
const double REMOVAL_Z_MIN = 0.0;
const double REMOVAL_Z_MAX = 1.0;
const double SIGMA = 0.15;
const double OBJECT_SIZE = 0.045;
const int NEIGHBOURS_NUMBER = 15;
const double SOURCE_WEIGHT = 1.09;

int main (int argc, char** argv)
{
  pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("../pcd/colored.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  executeMinCutSeg(cloud, FOREGROUND_POINT_X, FOREGROUND_POINT_Y, FOREGROUND_POINT_Z,
                   REMOVAL_Z_MIN, REMOVAL_Z_MAX, SIGMA,
                   OBJECT_SIZE, NEIGHBOURS_NUMBER, SOURCE_WEIGHT);

  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ()) {}

  return (0);
}
