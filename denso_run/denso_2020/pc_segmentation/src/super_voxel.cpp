#include <super_voxel_def.hpp>

int main (int argc, char **argv)
{

  float voxel_resolution = 0.000656593f;
  float seed_resolution = 0.0911658f;
  float color_importance = 0.0f;
  float spatial_importance = 0.299297f;
  float normal_importance = 0.130032f;
  float search_radius_coeff = 0.615881f;
  float min_points_coeff = 0.507035f;
  // std::string used_pcd_path = "../pcd/part_in_box_v3.pcd";
  // std::cout << std::getenv("HOME") << std::endl;
  std::string used_pcd_path = "../pcd/flat_hv8_v2.pcd";

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer
                                              ("3D Viewer"));
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> (used_pcd_path, *cloud);
  PointLCloudT::Ptr cluster_num =
    executeSuperVoxelSegmentation(cloud,
                                  voxel_resolution, seed_resolution, color_importance,
                                  spatial_importance, normal_importance,
                                  search_radius_coeff, min_points_coeff);
  pclVisualizerInitialize(viewer, cluster_num);
  while (!viewer->wasStopped ()) { viewer->spinOnce (100); }
  return (0);
}

