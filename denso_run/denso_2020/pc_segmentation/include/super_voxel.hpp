#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree.h>
#include <chrono>
#include <supervoxel_clustering_v2_def.hpp>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

bool pclVisualizerInitialize(pcl::visualization::PCLVisualizer::Ptr viewer, PointLCloudT::Ptr labeled_voxel_cloud);

PointLCloudT::Ptr executeSuperVoxelSegmentation(PointCloudT::Ptr input_cloud,
                                                float voxel_resolution, float seed_resolution, float color_importance,
                                                float spatial_importance, float normal_importance,
                                                float search_radius_coeff, float min_points_coeff);

int creatingSegPCD(PointLCloudT::Ptr result_voxel_cloud, std::string pcd_path);

std::chrono::system_clock::time_point start, end;

std::vector<std::vector<int>> countMatchPoints(const PointLCloudT::Ptr labeled_voxel_cloud,
                                               const std::string result_pcd_path,
                                               const std::string correct_pcd_path,
                                               const int correct_data_num, const int threshold);

int calculateFP(const pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud,
                const double octree_resolution);

double calculateIoU(std::vector<std::vector<int>> correspondence);

double calculateF1(std::vector<std::vector<int>> correspondence);

double evaluateSegbyIoU(PointCloudT::Ptr input_cloud,
                        float voxel_resolution, float seed_resolution, float color_importance,
                        float spatial_importance, float normal_importance,
                        float search_radius_coeff, float min_points_coeff,
                        const std::string result_pcd_path, const std::string correct_pcd_path,
                        const int correct_data_num, const int threshold);

double evaluateSegbyF1(PointCloudT::Ptr input_cloud,
                       float voxel_resolution, float seed_resolution, float color_importance,
                       float spatial_importance, float normal_importance,
                       float search_radius_coeff, float min_points_coeff,
                       const std::string result_pcd_path, const std::string correct_pcd_path,
                       const int correct_data_num, const int threshold);

