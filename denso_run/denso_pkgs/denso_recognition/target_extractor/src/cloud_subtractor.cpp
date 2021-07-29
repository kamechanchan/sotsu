#include <iostream>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

void Downsampling_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, float leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGBA> sor_model;
  sor_model.setInputCloud(cloud);
  sor_model.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor_model.filter(*cloud);
}

void Remove_outliers(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  sor.setInputCloud(cloud);  //外れ値を除去する点群を入力
  sor.setMeanK(50);          // MeanKを設定 点群数
  sor.setStddevMulThresh(0.1);
  sor.setNegative(false);  //外れ値を出力する場合はtrueにする
  sor.filter(*cloud);      //出力
}

void Clustering(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  // ec.setClusterTolerance(clusterTolerance_);
  // ec.setMinClusterSize(minSize_);
  // ec.setMaxClusterSize(maxSize_);
  // ec.setSearchMethod(tree);
  // ec.setInputCloud(cloud);
  // ec.extract(cluster_indices);
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(2500000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  uint32_t max_num = 0;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr max_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // std::cout << "begin indices : " << cluster_indices.begin() << std::endl;
  // std::cout << "end indices   : " << cluster_indices.end() << std::endl;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if (cloud_cluster->width > max_num)
    {
      max_num = cloud_cluster->width;
      max_cloud = cloud_cluster;
    }
  }

  copyPointCloud(*max_cloud, *cloud);

  // Viewer
  // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  // viewer->addCoordinateSystem (1.0);
  // viewer->initCameraParameters ();
  // viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  // viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z,
  // max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
  // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
  // pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

  // Empty Buffer
  cluster_indices.clear();
}

pcl::PointCloud<pcl::PointXYZRGBA> difference_extraction(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_base,
                                                         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_test)
{
  // cloud_baseは元となる点群
  // cloud_testは比較対象の点群
  // cloud_diffは比較の結果差分とされた点群

  double resolution = 0.02;  // Octreeの解像度を指定
  float leaf_size = 0.001;

  // Downsampling_cloud(cloud_base, leaf_size);
  // Downsampling_cloud(cloud_test, leaf_size);

  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> octree(resolution);  // Octreeを作成

  octree.setInputCloud(cloud_base);  //元となる点群を入力
  octree.addPointsFromInputCloud();

  octree.switchBuffers();  //バッファの切り替え

  octree.setInputCloud(cloud_test);  //比較対象の点群を入力
  octree.addPointsFromInputCloud();

  std::vector<int> newPointIdxVector;  //

  octree.getPointIndicesFromNewVoxels(newPointIdxVector);  //比較の結果差分と判断された点郡の情報を保管

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZRGBA>);  //出力先

  //保管先のサイズの設定
  cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
  cloud_diff->height = 1;
  cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);

  //保管先のサイズの設定
  cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
  cloud_diff->height = 1;
  cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);

  int n = 0;  //差分点群の数を保存する
  for (size_t i = 0; i < newPointIdxVector.size(); i++)
  {
    cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
    cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
    cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
    n++;
  }
  //差分点群のサイズの再設定
  cloud_diff->width = n;
  cloud_diff->height = 1;
  cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);

  std::cout << "original cloud size   : " << cloud_test->width << std::endl;
  std::cout << "difference cloud size : " << cloud_diff->width << std::endl;

  Remove_outliers(cloud_diff);
  Clustering(cloud_diff);

  std::cout << "final cloud size : " << cloud_diff->width << std::endl;
  return *cloud_diff;
}

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_test(new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::io::loadPCDFile(argv[1], *cloud_base);
  pcl::io::loadPCDFile(argv[2], *cloud_test);

  pcl::io::savePCDFileBinary("difference.pcd", difference_extraction(cloud_base, cloud_test));

  return (0);
}
