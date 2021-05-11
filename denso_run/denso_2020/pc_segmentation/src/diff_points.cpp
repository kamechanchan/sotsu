#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <vector>
#include <ctime>

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor(0, 0, 0);
  std::cout << "viewerOneOff" << std::endl;
}

void difference_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff)
{
  //cloud_baseは元となる点群
  //cloud_testは比較対象の点群
  //cloud_diffは比較の結果差分とされた点群

  double resolution = 0.0065;
  // double resolution = 0.00524;//Octreeの解像度を指定

  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);//Octreeを作成

  octree.setInputCloud (cloud_base);//元となる点群を入力
  octree.addPointsFromInputCloud ();

  octree.switchBuffers ();//バッファの切り替え

  octree.setInputCloud (cloud_test);//比較対象の点群を入力
  octree.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector;//

  octree.getPointIndicesFromNewVoxels (newPointIdxVector);//比較の結果差分と判断された点郡の情報を保管

  std::cout << newPointIdxVector.size() << std::endl;
 
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ> );//出力先


  //保管先のサイズの設定
  cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
  cloud_diff->height = 1;
  cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

  int n = 0;//差分点群の数を保存する
  for(size_t i = 0; i < newPointIdxVector.size (); i++)
  {
      cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
      cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
      cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
      n++;
  }
  //差分点群のサイズの再設定
  cloud_diff->width = n;
  cloud_diff->height = 1;
  cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);
}

int main(void)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ> );

  pcl::io::loadPCDFile("../pcd/hollowed_part_cloud/hollowed_part1_cloud.pcd",*cloud_base);
  pcl::io::loadPCDFile("../pcd/super_voxel_pcd/super_voxel_result_6.pcd", *cloud_test);
  difference_extraction(cloud_base, cloud_test, cloud_diff);

  std::cout << cloud_diff->size() << std::endl;
  // &cloud_diff = difference_extraction(cloud_base, cloud_test);

  pcl::visualization::CloudViewer viewer("PointCloudViewer");
  viewer.showCloud(cloud_diff);

  // ビューワー起動時の一回だけ呼ばれる関数をセット
  viewer.runOnVisualizationThreadOnce(viewerOneOff);

  // // ビューワー起動中の毎フレーム実行される関数をセット
  // viewer.runOnVisualizationThread(viewerPsycho);

  // ビューワー視聴用ループ
  while (!viewer.wasStopped())
  {

  }


  return (0);
}
