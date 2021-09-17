#include <pcd_ascii_save.hpp>

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor(0, 0, 0);
  std::cout << "viewerOneOff" << std::endl;
}

void addColor2PointCloud(const int i)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string pcd_name = "../pcd/part";
  pcd_name += std::to_string(i);
  pcd_name += ".pcd";
  pcl::io::loadPCDFile(pcd_name, *p_cloud);
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = p_cloud->points.begin(); pt < p_cloud->points.end(); pt++)
  {
    pcl::PointXYZRGB buffer_point;
    buffer_point.x = pt->x;
    buffer_point.y = pt->y;
    buffer_point.z = pt->z;
    buffer_point.r = color_data_[i-1][0];
    buffer_point.g = color_data_[i-1][1];
    buffer_point.b = color_data_[i-1][2];
    colored_points_.push_back(buffer_point);
  }
}

int main()
{
  for (int i = 1; i < 22; i++) { addColor2PointCloud(i); }
  // ビューワーの作成
  pcl::visualization::CloudViewer viewer("PointCloudViewer");
  viewer.showCloud(colored_points_.makeShared());
  pcl::io::savePCDFileASCII ("colored.pcd", colored_points_);

  // ビューワー起動時の一回だけ呼ばれる関数をセット
  viewer.runOnVisualizationThreadOnce(viewerOneOff);

  // // ビューワー起動中の毎フレーム実行される関数をセット
  // viewer.runOnVisualizationThread(viewerPsycho);

  // ビューワー視聴用ループ
  while (!viewer.wasStopped())
  {

  }
  return 0;
}
