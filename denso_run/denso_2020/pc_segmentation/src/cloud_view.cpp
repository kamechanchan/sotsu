#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

// ビューワー起動時の一回だけ呼ばれる
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor(0, 0, 0);
  std::cout << "viewerOneOff" << std::endl;
}

int main(int argc, char *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 作成したPointCloudを読み込む
  pcl::io::loadPCDFile(argv[1], *p_cloud);

  // ビューワーの作成
  pcl::visualization::CloudViewer viewer("PointCloudViewer");
  viewer.showCloud(p_cloud);

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
