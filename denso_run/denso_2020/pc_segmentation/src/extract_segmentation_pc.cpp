#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

const std::vector<std::vector<int>> color_data_ = {
                                      {255, 0, 0}, {0, 255, 0}, {0, 0, 255},
                                      {255, 255, 0}, {255, 0, 255}, {0, 255, 255},
                                      {128, 0, 0}, {0, 128, 0}, {0, 0, 128},
                                      {128, 128, 0}, {128, 0, 128}, {0, 128, 128},
                                      {128, 128, 255}, {128, 255, 128}, {255, 128, 128},
                                      {255, 255, 128}, {255, 128, 255}, {128, 255, 255},
                                      {0, 64, 128}, {64, 0, 128}, {64, 128, 0}
};

const unsigned int segmentation_number = 3;
pcl::PointCloud<pcl::PointXYZRGB> colored_points_;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor(0.2, 0.2, 0.2);
  std::cout << "viewerOneOff" << std::endl;
}

void extractSegPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud, unsigned int seg_num)
{
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = p_cloud->points.begin(); pt < p_cloud->points.end(); pt++)
  {
    if (pt->r == color_data_[seg_num-1][0] && pt->g == color_data_[seg_num-1][1] && pt->b == color_data_[seg_num-1][2])
    {
      pcl::PointXYZRGB buffer_point;
      buffer_point.x = pt->x;
      buffer_point.y = pt->y;
      buffer_point.z = pt->z;
      buffer_point.r = color_data_[seg_num-1][0];
      buffer_point.g = color_data_[seg_num-1][1];
      buffer_point.b = color_data_[seg_num-1][2];
      colored_points_.push_back(buffer_point);
    }
  }
}


int main(void)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // 作成したPointCloudを読み込む
  pcl::io::loadPCDFile("../pcd/colored.pcd", *p_cloud);
  extractSegPointCloud(p_cloud, segmentation_number);
  pcl::visualization::CloudViewer viewer("PointCloudViewer");
  viewer.showCloud(colored_points_.makeShared());
  viewer.runOnVisualizationThreadOnce(viewerOneOff);

  // // ビューワー起動中の毎フレーム実行される関数をセット
  // viewer.runOnVisualizationThread(viewerPsycho);

  // ビューワー視聴用ループ
  while (!viewer.wasStopped())
  {

  }
  return 0;
}
