#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include <vector>

pcl::PointCloud<pcl::PointXYZRGB> colored_points_;

const std::vector<std::vector<int>> color_data_ = {
                                      {255, 0, 0}, {0, 255, 0}, {0, 0, 255},
                                      {255, 255, 0}, {255, 0, 255}, {0, 255, 255},
                                      {128, 0, 0}, {0, 128, 0}, {0, 0, 128},
                                      {128, 128, 0}, {128, 0, 128}, {0, 128, 128},
                                      {128, 128, 255}, {128, 255, 128}, {255, 128, 128},
                                      {255, 255, 128}, {255, 128, 255}, {128, 255, 255},
                                      {0, 64, 128}, {64, 0, 128}, {64, 128, 0}
};
