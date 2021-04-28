#include <gtest/gtest.h>
#include <super_voxel_def.hpp>
#include <iostream>

class UnitTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  }
  // virtual void TearDown()  {}
  bool countSegPoints(const std::string dir_path, const std::string result_pcd_name)
  {
    int points_num = 0;
    PointLCloudT::Ptr result_voxel_cloud (new PointLCloudT);
    pcl::PointCloud <pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<PointLT> (result_pcd_name, *result_voxel_cloud);
    creatingSegPCD(result_voxel_cloud, dir_path);
    for (int num = 1; num <= result_voxel_cloud->points[result_voxel_cloud->size()-1].label; num++)
    {
      std::string part_result_pcd = dir_path;
      part_result_pcd += "super_voxel_result_";
      part_result_pcd += std::to_string(num);
      part_result_pcd += ".pcd";
      save_cloud->size();
      pcl::io::loadPCDFile<pcl::PointXYZ> (part_result_pcd, *save_cloud);
      points_num += save_cloud->size();
    }
    if (points_num == result_voxel_cloud->size())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
};

TEST_F(UnitTest, test1)
{
  PointLCloudT::Ptr cluster_num = executeSuperVoxelSegmentation(cloud,
                                                                0.000801949f, 0.0870592f, 0.0f,
                                                                0.36686f, 0.14321f,
                                                                0.464037f, 0.317611f);
  EXPECT_GE(cluster_num->size(), 0);
  PointLCloudT::Ptr cluster_num2 = executeSuperVoxelSegmentation(cloud,
                                                                 0.000533856f, 0.0889073f, 0.0f,
                                                                 0.890372f, 0.106314f,
                                                                 0.989651f, 0.911755f);
  EXPECT_EQ(cluster_num2->size(), 0);
}

TEST_F(UnitTest, test2)
{
  std::string dir_path = "../test/pcd/creatingPCD/";
  EXPECT_TRUE(countSegPoints(dir_path, "../test/pcd/creatingPCD_test.pcd"));
  EXPECT_TRUE(countSegPoints(dir_path, "../test/pcd/creatingPCD_test2.pcd"));
  EXPECT_TRUE(countSegPoints(dir_path, "../test/pcd/creatingPCD_test3.pcd"));
  EXPECT_TRUE(countSegPoints(dir_path, "../test/pcd/creatingPCD_test4.pcd"));
  EXPECT_TRUE(countSegPoints(dir_path, "../test/pcd/creatingPCD_test5.pcd"));
  EXPECT_TRUE(countSegPoints(dir_path, "../test/pcd/creatingPCD_test6.pcd"));
  EXPECT_TRUE(countSegPoints(dir_path, "../test/pcd/creatingPCD_test7.pcd"));
  EXPECT_TRUE(countSegPoints(dir_path, "../test/pcd/creatingPCD_test8.pcd"));
}

TEST_F(UnitTest, test3)
{
  constexpr double resolution = 0.00524;
  constexpr int abs_error = 100;
  const std::string part1_path = "../pcd/part_cloud/part1.pcd";
  const std::string part1_2_path = "../test/pcd/part1_2.pcd";
  pcl::PointCloud <pcl::PointXYZ>::Ptr buffer_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud <pcl::PointXYZ>::Ptr buffer2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (part1_path, *buffer_cloud);
  pcl::io::loadPCDFile<pcl::PointXYZ> (part1_2_path, *buffer2_cloud);
  const int FP = calculateFP(buffer2_cloud, buffer_cloud, resolution);
  EXPECT_NEAR(FP, buffer2_cloud->size()-buffer_cloud->size(), abs_error);
}

TEST_F(UnitTest, test4)
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer
                                              ("3D Viewer"));
  PointLCloudT::Ptr result_voxel_cloud (new PointLCloudT);
  pcl::io::loadPCDFile<PointLT> ("../test/pcd/segmentation_example/super_voxel_result.pcd", *result_voxel_cloud);
  EXPECT_TRUE(pclVisualizerInitialize(viewer, result_voxel_cloud));
}

TEST_F(UnitTest, test5)
{
  PointLCloudT::Ptr result_voxel_cloud (new PointLCloudT);
  pcl::io::loadPCDFile<PointLT> ("../test/pcd/segmentation_example/super_voxel_result.pcd", *result_voxel_cloud);
  std::vector<std::vector<int>> correspondence =
    countMatchPoints(result_voxel_cloud, "../test/pcd/segmentation_example/", "../pcd/part_cloud/", 21, 10);
  std::cout << "========================" << std::endl;
  for (std::vector<std::vector<int>>::iterator
         itr = correspondence.begin(); itr != correspondence.end(); ++itr)
  {
    std::cout << "Seg-No." << itr->at(0) << " --> Ref-No."
              << itr->at(1) << "|   TP: " << itr->at(2) << "points, FP: "
              << itr->at(3) << "points, FN: " << itr->at(4) << std::endl;
  }
  std::cout << "========================" << std::endl;
  std::vector<std::vector<int>> correct_data = {{8, 1, 10391, 2868, 1534},
                                                {12, 6, 8963, 0, 2343},
                                                {1, 3, 6194, 5104, 823},
                                                {4, 4, 5360, 0, 795},
                                                {5, 2, 4995, 115, 3000},
                                                {13, 11, 4251, 14, 760},
                                                {11, 14, 1900, 0, 1501},
                                                {3, 5, 1550, 0, 5109},
                                                {9, 16, 1203, 761, 1967}};
  EXPECT_TRUE(correspondence == correct_data);
}

TEST_F(UnitTest, test6)
{
  std::vector<std::vector<int>> demo_data = {{8, 1, 10391, 2868, 1534},
                                             {12, 6, 8963, 0, 2343},
                                             {1, 3, 6194, 5104, 823},
                                             {4, 4, 5360, 0, 795},
                                             {5, 2, 4995, 115, 3000},
                                             {13, 11, 4251, 14, 760},
                                             {11, 14, 1900, 0, 1501},
                                             {3, 5, 1550, 0, 5109},
                                             {9, 16, 1203, 761, 1967}};
  EXPECT_NEAR(calculateIoU(demo_data), 0.62670955, 0.001);
  EXPECT_NEAR(calculateF1(demo_data), 0.77052422, 0.001);
  demo_data.clear();
  EXPECT_NEAR(calculateIoU(demo_data), 0.0, 0.001);
}

TEST_F(UnitTest, test7)
{
  PointLCloudT::Ptr result_voxel_cloud (new PointLCloudT);
  std::vector<std::vector<int>> correspondence =
    countMatchPoints(result_voxel_cloud, "../test/pcd/segmentation_example/", "../pcd/part_cloud/", 21, 10);
  std::vector<std::vector<int>> empty_data;
  EXPECT_TRUE(correspondence == empty_data);
}

TEST_F(UnitTest, test8)
{
  double IoU_value = evaluateSegbyIoU(cloud, 0.000801949f, 0.0870592f, 0.0f, 0.36686f, 0.14321f, 0.464037f, 0.317611f,
                                      "../test/pcd/segmentation_result_place/", "../pcd/part_cloud/", 21, 10);
  EXPECT_NEAR(IoU_value, 0.62670955, 0.001);
  IoU_value = evaluateSegbyIoU(cloud, 0.000533856f, 0.0889073f, 0.0f, 0.890372f, 0.106314f, 0.989651f, 0.911755f,
                               "../test/pcd/segmentation_result_place/", "../pcd/part_cloud/", 21, 10);
  EXPECT_NEAR(IoU_value, 0.0, 0.001);
  IoU_value = evaluateSegbyIoU(cloud, 0.000801949f, 0.0870592f, 0.0f, 0.36686f, 0.14321f, 0.464037f, 0.317611f,
                               "../test/pcd/segmentation_result_place/", "../pcd/part_cloud/", 21, 10);
  EXPECT_NEAR(IoU_value, 0.62670955, 0.001);
}

TEST_F(UnitTest, test9)
{
  double F1_value = evaluateSegbyF1(cloud, 0.000801949f, 0.0870592f, 0.0f, 0.36686f, 0.14321f, 0.464037f, 0.317611f,
                                      "../test/pcd/segmentation_result_place/", "../pcd/part_cloud/", 21, 10);
  EXPECT_NEAR(F1_value, 0.77052422, 0.001);
  F1_value = evaluateSegbyF1(cloud, 0.000533856f, 0.0889073f, 0.0f, 0.890372f, 0.106314f, 0.989651f, 0.911755f,
                               "../test/pcd/segmentation_result_place/", "../pcd/part_cloud/", 21, 10);
  EXPECT_NEAR(F1_value, 0.0, 0.001);
  F1_value = evaluateSegbyF1(cloud, 0.000801949f, 0.0870592f, 0.0f, 0.36686f, 0.14321f, 0.464037f, 0.317611f,
                               "../test/pcd/segmentation_result_place/", "../pcd/part_cloud/", 21, 10);
  EXPECT_NEAR(F1_value, 0.77052422, 0.001);
}
