#include <gtest/gtest.h>
#include <abc_algorithm_def.hpp>
#include <iostream>

class UnitTest : public ::testing::Test
{
// public:
//   virtual void SetUp()
//   {
//     pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
//   }
//   // virtual void TearDown()  {}
//   bool countSegPoints(const std::string dir_path, const std::string result_pcd_name)
//   {
//     int points_num = 0;
//     PointLCloudT::Ptr result_voxel_cloud (new PointLCloudT);
//     pcl::PointCloud <pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile<PointLT> (result_pcd_name, *result_voxel_cloud);
//     creatingSegPCD(result_voxel_cloud, dir_path);
//     for (int num = 1; num <= result_voxel_cloud->points[result_voxel_cloud->size()-1].label; num++)
//     {
//       std::string part_result_pcd = dir_path;
//       part_result_pcd += "super_voxel_result_";
//       part_result_pcd += std::to_string(num);
//       part_result_pcd += ".pcd";
//       save_cloud->size();
//       pcl::io::loadPCDFile<pcl::PointXYZ> (part_result_pcd, *save_cloud);
//       points_num += save_cloud->size();
//     }
//     if (points_num == result_voxel_cloud->size())
//     {
//       return true;
//     }
//     else
//     {
//       return false;
//     }
//   }
//   PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
};

TEST_F(UnitTest, test1)
{
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  std::vector<float> buffer_vec = getParameters(cloud);
  for (int i = 0; i < buffer_vec.size(); i++)
  {
    std::cout << buffer_vec.at(i) << std::endl;
  }
}

TEST_F(UnitTest, test2)
{
  int bee_num = 2;
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  std::vector<std::vector<float>> buffer_vec = abc_algorithm_init(bee_num, cloud);
  EXPECT_EQ(buffer_vec.size(), bee_num);
}

TEST_F(UnitTest, test3)
{
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  std::vector<std::vector<float>> parameters_vec = {{0.000802847, 0.102486, 0, 0.81297, 0.904885, 0.396356, 0.68371},
                                                   {0.000521146, 0.0856555, 0, 0.392431, 0.362354, 0.0439183, 0.971675},
                                                   {0.000894813, 0.102434, 0, 0.554451, 0.417007, 0.39938, 0.171175}};
  std::vector<int> TC = {0,0,0};
  std::vector<std::vector<float>> result_parameters = abc_algorithm_employee_step(parameters_vec, TC, cloud);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      std::cout << result_parameters.at(i).at(j) << std::endl;
    }
    std::cout << "-------------" << std::endl;
  }
  for (int i = 0; i < 3; i++)
  {
    std::cout << TC.at(i) << std::endl;
  }
}

TEST_F(UnitTest, test4)
{
  std::vector<double> buff_vec = {0.4, 0.5, 0.5, 0.7, 0.8};
  std::cout << "roulette number: " << roulette_choice(buff_vec) << std::endl;
  std::vector<double> buff_vec2 = {0.8, 0.9, 1.5, 1.7};
  std::cout << "roulette number: " << roulette_choice(buff_vec2) << std::endl;
}

TEST_F(UnitTest, test5)
{
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  std::vector<std::vector<float>> parameters_vec = {{0.000802847, 0.102486, 0, 0.81297, 0.904885, 0.396356, 0.68371},
                                                   {0.000521146, 0.0856555, 0, 0.392431, 0.362354, 0.0439183, 0.971675},
                                                   {0.000894813, 0.102434, 0, 0.554451, 0.417007, 0.39938, 0.171175}};
  std::vector<int> TC = {0,0,0};
  std::vector<std::vector<float>> result_parameters = abc_algorithm_onlooker_step(parameters_vec, TC, cloud);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      std::cout << result_parameters.at(i).at(j) << std::endl;
    }
    std::cout << "-------------" << std::endl;
  }
  for (int i = 0; i < 3; i++)
  {
    std::cout << TC.at(i) << std::endl;
  }
}

TEST_F(UnitTest, test6)
{
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  std::vector<std::vector<float>> parameters_vec = {{0.000802847, 0.102486, 0, 0.81297, 0.904885, 0.396356, 0.68371},
                                                   {0.000521146, 0.0856555, 0, 0.392431, 0.362354, 0.0439183, 0.971675},
                                                   {0.000894813, 0.102434, 0, 0.554451, 0.417007, 0.39938, 0.171175}};
  std::vector<int> TC = {1,0,1};
  const int lim = 0;
  std::vector<std::vector<float>> result_parameters = abc_algorithm_scout_step(parameters_vec, TC, cloud, lim);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      std::cout << result_parameters.at(i).at(j) << std::endl;
    }
    std::cout << "-------------" << std::endl;
  }
  for (int i = 0; i < 3; i++)
  {
    std::cout << TC.at(i) << std::endl;
  }
}

TEST_F(UnitTest, test7)
{
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  std::vector<std::vector<float>> parameters_vec = {{0.000802847, 0.102486, 0, 0.81297, 0.904885, 0.396356, 0.68371},
                                                   {0.000521146, 0.0856555, 0, 0.392431, 0.362354, 0.0439183, 0.971675},
                                                   {0.000894813, 0.102434, 0, 0.554451, 0.417007, 0.39938, 0.171175}};
  std::vector<float> best_params_vec = {0};
  double IoU_score = 0;
  best_params_vec = abc_algorithm_get_best_parameters(parameters_vec, cloud, best_params_vec, IoU_score);
  std::cout << "----------------------" << std::endl;
  for (int i = 0; i < best_params_vec.size(); i++)
  {
    std::cout << best_params_vec.at(i) << std::endl;
  }
  std::cout << "----------------------" << std::endl;
  std::cout << "IoU: " << IoU_score << std::endl;
}

TEST_F(UnitTest, test8)
{
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  constexpr int bee_num = 2;
  constexpr int term = 2;
  constexpr int lim = 3;
  std::vector<float> best_params_vec = abc_algorithm(bee_num, term, lim, cloud);
  std::cout << "----------------------" << std::endl;
  for (int i = 0; i < best_params_vec.size(); i++)
  {
    std::cout << best_params_vec.at(i) << std::endl;
  }
  std::cout << "----------------------" << std::endl;
}
