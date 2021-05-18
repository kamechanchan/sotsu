#include <abc_algorithm_def.hpp>
#include <iostream>

int main()
{
  PointCloudT::Ptr cloud = std::make_unique<PointCloudT>();
  pcl::io::loadPCDFile<PointT> ("../pcd/part_in_box_v3.pcd", *cloud);
  constexpr int bee_num = 20;
  constexpr int term = 5;
  constexpr int lim = 3;
  std::vector<float> best_params_vec = abc_algorithm(bee_num, term, lim, cloud);
  std::cout << "----------------------" << std::endl;
  for (int i = 0; i < best_params_vec.size(); i++)
  {
    std::cout << best_params_vec.at(i) << std::endl;
  }
  std::cout << "----------------------" << std::endl;
}
