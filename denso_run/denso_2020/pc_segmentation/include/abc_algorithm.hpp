#include <super_voxel_def.hpp>

constexpr float voxel_resolution_min = 0.001;
constexpr float voxel_resolution_max = 0.0001;
constexpr float seed_resolution_min = 0.085;
constexpr float seed_resolution_max = 0.105;
constexpr float color_importance_min = 0;
constexpr float color_importance_max = 0;
constexpr float spatial_importance_min = 0.01;
constexpr float spatial_importance_max = 1;
constexpr float normal_importance_min = 0.01;
constexpr float normal_importance_max = 1;
constexpr float search_radius_coeff_min = 0.01;
constexpr float search_radius_coeff_max = 0.99;
constexpr float min_points_coeff_min = 0.01;
constexpr float min_points_coeff_max = 0.99;

// True data setting
const std::string result_pcd_path_name = "../pcd/super_voxel_pcd/";
const std::string correct_pcd_path_name = "../pcd/part_cloud/";
constexpr int correct_data_num = 21;
constexpr int threshold_value = 10;

int roulette_choice(std::vector<double> w);
std::vector<float> getParameters(const PointCloudT::Ptr cloud);

std::vector<float> abc_algorithm(const int bee_num, const int term, const PointCloudT::Ptr cloud);
std::vector<std::vector<float>> abc_algorithm_init(const int bee_num, PointCloudT::Ptr cloud);
std::vector<std::vector<float>> abc_algorithm_employee_step(std::vector<std::vector<float>> parameters_vec,
                                                            std::vector<int> &TC, const PointCloudT::Ptr cloud);
std::vector<std::vector<float>> abc_algorithm_onlooker_step(std::vector<std::vector<float>> parameters_vec,
                                                            std::vector<int> &TC, const PointCloudT::Ptr cloud);
std::vector<std::vector<float>> abc_algorithm_scout_step(std::vector<std::vector<float>> parameters_vec, std::vector<int> &TC,
                                                         const PointCloudT::Ptr cloud, const int lim);
std::vector<float> abc_algorithm_get_best_parameters(std::vector<std::vector<float>> parameters_vec,
                                                     const PointCloudT::Ptr cloud,
                                                     std::vector<float> best_param_vec, double &IoU_score);
