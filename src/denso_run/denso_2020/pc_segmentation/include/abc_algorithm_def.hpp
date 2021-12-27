#include <abc_algorithm.hpp>

std::vector<std::vector<float>> abc_algorithm_employee_step(std::vector<std::vector<float>> parameters_vec,
                                                            std::vector<int> &TC, const PointCloudT::Ptr cloud)
{
  std::cout << "============Employee Bee Step============" << std::endl;
  for (int i = 0; i < parameters_vec.size(); i++)
  {
    std::vector<std::vector<float>> buffer_vec = parameters_vec;
    int k = i;
    while (k == i) { k = (int)rand() % parameters_vec.size(); }
    srand((unsigned) time(NULL));
    double r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(i).at(0) = parameters_vec.at(i).at(0) + r * (parameters_vec.at(i).at(0) - parameters_vec.at(k).at(0));
    if (buffer_vec.at(i).at(0) < voxel_resolution_min) {buffer_vec.at(i).at(0) = voxel_resolution_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(i).at(1) = parameters_vec.at(i).at(1) + r * (parameters_vec.at(i).at(1) - parameters_vec.at(k).at(1));
    if (buffer_vec.at(i).at(1) < seed_resolution_min) {buffer_vec.at(i).at(1) = seed_resolution_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(i).at(2) = parameters_vec.at(i).at(2) + r * (parameters_vec.at(i).at(2) - parameters_vec.at(k).at(2));
    if (buffer_vec.at(i).at(2) < color_importance_min) {buffer_vec.at(i).at(2) = color_importance_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(i).at(3) = parameters_vec.at(i).at(3) + r * (parameters_vec.at(i).at(3) - parameters_vec.at(k).at(3));
    if (buffer_vec.at(i).at(3) < spatial_importance_min) {buffer_vec.at(i).at(3) = spatial_importance_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(i).at(4) = parameters_vec.at(i).at(4) + r * (parameters_vec.at(i).at(4) - parameters_vec.at(k).at(4));
    if (buffer_vec.at(i).at(4) < normal_importance_min) {buffer_vec.at(i).at(4) = normal_importance_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(i).at(5) = parameters_vec.at(i).at(5) + r * (parameters_vec.at(i).at(5) - parameters_vec.at(k).at(5));
    if (buffer_vec.at(i).at(5) < search_radius_coeff_min) {buffer_vec.at(i).at(5) = search_radius_coeff_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(i).at(6) = parameters_vec.at(i).at(6) + r * (parameters_vec.at(i).at(6) - parameters_vec.at(k).at(6));
    if (buffer_vec.at(i).at(6) < min_points_coeff_min) {buffer_vec.at(i).at(6) = min_points_coeff_min; }
    const double IoU_score = evaluateSegbyIoU(cloud, parameters_vec.at(i).at(0), parameters_vec.at(i).at(1),
                                              parameters_vec.at(i).at(2), parameters_vec.at(i).at(3),
                                              parameters_vec.at(i).at(4), parameters_vec.at(i).at(5),
                                              parameters_vec.at(i).at(6), result_pcd_path_name,
                                              "../pcd/part_cloud/", correct_data_num, threshold_value);
    const double buff_IoU_score = evaluateSegbyIoU(cloud, buffer_vec.at(i).at(0), buffer_vec.at(i).at(1),
                                                   buffer_vec.at(i).at(2), buffer_vec.at(i).at(3),
                                                   buffer_vec.at(i).at(4), buffer_vec.at(i).at(5),
                                                   buffer_vec.at(i).at(6), result_pcd_path_name,
                                                   "../pcd/part_cloud/", correct_data_num, threshold_value);
    if (IoU_score < buff_IoU_score)
    {
      parameters_vec.at(i) = buffer_vec.at(i);
      TC.at(i) = 0;
    }
    else { TC.at(i) += 1; }
  }
  return parameters_vec;
}

std::vector<std::vector<float>> abc_algorithm_init(const int bee_num, const PointCloudT::Ptr cloud)
{
  std::cout << "ABC algorithm initialize" << std::endl;
  std::vector<std::vector<float>> init_parameters;
  for (int i = 0; i < bee_num; i++) { init_parameters.push_back(getParameters(cloud)); }
  return init_parameters;
}

std::vector<std::vector<float>> abc_algorithm_onlooker_step(std::vector<std::vector<float>> parameters_vec,
                                                            std::vector<int> &TC, const PointCloudT::Ptr cloud)
{
  std::cout << "============Onlooker Bee Step============" << std::endl;
  for (int i = 0; i < parameters_vec.size(); i++)
  {
    std::vector<double> score_buff;
    for (int j = 0; j < parameters_vec.size(); j++)
    {
      score_buff.push_back(evaluateSegbyIoU(cloud, parameters_vec.at(i).at(0), parameters_vec.at(i).at(1),
                                            parameters_vec.at(i).at(2), parameters_vec.at(i).at(3),
                                            parameters_vec.at(i).at(4), parameters_vec.at(i).at(5),
                                            parameters_vec.at(i).at(6), result_pcd_path_name,
                                            "../pcd/part_cloud/", correct_data_num, threshold_value));
    }
    const int roulette_num = roulette_choice(score_buff);
    int k = roulette_num;
    std::cout << "Roulette's number: " << roulette_num << std::endl;
    while (k == roulette_num) { k = (int)rand() % parameters_vec.size(); }
    std::vector<std::vector<float>> buffer_vec = parameters_vec;
    srand((unsigned) time(NULL));
    double r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(roulette_num).at(0) = parameters_vec.at(roulette_num).at(0) +
      r * (parameters_vec.at(roulette_num).at(0) - parameters_vec.at(k).at(0));
    if (buffer_vec.at(roulette_num).at(0) < voxel_resolution_min) {buffer_vec.at(roulette_num).at(0) = voxel_resolution_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(roulette_num).at(1) = parameters_vec.at(roulette_num).at(1) +
      r * (parameters_vec.at(roulette_num).at(1) - parameters_vec.at(k).at(1));
    if (buffer_vec.at(roulette_num).at(1) < seed_resolution_min) {buffer_vec.at(roulette_num).at(1) = seed_resolution_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(roulette_num).at(2) = parameters_vec.at(roulette_num).at(2) +
      r * (parameters_vec.at(roulette_num).at(2) - parameters_vec.at(k).at(2));
    if (buffer_vec.at(roulette_num).at(2) < color_importance_min) {buffer_vec.at(roulette_num).at(2) = color_importance_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(roulette_num).at(3) = parameters_vec.at(roulette_num).at(3) +
      r * (parameters_vec.at(roulette_num).at(3) - parameters_vec.at(k).at(3));
    if (buffer_vec.at(roulette_num).at(3) < spatial_importance_min) {buffer_vec.at(roulette_num).at(3) = spatial_importance_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(roulette_num).at(4) = parameters_vec.at(roulette_num).at(4) +
      r * (parameters_vec.at(roulette_num).at(4) - parameters_vec.at(k).at(4));
    if (buffer_vec.at(roulette_num).at(4) < normal_importance_min) {buffer_vec.at(roulette_num).at(4) = normal_importance_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(roulette_num).at(5) = parameters_vec.at(roulette_num).at(5) +
      r * (parameters_vec.at(roulette_num).at(5) - parameters_vec.at(k).at(5));
    if (buffer_vec.at(roulette_num).at(5) < search_radius_coeff_min) {buffer_vec.at(roulette_num).at(5) = search_radius_coeff_min; }
    srand((unsigned) time(NULL));
    r = (double)rand() / RAND_MAX * 2 - 1;
    buffer_vec.at(roulette_num).at(6) = parameters_vec.at(roulette_num).at(6) +
      r * (parameters_vec.at(roulette_num).at(6) - parameters_vec.at(k).at(6));
    if (buffer_vec.at(roulette_num).at(6) < min_points_coeff_min) {buffer_vec.at(roulette_num).at(6) = min_points_coeff_min; }
    const double IoU_score =
      evaluateSegbyIoU(cloud, parameters_vec.at(roulette_num).at(0), parameters_vec.at(roulette_num).at(1),
                       parameters_vec.at(roulette_num).at(2), parameters_vec.at(roulette_num).at(3),
                       parameters_vec.at(roulette_num).at(4), parameters_vec.at(roulette_num).at(5),
                       parameters_vec.at(roulette_num).at(6), result_pcd_path_name,
                       "../pcd/part_cloud/", correct_data_num, threshold_value);
    const double buff_IoU_score =
      evaluateSegbyIoU(cloud, buffer_vec.at(roulette_num).at(0), buffer_vec.at(roulette_num).at(1),
                       buffer_vec.at(roulette_num).at(2), buffer_vec.at(roulette_num).at(3),
                       buffer_vec.at(roulette_num).at(4), buffer_vec.at(roulette_num).at(5),
                       buffer_vec.at(roulette_num).at(6), result_pcd_path_name,
                       "../pcd/part_cloud/", correct_data_num, threshold_value);
    if (IoU_score < buff_IoU_score)
    {
      std::cout << "No." << roulette_num << " parameters changed" << std::endl;
      parameters_vec.at(roulette_num) = buffer_vec.at(roulette_num);
      TC.at(roulette_num) = 0;
    }
    else { TC.at(roulette_num) += 1; }
  }
  return parameters_vec;
}

std::vector<float> getParameters(const PointCloudT::Ptr cloud)
{
  std::vector<float> buff_vec;
  srand((unsigned) time(NULL));
  buff_vec.push_back((voxel_resolution_max - voxel_resolution_min) *
                     (float)rand() / RAND_MAX + voxel_resolution_min);
  buff_vec.push_back((seed_resolution_max - seed_resolution_min) *
                     (float)rand() / RAND_MAX + seed_resolution_min);
  buff_vec.push_back((color_importance_max - color_importance_min) *
                     (float)rand() / RAND_MAX + color_importance_min);
  buff_vec.push_back((spatial_importance_max - spatial_importance_min) *
                     (float)rand() / RAND_MAX + spatial_importance_min);
  buff_vec.push_back((normal_importance_max - normal_importance_min) *
                     (float)rand() / RAND_MAX + normal_importance_min);
  buff_vec.push_back((search_radius_coeff_max - search_radius_coeff_min) *
                     (float)rand() / RAND_MAX + search_radius_coeff_min);
  buff_vec.push_back((min_points_coeff_max - min_points_coeff_min) *
                     (float)rand() / RAND_MAX + min_points_coeff_min);
  if (evaluateSegbyIoU(cloud, buff_vec.at(0), buff_vec.at(1), buff_vec.at(2),
                       buff_vec.at(3), buff_vec.at(4), buff_vec.at(5), buff_vec.at(6),
                       result_pcd_path_name, "../pcd/part_cloud/", correct_data_num, threshold_value) != 0)
  {
    return buff_vec;
  }
  else { getParameters(cloud); }
}

int roulette_choice(std::vector<double> w)
{
  std::vector<double> tot;
  double acc = 0;
  for (auto itr = w.begin(); itr != w.end(); ++itr)
  {
    acc += *itr;
    tot.push_back(acc);
  }
  double r = (double)rand() / RAND_MAX * acc;
  int count = 0;
  // std::cout << "roulette value: " << r << std::endl;
  for (auto itr = tot.begin(); itr != tot.end(); ++itr, count++)
  {
    if (r <= *itr)
    {
      return count;
    }
  }
}

std::vector<float> abc_algorithm_get_best_parameters(std::vector<std::vector<float>> parameters_vec,
                                                     const PointCloudT::Ptr cloud,
                                                     std::vector<float> best_param_vec, double &IoU_score)
{
  for (int i = 0; i < parameters_vec.size(); i++)
  {
    const double buff_IoU_score = evaluateSegbyIoU(cloud, parameters_vec.at(i).at(0), parameters_vec.at(i).at(1),
                                                   parameters_vec.at(i).at(2), parameters_vec.at(i).at(3),
                                                   parameters_vec.at(i).at(4), parameters_vec.at(i).at(5),
                                                   parameters_vec.at(i).at(6), result_pcd_path_name,
                                                   "../pcd/part_cloud/", correct_data_num, threshold_value);
    if (IoU_score < buff_IoU_score)
    {
      best_param_vec = parameters_vec.at(i);
      IoU_score = buff_IoU_score;
    }
  }
  return best_param_vec;
}


std::vector<float> abc_algorithm(const int bee_num, const int term, const int lim, const PointCloudT::Ptr cloud)
{
  std::vector<float> best_param_vec;
  std::string answer;
  std::cout << "Do you use the " << correct_data_num <<
    " data in " << correct_pcd_path_name << " as the correct data?[y/n]: ";
  std::cin >> answer;
  if (answer == "no" || answer == "NO" || answer == "N" || answer == "n")
  {
    return best_param_vec;
  }
  std::vector<std::vector<float>> parameters_vec = abc_algorithm_init(bee_num, cloud);
  std::vector<int> TC(bee_num);
  double IoU_score = 0;
  std::string filename = "./abc_algorithm_result.txt";
  std::ofstream writing_file;
  std::chrono::system_clock::time_point start, end;
  writing_file.open(filename, std::ios::out);
  start = std::chrono::system_clock::now();
  writing_file << "=======Initial Condition=======" << std::endl;
  writing_file << "Number of Term: " << term << std::endl;
  writing_file << "Number of bee: " <<  bee_num << std::endl;
  writing_file << "Number of limit: " <<  lim << std::endl;
  writing_file << "voxel resolution min: " << voxel_resolution_min << std::endl;
  writing_file << "voxel resolution max: " << voxel_resolution_max << std::endl;
  writing_file << "seed resolution min: " << seed_resolution_min << std::endl;
  writing_file << "seed resolution max: " << seed_resolution_max << std::endl;
  writing_file << "color importance min: " << color_importance_min << std::endl;
  writing_file << "color importance max: " << color_importance_max << std::endl;
  writing_file << "spatial_importance min: " << spatial_importance_min << std::endl;
  writing_file << "spatial_importance max: " << spatial_importance_max << std::endl;
  writing_file << "normal_importance min: " << normal_importance_min << std::endl;
  writing_file << "normal_importance max: " << normal_importance_max << std::endl;
  writing_file << "search_radius_coeff min: " << search_radius_coeff_min << std::endl;
  writing_file << "search_radius_coeff max: " << search_radius_coeff_max << std::endl;
  writing_file << "min_points_coeff min: " << min_points_coeff_min << std::endl;
  writing_file << "min_points_coeff max: " << min_points_coeff_max << std::endl;
  writing_file << "===============================" << std::endl << std::endl;;
  for (int g = 0; g < term; g++)
  {
    parameters_vec = abc_algorithm_employee_step(parameters_vec, TC, cloud);
    parameters_vec = abc_algorithm_onlooker_step(parameters_vec, TC, cloud);
    parameters_vec = abc_algorithm_scout_step(parameters_vec, TC, cloud, lim);
    best_param_vec = abc_algorithm_get_best_parameters(parameters_vec, cloud, best_param_vec, IoU_score);
    writing_file << "Term: " << g+1 << " result" << std::endl;
    writing_file << "=========================" << std::endl;
    writing_file << "voxel resolution: " << best_param_vec.at(0) << std::endl;
    writing_file << "seed resolution: " << best_param_vec.at(1) << std::endl;
    writing_file << "color importance: " << best_param_vec.at(2) << std::endl;
    writing_file << "spatial_importance: " << best_param_vec.at(3) << std::endl;
    writing_file << "normal_importance: " << best_param_vec.at(4) << std::endl;
    writing_file << "search_radius_coeff: " << best_param_vec.at(5) << std::endl;
    writing_file << "min_points_coeff: " << best_param_vec.at(6) << std::endl;
    writing_file << "IoU score: " << IoU_score << std::endl << std::endl;
  }
  end = std::chrono::system_clock::now();
  double time = static_cast<double>
    (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
  writing_file << "ABC-Algorithm Processing Time: " << time <<
    "[ms]" << std::endl << std::endl;
  return best_param_vec;
}

std::vector<std::vector<float>> abc_algorithm_scout_step(std::vector<std::vector<float>> parameters_vec, std::vector<int> &TC,
                                                         const PointCloudT::Ptr cloud, const int lim)
{
  std::cout << "============Scout Bee Step============" << std::endl;
  for (int i = 0; i < parameters_vec.size(); i++)
  {
    if (TC.at(i) > lim)
    {
      parameters_vec.at(i) = getParameters(cloud);
      TC.at(i) = 0;
    }
  }
  return parameters_vec;
}
