#include <super_voxel.hpp>

bool pclVisualizerInitialize(pcl::visualization::PCLVisualizer::Ptr viewer, PointLCloudT::Ptr labeled_voxel_cloud)
{
  try
  {
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");
    return true;
  }
  catch (std::exception ex)
  {
    std::cout << "Error: " << ex.what() << std::endl;
    return false;
  }
}

PointLCloudT::Ptr executeSuperVoxelSegmentation(PointCloudT::Ptr input_cloud,
                                                float voxel_resolution, float seed_resolution, float color_importance,
                                                float spatial_importance, float normal_importance,
                                                float search_radius_coeff, float min_points_coeff)
{
  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
  PointLCloudT::Ptr labeled_voxel_cloud;
  super.setInputCloud (input_cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  search_radius = search_radius_coeff * seed_resolution;
  min_points = min_points_coeff * (search_radius)*(search_radius) *
    3.1415926536f  / (voxel_resolution * voxel_resolution);
  super.extract (supervoxel_clusters);
  labeled_voxel_cloud = super.getLabeledVoxelCloud();
  std::cout << "=====Segmentation Result=====" << std::endl;
  std::cout << "Number of SuperVoxels: " << supervoxel_clusters.size() << std::endl;
  std::cout << "Size of PointCloud: " << labeled_voxel_cloud->size() << std::endl;
  std::cout << "=============================" << std::endl;
  return labeled_voxel_cloud;
}

int creatingSegPCD(PointLCloudT::Ptr result_voxel_cloud, std::string pcd_path)
{
  int count = 1;
  pcl::PointCloud <pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const char* path = pcd_path.c_str();
  struct stat st;
  if (stat(path, &st) != 0)
  {
    mkdir(path, 0777);
    std::cout << "Dataset directory created!!" << std::endl;
  }
  else
  {
    std::string sys_call = "rm ";
    sys_call += pcd_path;
    sys_call += "*.pcd";
    std::system(sys_call.c_str());
    std::cout << "Delete PCD files" << std::endl;
  }

  if (result_voxel_cloud->size() == 0)
  {
    return 0;
  }
  else
  {
    std::string result_pcd_name = pcd_path;
    std::string part_pcd_name = pcd_path;
    result_pcd_name += "super_voxel_result.pcd";
    part_pcd_name += "super_voxel_result_";
    pcl::io::savePCDFileASCII (result_pcd_name, *result_voxel_cloud);
    for (pcl::PointCloud<pcl::PointXYZL>::iterator pt = result_voxel_cloud->points.begin();
         pt < result_voxel_cloud->points.end(); pt++)
    {
      pcl::PointXYZ buffer_point;
      buffer_point.x = pt->x;
      buffer_point.y = pt->y;
      buffer_point.z = pt->z;
      if (pt->label == count)
      {
        save_cloud->push_back(buffer_point);
      }
      else
      {
        std::string out_pcd_name = part_pcd_name;
        out_pcd_name += std::to_string(count);
        out_pcd_name += ".pcd";
        pcl::io::savePCDFileASCII (out_pcd_name, *save_cloud);
        save_cloud->clear();
        save_cloud->push_back(buffer_point);
        count++;
      }
    }
    std::string out_pcd_name = part_pcd_name;
    out_pcd_name += std::to_string(count);
    out_pcd_name += ".pcd";
    pcl::io::savePCDFileASCII (out_pcd_name, *save_cloud);
  }
  return count;
}


std::vector<std::vector<int>> countMatchPoints(const PointLCloudT::Ptr labeled_voxel_cloud,
                                               const std::string result_pcd_path,
                                               const std::string correct_pcd_path,
                                               const int correct_data_num, const int threshold)
{
  std::vector<std::vector<int>> correspondence;
  pcl::PointCloud<pcl::PointXYZ>::Ptr buff_cloud (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr buff_cloud2 (new pcl::PointCloud<pcl::PointXYZ> );
  if (labeled_voxel_cloud->size() == 0)
  {
    return correspondence;
  }
  for (int i = 1; i <= labeled_voxel_cloud->points[labeled_voxel_cloud->size()-1].label; i++)
  {
    std::string input_pcd_name = result_pcd_path;
    input_pcd_name += "super_voxel_result_";
    input_pcd_name += std::to_string(i);
    input_pcd_name += ".pcd";
    buff_cloud->clear();
    pcl::io::loadPCDFile(input_pcd_name, *buff_cloud);
    for (int j = 1; j <= correct_data_num; j++)
    {
      std::string ref_pcd_name = correct_pcd_path;
      ref_pcd_name += "part";
      ref_pcd_name += std::to_string(j);
      ref_pcd_name += ".pcd";
      buff_cloud2->clear();
      pcl::io::loadPCDFile(ref_pcd_name, *buff_cloud2);
      int FP_buff = calculateFP(buff_cloud, buff_cloud2, 0.0065);
      int TP_buff = buff_cloud->size() - FP_buff;
      if (TP_buff > threshold)
      {
        std::vector<int> buff_vec;
        buff_vec.push_back(i); // Seg-Num
        buff_vec.push_back(j); // Ref-Num
        buff_vec.push_back(TP_buff); // TP
        buff_vec.push_back(FP_buff); // FP
        buff_vec.push_back(buff_cloud2->size() - TP_buff); // FN
        correspondence.push_back(buff_vec);
      }
    }
  }
  std::sort(correspondence.begin(),correspondence.end(),[]
            (const std::vector<int> &alpha,const std::vector<int> &beta)
                                                        {return alpha[2] > beta[2];});
  std::vector<int> detected_num;
  std::vector<std::vector<int>> segmentation_data;
  for (std::vector<std::vector<int>>::iterator
         itr = correspondence.begin(); itr != correspondence.end(); ++itr)
  {
    auto result = std::find(detected_num.begin(), detected_num.end(), itr->at(0));
    if (result == detected_num.end())
    {
      std::vector<int> seg_data_buff;
      seg_data_buff.push_back(itr->at(0));
      seg_data_buff.push_back(itr->at(1));
      seg_data_buff.push_back(itr->at(2));
      seg_data_buff.push_back(itr->at(3));
      seg_data_buff.push_back(itr->at(4));
      segmentation_data.push_back(seg_data_buff);
      detected_num.push_back(itr->at(0));
    }
  }
  return segmentation_data;
}

int calculateFP(const pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud,
                const double octree_resolution)
{
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (octree_resolution);//Octreeを作成
  octree.setInputCloud (diff_cloud);
  octree.addPointsFromInputCloud ();
  octree.switchBuffers ();
  octree.setInputCloud (orig_cloud);
  octree.addPointsFromInputCloud ();
  std::vector<int> newPointIdxVector;
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);//比較の結果差分と判断された点郡の情報を保管
  return  newPointIdxVector.size();
}

double calculateIoU(std::vector<std::vector<int>> correspondence)
{
  try
  {
    long unsigned int all_TP = 0;
    long unsigned int all_FP = 0;
    long unsigned int all_FN = 0;
    for (std::vector<std::vector<int>>::iterator
           itr = correspondence.begin(); itr != correspondence.end(); ++itr)
    {
      all_TP += itr->at(2);
      all_FP += itr->at(3);
      all_FN += itr->at(4);
    }
    if (all_TP == 0 && all_FP == 0 && all_FP == 0) { return 0; }
    double IoU = (double)all_TP / (double)(all_TP + all_FP + all_FN);
    return IoU;
  }
  catch (std::exception ex)
  {
    return 0;
  }
}

double calculateF1(std::vector<std::vector<int>> correspondence)
{
  try
  {
    long unsigned int all_TP = 0;
    long unsigned int all_FP = 0;
    long unsigned int all_FN = 0;
    for (std::vector<std::vector<int>>::iterator
           itr = correspondence.begin(); itr != correspondence.end(); ++itr)
    {
      all_TP += itr->at(2);
      all_FP += itr->at(3);
      all_FN += itr->at(4);
    }
    if (all_TP == 0 && all_FP == 0 && all_FP == 0) { return 0; }
    long double precision = (double)all_TP / (all_TP + all_FP);
    long double recall = (double)all_TP / (all_TP + all_FN);
    double F1 = 2 * ((precision * recall) / (precision + recall));
    return F1;
  }
  catch (std::exception ex)
  {
    return 0;
  }
}

double evaluateSegbyIoU(PointCloudT::Ptr input_cloud,
                        float voxel_resolution, float seed_resolution, float color_importance,
                        float spatial_importance, float normal_importance,
                        float search_radius_coeff, float min_points_coeff,
                        const std::string result_pcd_path, const std::string correct_pcd_path,
                        const int correct_data_num, const int threshold)
{
  PointLCloudT::Ptr result_voxel_cloud
    = executeSuperVoxelSegmentation(input_cloud, voxel_resolution, seed_resolution, color_importance,
                                    spatial_importance, normal_importance, search_radius_coeff, min_points_coeff);
  int segmentation_num = creatingSegPCD(result_voxel_cloud, result_pcd_path);
  if (segmentation_num == 0) { return 0; }
  std::vector<std::vector<int>> segmentation_data
    = countMatchPoints(result_voxel_cloud, result_pcd_path, correct_pcd_path, correct_data_num, threshold);
  return calculateIoU(segmentation_data);
}

double evaluateSegbyF1(PointCloudT::Ptr input_cloud,
                       float voxel_resolution, float seed_resolution, float color_importance,
                       float spatial_importance, float normal_importance,
                       float search_radius_coeff, float min_points_coeff,
                       const std::string result_pcd_path, const std::string correct_pcd_path,
                       const int correct_data_num, const int threshold)
{
  PointLCloudT::Ptr result_voxel_cloud
    = executeSuperVoxelSegmentation(input_cloud, voxel_resolution, seed_resolution, color_importance,
                                    spatial_importance, normal_importance, search_radius_coeff, min_points_coeff);
  int segmentation_num = creatingSegPCD(result_voxel_cloud, result_pcd_path);
  if (segmentation_num == 0) { return 0; }
  std::vector<std::vector<int>> segmentation_data
    = countMatchPoints(result_voxel_cloud, result_pcd_path, correct_pcd_path, correct_data_num, threshold);
  return calculateF1(segmentation_data);
}

