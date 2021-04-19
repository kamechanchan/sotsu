#include <target_extractor/bulk_extractor.h>
#include <time.h>

using bulk_extractor::BulkExtractor;

// Class methods definitions
BulkExtractor::BulkExtractor(ros::NodeHandle& nh, ros::NodeHandle& n)
  : nh_(nh)
  , base_flag_(false)
  , octree_resolution_(0.02)
  , leaf_size_(0.004)
  , cluster_tolerance_(0.02)
  , cluster_min_size_(50)
  , cluster_max_size_(250000)
  , base_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
  , diff_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
  , extracted_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  extracted_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("bulk_cloud", 10);
  base_cloud_sub_ =
      nh.subscribe(n.param<std::string>("base_pc_topic_name", "/base_cloud"), 1, &BulkExtractor::updateBaseCloud, this);
  diff_filename_ = n.param<std::string>("diff_pc_filename", "diff_cloud.pcd");
}

void BulkExtractor::publish(void)
{
  clock_t start = clock();
  clock_t process1 = clock();
  clock_t process2 = clock();
  clock_t process3 = clock();
  clock_t process4 = clock();
  clock_t process5 = clock();
  clock_t process6 = clock();
  clock_t end = clock();

  if (base_flag_)
  {
    if (!crop_flag_)
    {
      loadDiffCloud();
      process1 = clock();
      extractDifference(base_cloud_, diff_cloud_);
      process2 = clock();
      removeOutlier(extracted_cloud_);
      process3 = clock();
      clustering(extracted_cloud_);
      process4 = clock();
      getCropCoordinate(extracted_cloud_);
      process5 = clock();
    }
    else
    {
      cropBulk(base_cloud_);
      process6 = clock();
    }

    extracted_cloud_->header.frame_id = base_cloud_->header.frame_id;

    // publish
    sensor_msgs::PointCloud2 published_cloud;
    pcl::toROSMsg(*extracted_cloud_, published_cloud);
    extracted_cloud_pub_.publish(published_cloud);

    // std::string model_filepath = ros::package::getPath("target_extractor") + "/pcd/";
    // std::string model_filename = "bulk_cloud.pcd";
    // pcl::io::savePCDFileASCII (model_filepath + model_filename, *extracted_cloud_);
    // std::cout << "save " << model_filename << std::endl;

    std::cout << "extracted bulk cloud" << std::endl;

    clock_t end = clock();
    const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
    const double load_time = static_cast<double>(process1 - start) / CLOCKS_PER_SEC * 1000.0;
    const double subtract_time = static_cast<double>(process2 - process1) / CLOCKS_PER_SEC * 1000.0;
    const double removal_time = static_cast<double>(process3 - process2) / CLOCKS_PER_SEC * 1000.0;
    const double clustering_time = static_cast<double>(process4 - process3) / CLOCKS_PER_SEC * 1000.0;
    const double getcoodinate_time = static_cast<double>(process5 - process4) / CLOCKS_PER_SEC * 1000.0;
    const double crop_time = static_cast<double>(process6 - start) / CLOCKS_PER_SEC * 1000.0;
    printf("time %lf[ms]\n\n", time);
    printf("load_time %lf[ms]\n\n", load_time);
    printf("subtract_time %lf[ms]\n\n", subtract_time);
    printf("removal_time %lf[ms]\n\n", removal_time);
    printf("clustering_time %lf[ms]\n\n", clustering_time);
    printf("getcoodinate_time %lf[ms]\n\n", getcoodinate_time);
    printf("crop_time %lf[ms]\n\n", crop_time);
  }
  else
  {
    std::cout << "Waiting for input base cloud..." << std::endl;
  }
}

void BulkExtractor::updateBaseCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *base_cloud_);
  downsampleCloud(base_cloud_, leaf_size_);
  base_flag_ = true;
}

void BulkExtractor::loadDiffCloud()
{
  std::cout << "Loading diff cloud..." << std::endl;
  std::string diff_filepath = ros::package::getPath("target_extractor") + "/pcd/";
  pcl::io::loadPCDFile(diff_filepath + diff_filename_, *diff_cloud_);
}

void BulkExtractor::downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, const double leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*input_cloud);
}

void BulkExtractor::segmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);  // 検出するモデルのタイプを指定
  seg.setMethodType(pcl::SAC_RANSAC);     // 検出に使用する方法を指定
  seg.setDistanceThreshold(0.01);         // RANSACの最小二乗法の許容誤差範囲
  seg.setMaxIterations(2000);
  seg.setProbability(0.95);

  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
  extract.filter(*input_cloud);
}

void BulkExtractor::removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(input_cloud);  // 外れ値を除去する点群を入力
  sor.setMeanK(50);                // MeanKを設定 点群数
  sor.setStddevMulThresh(0.1);
  sor.setNegative(false);    // 外れ値を出力する場合はtrueにする
  sor.filter(*input_cloud);  // 出力
}

void BulkExtractor::clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(cluster_min_size_);
  ec.setMaxClusterSize(cluster_max_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  uint32_t max_num = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr max_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // std::cout << "begin indices : " << cluster_indices.begin() << std::endl;
  // std::cout << "end indices   : " << cluster_indices.end() << std::endl;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if (cloud_cluster->width > max_num)
    {
      max_num = cloud_cluster->width;
      max_cloud = cloud_cluster;
    }
  }

  copyPointCloud(*max_cloud, *cloud);

  // Empty Buffer
  cluster_indices.clear();
}

void BulkExtractor::extractDifference(pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud)
{
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(octree_resolution_);  // Octreeを作成
  octree.setInputCloud(diff_cloud);  // 元となる点群を入力
  octree.addPointsFromInputCloud();
  octree.switchBuffers();            // バッファの切り替え
  octree.setInputCloud(base_cloud);  // 比較対象の点群を入力
  octree.addPointsFromInputCloud();

  std::vector<int> newPointIdxVector;
  octree.getPointIndicesFromNewVoxels(newPointIdxVector);  // 比較の結果差分と判断された点郡の情報を保管

  // 保管先のサイズの設定
  extracted_cloud_->width = diff_cloud->points.size() + base_cloud->points.size();
  extracted_cloud_->height = 1;
  extracted_cloud_->points.resize(extracted_cloud_->width * extracted_cloud_->height);

  int n = 0;  // 差分点群の数を保存する
  for (size_t i = 0; i < newPointIdxVector.size(); i++)
  {
    extracted_cloud_->points[i].x = base_cloud->points[newPointIdxVector[i]].x;
    extracted_cloud_->points[i].y = base_cloud->points[newPointIdxVector[i]].y;
    extracted_cloud_->points[i].z = base_cloud->points[newPointIdxVector[i]].z;
    n++;
  }
  // 差分点群のサイズの再設定
  extracted_cloud_->width = n;
  extracted_cloud_->height = 1;
  extracted_cloud_->points.resize(extracted_cloud_->width * extracted_cloud_->height);

  std::cout << "original cloud size  : " << base_cloud->width << std::endl;
  std::cout << "extracted cloud size : " << extracted_cloud_->width << std::endl;
}

void BulkExtractor::getCropCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
  pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
  feature_extractor.setInputCloud(input_cloud);
  feature_extractor.compute();

  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;
  pcl::PointXYZRGB min_point_AABB;
  pcl::PointXYZRGB max_point_AABB;
  pcl::PointXYZRGB min_point_OBB;
  pcl::PointXYZRGB max_point_OBB;
  pcl::PointXYZRGB position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;

  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  double crop_offset_x = 0.02;
  double crop_offset_y = 0.02;
  double crop_offset_z = 0;
  min_point_[0] = min_point_OBB.x + crop_offset_x;
  min_point_[1] = min_point_OBB.y + crop_offset_y;
  min_point_[2] = min_point_OBB.z + crop_offset_z;
  max_point_[0] = max_point_OBB.x - crop_offset_x;
  max_point_[1] = max_point_OBB.y - crop_offset_y;
  max_point_[2] = max_point_OBB.z - crop_offset_z;

  Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
  crop_translation_ = position;
  Eigen::AngleAxisf euler(rotational_matrix_OBB);
  crop_rotation_[0] = euler.axis()[0];
  crop_rotation_[1] = euler.axis()[1];
  crop_rotation_[2] = euler.axis()[2];

  crop_flag_ = true;
}

void BulkExtractor::cropBulk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
  pcl::CropBox<pcl::PointXYZRGB> cropFilter;
  cropFilter.setInputCloud(input_cloud);
  cropFilter.setMin(min_point_);
  cropFilter.setMax(max_point_);
  cropFilter.setTranslation(crop_translation_);
  cropFilter.setRotation(crop_rotation_);
  cropFilter.filter(*extracted_cloud_);
}
