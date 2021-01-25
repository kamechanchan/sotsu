#include <cnn_pose_estimator/registration_pose_server.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>
#include <time.h>

using registration_pose_server::RegistrationPoseServer;

RegistrationPoseServer::RegistrationPoseServer(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh)
  , model_filename_(n.param<std::string>("model_name", "hv6"))
  , algorithm_name_(n.param<std::string>("method_name", "ICP"))
  , estimated_frame_id_(n.param<std::string>("estimate_frame_id", "estimate_tf"))
  , sensor_frame_id_(n.param<std::string>("sensor_frame_id", "photoneo_center_optical_frame"))
  , registrated_frame_id_(n.param<std::string>("registrated_frame_id", "jig_HV6_0"))
  , max_icp_iteration_(n.param<int>("max_icp_iteration", 500))
  , leaf_scene_(n.param<float>("leaf_scene", 0))
  , leaf_model_(n.param<float>("leaf_model", 0))
  , model_plane_threshold_(n.param<float>("model_plane_threshold", 0.005))  // HV6
  // ,plane_threshold_(0.016) // HV8
  , scene_plane_threshold_(n.param<float>("scene_plane_threshold", 0.003))  // HV6
  // ,scene_plane_threshold_(0.012); // HV8
  , model_planable_ratio_(n.param<float>("model_planable_ratio", 0.1))
  , scene_planable_ratio_(n.param<float>("scene_planable_ratio", 0.3))
  , use_remove_plane_(n.param<bool>("use_remove_plane", false))
  , is_ok_(false)
  , registrate_transform_()
  , up_model_cloud_(new pcl::PointCloud<PointInT>)
  , down_model_cloud_(new pcl::PointCloud<PointInT>)
  , scene_cloud_(new pcl::PointCloud<PointInT>)
  , converted_up_model_(new pcl::PointCloud<PointInT>)
  , converted_down_model_(new pcl::PointCloud<PointInT>)
  , converted_scene_(new pcl::PointCloud<PointInT>)
  , registrated_cloud_(new pcl::PointCloud<PointInT>)
{
  transformed_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/model_pc", 1);
  registrated_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/registrated_model_pc", 1);
  object_cloud_sub_ = nh.subscribe(n.param<std::string>("source_pc", "/filtered_pointcloud"), 1,
                                   &RegistrationPoseServer::getSceneCloud, this);
  registrate_pose_server_ =
      nh.advertiseService("/cnn_pose_estimator/registrate_pose", &RegistrationPoseServer::registratePose, this);

  RegistrationPoseServer::loadModelCloud(up_model_cloud_, "up");
  RegistrationPoseServer::loadModelCloud(down_model_cloud_, "down");
}

void RegistrationPoseServer::getSceneCloud(const sensor_msgs::PointCloud2::ConstPtr& scene)
{
  pcl::fromROSMsg(*scene, *scene_cloud_);
}

bool RegistrationPoseServer::surfaceRemove(pcl::PointCloud<PointInT>::Ptr cloud, double threshold, double ratio)
{
  // 点群が0でないか確認
  if (cloud->points.size() == 0)
  {
    std::cout << "no cloud data" << std::endl;
    return false;
  }

  // 平面検出（pclのチュートリアル通り）
  pcl::SACSegmentation<PointInT> seg;
  seg.setOptimizeCoefficients(true);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  seg.setInputCloud(cloud);
  seg.setModelType(pcl::SACMODEL_PLANE);  //モデル
  seg.setMethodType(pcl::SAC_RANSAC);     //検出手法
  seg.setMaxIterations(200);
  seg.setDistanceThreshold(threshold);  //閾値
  seg.segment(*inliers, *coefficients);

  // 平面除去を終わらせるかどうか：検出した平面が，前の平面除去した状態の点群のfinishRemovePlaneRatioで指定された割合未満の点群サイズであれば，それは平面でないとみなして点群除去処理を終了（finishRemovePlaneRatioは0.0~1.0:環境に合わせてください）
  if (inliers->indices.size() < cloud->points.size() * ratio)
  {
    return false;
  }

  // 平面除去
  pcl::ExtractIndices<PointInT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true にすると平面を除去、false にすると平面以外を除去
  extract.filter(*cloud);
  std::cout << "remove plane" << std::endl;

  return true;
}

void RegistrationPoseServer::downsampleCloud(pcl::PointCloud<PointInT>::Ptr input_cloud,
                                             pcl::PointCloud<PointInT>::Ptr output_cloud, float leaf)
{
  // scene downsampling
  pcl::ApproximateVoxelGrid<PointInT> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(leaf, leaf, leaf);
  sor.filter(*output_cloud);
}

void RegistrationPoseServer::loadModelCloud(pcl::PointCloud<PointInT>::Ptr cloud, std::string direction)
{
  std::cout << "algorithm：" << algorithm_name_ << std::endl;
  std::string model_filepath = ros::package::getPath("cnn_pose_estimator") + "/pcds/";
  std::string filepath = model_filepath + model_filename_ + "_" + direction + ".pcd";
  pcl::io::loadPCDFile(filepath, *cloud);
  std::cout << "load_" << direction << "_model_cloud : " << cloud->size() << std::endl;

  // 点群の中からnanを消す
  std::vector<int> dummy;
  std::cout << direction << "_model_cloud : " << cloud->size() << std::endl;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, dummy);

  // 平面除去
  while (true)
  {
    bool model_flag = surfaceRemove(cloud, model_plane_threshold_, model_planable_ratio_);
    model_flag = true;
    if (model_flag == true)
    {
      break;
    }
  }
  std::cout << "removed_" << direction << "_model_cloud : " << cloud->size() << std::endl;
  // model downsampling
  downsampleCloud(cloud, cloud, leaf_model_);
  std::cout << "model leaf size : " << leaf_model_ << std::endl;
  std::cout << "downsampled_" << direction << "_model_cloud : " << cloud->size() << std::endl;
  std::cout << std::endl;
}

void RegistrationPoseServer::transformCloudFrame(pcl::PointCloud<PointInT>::Ptr input_cloud,
                                                 pcl::PointCloud<PointInT>::Ptr transformed_cloud,
                                                 std::string from_frame, std::string to_frame)
{
  sensor_msgs::PointCloud2 input_cloud_msg;
  sensor_msgs::PointCloud2 transformed_cloud_msg;
  pcl::toROSMsg(*input_cloud, input_cloud_msg);
  input_cloud_msg.header.frame_id = from_frame;
  try
  {
    pcl_ros::transformPointCloud(to_frame, input_cloud_msg, transformed_cloud_msg, tf_listener_);
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }
  pcl::fromROSMsg(transformed_cloud_msg, *transformed_cloud);
}

void RegistrationPoseServer::registrateCloud(pcl::PointCloud<PointInT>::Ptr input_cloud,
                                             pcl::PointCloud<PointInT>::Ptr target_cloud, std::string method)
{
  if (method == "ICP")
  {
    // ICP algorithm
    pcl::IterativeClosestPoint<PointInT, PointInT> icp;
    icp.setInputSource(input_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaximumIterations(max_icp_iteration_);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-10);

    //変換matrixを表示する
    icp.align(*registrated_cloud_);
    Eigen::Matrix4d registrate_transform_inverse = Eigen::Matrix4d::Identity();
    registrate_transform_inverse = (icp.getFinalTransformation()).cast<double>();
    registrate_transform_ = registrate_transform_inverse.inverse();

    std::cout << "Transformation matrix:" << std::endl;
    std::cout << registrate_transform_ << std::endl;
    ROS_INFO_STREAM("score : " << icp.getFitnessScore());
  }
  else if (method == "NDT")
  {
    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointInT, PointInT> ndt;

    // Setting scale dependent NDT parameters
    ndt.setTransformationEpsilon(1e-3);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.01);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(0.01);
    // ndt.setMaximumIterations (35);

    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(target_cloud);

    // Calculating required rigid transform to align the input cloud to the
    // target cloud.
    ndt.align(*registrated_cloud_);
    Eigen::Matrix4d registrate_transform_inverse = Eigen::Matrix4d::Identity();
    registrate_transform_inverse = (ndt.getFinalTransformation()).cast<double>();
    registrate_transform_ = registrate_transform_inverse.inverse();
  }
}

bool RegistrationPoseServer::broadcastRegistratedPose()
{
  ROS_INFO("Conversion Start!!");
  tf::StampedTransform sensor_to_estimated_tf;
  try
  {
    tf_listener_.lookupTransform(sensor_frame_id_, estimated_frame_id_, ros::Time(0), sensor_to_estimated_tf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  Eigen::Affine3d sensor_to_estimated_affine;
  tf::transformTFToEigen(sensor_to_estimated_tf, sensor_to_estimated_affine);
  Eigen::Matrix4d sensor_to_estimated_matrix = sensor_to_estimated_affine.matrix();
  Eigen::Matrix4d sensor_to_final_matrix = sensor_to_estimated_matrix * registrate_transform_;
  Eigen::Affine3d eigen_affine3d(sensor_to_final_matrix);
  tf::Transform sensor_to_final_tf;
  tf::transformEigenToTF(eigen_affine3d, sensor_to_final_tf);
  geometry_msgs::Transform sensor_to_final_msg;
  tf::transformTFToMsg(sensor_to_final_tf, sensor_to_final_msg);
  registrated_frame_transform_stamped_.transform = sensor_to_final_msg;
  registrated_frame_transform_stamped_.header.frame_id = sensor_frame_id_;
  registrated_frame_transform_stamped_.child_frame_id = registrated_frame_id_;
  return true;
}

float RegistrationPoseServer::checkUpsideDown()
{
  pcl::PointCloud<PointInT>::Ptr z_vector(new pcl::PointCloud<PointInT>);
  z_vector->width = 2;
  z_vector->height = 1;
  z_vector->points.resize(z_vector->width * z_vector->height);
  z_vector->points[0].x = 0.0;
  z_vector->points[0].y = 0.0;
  z_vector->points[0].z = 0.0;
  z_vector->points[1].x = 0.0;
  z_vector->points[1].y = 0.0;
  z_vector->points[1].z = 1.0;

  transformCloudFrame(z_vector, z_vector, "/world", estimated_frame_id_);

  float vec_z = z_vector->points[1].z - z_vector->points[0].z;
  float sign;
  if (vec_z > 0)
  {
    sign = 1;
    std::cout << "object' direction is upside" << std::endl;
  }
  else
  {
    sign = -1;
    std::cout << "object' direction is down" << std::endl;
  }
  return sign;
}

bool RegistrationPoseServer::registratePose(denso_recognition_srvs::RegistrationPose::Request& req,
                                            denso_recognition_srvs::RegistrationPose::Response& res)
{
  is_ok_ = false;
  clock_t start = clock();
  if (use_remove_plane_)
  {
    while (!surfaceRemove(scene_cloud_, scene_plane_threshold_, scene_planable_ratio_))
    {
    }
  }

  pcl::PointCloud<PointInT>::Ptr downsampled_scene_cloud(new pcl::PointCloud<PointInT>);
  // downsample scene cloud
  downsampleCloud(scene_cloud_, downsampled_scene_cloud, leaf_scene_);
  std::cout << "scene leaf size : " << leaf_scene_ << std::endl;
  std::cout << "downsampled_scene_cloud : " << downsampled_scene_cloud->size() << std::endl;
  // calculate direction of estimated_tf's axis z
  float direction_z = checkUpsideDown();
  // transform cloud from sensor frame to world
  transformCloudFrame(downsampled_scene_cloud, converted_scene_, "world", estimated_frame_id_);

  // convert model cloud to cnn estimated_tf
  if (direction_z > 0)
  {
    std::cout << "ICP up " << std::endl;
    pcl::toROSMsg(*up_model_cloud_, model_cloud_);
    // 精密位置合わせを行う estimated_tf frameで行う
    registrateCloud(converted_scene_, up_model_cloud_, algorithm_name_);
  }
  else
  {
    std::cout << "ICP down " << std::endl;
    pcl::toROSMsg(*down_model_cloud_, model_cloud_);
    // 精密位置合わせを行う
    registrateCloud(converted_scene_, down_model_cloud_, algorithm_name_);
  }

  res.success = broadcastRegistratedPose();

  clock_t end = clock();
  const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
  printf("time %lf[ms]\n\n", time);

  if (res.success)
  {
    model_cloud_.header.frame_id = estimated_frame_id_;
    pcl::toROSMsg(*registrated_cloud_, transformed_cloud_);
    transformed_cloud_.header.frame_id = estimated_frame_id_;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to registrate !!");
    is_ok_ = false;
    return res.success;
  }

  is_ok_ = true;
  return res.success;
}

void RegistrationPoseServer::publishRegistratedCloudAndTF()
{
  if (is_ok_)
  {
    model_cloud_.header.stamp = ros::Time::now();
    transformed_cloud_pub_.publish(model_cloud_);

    transformed_cloud_.header.stamp = ros::Time::now();
    registrated_cloud_pub_.publish(transformed_cloud_);

    registrated_frame_transform_stamped_.header.stamp = ros::Time::now();
    br2_.sendTransform(registrated_frame_transform_stamped_);
  }
}
