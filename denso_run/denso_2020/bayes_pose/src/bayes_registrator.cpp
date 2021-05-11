#include <bayes_pose/bayes_registrator.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

using bayes_pose::BayesRegistrator;
using bayes_pose::PositionOptimizer;
using bayes_pose::OrientationOptimizer;
using bayes_pose::PostureOptimizer;

BayesRegistrator::BayesRegistrator(ros::NodeHandle nh, const std::string& compared_pc_topic_name,
                                   const std::string& reference_pc_topic_name,
                                   const std::string& registrated_pc_topic_name, bayesopt::Parameters pos_params,
                                   bayesopt::Parameters ori_params, bayesopt::Parameters posture_params)
  : nh_(nh)
  , is_init_(false)
  , is_tf_init_(false)
  , is_ready_(false)
  , stop_estimate_trigger_(false)
  , registrated_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
  , registrated_pc_pub_(nh_.advertise<sensor_msgs::PointCloud2>(registrated_pc_topic_name, 1))
  , reference_pc_sub_(nh_.subscribe(reference_pc_topic_name, 1, &BayesRegistrator::referenceCB, this))
  , compared_pc_sub_(nh_.subscribe(compared_pc_topic_name, 1, &BayesRegistrator::registrate, this))
  , pos_opt_(pos_params)
  , ori_opt_(ori_params)
  , posture_opt_(posture_params)
  , tf_listener_(tf_buffer_)
{
  ros::param::param<std::string>("~object_frame_name", reference_frame_name_, "HV6_00");
  ros::param::set("stop_estimate_trigger", false);
  const double pi = boost::math::constants::pi<double>();
  vectord lower_pos(3);
  vectord upper_pos(3);
  vectord lower_ori(3);
  vectord upper_ori(3);
  lower_pos[0] = -0.2;
  lower_pos[1] = -0.2;
  lower_pos[2] = -0.2;
  upper_pos[0] = 0.2;
  upper_pos[1] = 0.2;
  upper_pos[2] = 0.2;
  // lower_ori[0] = -pi;
  // lower_ori[1] = -1.0 * (pi / 2.0);
  lower_ori[0] = 0.0;
  lower_ori[1] = 0.0;
  lower_ori[2] = -pi;
  // upper_ori[0] = pi;
  // upper_ori[1] = pi / 2.0;
  upper_ori[0] = 0.0;
  upper_ori[1] = 0.0;
  upper_ori[2] = pi;
  pos_opt_.setBoundingBox(lower_pos, upper_pos);
  ori_opt_.setBoundingBox(lower_ori, upper_ori);
  ROS_INFO_STREAM("Bayes Registrator ready.");
}

void BayesRegistrator::referenceCB(const sensor_msgs::PointCloud2ConstPtr& reference_pc)
{
  reference_pc_ = *reference_pc;
  source_frame_name_ = reference_pc_.header.frame_id;
  is_init_ = true;
}

void BayesRegistrator::registrate(const sensor_msgs::PointCloud2ConstPtr& compared_pc)
{
  if (is_init_)
  {
    if (!is_tf_init_)
    {
      lookupInitTransform();
      is_tf_init_ = true;
    }
    pcl::PointCloud<pcl::PointXYZ> compared_pc_pcl, reference_pc_pcl, tmp_pc_pcl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc_pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*compared_pc, compared_pc_pcl);
    pcl::fromROSMsg(reference_pc_, reference_pc_pcl);

    *reference_pc_pcl_ptr = reference_pc_pcl;

    std::vector<int> nan_index;
    pcl::removeNaNFromPointCloud(compared_pc_pcl, compared_pc_pcl, nan_index);
    pcl::removeNaNFromPointCloud(*reference_pc_pcl_ptr, *reference_pc_pcl_ptr, nan_index);

    if (compared_pc_pcl.points.size() == 0 || reference_pc_pcl_ptr->points.size() == 0)
    {
      ROS_WARN("No pointcloud.");
      return;
    }

    // ROS_WARN("==========================");
    // ROS_WARN("=         OPTIMIZE       =");
    // ROS_WARN("==========================");
    //
    // vectord posture_result(6);
    // posture_opt_.setInputClouds(compared_pc_pcl, *reference_pc_pcl_ptr);
    // if (is_ready_)
    // {
    //   ROS_WARN("=          READY         =");
    //   ROS_WARN("==========================");
    //   posture_opt_.setInputClouds(compared_pc_pcl, *registrated_pc_pcl_);
    // }
    // else
    // {
    //   ROS_WARN("=          INIT          =");
    //   ROS_WARN("==========================");
    //   posture_opt_.setInputClouds(compared_pc_pcl, *reference_pc_pcl_ptr);
    // }
    // posture_opt_.optimize(posture_result);
    // ROS_INFO_STREAM("xyzrpy: " << posture_result[0] << ", " << posture_result[1] << ", " << posture_result[2] << ", "
    //                            << posture_result[3] << ", " << posture_result[4] << ", " << posture_result[5]);
    // applyTransform(posture_result[0], posture_result[1], posture_result[2], posture_result[3], posture_result[4],
    //                posture_result[5], reference_pc_pcl_ptr);
    // pcl::toROSMsg(*registrated_pc_pcl_, registrated_pc_);
    // is_ready_ = true;
    // registrated_pc_pub_.publish(registrated_pc_);

    vectord pos_result(3);
    vectord ori_result(3);

    ROS_WARN("==========================");
    ROS_WARN("=       POSITION 1       =");
    ROS_WARN("==========================");

    if (is_ready_)
    {
      ROS_WARN("=          READY         =");
      ROS_WARN("==========================");
      pos_opt_.setInputClouds(compared_pc_pcl, *registrated_pc_pcl_);
    }
    else
    {
      ROS_WARN("=          INIT          =");
      ROS_WARN("==========================");
      pos_opt_.setInputClouds(compared_pc_pcl, *reference_pc_pcl_ptr);
    }
    pos_opt_.optimize(pos_result);

    // if (is_ready_)
    // {
    //   applyTranslation(pos_result[0], pos_result[1], pos_result[2], registrated_pc_pcl_);
    // }
    // else
    // {
    //   applyTranslation(pos_result[0], pos_result[1], pos_result[2], reference_pc_pcl_ptr);
    // }

    ros::param::get("stop_estimate_trigger", stop_estimate_trigger_);
    if (stop_estimate_trigger_)
    {
      reference_pc_sub_.shutdown();
      compared_pc_sub_.shutdown();
      is_init_ = false;
      ROS_INFO_STREAM("***ESTIMATION END***");
      return;
    }
    *registrated_pc_pcl_ = pos_opt_.getOptimumRefCloud(pos_result);
    pos_opt_.applyTranslation(pos_result[0], pos_result[1], pos_result[2], output_affine3d_);
    broadcast_tf();

    pcl::toROSMsg(*registrated_pc_pcl_, registrated_pc_);
    registrated_pc_pub_.publish(registrated_pc_);

    ROS_INFO_STREAM("POS1: " << pos_result[0] << ", " << pos_result[1] << ", " << pos_result[2]);
    ROS_INFO_STREAM("COST: " << pos_opt_.evaluateSample(pos_result));

    ROS_WARN("==========================");
    ROS_WARN("=      ORIENTATION 1     =");
    ROS_WARN("==========================");

    ori_opt_.setInputClouds(compared_pc_pcl, *registrated_pc_pcl_);
    ori_opt_.optimize(ori_result);
    // applyRotation(ori_result[0], ori_result[1], ori_result[2], registrated_pc_pcl_);
    ros::param::get("stop_estimate_trigger", stop_estimate_trigger_);
    if (stop_estimate_trigger_)
    {
      reference_pc_sub_.shutdown();
      compared_pc_sub_.shutdown();
      is_init_ = false;
      ROS_INFO_STREAM("***ESTIMATION END***");
      return;
    }
    *registrated_pc_pcl_ = ori_opt_.getOptimumRefCloud(ori_result);
    ori_opt_.applyRotation(ori_result[0], ori_result[1], ori_result[2], output_affine3d_);
    broadcast_tf();

    pcl::toROSMsg(*registrated_pc_pcl_, registrated_pc_);
    registrated_pc_pub_.publish(registrated_pc_);

    ROS_INFO_STREAM("ORI1: " << ori_result[0] << ", " << ori_result[1] << ", " << ori_result[2]);
    ROS_INFO_STREAM("COST: " << ori_opt_.evaluateSample(ori_result));

    ROS_WARN("==========================");
    ROS_WARN("=       POSITION 2       =");
    ROS_WARN("==========================");
    pos_opt_.setInputClouds(compared_pc_pcl, *registrated_pc_pcl_);
    pos_opt_.optimize(pos_result);
    // applyTranslation(pos_result[0], pos_result[1], pos_result[2], registrated_pc_pcl_);
    ros::param::get("stop_estimate_trigger", stop_estimate_trigger_);
    if (stop_estimate_trigger_)
    {
      reference_pc_sub_.shutdown();
      compared_pc_sub_.shutdown();
      is_init_ = false;
      ROS_INFO_STREAM("***ESTIMATION END***");
      return;
    }
    *registrated_pc_pcl_ = pos_opt_.getOptimumRefCloud(pos_result);
    pos_opt_.applyTranslation(pos_result[0], pos_result[1], pos_result[2], output_affine3d_);
    broadcast_tf();

    pcl::toROSMsg(*registrated_pc_pcl_, registrated_pc_);
    registrated_pc_pub_.publish(registrated_pc_);

    ROS_INFO_STREAM("POS2: " << pos_result[0] << ", " << pos_result[1] << ", " << pos_result[2]);
    ROS_INFO_STREAM("COST: " << pos_opt_.evaluateSample(pos_result));

    ROS_WARN("==========================");
    ROS_WARN("=      ORIENTATION 2     =");
    ROS_WARN("==========================");

    ori_opt_.setInputClouds(compared_pc_pcl, *registrated_pc_pcl_);
    ori_opt_.optimize(ori_result);
    // applyRotation(ori_result[0], ori_result[1], ori_result[2], registrated_pc_pcl_);
    ros::param::get("stop_estimate_trigger", stop_estimate_trigger_);
    if (stop_estimate_trigger_)
    {
      reference_pc_sub_.shutdown();
      compared_pc_sub_.shutdown();
      is_init_ = false;
      ROS_INFO_STREAM("***ESTIMATION END***");
      return;
    }
    *registrated_pc_pcl_ = ori_opt_.getOptimumRefCloud(ori_result);
    ori_opt_.applyRotation(ori_result[0], ori_result[1], ori_result[2], output_affine3d_);
    broadcast_tf();

    pcl::toROSMsg(*registrated_pc_pcl_, registrated_pc_);
    registrated_pc_pub_.publish(registrated_pc_);

    ROS_INFO_STREAM("ORI2: " << ori_result[0] << ", " << ori_result[1] << ", " << ori_result[2]);
    ROS_INFO_STREAM("COST: " << ori_opt_.evaluateSample(ori_result));

    is_ready_ = true;

    pcl::toROSMsg(*registrated_pc_pcl_, registrated_pc_);
  }
}

void BayesRegistrator::applyTranslation(const double x, const double y, const double z,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_pc)
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() << x, y, z;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*transform_pc, *registrated_pc_pcl_, transform);
}

void BayesRegistrator::applyRotation(const double roll, const double pitch, const double yaw,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_pc)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cog_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Vector4f reference_center;
  pcl::compute3DCentroid(*transform_pc, reference_center);
  Eigen::Affine3d transform_to_base = Eigen::Affine3d::Identity();
  transform_to_base.translation() << -reference_center(0, 0), -reference_center(1, 0), -reference_center(2, 0);

  pcl::transformPointCloud(*transform_pc, *cog_aligned_cloud, transform_to_base);

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  transform.rotate(rotation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cog_aligned_cloud, *transformed_cloud, transform);

  Eigen::Affine3d transform_from_base = Eigen::Affine3d::Identity();
  transform_from_base.translation() << reference_center(0, 0), reference_center(1, 0), reference_center(2, 0);

  pcl::transformPointCloud(*transformed_cloud, *registrated_pc_pcl_, transform_from_base);
}

void BayesRegistrator::applyTransform(const double x, const double y, const double z, const double roll,
                                      const double pitch, const double yaw,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_pc)
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  transform.translation() << x, y, z;

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  transform.rotate(rotation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*transform_pc, *registrated_pc_pcl_, transform);
}

void BayesRegistrator::publish()
{
  if (is_ready_)
  {
    registrated_pc_pub_.publish(registrated_pc_);
    is_ready_ = false;
  }
}

void BayesRegistrator::broadcast_tf()
{
  if (is_tf_init_)
  {
    geometry_msgs::TransformStamped ts;
    ts = tf2::eigenToTransform(output_affine3d_);
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = source_frame_name_;
    ts.child_frame_id = reference_frame_name_ + "_estimate";
    ROS_DEBUG("[bayes_registrator] broadcast tf");
    br_.sendTransform(ts);
  }
}

void BayesRegistrator::lookupInitTransform()
{
  geometry_msgs::TransformStamped transform;
  geometry_msgs::Pose pose;
  while (true)
  {
    try
    {
      transform = tf_buffer_.lookupTransform(source_frame_name_, reference_frame_name_, ros::Time(0));
      break;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      continue;
    }
  }
  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;
  pose.orientation.x = transform.transform.rotation.x;
  pose.orientation.y = transform.transform.rotation.y;
  pose.orientation.z = transform.transform.rotation.z;
  pose.orientation.w = transform.transform.rotation.w;
  tf2::fromMsg(pose, output_affine3d_);
}

PositionOptimizer::PositionOptimizer(bayesopt::Parameters params)
  : ContinuousModel(3, params)
  , reference_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
  , compared_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
  , transformed_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
{
}

void PositionOptimizer::setInputClouds(const sensor_msgs::PointCloud2& compared_pc,
                                       const sensor_msgs::PointCloud2& reference_pc)
{
  pcl::fromROSMsg(compared_pc, *compared_pc_pcl_);
  pcl::fromROSMsg(reference_pc, *reference_pc_pcl_);
}

void PositionOptimizer::setInputClouds(const pcl::PointCloud<pcl::PointXYZ>& compared_pc,
                                       const pcl::PointCloud<pcl::PointXYZ>& reference_pc)
{
  *compared_pc_pcl_ = compared_pc;
  *reference_pc_pcl_ = reference_pc;
}

double PositionOptimizer::evaluateSample(const vectord& xin)
{
  if (xin.size() != 3)
  {
    std::cout << "WARNING: This only works for 3D(x, y, z) inputs." << std::endl
              << "WARNING: Using only first three components." << std::endl;
  }
  applyTranslation(xin[0], xin[1], xin[2]);
  return calcC2CDistance();
}

bool PositionOptimizer::checkReachability(const vectord& query)
{
  return true;
}

void PositionOptimizer::applyTranslation(const double x, const double y, const double z)
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() << x, y, z;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*reference_pc_pcl_, *transformed_cloud, transform);
  *transformed_pc_pcl_ = *transformed_cloud;
}

void PositionOptimizer::applyTranslation(const double x, const double y, const double z, Eigen::Affine3d& matrix)
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() << x, y, z;
  // matrix = matrix * transform;
  matrix = transform * matrix;
}

double PositionOptimizer::calcC2CDistance()
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(transformed_pc_pcl_->makeShared());
  const int K = 1;

  std::vector<int> nearest_point_index;
  std::vector<float> nearest_point_distance;
  double c2c_distance = 0.0;
  int point_size = 0;

  for (auto compared_point : compared_pc_pcl_->points)
  {
    if (kdtree.nearestKSearch(compared_point, K, nearest_point_index, nearest_point_distance) > 0)
    {
      c2c_distance += nearest_point_distance[0];
      point_size++;
    }
    nearest_point_index.clear();
    nearest_point_distance.clear();
  }
  // ROS_INFO_STREAM("point_size: " << point_size);
  // ROS_INFO_STREAM("c2c_distance: " << c2c_distance);
  // ROS_INFO_STREAM("c2c_distance(mean): " << c2c_distance / point_size);

  return (c2c_distance / point_size);
}

pcl::PointCloud<pcl::PointXYZ> PositionOptimizer::getOptimumRefCloud(const vectord& opt_param)
{
  applyTranslation(opt_param[0], opt_param[1], opt_param[2]);
  return *transformed_pc_pcl_;
}

OrientationOptimizer::OrientationOptimizer(bayesopt::Parameters params)
  : ContinuousModel(3, params)
  , reference_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
  , compared_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
  , transformed_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
{
}

void OrientationOptimizer::setInputClouds(const sensor_msgs::PointCloud2& compared_pc,
                                          const sensor_msgs::PointCloud2& reference_pc)
{
  pcl::fromROSMsg(compared_pc, *compared_pc_pcl_);
  pcl::fromROSMsg(reference_pc, *reference_pc_pcl_);
}

void OrientationOptimizer::setInputClouds(const pcl::PointCloud<pcl::PointXYZ>& compared_pc,
                                          const pcl::PointCloud<pcl::PointXYZ>& reference_pc)
{
  *compared_pc_pcl_ = compared_pc;
  *reference_pc_pcl_ = reference_pc;
}

double OrientationOptimizer::evaluateSample(const vectord& xin)
{
  if (xin.size() != 3)
  {
    std::cout << "WARNING: This only works for 3D(roll, pitch, yaw) inputs." << std::endl
              << "WARNING: Using only first three components." << std::endl;
  }
  applyRotation(xin[0], xin[1], xin[2]);
  // double cost = calcC2CDistance();
  // ROS_INFO_STREAM(xin[0] << " " << xin[1] << " " << xin[2] << ": " << cost);
  // return cost;
  return calcC2CDistance();
}

bool OrientationOptimizer::checkReachability(const vectord& query)
{
  const double pi = boost::math::constants::pi<double>();
  if ((query[0] < (-1.0 * pi)) || (query[0] > pi))
  {
    return false;
  }
  if ((query[1] < (-1.0 * (pi / 2.0))) || (query[1] > (pi / 2.0)))
  {
    return false;
  }
  if ((query[2] < (-1.0 * pi)) || (query[2] > pi))
  {
    return false;
  }
  return true;
}

void OrientationOptimizer::applyRotation(const double roll, const double pitch, const double yaw)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cog_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Vector4f reference_center;
  pcl::compute3DCentroid(*reference_pc_pcl_, reference_center);
  Eigen::Affine3d transform_to_base = Eigen::Affine3d::Identity();
  transform_to_base.translation() << -reference_center(0, 0), -reference_center(1, 0), -reference_center(2, 0);

  pcl::transformPointCloud(*reference_pc_pcl_, *cog_aligned_cloud, transform_to_base);

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  transform.rotate(rotation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rot_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cog_aligned_cloud, *rot_aligned_cloud, transform);

  Eigen::Affine3d transform_from_base = Eigen::Affine3d::Identity();
  transform_from_base.translation() << reference_center(0, 0), reference_center(1, 0), reference_center(2, 0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*rot_aligned_cloud, *transformed_cloud, transform_from_base);
  *transformed_pc_pcl_ = *transformed_cloud;
}

void OrientationOptimizer::applyRotation(const double roll, const double pitch, const double yaw, Eigen::Affine3d& matrix)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cog_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Vector4f reference_center;
  pcl::compute3DCentroid(*reference_pc_pcl_, reference_center);
  Eigen::Affine3d transform_to_base = Eigen::Affine3d::Identity();
  transform_to_base.translation() << -reference_center(0, 0), -reference_center(1, 0), -reference_center(2, 0);

  pcl::transformPointCloud(*reference_pc_pcl_, *cog_aligned_cloud, transform_to_base);
  // matrix = matrix * transform_to_base;
  matrix = transform_to_base * matrix;

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  transform.rotate(rotation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rot_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cog_aligned_cloud, *rot_aligned_cloud, transform);
  // matrix = matrix * transform;
  matrix = transform * matrix;

  Eigen::Affine3d transform_from_base = Eigen::Affine3d::Identity();
  transform_from_base.translation() << reference_center(0, 0), reference_center(1, 0), reference_center(2, 0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*rot_aligned_cloud, *transformed_cloud, transform_from_base);
  // matrix = matrix * transform_from_base;
  matrix = transform_from_base * matrix;
}

double OrientationOptimizer::calcC2CDistance()
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(transformed_pc_pcl_->makeShared());
  const int K = 1;

  std::vector<int> nearest_point_index;
  std::vector<float> nearest_point_distance;
  double c2c_distance = 0.0;
  int point_size = 0;

  for (auto compared_point : compared_pc_pcl_->points)
  {
    if (kdtree.nearestKSearch(compared_point, K, nearest_point_index, nearest_point_distance) > 0)
    {
      c2c_distance += nearest_point_distance[0];
      point_size++;
    }
    nearest_point_index.clear();
    nearest_point_distance.clear();
  }
  // ROS_INFO_STREAM("point_size: " << point_size);
  // ROS_INFO_STREAM("c2c_distance: " << c2c_distance);
  // ROS_INFO_STREAM("c2c_distance(mean): " << c2c_distance / point_size);

  return (c2c_distance / point_size);
}

pcl::PointCloud<pcl::PointXYZ> OrientationOptimizer::getOptimumRefCloud(const vectord& opt_param)
{
  applyRotation(opt_param[0], opt_param[1], opt_param[2]);
  return *transformed_pc_pcl_;
}

PostureOptimizer::PostureOptimizer(bayesopt::Parameters params)
  : ContinuousModel(6, params)
  , reference_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
  , compared_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
  , transformed_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
{
}

void PostureOptimizer::setInputClouds(const sensor_msgs::PointCloud2& compared_pc,
                                      const sensor_msgs::PointCloud2& reference_pc)
{
  pcl::fromROSMsg(compared_pc, *compared_pc_pcl_);
  pcl::fromROSMsg(reference_pc, *reference_pc_pcl_);
}

void PostureOptimizer::setInputClouds(const pcl::PointCloud<pcl::PointXYZ>& compared_pc,
                                      const pcl::PointCloud<pcl::PointXYZ>& reference_pc)
{
  *compared_pc_pcl_ = compared_pc;
  *reference_pc_pcl_ = reference_pc;
}

double PostureOptimizer::evaluateSample(const vectord& xin)
{
  if (xin.size() != 6)
  {
    std::cout << "WARNING: This only works for 6D(x, y, z, roll, pitch, yaw) inputs." << std::endl
              << "WARNING: Using only first six components." << std::endl;
  }
  applyTransform(xin[0], xin[1], xin[2], xin[3], xin[4], xin[5]);
  return calcC2CDistance();
}

bool PostureOptimizer::checkReachability(const vectord& query)
{
  const double pi = boost::math::constants::pi<double>();
  if ((query[3] < (-1.0 * pi)) || (query[3] > pi))
  {
    return false;
  }
  if ((query[4] < (-1.0 * (pi / 2.0))) || (query[4] > (pi / 2.0)))
  {
    return false;
  }
  if ((query[5] < (-1.0 * pi)) || (query[5] > pi))
  {
    return false;
  }
  return true;
}

void PostureOptimizer::applyTransform(const double x, const double y, const double z, const double roll,
                                      const double pitch, const double yaw)
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  transform.translation() << x, y, z;

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  transform.rotate(rotation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*reference_pc_pcl_, *transformed_cloud, transform);
  *transformed_pc_pcl_ = *transformed_cloud;
}

double PostureOptimizer::calcC2CDistance()
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(transformed_pc_pcl_->makeShared());
  const int K = 1;

  std::vector<int> nearest_point_index;
  std::vector<float> nearest_point_distance;
  double c2c_distance = 0.0;
  int point_size = 0;

  for (auto compared_point : compared_pc_pcl_->points)
  {
    if (kdtree.nearestKSearch(compared_point, K, nearest_point_index, nearest_point_distance) > 0)
    {
      c2c_distance += nearest_point_distance[0];
      point_size++;
    }
    nearest_point_index.clear();
    nearest_point_distance.clear();
  }
  // ROS_INFO_STREAM("point_size: " << point_size);
  // ROS_INFO_STREAM("c2c_distance: " << c2c_distance);
  // ROS_INFO_STREAM("c2c_distance(mean): " << c2c_distance / point_size);

  return (c2c_distance / point_size);
}
