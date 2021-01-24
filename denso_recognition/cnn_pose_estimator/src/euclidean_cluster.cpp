#include "cnn_pose_estimator/euclidean_cluster.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>

using cluster_cloud::ClusterCloud;

ClusterCloud::ClusterCloud(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh)
  , rate_(n.param("loop_rate", 10))
  , frame_id_(n.param<std::string>("clustering_frame_id", "photoneo_test_optical_frame"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/photoneo_test/pointcloud"), 1,
                                 &ClusterCloud::euclideanCallback, this);
  fileterd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      n.param<std::string>("filtered_pc_topic_name", "/filtered_pointcloud"), 1);
  euclidean_cluster_pub_ = nh_.advertise<denso_recognition_msgs::BoundingBoxArray>(
      n.param<std::string>("box_name", "/clustering_result"), 1);

  // クラスタリングのパラメータを初期化
  n.param<double>("clusterTolerance", clusterTolerance_, 0.02);
  n.param<int>("minSize", minSize_, 100);
  n.param<int>("maxSize", maxSize_, 250000);
  // clopboxを当てはめるエリアを定義
  n.param<float>("crop_x_min", crop_min_.x, 0.15);
  n.param<float>("crop_x_max", crop_max_.x, 1.5);
  n.param<float>("crop_y_min", crop_min_.y, -1.5);
  n.param<float>("crop_y_max", crop_max_.y, 1.5);
  n.param<float>("crop_z_min", crop_min_.z, 0.01);
  n.param<float>("crop_z_max", crop_max_.z, 0.5);

  n.param<float>("clustering_min_height", min_height_, 0.2);
}

void ClusterCloud::euclideanCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  ros::Time t0 = ros::Time::now();

  //点群をsensor座標系からWorld座標系に変換
  //変換されたデータはtrans_pcに格納される．
  sensor_msgs::PointCloud2 trans_pc;
  try
  {
    pcl_ros::transformPointCloud("world", *source_pc, trans_pc, tf_);
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<pcl::PointXYZ>(pcl_source));

  //  // Create the filtering object
  //  pcl::StatisticalOutlierRemoval<pcl::pcl::PointXYZ> sor;
  //  sor.setInputCloud (pcl_source_ptr);
  //  sor.setMeanK (100);
  //  sor.setStddevMulThresh (0.1);
  //  sor.filter (*pcl_source_ptr);

  // 平面をしきい値で除去する→cropboxで
  cropBox(pcl_source_ptr, crop_min_, crop_max_);

  // 点群データをsensor座標系に変換
  sensor_msgs::PointCloud2 filtered_pc;
  pcl::toROSMsg(*pcl_source_ptr, filtered_pc);
  sensor_msgs::PointCloud2 trans_pc_1;
  try
  {
    pcl_ros::transformPointCloud("photoneo_test_optical_frame", filtered_pc, trans_pc_1, tf_);
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ> pcl_source_1;
  pcl::fromROSMsg(trans_pc_1, pcl_source_1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source_ptr_1(new pcl::PointCloud<pcl::PointXYZ>(pcl_source_1));

  ros::Time t1 = ros::Time::now();
  // Creating the KdTree object for the search method of the extraction
  clustering(pcl_source_ptr_1);

  // crop後にsensor座標系に戻した点群をpublish
  trans_pc_1.header.stamp = ros::Time::now();
  trans_pc_1.header.frame_id = frame_id_;
  fileterd_cloud_pub_.publish(trans_pc_1);

  ros::Time t2 = ros::Time::now();
  std::cout << "前処理時間 : " << t1 - t0 << std::endl;
  std::cout << "クラスタリング時間 : " << t2 - t1 << std::endl;
}

void ClusterCloud::cropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max)
{
  Eigen::Vector4f minPoint;

  minPoint[0] = min.x;  // define minimum point x
  minPoint[1] = min.y;  // define minimum point y
  minPoint[2] = min.z;  // define minimum point z

  Eigen::Vector4f maxPoint;
  maxPoint[0] = max.x;  // define max point x
  maxPoint[1] = max.y;  // define max point y
  maxPoint[2] = max.z;  // define max point z

  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = 0;
  boxTranslatation[1] = 0;
  boxTranslatation[2] = 0;

  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0;  // rotation around x-axis
  boxRotation[1] = 0;  // rotation around y-axis
  boxRotation[2] = 0;  // in radians rotation around z-axis. this rotates your

  Eigen::Affine3f boxTransform;

  pcl::CropBox<pcl::PointXYZ> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);
  cropFilter.setRotation(boxRotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  cropFilter.filter(*cloud_filtered);
  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*cloud_filtered, *cloud);
}

void ClusterCloud::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(clusterTolerance_);
  ec.setMinClusterSize(minSize_);
  ec.setMaxClusterSize(maxSize_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  int j = 0;
  denso_recognition_msgs::BoundingBoxArray box_array;  // clustering結果をぶち込む配列

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // 一つのclusterをpushback
    if (minAreaRect(cloud_cluster, j))
    {
      box_array.boxes.push_back(box_);
      j++;
    }
  }

  // int clusterLength = clusterIndices.size();
  ROS_INFO("Found %d clusters:", j);

  // publish
  box_array.header.stamp = ros::Time::now();
  box_array.header.frame_id = frame_id_;
  euclidean_cluster_pub_.publish(box_array);

  // Empty Buffer
  cluster_indices.clear();
}

bool ClusterCloud::minAreaRect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cluster_cnt)
{
  // PCLによる点群の最大最小領域を取得
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  // AABBは各軸に平行な四角形、または直方体

  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 size;

  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);

  //  if(min_point_AABB.z < min_height_){
  if (1)
  {
    // ROS_INFO("Center of mass (x, y) = (%f, %f)", rrect.center.x,
    // rrect.center.y);
    // ROS_INFO("Height = %f Width =  %f", rrect.size.height, rrect.size.width);
    // ROS_INFO("Angle = %f [deg]", rrect.angle);

    // denso_recognition_msgs::BoundingBoxの型に合わせて代入していく
    //    pose.position.x = (min_point_AABB.x + max_point_AABB.x) / 2.0;
    //    pose.position.y = (min_point_AABB.y + max_point_AABB.y) / 2.0;
    //    pose.position.z = (min_point_AABB.z + max_point_AABB.z) / 2.0;
    pose.position.x = min_point_AABB.x;
    pose.position.y = min_point_AABB.y;
    pose.position.z = min_point_AABB.z;

    Eigen::Matrix3f AxisAngle;
    Eigen::Vector3f axis(0, 0, 1);           // z 軸を指定
    AxisAngle = Eigen::AngleAxisf(0, axis);  // z軸周りに90度反時計回りに回転
    Eigen::Quaternionf quat(AxisAngle);      // クォータニオンに変換
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    size.x = max_point_AABB.x - min_point_AABB.x;
    size.y = max_point_AABB.x - min_point_AABB.y;
    size.z = max_point_AABB.z - min_point_AABB.z;

    // TFの名前付け
    std::stringstream ss;
    std::string object_name;
    ss << cluster_cnt + 1;
    object_name = "object_" + ss.str();

    br_.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                      tf::Vector3(pose.position.x, pose.position.y, max_point_AABB.z)),
        ros::Time::now(), frame_id_, object_name));

    box_.header.frame_id = frame_id_;
    box_.pose = pose;
    box_.dimensions = size;
    box_.label = cluster_cnt;

    return true;
  }
  else
  {
    return false;
  }
}

bool ClusterCloud::momentOfInertia_AABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cluster_cnt)
{
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;

  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 size;

  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);

  if (min_point_AABB.z < min_height_)
  {
    pose.position.x = (min_point_AABB.x + max_point_AABB.x) / 2.0;
    pose.position.y = (min_point_AABB.y + max_point_AABB.y) / 2.0;
    pose.position.z = (min_point_AABB.z + max_point_AABB.z) / 2.0;
    // std::cout << pose.position.x << ", " << pose.position.y << ", " <<
    // pose.position.z << std::endl;

    size.x = max_point_AABB.x - min_point_AABB.x;
    size.y = max_point_AABB.y - min_point_AABB.y;
    size.z = max_point_AABB.z - min_point_AABB.z;
    // std::cout << size.x << ", " << size.y << ", " << size.z << std::endl;
    // std::cout << std::endl;

    // TFの名前付け
    std::stringstream ss;
    std::string object_name;
    ss << cluster_cnt;
    object_name = "object_" + ss.str();

    br_.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(pose.position.x, pose.position.y, max_point_AABB.z)),
        ros::Time::now(), frame_id_, object_name));

    box_.header.frame_id = frame_id_;
    box_.pose = pose;
    box_.dimensions = size;
    box_.label = cluster_cnt;

    return true;
  }
  else
  {
    return false;
  }
}

denso_recognition_msgs::BoundingBox ClusterCloud::momentOfInertia_OBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 size;

  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues(major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);
  Eigen::Quaternionf quat(rotational_matrix_OBB);

  pose.position.x = mass_center(0);
  pose.position.y = mass_center(1);
  pose.position.z = mass_center(2);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  // std::cout << pose.position.x << ", " << pose.position.y << ", " <<
  // pose.position.z << std::endl;

  size.x = max_point_OBB.x - min_point_OBB.x;
  size.y = max_point_OBB.y - min_point_OBB.y;
  size.z = max_point_OBB.z - min_point_OBB.z;
  // std::cout << size.x << ", " << size.y << ", " << size.z << std::endl;
  // std::cout << std::endl;

  denso_recognition_msgs::BoundingBox box;
  box.header.frame_id = frame_id_;
  box.pose = pose;
  box.dimensions = size;

  return box;
}
