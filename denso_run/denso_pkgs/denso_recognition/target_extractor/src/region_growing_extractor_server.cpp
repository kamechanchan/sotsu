#include <target_extractor/region_growing_extractor_server.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>

using region_growing_extractor_server::RegionGrowingExtractorServer;

RegionGrowingExtractorServer::RegionGrowingExtractorServer(ros::NodeHandle& nh)
  : nh_(nh), is_ok_(false), scene_cloud_(new pcl::PointCloud<PointInT>)
{
  ros::param::param<int>("~cluster_cloud_size_min", cluster_cloud_size_min_, 50);
  ros::param::param<int>("~cluster_cloud_size_max", cluster_cloud_size_max_, 1000000);
  ros::param::param<int>("~k_search_size", k_search_size_, 50);
  ros::param::param<int>("~number_of_neighbours", number_of_neighbours_, 30);
  ros::param::param<int>("~extract_cluster_cloud_size_min", extract_cluster_cloud_size_min_, 50);
  ros::param::param<int>("~extract_cluster_cloud_size_max", extract_cluster_cloud_size_max_, 10000);
  ros::param::param<double>("~smooth_threshold_deg", smooth_threshold_deg_, 3.0);
  ros::param::param<double>("~curvature_threshold", curvature_threshold_, 1.0);
  ros::param::param<std::string>("~scene_topic_name", scene_topic_name_, "/pcd_data");
  ros::param::param<std::string>("~extract_cloud_topic_name", extract_cloud_topic_name_, "/extract_cloud");
  ros::param::param<std::string>("~extract_cloud_frame_id", extract_cloud_frame_id_, "world");
  cluster_clouds_.clear();
  extract_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(extract_cloud_topic_name_, 1);
  scene_topic_sub_ = nh.subscribe(scene_topic_name_, 1, &RegionGrowingExtractorServer::receiveSceneTopicCallback, this);
  get_top_cluster_server_ = nh.advertiseService("/target_extractor/region_growing/get_top_cluster",
                                                &RegionGrowingExtractorServer::getTopCluster, this);
  get_cluster_server_ = nh.advertiseService("/target_extractor/region_growing/get_cluster",
                                            &RegionGrowingExtractorServer::getCluster, this);
  set_target_cluster_server_ = nh.advertiseService("/target_extractor/region_growing/set_targer_cluster",
                                                   &RegionGrowingExtractorServer::setTargetCluster, this);
  ROS_INFO("RegionGrowingExtractorServer create instance!!");
}

void RegionGrowingExtractorServer::receiveSceneTopicCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  pcl::fromROSMsg(*cloud, *scene_cloud_);
}

bool RegionGrowingExtractorServer::getTopCluster(denso_recognition_srvs::ExtractObject::Request& req,
                                                 denso_recognition_srvs::ExtractObject::Response& res)
{
  is_ok_ = false;
  pcl::PointCloud<PointInT>::Ptr cloud(new pcl::PointCloud<PointInT>);
  *cloud = *scene_cloud_;

  if (cloud->points.size() == 0)
  {
    ROS_ERROR_STREAM("No input cloud !!");
    res.success = false;
    return res.success;
  }

  pcl::search::Search<PointInT>::Ptr tree(new pcl::search::KdTree<PointInT>);
  pcl::PointCloud<PointNT>::Ptr normals(new pcl::PointCloud<PointNT>);
  pcl::NormalEstimation<PointInT, PointNT> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(k_search_size_);
  normal_estimator.compute(*normals);

  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::PassThrough<PointInT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*indices);

  pcl::RegionGrowing<PointInT, PointNT> reg;
  reg.setMinClusterSize(cluster_cloud_size_min_);
  reg.setMaxClusterSize(cluster_cloud_size_max_);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(number_of_neighbours_);
  reg.setInputCloud(cloud);
  reg.setIndices(indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(smooth_threshold_deg_ / 180.0 * M_PI);
  reg.setCurvatureThreshold(curvature_threshold_);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" << std::endl
            << "cloud that belong to the first cluster:" << std::endl;

  std::vector<pcl::PointCloud<PointInT>::Ptr> cluster_clouds;
  cluster_clouds.clear();
  for (int i = 0; i < clusters.size(); i++)
  {
    pcl::PointCloud<PointInT>::Ptr cluster_cloud(new pcl::PointCloud<PointInT>);
    cluster_cloud->header.frame_id = cloud->header.frame_id;
    cluster_cloud->height = 1;
    cluster_cloud->width = clusters[i].indices.size();
    cluster_cloud->points.resize(clusters[i].indices.size());
    int extract_cloud_size = clusters[i].indices.size();
    if (extract_cluster_cloud_size_min_ < extract_cloud_size && extract_cloud_size < extract_cluster_cloud_size_max_)
    {
      for (int j = 0; j < extract_cloud_size; j++)
      {
        cluster_cloud->points[j].x = cloud->points[clusters[i].indices[j]].x;
        cluster_cloud->points[j].y = cloud->points[clusters[i].indices[j]].y;
        cluster_cloud->points[j].z = cloud->points[clusters[i].indices[j]].z;
      }
      cluster_clouds.push_back(cluster_cloud);
    }
    else
    {
      continue;
    }
  }

  if (!cluster_clouds.empty())
  {
    std::vector<double> z_ave_vector;
    z_ave_vector.clear();
    for (int i = 0; i < cluster_clouds.size(); i++)
    {
      double z_sum;
      for (int j = 0; j < cluster_clouds[i]->points.size(); j++)
      {
        z_sum += cluster_clouds[i]->points[j].z;
      }
      z_ave_vector.push_back(z_sum / cluster_clouds[i]->points.size());
    }

    std::vector<double>::iterator itr = std::max_element(z_ave_vector.begin(), z_ave_vector.end());
    size_t index = std::distance(z_ave_vector.begin(), itr);

    std::cout << "Top cluster is cluster " << index << std::endl;

    pcl::PointCloud<PointInT> select_cluster_cloud;
    select_cluster_cloud = *cluster_clouds[index];
    pcl::toROSMsg(select_cluster_cloud, extract_cloud_);
    extract_cloud_.header.frame_id = extract_cloud_frame_id_;
    is_ok_ = true;
    res.success = true;
    return res.success;
  }
  else
  {
    std::cout << "No extract cluster !!" << std::endl;
    is_ok_ = false;
    res.success = false;
    return res.success;
  }
}

bool RegionGrowingExtractorServer::getCluster(denso_recognition_srvs::ExtractObject::Request& req,
                                              denso_recognition_srvs::ExtractObject::Response& res)
{
  is_ok_ = false;
  pcl::PointCloud<PointInT>::Ptr cloud(new pcl::PointCloud<PointInT>);
  *cloud = *scene_cloud_;

  if (cloud->points.size() == 0)
  {
    ROS_ERROR_STREAM("No input cloud !!");
    res.success = false;
    return res.success;
  }

  pcl::search::Search<PointInT>::Ptr tree(new pcl::search::KdTree<PointInT>);
  pcl::PointCloud<PointNT>::Ptr normals(new pcl::PointCloud<PointNT>);
  pcl::NormalEstimation<PointInT, PointNT> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(k_search_size_);
  normal_estimator.compute(*normals);

  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::PassThrough<PointInT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*indices);

  pcl::RegionGrowing<PointInT, PointNT> reg;
  reg.setMinClusterSize(cluster_cloud_size_min_);
  reg.setMaxClusterSize(cluster_cloud_size_max_);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(number_of_neighbours_);
  reg.setInputCloud(cloud);
  reg.setIndices(indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(smooth_threshold_deg_ / 180.0 * M_PI);
  reg.setCurvatureThreshold(curvature_threshold_);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" << std::endl
            << "cloud that belong to the first cluster:" << std::endl;

  for (int i = 0; i < clusters.size(); i++)
  {
    pcl::PointCloud<PointInT>::Ptr cluster_cloud(new pcl::PointCloud<PointInT>);
    cluster_cloud->header.frame_id = cloud->header.frame_id;
    cluster_cloud->height = 1;
    cluster_cloud->width = clusters[i].indices.size();
    cluster_cloud->points.resize(clusters[i].indices.size());
    int extract_cloud_size = clusters[i].indices.size();
    if (extract_cluster_cloud_size_min_ < extract_cloud_size && extract_cloud_size < extract_cluster_cloud_size_max_)
    {
      for (int j = 0; j < extract_cloud_size; j++)
      {
        cluster_cloud->points[j].x = cloud->points[clusters[i].indices[j]].x;
        cluster_cloud->points[j].y = cloud->points[clusters[i].indices[j]].y;
        cluster_cloud->points[j].z = cloud->points[clusters[i].indices[j]].z;
      }
      cluster_clouds_.push_back(cluster_cloud);
    }
    else
    {
      continue;
    }
  }

  if (cluster_clouds_.empty())
  {
    std::cout << "No extract cluster !!" << std::endl;
    is_ok_ = false;
    res.success = false;
    return res.success;
  }

  res.success = true;
  return res.success;
}

bool RegionGrowingExtractorServer::setTargetCluster(denso_recognition_srvs::ExtractObject::Request& req,
                                                    denso_recognition_srvs::ExtractObject::Response& res)
{
  is_ok_ = false;
  if (!cluster_clouds_.empty())
  {
    std::vector<double> z_ave_vector;
    z_ave_vector.clear();
    for (int i = 0; i < cluster_clouds_.size(); i++)
    {
      double z_sum;
      for (int j = 0; j < cluster_clouds_[i]->points.size(); j++)
      {
        z_sum += cluster_clouds_[i]->points[j].z;
      }
      z_ave_vector.push_back(z_sum / cluster_clouds_[i]->points.size());
    }

    std::vector<double>::iterator itr = std::max_element(z_ave_vector.begin(), z_ave_vector.end());
    size_t index = std::distance(z_ave_vector.begin(), itr);

    pcl::PointCloud<PointInT> select_cluster_cloud;
    select_cluster_cloud = *cluster_clouds_[index];
    pcl::toROSMsg(select_cluster_cloud, extract_cloud_);
    extract_cloud_.header.frame_id = extract_cloud_frame_id_;
    cluster_clouds_.erase(cluster_clouds_.begin() + index);
    is_ok_ = true;
    res.success = true;
    return res.success;
  }
  else
  {
    std::cout << "No extract cluster !!" << std::endl;
    is_ok_ = false;
    res.success = false;
    return res.success;
  }
}

void RegionGrowingExtractorServer::publishExtractCloud()
{
  if (is_ok_)
  {
    extract_cloud_pub_.publish(extract_cloud_);
  }
}