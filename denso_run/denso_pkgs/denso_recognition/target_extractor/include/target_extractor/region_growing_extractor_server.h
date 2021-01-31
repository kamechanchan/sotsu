#ifndef REGION_GROWING_EXTRACTOR_SERVER_H
#define REGION_GROWING_EXTRACTOR_SERVER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <denso_recognition_srvs/ExtractObject.h>

typedef pcl::PointXYZ PointInT;
typedef pcl::PointXYZ PointOutT;
typedef pcl::Normal PointNT;

namespace region_growing_extractor_server
{
class RegionGrowingExtractorServer
{
public:
  RegionGrowingExtractorServer(ros::NodeHandle& nh);
  void publishExtractCloud();

private:
  void receiveSceneTopicCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  bool getTopCluster(denso_recognition_srvs::ExtractObject::Request& req,
                     denso_recognition_srvs::ExtractObject::Response& res);
  bool getCluster(denso_recognition_srvs::ExtractObject::Request& req,
                  denso_recognition_srvs::ExtractObject::Response& res);
  bool setTargetCluster(denso_recognition_srvs::ExtractObject::Request& req,
                        denso_recognition_srvs::ExtractObject::Response& res);

private:
  ros::NodeHandle nh_;
  ros::Publisher extract_cloud_pub_;
  ros::Subscriber scene_topic_sub_;
  ros::ServiceServer get_top_cluster_server_;
  ros::ServiceServer get_cluster_server_;
  ros::ServiceServer set_target_cluster_server_;
  bool is_ok_;
  int cluster_cloud_size_min_;
  int cluster_cloud_size_max_;
  int k_search_size_;
  int number_of_neighbours_;
  int extract_cluster_cloud_size_min_;
  int extract_cluster_cloud_size_max_;
  double smooth_threshold_deg_;
  double curvature_threshold_;
  std::string scene_topic_name_;
  std::string extract_cloud_topic_name_;
  std::string extract_cloud_frame_id_;
  pcl::PointCloud<PointInT>::Ptr scene_cloud_;
  sensor_msgs::PointCloud2 extract_cloud_;
  std::vector<pcl::PointCloud<PointInT>::Ptr> cluster_clouds_;
};
}  // namespace region_growing_extractor_server

#endif  // REGION_GROWING_EXTRACTOR_SERVER_H