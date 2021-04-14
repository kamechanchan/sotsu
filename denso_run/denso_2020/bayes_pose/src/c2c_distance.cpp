#include <bayes_pose/c2c_distance.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

using bayes_pose::C2CDistance;

C2CDistance::C2CDistance(ros::NodeHandle nh, const std::string& compared_pc_topic_name,
                         const std::string& reference_pc_topic_name)
  : nh_(nh)
  , compared_pc_sub_(nh_, compared_pc_topic_name, 1)
  , reference_pc_sub_(nh_, reference_pc_topic_name, 1)
  , sync_(SyncPolicy(10), compared_pc_sub_, reference_pc_sub_)
{
  sync_.registerCallback(boost::bind(&C2CDistance::calculate, this, _1, _2));
}

void C2CDistance::calculate(const sensor_msgs::PointCloud2ConstPtr& compared_pc,
                            const sensor_msgs::PointCloud2ConstPtr& reference_pc)
{
  pcl::PointCloud<pcl::PointXYZ> compared_pc_pcl, reference_pc_pcl;
  pcl::fromROSMsg(*compared_pc, compared_pc_pcl);
  pcl::fromROSMsg(*reference_pc, reference_pc_pcl);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(reference_pc_pcl.makeShared());
  const int K = 1;

  std::vector<int> nearest_point_index;
  std::vector<float> nearest_point_distance;
  double c2c_distance = 0.0;
  int point_size = 0;

  for (auto compared_point : compared_pc_pcl.points)
  {
    if (kdtree.nearestKSearch(compared_point, K, nearest_point_index, nearest_point_distance) > 0)
    {
      c2c_distance += nearest_point_distance[0];
      point_size++;
    }
    nearest_point_index.clear();
    nearest_point_distance.clear();
  }
  ROS_INFO_STREAM("point_size: " << point_size);
  ROS_INFO_STREAM("c2c_distance: " << c2c_distance);
  ROS_INFO_STREAM("c2c_distance(mean): " << c2c_distance / point_size);

  return;
}
