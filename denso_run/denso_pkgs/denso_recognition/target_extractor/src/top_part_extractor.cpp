#include <target_extractor/top_part_extractor.h>

using top_part_extractor::TopPartExtractor;

// Class methods definitions
TopPartExtractor::TopPartExtractor(ros::NodeHandle& nh)
  : nh_(nh)
  , flag_(false)
  , leaf_size_(0.004)
  , bulk_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
  , part_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output_cloud", 10);
  cloud_sub_ = nh.subscribe("/input_cloud", 1, &TopPartExtractor::updatePointCloud, this);
}

void TopPartExtractor::publish(void)
{
  if (!flag_)
    return;

  const int extracting_point_number = 12500;
  part_cloud_ = TopPartExtractor::extractCloud(getHighestPoint(bulk_cloud_), extracting_point_number);

  // publish
  sensor_msgs::PointCloud2 cloud_msg;
  part_cloud_->header.frame_id = bulk_cloud_->header.frame_id;
  pcl::toROSMsg(*part_cloud_, cloud_msg);
  cloud_pub_.publish(cloud_msg);
}

void TopPartExtractor::updatePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // worldと点群のframe_id間のTFが取得できるまで繰り返す
  while (true)
  {
    try
    {
      listener_.lookupTransform("world", cloud_msg->header.frame_id, ros::Time(0), transform_);
      ROS_INFO_ONCE("I got a transform!");
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  sensor_msgs::PointCloud2 cloud_msg_transformed;
  pcl_ros::transformPointCloud("world", transform_, *cloud_msg, cloud_msg_transformed);

  pcl::fromROSMsg(cloud_msg_transformed, *bulk_cloud_);
  // downsampleCloud(bulk_cloud_, leaf_size_);
  flag_ = true;
}

void TopPartExtractor::downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, const double leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*input_cloud);
}

geometry_msgs::Vector3 TopPartExtractor::getHighestPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  part_cloud_->clear();
  geometry_msgs::Vector3 top_point;
  top_point.z = 0;

  const char RED_RENGE = 100;

  for (auto pt : cloud->points)
  {
    if (pt.r <= RED_RENGE)  continue; // if the color is NOT RED

    if (pt.z <= top_point.z)  continue;  // if z is LOWER than top_z

    // if both "color is RED" and "z is taller than z" are simultaneously true
    top_point.x = pt.x;
    top_point.y = pt.y;
    top_point.z = pt.z;
  };

  std::cout << "The highest RED point's geometry is ..." << std::endl;
  std::cout << "x : " << top_point.x << std::endl;
  std::cout << "y : " << top_point.y << std::endl;
  std::cout << "z : " << top_point.z << std::endl;

  return top_point;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TopPartExtractor::extractCloud(const geometry_msgs::Vector3 point,
                                                                         const int radius)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // k-d木オブジェクト
  pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(bulk_cloud_);

  // 次のポイントに対する最近傍点を5個見つける(これはクラウドの点である必要はない)
  pcl::PointXYZRGB pcl_point;
  pcl_point.x = point.x;
  pcl_point.y = point.y;
  pcl_point.z = point.z;
  // 出力の近傍点を記憶するためのベクトル
  std::vector<int> pointIndices(5);
  // このベクトルは探索ポイントへの距離の自乗を記憶する
  std::vector<float> squaredDistances(5);

  pcl::PointXYZRGB buf_cloud;
  // 探索を行い結果を出力する
  if (kdtree.nearestKSearch(pcl_point, radius, pointIndices, squaredDistances) > 0)
  {
    std::cout << "5 nearest neighbors of the point:" << std::endl;
    for (size_t i = 0; i < pointIndices.size(); ++i)
    {
      buf_cloud.x = bulk_cloud_->points[pointIndices[i]].x;
      buf_cloud.y = bulk_cloud_->points[pointIndices[i]].y;
      buf_cloud.z = bulk_cloud_->points[pointIndices[i]].z;
      buf_cloud.r = 0;
      buf_cloud.g = 0;
      buf_cloud.b = 255;
      extracted_cloud->push_back(buf_cloud);
    }
  }

  // // このポイントから3cm内の近傍点をすべて見つける(ポイントを中心として半径3cmの球内の点)
  // if (kdtree.radiusSearch(pcl_point, 0.03, pointIndices, squaredDistances) > 0)
  // {
  // 	std::cout << "Neighbors within 3cm:" << std::endl;
  // 	for (size_t i = 0; i < pointIndices.size(); ++i)
  // 		std::cout << "\t" << bulk_cloud_->points[pointIndices[i]].x
  // 				      << "  " << bulk_cloud_->points[pointIndices[i]].y
  // 				      << "  " << bulk_cloud_->points[pointIndices[i]].z
  // 				      << " (squared distance: " << squaredDistances[i] << ")" << std::endl;
  // }

  return extracted_cloud;
}
