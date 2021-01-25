#include <pointcloud_processing/crop_cloud.h>

using crop_cloud::CropCloud;

CropCloud::CropCloud(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh)
  , rate_(n.param("loop_rate", 10))
  , frame_id_(n.param<std::string>("crop_frame_id", "world"))
  , use_rgb_(n.param<bool>("use_rgb", "false"))
{
  source_pc_sub_ =
      nh_.subscribe(n.param<std::string>("crop_pc_src", "/merged_cloud"), 1, &CropCloud::cropCallback, this);
  fileterd_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("crop_pc_dst", "/filtered_pointcloud"), 1);

  // clopboxを当てはめるエリアを定義
  n.param<float>("crop_x_min", crop_min_rgb_.x, 0.15);
  n.param<float>("crop_x_max", crop_max_rgb_.x, 1.5);
  n.param<float>("crop_y_min", crop_min_rgb_.y, -1.5);
  n.param<float>("crop_y_max", crop_max_rgb_.y, 1.5);
  n.param<float>("crop_z_min", crop_min_rgb_.z, 0.01);
  n.param<float>("crop_z_max", crop_max_rgb_.z, 0.5);

  n.param<float>("crop_x_min", crop_min_.x, 0.15);
  n.param<float>("crop_x_max", crop_max_.x, 1.5);
  n.param<float>("crop_y_min", crop_min_.y, -1.5);
  n.param<float>("crop_y_max", crop_max_.y, 1.5);
  n.param<float>("crop_z_min", crop_min_.z, 0.01);
  n.param<float>("crop_z_max", crop_max_.z, 0.5);
}

void CropCloud::cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  //点群をKinect座標系からWorld座標系に変換
  //変換されたデータはtrans_pcに格納される．
  sensor_msgs::PointCloud2 trans_pc;
  try
  {
    pcl_ros::transformPointCloud(frame_id_, *source_pc, trans_pc, tf_);
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZRGB> pcl_source_rgb;
  pcl::PointCloud<pcl::PointXYZ> pcl_source;
  if (use_rgb_)
  {
    pcl::fromROSMsg(trans_pc, pcl_source_rgb);
  }
  else
  {
    pcl::fromROSMsg(trans_pc, pcl_source);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_source_ptr_rgb(new pcl::PointCloud<pcl::PointXYZRGB>(pcl_source_rgb));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<pcl::PointXYZ>(pcl_source));

  if (use_rgb_)
  {
    cropBox(pcl_source_ptr_rgb, crop_min_rgb_, crop_max_rgb_);
  }
  else
  {
    cropBox(pcl_source_ptr, crop_min_, crop_max_);
  }

  // 処理後の点群をpublish
  sensor_msgs::PointCloud2 filtered_pc2;
  if (use_rgb_)
  {
    pcl::toROSMsg(*pcl_source_ptr_rgb, filtered_pc2);
  }
  else
  {
    pcl::toROSMsg(*pcl_source_ptr, filtered_pc2);
  }
  filtered_pc2.header.stamp = ros::Time::now();
  filtered_pc2.header.frame_id = "world";
  fileterd_cloud_pub_.publish(filtered_pc2);
}

void CropCloud::cropBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB min,
                        const pcl::PointXYZRGB max)
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

  pcl::CropBox<pcl::PointXYZRGB> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);
  cropFilter.setRotation(boxRotation);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  cropFilter.filter(*cloud_filtered);
  pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*cloud_filtered, *cloud);
}

void CropCloud::cropBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ min,
                        const pcl::PointXYZ max)
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

void CropCloud::run()
{
  while (nh_.ok())
  {
    ros::spinOnce();
    rate_.sleep();
  }
}
