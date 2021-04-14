#include <pointcloud_registrator/pointcloud_registrator.h>

#include <ros/package.h>

#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

using pointcloud_registrator::PointCloudRegistrator;

PointCloudRegistrator::PointCloudRegistrator(ros::NodeHandle& nh)
  : nh_(nh)
  , pointcloud1_sub_(nh_, "/mesh_cloud", 1)
  , pointcloud2_sub_(nh_, "/extracted_pointcloud", 1)
  , sync_(MySyncPolicy(10), pointcloud1_sub_, pointcloud2_sub_)
{
  icp_data_pub_ = nh.advertise<std_msgs::Float64MultiArray>("icp_data_array", 10);
  sync_.registerCallback(boost::bind(&PointCloudRegistrator::sensorAndMeshPointCloudCallback, this, _1, _2));
}

PointCloudRegistrator::~PointCloudRegistrator()
{
}

void PointCloudRegistrator::runICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_mesh_pcl,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sensor_pcl)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(1000);

  if (pc_sensor_pcl->points.size() == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> Final;
  Eigen::Vector4f c_mesh, c_sensor;
  std_msgs::Float64MultiArray icp_result_data;
  pcl::compute3DCentroid(*pc_mesh_pcl, c_mesh);      // 重心計算して代入
  pcl::compute3DCentroid(*pc_sensor_pcl, c_sensor);  // フィルタかけた点群の重心計算
  Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(c_mesh(0, 0) - c_sensor(0, 0), c_mesh(1, 0) - c_sensor(1, 0),
                                        c_mesh(2, 0) - c_sensor(2, 0));
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  ROS_INFO_STREAM("Matching Start!!!");
  icp.setInputSource(pc_sensor_pcl);
  icp.setInputTarget(pc_mesh_pcl);
  icp.setTransformationEpsilon(1e-13);
  icp.setEuclideanFitnessEpsilon(1e-10);
  icp.align(Final, init_guess);
  ROS_INFO_STREAM("has converged : " << icp.hasConverged());
  ROS_INFO_STREAM("score : " << icp.getFitnessScore());

  Eigen::Matrix4d sensor_to_mesh =
      (icp.getFinalTransformation()).cast<double>();  //最終的に得られたICPによる変換行列を変数として置く
  std::cout << "----ICP's matrix------------------------" << std::endl;
  std::cout << sensor_to_mesh << std::endl;
  std::cout << "----------------------------------------" << std::endl;

  tf::matrixEigenToMsg(sensor_to_mesh, icp_result_data);
  icp_data_pub_.publish(icp_result_data);
}

void PointCloudRegistrator::sensorAndMeshPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_mesh_ros,
                                                            const sensor_msgs::PointCloud2::ConstPtr& pc_sensor_ros)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_mesh_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sensor_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*pc_mesh_ros, *pc_mesh_pcl);
  pcl::fromROSMsg(*pc_sensor_ros, *pc_sensor_pcl);

  std::vector<int> nan_index;
  pcl::removeNaNFromPointCloud(*pc_mesh_pcl, *pc_mesh_pcl, nan_index);
  pcl::removeNaNFromPointCloud(*pc_sensor_pcl, *pc_sensor_pcl, nan_index);

  // Start ICP
  runICP(pc_mesh_pcl, pc_sensor_pcl);
}
