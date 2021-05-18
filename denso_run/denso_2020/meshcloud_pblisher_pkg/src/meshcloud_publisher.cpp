#include <meshcloud_publisher/meshcloud_publisher.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using meshcloud_publisher::MeshCloudPublisher;
using meshcloud_sampler::uniform_sampling;

MeshCloudPublisher::MeshCloudPublisher(ros::NodeHandle& nh)
  : nh_(nh), downsampled_mesh_pointcloud_ptr_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
{
  ROS_INFO("const");
  ros::param::param<std::string>("~node_id", node_id_, "0");
  ros::param::param<std::string>("~yaml_file", yaml_file_, "four_AR_cubes.yaml");
  ros::param::param<std::string>("~parent_frame", mesh_pc_parent_frame_, "world");
  ros::param::param<double>("~leaf_size", leaf_size_, 0.01);
  ros::param::param<int>("~sampling_points", sampling_points_, 10000);

  model_names_.clear();
  base_frames_.clear();
  mesh_pkgs_.clear();
  to_mesh_paths_.clear();
  mesh_names_.clear();
  meshes_x_.clear();
  meshes_y_.clear();
  meshes_z_.clear();
  meshes_qx_.clear();
  meshes_qy_.clear();
  meshes_qz_.clear();
  meshes_qw_.clear();
  ts_.clear();

  yaml_path_ = ros::package::getPath("meshcloud_publisher_pkg") + "/config/" + yaml_file_;
  ROS_INFO_STREAM("YAML file path: " << yaml_path_);

  mesh_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/mesh_cloud_id_" + node_id_ , 1);
}

MeshCloudPublisher::~MeshCloudPublisher()
{
}

bool MeshCloudPublisher::setMeshCloud()
{
  if (!loadMeshInfoFromYAML(yaml_path_))
  {
    ROS_ERROR("Failed to load yaml file");
    return false;
  }

  for (int i = 0; i < model_number_; i++)
  {
    getMesh(ros::package::getPath(mesh_pkgs_[i]) + "/" + to_mesh_paths_[i] + "/" + mesh_names_[i]);
    geometry_msgs::TransformStamped t;
    t.header.frame_id = base_frames_[i];
    t.child_frame_id = model_names_[i];
    t.transform.translation.x = meshes_x_[i];
    t.transform.translation.y = meshes_y_[i];
    t.transform.translation.z = meshes_z_[i];
    t.transform.rotation.x = meshes_qx_[i];
    t.transform.rotation.y = meshes_qy_[i];
    t.transform.rotation.z = meshes_qz_[i];
    t.transform.rotation.w = meshes_qw_[i];
    ts_.push_back(t);
  }

  transformMesh();

  return true;
}

void MeshCloudPublisher::getMesh(const std::string dir_path)
{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileSTL(dir_path, mesh);
  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh, polydata1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr parts_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr parts_cloud_filterd(new pcl::PointCloud<pcl::PointXYZ>);
  uniform_sampling(polydata1, sampling_points_, *parts_cloud);

  // passthrough filter
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(parts_cloud);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(0.0, FLT_MAX);
  // pass.filter(*parts_cloud_filterd);
  // parts_clouds_.push_back(*parts_cloud_filterd);

  parts_clouds_.push_back(*parts_cloud);
}

void MeshCloudPublisher::transformMesh()
{
  int itr = 0;
  for (auto ts : ts_)
  {
    ROS_INFO("loop start");
    tf::Transform pose;
    tf::Vector3 position(ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z);
    // tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);
    tf::Quaternion orientation(ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w);
    pose.setOrigin(position);
    pose.setRotation(orientation);
    tf::StampedTransform transform(pose, ros::Time::now(), base_frames_[itr], ts.child_frame_id);
    pcl_ros::transformPointCloud(parts_clouds_[itr], mesh_pointcloud_pcl_, transform);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_pointcloud_ptr_pcl(new pcl::PointCloud<pcl::PointXYZ>(mesh_pointcloud_pcl_));
    ROS_INFO("downSample");
    downSample(mesh_pointcloud_ptr_pcl, downsampled_mesh_pointcloud_ptr_pcl_);
    ROS_INFO("downSample done");
    pcl::toROSMsg(*downsampled_mesh_pointcloud_ptr_pcl_, mesh_pointcloud_ros_);
    ROS_INFO("toROSMsg");
    sensor_msgs::PointCloud2 tmp;
    tmp = merged_pointcloud_ros_;
    pcl::concatenatePointCloud(mesh_pointcloud_ros_, tmp, merged_pointcloud_ros_);
    ROS_INFO("concatenatePointCloud");
    itr += 1;
  }
}

void MeshCloudPublisher::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  sor.filter(*cloud_filtered);
  std::cout << "------------------------------------" << std::endl;
  std::cout << "DownSampled cloud point size : " << cloud_filtered->points.size() << std::endl;
}

void MeshCloudPublisher::publishCloud()
{
  merged_pointcloud_ros_.header.stamp = ros::Time::now();
  merged_pointcloud_ros_.header.frame_id = mesh_pc_parent_frame_;
  mesh_pointcloud_publisher_.publish(merged_pointcloud_ros_);
  std::cout << "===================================" << std::endl;
}

void MeshCloudPublisher::broadcastTF()
{
  for (auto ts : ts_)
  {
    ts.header.stamp = ros::Time::now();
    br_.sendTransform(ts);
  }
}

bool MeshCloudPublisher::loadMeshInfoFromYAML(const std::string& file_path)
{
  try
  {
    YAML::Node node = YAML::LoadFile(file_path);
    model_number_ = node["meshes"].size();
    for (int i = 0; i < model_number_; i++)
    {
      model_names_.push_back(node["meshes"][i]["name"].as<std::string>());
      base_frames_.push_back(node["meshes"][i]["base_frame"].as<std::string>());
      mesh_pkgs_.push_back(node["meshes"][i]["mesh_pkg"].as<std::string>());
      to_mesh_paths_.push_back(node["meshes"][i]["path_from_pkg_to_mesh"].as<std::string>());
      mesh_names_.push_back(node["meshes"][i]["mesh_name"].as<std::string>());
      meshes_x_.push_back(node["meshes"][i]["x"].as<double>());
      meshes_y_.push_back(node["meshes"][i]["y"].as<double>());
      meshes_z_.push_back(node["meshes"][i]["z"].as<double>());
      meshes_qx_.push_back(node["meshes"][i]["qx"].as<double>());
      meshes_qy_.push_back(node["meshes"][i]["qy"].as<double>());
      meshes_qz_.push_back(node["meshes"][i]["qz"].as<double>());
      meshes_qw_.push_back(node["meshes"][i]["qw"].as<double>());
    }
  }
  catch (YAML::Exception& e)
  {
    ROS_ERROR_STREAM("YAML file loading error: " << e.what());
    return false;
  }

  return true;
}
