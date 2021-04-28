#include <meshcloud_publisher/meshcloud_publisher.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

using meshcloud_publisher::MeshCloudPublisher;
using meshcloud_publisher::MeshCloudParam;
using meshcloud_sampler::uniform_sampling;

const int sampling_points = 100000;

const int MeshCloudParam::RETRY_COUNT_LIMIT = 10;
const float MeshCloudParam::DURATION_TIME = 1.0;
const float MeshCloudParam::LEAF_SIZE = 0.0003f;

MeshCloudPublisher::MeshCloudPublisher(ros::NodeHandle& nh)
  : nh_(nh), downsampled_mesh_pointcloud_ptr_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
{
  const std::vector<int> ar_marker_indice = { 0, 2, 3, 5 };

  const std::string link_name_prefix = "ARmarker_";
  const std::string link_name_extention = ".stl";

  /*for (auto it = std::begin(ar_marker_indice); it != std::end(ar_marker_indice); ++it)
  {
    std::stringstream ss;
    ss << std::setfill('0') << std::right << std::setw(2) << *it;
    link_names_.push_back("ARmarker_" + ss.str() + ".stl");
    frame_names_.push_back("/true_ar_marker_" + std::to_string(*it) + "_center");

   // const std::string relative_path = "/meshes/ar_marker_cube_id" + ss.str() + "/" + link_names_.back();
   // this->getMesh(ros::package::getPath("ar_marker_urdf") + relative_path);
    const std::string relative_path = "/object_description/meshes/STL/HV8.stl";
    this->getMesh(ros::package::getPath("denso_descriptions") + relative_path);
  }*/
  const std::string relative_path = "/object_description/meshes/STL/HV8.stl";
  //this->getMesh(ros::package::getPath("denso_descriptions") + relative_path);
  this->getMesh("/home/ericlab/HV8.stl");
  frame_names_.push_back("photoneo_center_camera_frame");

  this->transformMesh();

  mesh_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/mesh_cloud", 1);
}

void MeshCloudPublisher::getMesh(const std::string dir_path)
{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileSTL(dir_path, mesh);
  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh, polydata1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr parts_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  uniform_sampling(polydata1, sampling_points, *parts_cloud);
  parts_clouds_.push_back(*parts_cloud);
}

void MeshCloudPublisher::transformMesh()
{
  pcl::PointCloud<pcl::PointXYZ> transformed_parts_cloud;

  for (int retryCount = 0; retryCount < MeshCloudParam::RETRY_COUNT_LIMIT; ++retryCount)
  {
    for (size_t i = 0; i < parts_clouds_.size(); ++i)
    {
      try
      {
        tf::StampedTransform transform;
        tf_.lookupTransform("/world", frame_names_[i], ros::Time(0), transform);
        // tf_.lookupTransform("/mhand_body_link", frame_names_[i],
        //                     ros::Time(0), transform);
        pcl_ros::transformPointCloud(parts_clouds_[i], transformed_parts_cloud, transform);
        mesh_pointcloud_pcl_ += transformed_parts_cloud;
        ROS_INFO_STREAM("Get transform : " << frame_names_[i]);
      }
      catch (tf2::LookupException e)
      {
        ROS_ERROR("pcl::ros %s", e.what());
        ros::Duration(MeshCloudParam::DURATION_TIME).sleep();
        mesh_pointcloud_pcl_.clear();
        break;
      }
      catch (tf2::ExtrapolationException e)
      {
        ROS_ERROR("pcl::ros %s", e.what());
        ros::Duration(MeshCloudParam::DURATION_TIME).sleep();
        mesh_pointcloud_pcl_.clear();
        break;
      }
      catch (...)
      {
        ros::Duration(MeshCloudParam::DURATION_TIME).sleep();
        mesh_pointcloud_pcl_.clear();
        break;
      }
    }
    if (mesh_pointcloud_pcl_.points.size() == (sampling_points * parts_clouds_.size()))
    {
      std::cout << "0) mesh : " << mesh_pointcloud_pcl_.points.size() << std::endl;

      pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_pointcloud_ptr_pcl(
          new pcl::PointCloud<pcl::PointXYZ>(mesh_pointcloud_pcl_));
      downSample(mesh_pointcloud_ptr_pcl, downsampled_mesh_pointcloud_ptr_pcl_);

      return;
    }
  }
  ROS_WARN_STREAM("try tf transform 5times, but failed");
  exit(-1);
  return;
}

void MeshCloudPublisher::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(MeshCloudParam::LEAF_SIZE, MeshCloudParam::LEAF_SIZE, MeshCloudParam::LEAF_SIZE);
  sor.filter(*cloud_filtered);
  std::cout << "------------------------------------" << std::endl;
  std::cout << "DownSampled cloud point size : " << cloud_filtered->points.size() << std::endl;
}

void MeshCloudPublisher::publishCloud()
{
  // pcl::toROSMsg(mesh_pointcloud_pcl_, mesh_pointcloud_ros_);
  pcl::toROSMsg(*downsampled_mesh_pointcloud_ptr_pcl_, mesh_pointcloud_ros_);
  mesh_pointcloud_ros_.header.stamp = ros::Time::now();
  mesh_pointcloud_ros_.header.frame_id = "world";
  mesh_pointcloud_publisher_.publish(mesh_pointcloud_ros_);
  std::cout << "===================================" << std::endl;
}
