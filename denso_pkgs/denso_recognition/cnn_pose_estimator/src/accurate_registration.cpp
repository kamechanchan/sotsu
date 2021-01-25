#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkVersion.h>

#include <pcl/common/io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include <geometry_msgs/Pose.h>

const int sampling_points = 10000;

// #define gazebo

inline double uniform_deviate(int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2,
                                float c3, Eigen::Vector4f& p)
{
  float r1 = static_cast<float>(uniform_deviate(rand()));
  float r2 = static_cast<float>(uniform_deviate(rand()));
  float r1sqr = sqrtf(r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}

inline void randPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas, double totalArea,
                         Eigen::Vector4f& p)
{
  float r = static_cast<float>(uniform_deviate(rand()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
  vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType* ptIds = NULL;
  polydata->GetCellPoints(el, npts, ptIds);
  polydata->GetPoint(ptIds[0], A);
  polydata->GetPoint(ptIds[1], B);
  polydata->GetPoint(ptIds[2], C);
  randomPointTriangle(float(A[0]), float(A[1]), float(A[2]), float(B[0]), float(B[1]), float(B[2]), float(C[0]),
                      float(C[1]), float(C[2]), p);
}

void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples,
                      pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
  polydata->BuildCells();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); i++)
  {
    polydata->GetPoint(ptIds[0], p1);
    polydata->GetPoint(ptIds[1], p2);
    polydata->GetPoint(ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }

  cloud_out.points.resize(n_samples);
  cloud_out.width = static_cast<pcl::uint32_t>(n_samples);
  cloud_out.height = 1;

  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    randPSurface(polydata, &cumulativeAreas, totalArea, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}

class RobotMeshCloud
{
public:
  RobotMeshCloud() : rate_(1), pcl_shifted_cloud_(new pcl::PointCloud<pcl::PointXYZ>()), init_icp_finished_(false)
  {
    // link_names_.push_back("base_link.stl");
    // frame_names_.push_back("/base_link");
    // link_names_.push_back("J1.stl");
    // frame_names_.push_back("/J1");
    // link_names_.push_back("J2.stl");
    // frame_names_.push_back("/J2");
    // link_names_.push_back("J3.stl");
    // frame_names_.push_back("/J3");
    // link_names_.push_back("J4.stl");
    // frame_names_.push_back("/J4");
    // link_names_.push_back("J5.stl");
    // frame_names_.push_back("/J5");
    // link_names_.push_back("J6.stl");
    // frame_names_.push_back("/J6");

    corrected_cloud_frame_ = "kinect2_rgb_optical_frame";

    // for(int i = 0; i < link_names_.size(); ++i){
    //   this->getMesh(ros::package::getPath("automatic_calibration")+"/STL/vs087/"+link_names_[i],
    //   frame_names_[i]);
    // }

    // mhand 表示
    // frame_names_.push_back("hand_link");
    frame_names_.push_back("mhand_body_link");
    frame_names_.push_back("mhand_finger_A");
    frame_names_.push_back("mhand_finger_B");
    frame_names_.push_back("mhand_finger_C");

    this->getMesh(ros::package::getPath("mhand_description") + "/meshes/STL/mhand_body.stl", "mhand_body_link");
    this->getMesh(ros::package::getPath("mhand_description") + "/meshes/STL/mhand_finger.stl", "mhand_finger_A");
    this->getMesh(ros::package::getPath("mhand_description") + "/meshes/STL/mhand_finger.stl", "mhand_finger_B");
    this->getMesh(ros::package::getPath("mhand_description") + "/meshes/STL/mhand_finger.stl", "mhand_finger_C");

    this->transformMesh();
    // ### Node Handles ###
    mesh_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/mesh_cloud", 1);
    shifted_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/shifted_cloud", 1);
    corrected_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 1);

    kinect_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/kinect_center/qhd/points", 10, boost::bind(&RobotMeshCloud::pointCloudCallback, this, _1));
    // frame_timer_ = nh_.createTimer(ros::Duration(0.01),
    // boost::bind(&RobotMeshCloud::frameCallback, this, _1));
    result_transform_publisher_ = nh_.advertise<geometry_msgs::Pose>("/result_transform", 1);
  }
  ~RobotMeshCloud()
  {
  }
  void getMesh(std::string dir_path, std::string frame_name)
  {
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(dir_path, mesh);
    vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
    ;
    pcl::io::mesh2vtk(mesh, polydata1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr parts_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    uniform_sampling(polydata1, sampling_points, *parts_cloud);
    parts_clouds_.push_back(*parts_cloud);
  }

  void transformMesh()
  {
    pcl::PointCloud<pcl::PointXYZ> transformed_parts_cloud;
    for (int retryCount = 0; retryCount < 10; ++retryCount)
    {
      for (size_t i = 0; i < parts_clouds_.size(); ++i)
      {
        try
        {
          tf::StampedTransform transform;
          // tf_.lookupTransform("/base_link", frame_names_[i],
          //                     ros::Time(0), transform);
          tf_.lookupTransform("/mhand_body_link", frame_names_[i], ros::Time(0), transform);
          pcl_ros::transformPointCloud(parts_clouds_[i], transformed_parts_cloud, transform);
          sia5_cloud_ += transformed_parts_cloud;
          ROS_INFO_STREAM("Get transform : " << frame_names_[i]);
        }
        catch (tf2::LookupException e)
        {
          ROS_ERROR("pcl::ros %s", e.what());
          ros::Duration(1.0).sleep();
          sia5_cloud_.clear();
          break;
        }
        catch (tf2::ExtrapolationException e)
        {
          ROS_ERROR("pcl::ros %s", e.what());
          ros::Duration(1.0).sleep();
          sia5_cloud_.clear();
          break;
        }
        catch (...)
        {
          ros::Duration(1.0).sleep();
          sia5_cloud_.clear();
          break;
        }
      }
      if (sia5_cloud_.points.size() == (sampling_points * parts_clouds_.size()))
      {
        std::cout << "0) mesh : " << sia5_cloud_.points.size() << std::endl;
        return;
      }
    }
    ROS_WARN_STREAM("try tf transform 5times, but failed");
    exit(-1);
    return;
  }

  void cropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
  {
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(0.1, 1.5);
    passthrough_filter.setFilterLimitsNegative(false);
    passthrough_filter.filter(*cloud);

    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.setFilterFieldName("x");
    passthrough_filter.setFilterLimits(-0.3, 0.7);
    passthrough_filter.setFilterLimitsNegative(false);
    passthrough_filter.filter(*cloud);

    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.setFilterFieldName("y");
    passthrough_filter.setFilterLimits(-0.15, 0.15);
    passthrough_filter.setFilterLimitsNegative(false);
    passthrough_filter.filter(*cloud);
    std::cout << "------------------------------------" << std::endl;
    std::cout << "filtered cloud point size : " << cloud->points.size() << std::endl;

    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ search_point;
    search_point.x = 0;
    search_point.y = 0;
    search_point.z = 0;
    double search_radius = 1.0;
    std::vector<float> point_radius_squared_distance;
    pcl::PointIndices::Ptr point_idx_radius_search(new pcl::PointIndices());

    if (kdtree.radiusSearch(search_point, search_radius, point_idx_radius_search->indices,
                            point_radius_squared_distance) > 0)
    {
      pcl::ExtractIndices<pcl::PointXYZ> extractor;
      extractor.setInputCloud(cloud);
      extractor.setIndices(point_idx_radius_search);
      extractor.setNegative(false);
      extractor.filter(*cloud_filtered);
    }
  }

  void downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
  {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
    std::cout << "------------------------------------" << std::endl;
    std::cout << "DownSampled  cloud point size : " << cloud_filtered->points.size() << std::endl;
  }

  pcl::PointCloud<pcl::PointXYZ> shiftPointCloud(pcl::PointCloud<pcl::PointXYZ> points, double x, double y, double z,
                                                 double roll, double pitch, double yaw)
  {
    tf::Matrix3x3 init_rotation;
    Eigen::AngleAxisf rotation_x(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_y(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_z(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(x, y, z);
    Eigen::Matrix4f noize_matrix = (translation * rotation_z * rotation_y * rotation_x).matrix();
    pcl::PointCloud<pcl::PointXYZ> shifted_cloud;
    pcl::transformPointCloud(points, shifted_cloud, noize_matrix);
    return shifted_cloud;
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& kinect_pointcloud)
  {
    if (init_icp_finished_)
    {
      return;
    }
    else
    {
      // ROSのメッセージからPCLの形式に変換
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*kinect_pointcloud, *pcl_pc);
      tf::StampedTransform transform;
      // world 座標系に変換
      try
      {
        tf_.waitForTransform("/world", kinect_pointcloud->header.frame_id, ros::Time(0), ros::Duration(10.0));
        tf_.lookupTransform("/world", kinect_pointcloud->header.frame_id, ros::Time(0), transform);
        Eigen::Matrix4f eigen_transform;
        pcl_ros::transformAsMatrix(transform, eigen_transform);

        std::cout << "----Initial position--------------------" << std::endl;
        std::cout << eigen_transform << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "kinect :" << kinect_pointcloud->data.size() << std::endl;

        Eigen::Affine3f eigen_affine_transform(eigen_transform);
        pcl::transformPointCloud(*pcl_pc, *pcl_shifted_cloud_, eigen_affine_transform);

        this->cropBox(pcl_shifted_cloud_, pcl_shifted_cloud_);
        this->downSampling(pcl_shifted_cloud_, pcl_shifted_cloud_);

        ROS_INFO_STREAM("shifted_cloud size : " << pcl_shifted_cloud_->points.size());
        sensor_msgs::PointCloud2 ros_shifted_cloud;
        pcl::toROSMsg(*pcl_shifted_cloud_, ros_shifted_cloud);  // pclで処理したやつをros形式へ変換
        ros_shifted_cloud.header.stamp = ros::Time::now();
        ros_shifted_cloud.header.frame_id = "/world";
        corrected_cloud_frame_ = "/world";
        shifted_cloud_publisher_.publish(ros_shifted_cloud);  // デバッグのためにずらしたやつのpublish
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }
  }

  // void frameCallback(const ros::TimerEvent&)
  // {
  //   std::cout << "frame call back" << std::endl;
  //   ros::Time time = ros::Time::now();
  //   br_.sendTransform(tf::StampedTransform(fixed_kinect_frame_, time,
  //   "world", "fixed_kinect_frame")); // Rvizへの出力?
  // }

  void run()
  {
    while (ros::ok())
    {
      pcl::toROSMsg(sia5_cloud_, mesh_pointcloud_);
      mesh_pointcloud_.header.stamp = ros::Time::now();
      mesh_pointcloud_.header.frame_id = "/world";
      mesh_pointcloud_publisher_.publish(mesh_pointcloud_);
      // ### ICP Argolithm ####
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      pcl::PointCloud<pcl::PointXYZ>::Ptr sia5_ptr(new pcl::PointCloud<pcl::PointXYZ>(sia5_cloud_));
      std::vector<int> nan_index;
      pcl::removeNaNFromPointCloud(*pcl_shifted_cloud_, *pcl_shifted_cloud_, nan_index);
      pcl::removeNaNFromPointCloud(*sia5_ptr, *sia5_ptr, nan_index);

      icp.setMaximumIterations(1000);
      if (pcl_shifted_cloud_->points.size() != 0)
      {
        pcl::PointCloud<pcl::PointXYZ> Final;
        Eigen::Vector4f c_sia5, c_kinect;
        pcl::compute3DCentroid(*sia5_ptr, c_sia5);              // 重心計算して代入
        pcl::compute3DCentroid(*pcl_shifted_cloud_, c_kinect);  // フィルタかけた点群の重心計算
        Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f init_translation(c_sia5(0, 0) - c_kinect(0, 0), c_sia5(1, 0) - c_kinect(1, 0),
                                              c_sia5(2, 0) - c_kinect(2, 0));
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
        ROS_INFO_STREAM("Matching Start!!!");
        icp.setInputSource(pcl_shifted_cloud_);
        icp.setInputTarget(sia5_ptr);
        icp.setTransformationEpsilon(1e-13);
        icp.setEuclideanFitnessEpsilon(1e-10);
        icp.align(Final, init_guess);
        ROS_INFO_STREAM("has converged : " << icp.hasConverged());
        ROS_INFO_STREAM("score : " << icp.getFitnessScore());

        try
        {
          tf::StampedTransform transform;  // icpかました後のカメラ位置推定処理
          tf_.lookupTransform("/world", "kinect_center_link", ros::Time::now(), transform);
          Eigen::Affine3d kinect_to_world_transform;
          tf::transformTFToEigen(transform, kinect_to_world_transform);  // Initial position
          Eigen::Matrix4d matrix_kinect_to_world =
              kinect_to_world_transform.matrix();  // kinectからワールド座標系への変換行列の定義
          Eigen::Matrix4d kinect_to_corrected =
              (icp.getFinalTransformation()).cast<double>();  //最終的に得られたICPによる変換行列を変数として置く
          std::cout << "----ICP's matrix------------------------" << std::endl;
          std::cout << kinect_to_corrected << std::endl;
          std::cout << "----------------------------------------" << std::endl;
          /*Eigen::Matrix4d kinect_center_to_corrected =
           * world_to_corrected*matrix_kinect_to_world; //座標変換*/
          Eigen::Matrix4d kinect_center_to_corrected = kinect_to_corrected * matrix_kinect_to_world;

          std::cout << "----Corrected position------------------" << std::endl;
          std::cout << kinect_center_to_corrected << std::endl;
          std::cout << "----------------------------------------" << std::endl;
          // Eigen::Matrix4d kinect_center_corrected =
          // kinect_center_to_corrected.normalized(); //正規化失敗はここ？
          // Eigen::Affine3d eigen_affine3d(kinect_center_corrected);
          Eigen::Affine3d eigen_affine3d(kinect_center_to_corrected);
          tf::transformEigenToTF(eigen_affine3d, fixed_kinect_frame_);

          tf::poseTFToMsg(fixed_kinect_frame_, result_transform_);
          result_transform_publisher_.publish(result_transform_);

          Eigen::Matrix3d rotation_matrix =
              kinect_center_to_corrected.block(0, 0, 3, 3);  // matrix of 4 times 4 started from (0,0)
          Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  //回転行列からオイラー角を計算
          std::cout << "<origin xyz=\"" << kinect_center_to_corrected(0, 3) << " " << kinect_center_to_corrected(1, 3)
                    << " " << kinect_center_to_corrected(2, 3) << "\" rpy=\"" << euler_angles(2) << " "
                    << euler_angles(1) << " " << euler_angles(0) << "\" />" << std::endl;
        }
        catch (...)
        {
          ROS_ERROR("tf fail");
        }
        sensor_msgs::PointCloud2 ros_corrected_cloud;
        pcl::toROSMsg(Final, ros_corrected_cloud);  // ICPこの関数の最初で定義した点群を新しい文字に代入
        ros_corrected_cloud.header.stamp = ros::Time::now();
        // ros_corrected_cloud.header.frame_id = "/base_link";
        ros_corrected_cloud.header.frame_id = "/world";
        corrected_cloud_publisher_.publish(ros_corrected_cloud);  //変換した点群をpublish
      }
      ros::spinOnce();
      rate_.sleep();
      std::cout << "===================================" << std::endl;
    }
  }

private:
  std::vector<pcl::PolygonMesh> meshes_;
  sensor_msgs::PointCloud2 mesh_pointcloud_;
  geometry_msgs::Pose result_transform_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  tf::TransformBroadcaster br_;
  ros::Rate rate_;
  ros::Publisher mesh_pointcloud_publisher_;
  ros::Publisher shifted_cloud_publisher_;
  ros::Publisher corrected_cloud_publisher_;
  ros::Publisher result_transform_publisher_;
  ros::Timer frame_timer_;
  ros::Subscriber kinect_subscriber_;
  std::vector<std::string> link_names_;
  std::vector<std::string> frame_names_;
  pcl::PointCloud<pcl::PointXYZ> sia5_cloud_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> parts_clouds_;
  sensor_msgs::PointCloud2 kinect_pointcloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_shifted_cloud_;
  std::string corrected_cloud_frame_;
  bool init_icp_finished_;
  tf::Transform fixed_kinect_frame_;
  tf::Transform kinect_optical_frame_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "registration");
  RobotMeshCloud mesh_cloud;
  mesh_cloud.run();
  return 0;
}
