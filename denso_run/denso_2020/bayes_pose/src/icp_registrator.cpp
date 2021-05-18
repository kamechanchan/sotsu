#include <bayes_pose/icp_registrator.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

using bayes_pose::ICPRegistrator;

ICPRegistrator::ICPRegistrator(ros::NodeHandle nh, const std::string& compared_pc_topic_name,
                               const std::string& reference_pc_topic_name, const std::string& registrated_pc_topic_name)
  : nh_(nh)
  , is_init_(false)
  , is_ready_(false)
  , registrated_pc_pub_(nh_.advertise<sensor_msgs::PointCloud2>(registrated_pc_topic_name, 1))
  , reference_pc_sub_(nh_.subscribe(reference_pc_topic_name, 1, &ICPRegistrator::referenceCB, this))
  , compared_pc_sub_(nh_.subscribe(compared_pc_topic_name, 1, &ICPRegistrator::registrate, this))
  , tf_listener_(tf_buffer_)
{
  ros::param::param<std::string>("~object_frame_name", reference_frame_name_, "HV6_00");
  ROS_INFO_STREAM("ICPRegistrator ready.");
}

void ICPRegistrator::referenceCB(const sensor_msgs::PointCloud2ConstPtr& reference_pc)
{
  reference_pc_ = *reference_pc;
  source_frame_name_ = reference_pc_.header.frame_id;
  is_init_ = true;
}

void ICPRegistrator::registrate(const sensor_msgs::PointCloud2ConstPtr& compared_pc)
{
  if (is_init_)
  {
    lookupInitTransform();
    pcl::PointCloud<pcl::PointXYZ> compared_pc_pcl, reference_pc_pcl;
    pcl::fromROSMsg(*compared_pc, compared_pc_pcl);
    pcl::fromROSMsg(reference_pc_, reference_pc_pcl);

    std::vector<int> nan_index;
    pcl::removeNaNFromPointCloud(compared_pc_pcl, compared_pc_pcl, nan_index);
    pcl::removeNaNFromPointCloud(reference_pc_pcl, reference_pc_pcl, nan_index);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(1000);

    if (compared_pc_pcl.points.size() == 0 || reference_pc_pcl.points.size() == 0)
    {
      ROS_WARN("No pointcloud.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> result_cloud_pcl;
    Eigen::Vector4f reference_center, compared_center;
    pcl::compute3DCentroid(reference_pc_pcl, reference_center);
    pcl::compute3DCentroid(compared_pc_pcl, compared_center);
    Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(compared_center(0, 0) - reference_center(0, 0),
                                          compared_center(1, 0) - reference_center(1, 0),
                                          compared_center(2, 0) - reference_center(2, 0));
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    ROS_INFO_STREAM("Matching start");
    icp.setInputSource(reference_pc_pcl.makeShared());
    icp.setInputTarget(compared_pc_pcl.makeShared());
    icp.setTransformationEpsilon(1e-13);
    icp.setEuclideanFitnessEpsilon(1e-10);
    icp.align(result_cloud_pcl, init_guess);
    ROS_INFO_STREAM("has converged : " << icp.hasConverged());
    ROS_INFO_STREAM("score : " << icp.getFitnessScore());

    Eigen::Matrix4d final_translation = (icp.getFinalTransformation()).cast<double>();
    Eigen::Affine3d final_affine3d;
    final_affine3d = final_translation;
    output_affine3d_ = final_affine3d * output_affine3d_;

    std::cout << "----ICP's matrix------------------------" << std::endl;
    std::cout << final_translation << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    pcl::toROSMsg(result_cloud_pcl, registrated_pc_);
    is_ready_ = true;
  }
}

void ICPRegistrator::publish()
{
  if (is_ready_)
  {
    registrated_pc_pub_.publish(registrated_pc_);
  }
}

void ICPRegistrator::broadcast_tf()
{
  if (is_ready_)
  {
    geometry_msgs::TransformStamped ts;
    ts = tf2::eigenToTransform(output_affine3d_);
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = source_frame_name_;
    ts.child_frame_id = reference_frame_name_ + "_estimate";
    ROS_DEBUG("[bayes_registrator] broadcast tf");
    br_.sendTransform(ts);
  }
}

void ICPRegistrator::lookupInitTransform()
{
  geometry_msgs::TransformStamped transform;
  geometry_msgs::Pose pose;
  while (true)
  {
    try
    {
      transform = tf_buffer_.lookupTransform(source_frame_name_, reference_frame_name_, ros::Time(0));
      break;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      continue;
    }
  }
  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;
  pose.orientation.x = transform.transform.rotation.x;
  pose.orientation.y = transform.transform.rotation.y;
  pose.orientation.z = transform.transform.rotation.z;
  pose.orientation.w = transform.transform.rotation.w;
  tf2::fromMsg(pose, output_affine3d_);
}
