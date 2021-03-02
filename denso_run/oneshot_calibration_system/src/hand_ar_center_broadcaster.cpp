#include <oneshot_calibration_system/ar_center_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Eigenvalues>

using ar_center_broadcaster::ARCenterBroadcaster;
using ar_center_broadcaster::BroadcasterParam;

const int BroadcasterParam::MATRIX_SIZE = 4;
const int BroadcasterParam::START_TIME = 0;
const float BroadcasterParam::DURATION_TIME = 1.0;

// Class methods definitions
ARCenterBroadcaster::ARCenterBroadcaster(ros::NodeHandle& nh) : nh_(nh), tfListener_(tfBuffer_)
{
  ar_transform_vector_.clear();
}

void ARCenterBroadcaster::broadcast(void)
{
  updateARCenterTransform();
  ar_center_transform_.header.stamp = ros::Time::now();
  br_.sendTransform(ar_center_transform_);
}

void ARCenterBroadcaster::updateARCenterTransform(void)
{
  updateARTransformVector();

  geometry_msgs::Vector3 position;
  geometry_msgs::Quaternion center_quat;
  int vector_size = ar_transform_vector_.size();

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(BroadcasterParam::MATRIX_SIZE, BroadcasterParam::MATRIX_SIZE);

  for (const auto& ar_transform : ar_transform_vector_)
  {
    // calculate center position
    position.x += ar_transform.transform.translation.x / vector_size;
    position.y += ar_transform.transform.translation.y / vector_size;
    position.z += ar_transform.transform.translation.z / vector_size;

    // calculate center orientation
    tf::Quaternion quat(ar_transform.transform.rotation.x, ar_transform.transform.rotation.y,
                        ar_transform.transform.rotation.z, ar_transform.transform.rotation.w);
    Eigen::Quaterniond q;
    tf::quaternionTFToEigen(quat, q);
    q.normalize();
    Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
    Eigen::Matrix4d q_mat = q_vec * q_vec.transpose();
    // std::cout << "q_vec" << std::endl;
    // std::cout << q_vec << std::endl;
    M += q_mat;
  }

  // std::cout << "M" << std::endl;
  // std::cout << M << std::endl;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(M);
  // std::cout << "solver.eigenvectors()" << std::endl;
  // std::cout << solver.eigenvectors() << std::endl;
  Eigen::VectorXd max_eigenvector = solver.eigenvectors().col(solver.eigenvectors().row(0).size() - 1);
  Eigen::Quaterniond q_ave(max_eigenvector(0), max_eigenvector(1), max_eigenvector(2), max_eigenvector(3));
  tf::Quaternion quat_ave;
  tf::quaternionEigenToTF(q_ave, quat_ave);
  quaternionTFToMsg(quat_ave, center_quat);
  // std::cout << "center_quat" << std::endl;
  // std::cout << center_quat << std::endl;

  std::string frame_id, child_frame_id;
  if (nh_.getParam("/ar_oneshot_calibrator/sensor_frame_id", frame_id))
  {
    if (nh_.getParam("/ar_oneshot_calibrator/targetA_frame_id", child_frame_id))
    {
      ar_center_transform_.header.frame_id = frame_id;
      ar_center_transform_.child_frame_id = child_frame_id;
      ar_center_transform_.transform.translation.x = position.x;
      ar_center_transform_.transform.translation.y = position.y;
      ar_center_transform_.transform.translation.z = position.z;
      ar_center_transform_.transform.rotation.x = center_quat.x;
      ar_center_transform_.transform.rotation.y = center_quat.y;
      ar_center_transform_.transform.rotation.z = center_quat.z;
      ar_center_transform_.transform.rotation.w = center_quat.w;
    }
  }

  ar_transform_vector_.clear();
}

bool ARCenterBroadcaster::getTransformInfoFromYAML(std::string marker, geometry_msgs::TransformStamped& transform)
{
  std::string transform_arg;

  transform_arg = "/ar_oneshot_calibrator/sensor_frame_id";
  if (!nh_.getParam(transform_arg, transform.header.frame_id))
  {
    return false;
  }
  transform_arg = "/ar_center_broadcaster/marker_frame" + marker + "/child_frame_id";
  if (!nh_.getParam(transform_arg, transform.child_frame_id))
  {
    return false;
  }

  return true;
}

bool ARCenterBroadcaster::lookupTransform(geometry_msgs::TransformStamped& transform, const ros::Time& time)
{
  bool tf_flag = true;
  int i = 0;

  while (tf_flag)
  {
    if (i > 30)  // 30 times * Duretion times
    {
      return false;
    }

    try
    {
      transform =
          tfBuffer_.lookupTransform(transform.header.frame_id, transform.child_frame_id, time, ros::Duration(BroadcasterParam::DURATION_TIME));
      tf_flag = false;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s Retry...", ex.what());
      tf_flag = true;
      continue;
      i++;
    }
  }

  return true;
}

void ARCenterBroadcaster::updateARTransformVector(void)
{
  ros::Time time = ros::Time(BroadcasterParam::START_TIME);

  for (int i = 0;; i++)
  {
    geometry_msgs::TransformStamped transform;

    std::string marker = "/marker_" + std::to_string(i);

    if (getTransformInfoFromYAML(marker, transform))
    {
      if (!lookupTransform(transform, time))
      {
        time = ros::Time(BroadcasterParam::START_TIME);
        i = -1;  // retry lookupTransform from marker_0
        ar_transform_vector_.clear();
      }
    }
    else
    {
      ROS_INFO_STREAM_ONCE("Found " << std::to_string(i) << "AR marker parameter.");
      break;
    }
    ar_transform_vector_.push_back(transform);
  }
}
