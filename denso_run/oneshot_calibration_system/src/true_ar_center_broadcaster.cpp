#include <oneshot_calibration_system/true_ar_center_broadcaster.h>

using true_ar_center_broadcaster::TrueARCenterBroadcaster;
using true_ar_center_broadcaster::BroadcasterParam;

const int BroadcasterParam::MATRIX_SIZE = 4;

// Class methods definitions
TrueARCenterBroadcaster::TrueARCenterBroadcaster(ros::NodeHandle& nh) : nh_(nh)
{
  true_ar_transform_vector_.clear();
}

void TrueARCenterBroadcaster::publish(void)
{
  updateCenterTransform();
  true_ar_center_transform_.header.stamp = ros::Time::now();
  br_.sendTransform(true_ar_center_transform_);
}

bool TrueARCenterBroadcaster::getTransformInfoFromYAML(std::string marker, geometry_msgs::TransformStamped& transform)
{
  std::string transform_arg;

  transform_arg = "/ar_oneshot_calibrator/origin_frame_id";
  if (!nh_.getParam(transform_arg, transform.header.frame_id))
  {
    return false;
  }
  transform_arg = "/true_ar_center_broadcaster/marker_frame" + marker + "/child_frame_id";
  if (!nh_.getParam(transform_arg, transform.child_frame_id))
  {
    return false;
  }

  return true;
}

void TrueARCenterBroadcaster::getTransformFromYAML(std::string marker, geometry_msgs::TransformStamped& transform)
{
  std::string transform_arg;

  transform_arg = "/true_ar_center_broadcaster/marker_frame" + marker + "/position_x";
  if (!nh_.getParam(transform_arg, transform.transform.translation.x))
  {
    ROS_INFO_STREAM("Acquisition Failure : " + transform_arg);
  }
  transform_arg = "/true_ar_center_broadcaster/marker_frame" + marker + "/position_y";
  if (!nh_.getParam(transform_arg, transform.transform.translation.y))
  {
    ROS_INFO_STREAM("Acquisition Failure : " + transform_arg);
  }
  transform_arg = "/true_ar_center_broadcaster/marker_frame" + marker + "/position_z";
  if (!nh_.getParam(transform_arg, transform.transform.translation.z))
  {
    ROS_INFO_STREAM("Acquisition Failure : " + transform_arg);
  }
  transform_arg = "/true_ar_center_broadcaster/marker_frame" + marker + "/orientation_x";
  if (!nh_.getParam(transform_arg, transform.transform.rotation.x))
  {
    ROS_INFO_STREAM("Acquisition Failure : " + transform_arg);
  }
  transform_arg = "/true_ar_center_broadcaster/marker_frame" + marker + "/orientation_y";
  if (!nh_.getParam(transform_arg, transform.transform.rotation.y))
  {
    ROS_INFO_STREAM("Acquisition Failure : " + transform_arg);
  }
  transform_arg = "/true_ar_center_broadcaster/marker_frame" + marker + "/orientation_z";
  if (!nh_.getParam(transform_arg, transform.transform.rotation.z))
  {
    ROS_INFO_STREAM("Acquisition Failure : " + transform_arg);
  }
  transform_arg = "/true_ar_center_broadcaster/marker_frame" + marker + "/orientation_w";
  if (!nh_.getParam(transform_arg, transform.transform.rotation.w))
  {
    ROS_INFO_STREAM("Acquisition Failure : " + transform_arg);
  }
}

void TrueARCenterBroadcaster::updateTrueARTransformArray(void)
{
  for (int i = 0;; i++)
  {
    geometry_msgs::TransformStamped transform;

    std::string marker = "/marker_" + std::to_string(i);
    if (getTransformInfoFromYAML(marker, transform))
    {
      getTransformFromYAML(marker, transform);
    }
    else
    {
      ROS_INFO_STREAM_ONCE("Found " << std::to_string(i) << "true AR marker parameter.");
      break;
    }
    true_ar_transform_vector_.push_back(transform);
  }
}

void TrueARCenterBroadcaster::updateCenterTransform(void)
{
  updateTrueARTransformArray();

  geometry_msgs::Vector3 position;
  geometry_msgs::Quaternion center_quat;
  int vector_size = true_ar_transform_vector_.size();

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(BroadcasterParam::MATRIX_SIZE, BroadcasterParam::MATRIX_SIZE);


  for (const auto& true_ar_transform : true_ar_transform_vector_)
  {
    // calculate center position
    position.x += true_ar_transform.transform.translation.x / vector_size;
    position.y += true_ar_transform.transform.translation.y / vector_size;
    position.z += true_ar_transform.transform.translation.z / vector_size;

    // calculate center orientation
    tf::Quaternion quat(true_ar_transform.transform.rotation.x, true_ar_transform.transform.rotation.y,
                        true_ar_transform.transform.rotation.z, true_ar_transform.transform.rotation.w);
    Eigen::Quaterniond q;
    tf::quaternionTFToEigen(quat, q);
    q.normalize();
    Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
    Eigen::Matrix4d q_mat = q_vec * q_vec.transpose();
    M += q_mat;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(M);
  Eigen::VectorXd max_eigenvector = solver.eigenvectors().col(solver.eigenvectors().row(0).size() - 1);
  Eigen::Quaterniond q_ave(max_eigenvector(0), max_eigenvector(1), max_eigenvector(2), max_eigenvector(3));
  tf::Quaternion quat_ave;
  tf::quaternionEigenToTF(q_ave, quat_ave);
  quaternionTFToMsg(quat_ave, center_quat);

  std::string frame_id, child_frame_id;
  if (nh_.getParam("/ar_oneshot_calibrator/origin_frame_id", frame_id))
  {
    if (nh_.getParam("/ar_oneshot_calibrator/targetB_frame_id", child_frame_id))
    {
      true_ar_center_transform_.header.frame_id = frame_id;
      true_ar_center_transform_.child_frame_id = child_frame_id;
      true_ar_center_transform_.transform.translation.x = position.x;
      true_ar_center_transform_.transform.translation.y = position.y;
      true_ar_center_transform_.transform.translation.z = position.z;
      true_ar_center_transform_.transform.rotation.x = center_quat.x;
      true_ar_center_transform_.transform.rotation.y = center_quat.y;
      true_ar_center_transform_.transform.rotation.z = center_quat.z;
      true_ar_center_transform_.transform.rotation.w = center_quat.w;
    }
  }

  true_ar_transform_vector_.clear();
}
