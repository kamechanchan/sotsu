#ifndef INCLUDE_JACOBI_HANDLER
#define INCLUDE_JACOBI_HANDLER

#include "util.h"

#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace jacobi_handler
{
//------------------　joint_trajectory から　RobotStateを返す関数　--------------------
void getRobotState(const moveit_msgs::RobotTrajectory& m_traj, robot_state::RobotState& state,
                   std::vector<robot_state::RobotState>& vRobotStates)
{
  trajectory_msgs::JointTrajectory m_trajectory = m_traj.joint_trajectory;
  size_t node_count = m_trajectory.points.size();

  for (int i = 0; i < node_count; ++i)
  {
    moveit::core::jointTrajPointToRobotState(m_trajectory, i, state);
    vRobotStates.push_back(state);
  }
}

//------------------  RobotStateからjacobi行列を得ifndefる関数　--------------------
void getJacobian(std::vector<std::vector<double>>& vvjacobian, std::vector<robot_state::RobotState> vstates,
                 const robot_state::JointModelGroup* joint_model_group)
{
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd traj_jacobian;
  std::vector<double> vjacobian(36, 0);

  for (state : vstates)
  {
    state.getJacobian(joint_model_group, state.getLinkModel(joint_model_group->getLinkModelNames().back()),
                      reference_point_position, jacobian);
    Map<Eigen::MatrixXd>(&vjacobian[0], 6, 6) = jacobian;
    vvjacobian.push_back(vjacobian);
  }
}

//------------------  ジョイント角の移動量を出す関数  --------------------

void diffAngle(trajectory_msgs::JointTrajectory& traj, std::vector<double>& vdiff)
{
  for (int i = 0; i < traj.points.size() - 1; i++)
  {
    std::vector<double> vpre_positions;
    std::vector<double> vpost_positions;

    copy(traj.points.at(i).positions.begin(), traj.points.at(i).positions.end(), back_inserter(vpre_positions));
    copy(traj.points.at(i + 1).positions.begin(), traj.points.at(i + 1).positions.end(),
         back_inserter(vpost_positions));
    std::transform(vpre_positions.begin(), vpre_positions.end(), vpost_positions.begin(), std::back_inserter(vdiff),
                   std::minus<double>());
  }
}

//------------------  ジョイント角の移動量Δθとヤコビ行列を与えると１つの軌道のコストを返す関数   --------------------
double calcCost(trajectory_msgs::JointTrajectory& m_traj, std::vector<std::vector<double>>& vvjacobian)
{
  std::vector<double> vdiff;
  double cost = 0;
  Eigen::MatrixXd diff_theta;
  Eigen::MatrixXd jacobi_col_sum;

  trajectory_msgs::JointTrajectory m_trajectory = m_traj;
  diffAngle(m_trajectory, vdiff);
  Eigen::MatrixXd Ediff = Map<Eigen::MatrixXd>(&vdiff[0], vdiff.size() / 6, 6);

  for (int i = 0; i < Ediff.rows(); i++)
  {
    Matrix6d jacobi;
    std::vector<double> vjacobi;

    vjacobi = vvjacobian.at(i);
    jacobi = Map<Matrix6d>(&vjacobi[0], 6, 6);
    jacobi_col_sum = jacobi.cwiseAbs().colwise().sum();
    diff_theta = Ediff.cwiseAbs().row(i).transpose();
    cost += (jacobi_col_sum * diff_theta)(0);
  }

  return cost;
}
} //jacobi_handler

#endif  // INCLUDE_JACOBI_HANDLER
