#ifndef MOVING_BEHAVIOR_H
#define MOVING_BEHAVIOR_H

#include <denso_state_srvs/Moving.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>

typedef std::pair<moveit_msgs::RobotTrajectory, double> Traj_Cost;

namespace moving_behavior
{
class MovingBehavior
{
public:
  MovingBehavior(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  bool moving(denso_state_srvs::Moving::Request& req, denso_state_srvs::Moving::Response& res);
  bool opt_moving(denso_state_srvs::Moving::Request& req, denso_state_srvs::Moving::Response& res);

private:
  moveit::planning_interface::MoveGroupInterface::Plan
  scaling_exec_speed(const moveit::planning_interface::MoveGroupInterface::Plan initial_plan);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer service_;
  std::vector<std::string> joint_names_;
  std::map<std::string, double> joint_values_;
  double exe_speed_rate_;
  moveit::planning_interface::MoveGroupInterface group_;
  moveit::planning_interface::MoveGroupInterface::Plan initial_plan_;
  moveit::planning_interface::MoveGroupInterface::Plan new_plan_;
  moveit_msgs::RobotTrajectory mrobot_traj_;
  std::vector<Traj_Cost> vmtraj_dataset_;
  robot_state::RobotState state_;
  const robot_state::JointModelGroup* joint_model_group_;
};
}  // namespace moving_behavior

#endif  // MOVING_BEHAVIOR_H
