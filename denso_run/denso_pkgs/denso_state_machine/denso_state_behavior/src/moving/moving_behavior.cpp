#include <denso_state_behavior/moving/moving_behavior.h>
#include <denso_state_behavior/moving/jacobi_handler.h>
#include <denso_state_behavior/moving/util.h>

#include <map>

typedef std::pair<moveit_msgs::RobotTrajectory, double> Traj_Cost;

using moving_behavior::MovingBehavior;
using namespace jacobi_handler;
using namespace utils;
// Class methods definitions
MovingBehavior::MovingBehavior(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , joint_names_({ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" })
  , group_("arm")
  , state_(group_.getRobotModel())
{
  group_.setPlannerId("ompl/RRTConnectkConfigDefault");
  joint_model_group_ = group_.getCurrentState()->getJointModelGroup("arm");
  private_nh_.param<double>("exe_speed_rate", exe_speed_rate_, 1.0);
  service_ = nh_.advertiseService("/state_behavior/moving", &MovingBehavior::opt_moving, this);
}

moveit::planning_interface::MoveGroupInterface::Plan
MovingBehavior::scaling_exec_speed(const moveit::planning_interface::MoveGroupInterface::Plan initial_plan)
{
  moveit_msgs::RobotTrajectory initial_trajectory;
  moveit_msgs::RobotTrajectory new_trajectory;

  initial_trajectory = initial_plan.trajectory_;
  new_trajectory = initial_trajectory;

  int n_joints = initial_trajectory.joint_trajectory.joint_names.size();
  int n_points = initial_trajectory.joint_trajectory.points.size();

  for (int i = 1; i < n_points; i++)
  {
    ros::Duration start_time(initial_trajectory.joint_trajectory.points[i].time_from_start.toSec() / exe_speed_rate_);
    new_trajectory.joint_trajectory.points[i].time_from_start = start_time;

    for (int j = 0; j < n_joints; j++)
    {
      new_trajectory.joint_trajectory.points[i].velocities[j] =
          initial_trajectory.joint_trajectory.points[i].velocities[j] * exe_speed_rate_;
      new_trajectory.joint_trajectory.points[i].accelerations[j] =
          initial_trajectory.joint_trajectory.points[i].accelerations[j] * exe_speed_rate_ * exe_speed_rate_;
      new_trajectory.joint_trajectory.points[i].positions[j] =
          initial_trajectory.joint_trajectory.points[i].positions[j];
    }
  }

  moveit::planning_interface::MoveGroupInterface::Plan new_plan;
  new_plan = initial_plan;
  new_plan.trajectory_ = new_trajectory;
  return new_plan;
}

bool MovingBehavior::moving(denso_state_srvs::Moving::Request& req, denso_state_srvs::Moving::Response& res)
{
  group_.setStartState(*group_.getCurrentState());

  robot_state::RobotState robot_state_goal(*group_.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = group_.getCurrentState()->getJointModelGroup("arm");

  bool ik = robot_state_goal.setFromIK(joint_model_group, req.target_pose.pose, 4, 1);

  if (!ik)
  {
    res.success = false;
    return true;
  }

  for (auto joint_name : joint_names_)
  {
    joint_values_[joint_name] = robot_state_goal.getVariablePosition(joint_name);
  }
  group_.setJointValueTarget(joint_values_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  group_.plan(plan);

  moveit::planning_interface::MoveGroupInterface::Plan scaled_plan = this->scaling_exec_speed(plan);

  group_.execute(scaled_plan);
  res.success = true;
  return true;
}

bool MovingBehavior::opt_moving(denso_state_srvs::Moving::Request& req, denso_state_srvs::Moving::Response& res)
{
  unsigned int num_generate_traj = 50;
  if(req.num_generate_traj != 0)
  {
    num_generate_traj = req.num_generate_traj;
  }

  group_.setStartState(*group_.getCurrentState());

  robot_state::RobotState robot_state_goal(*group_.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = group_.getCurrentState()->getJointModelGroup("arm");

  group_.setPlannerId("ompl/RRTConnectkConfigDefault");

  std::cout << "joint_names: " << req.joint_names.size() << std::endl;
  std::cout << "joint_values: " << req.target_joint_values.size() << std::endl;
  if (req.target_joint_values.size() != 0)
  {
    if (req.joint_names.size() != req.target_joint_values.size())
    {
      ROS_ERROR("Unmatch size between joint_names and target_joint_values");
      res.success = false;
      return false;
    }
    for (int i=0; i < req.joint_names.size(); i++)
    {
      joint_values_[req.joint_names[i]] = req.target_joint_values[i];
    }
    group_.setJointValueTarget(joint_values_);
  }
  else
  {
    bool ik = robot_state_goal.setFromIK(joint_model_group, req.target_pose.pose, 4, 1);

    if (!ik)
    {
      res.success = false;
      return true;
    }

    for (auto joint_name : joint_names_)
    {
      joint_values_[joint_name] = robot_state_goal.getVariablePosition(joint_name);
    }
    group_.setJointValueTarget(joint_values_);
  }

  ros::WallTime wall_begin = ros::WallTime::now();
  int data_num = 0;

  std::vector<Traj_Cost> vmtraj_dataset;
  moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
  moveit::planning_interface::MoveGroupInterface::Plan new_plan;

  // generate_traj and pick up opt pa
  while (data_num < num_generate_traj && ros::ok())
  {
    bool success = (group_.plan(initial_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    bool in_range = eef_limit_check(5, -1.5708, 4.71239, initial_plan.trajectory_.joint_trajectory);
    std::cout << in_range << std::endl;
    if (success && in_range)
    {
      data_num += 1;
      double cost = 0;
      std::vector<robot_state::RobotState> V_Robot_State;
      std::vector<std::vector<double> > VV_jacobian;

      get_robot_state(initial_plan.trajectory_, state_, V_Robot_State);
      get_jacobian(VV_jacobian, V_Robot_State, joint_model_group_);
      cost = calc_cost(initial_plan.trajectory_.joint_trajectory, VV_jacobian);
      vmtraj_dataset.push_back(make_pair(initial_plan.trajectory_, cost));

      ros::WallDuration wall_duration = ros::WallTime::now() - wall_begin;
    }
    else
    {
      std::cout << "Failer: "
                << "%d" << data_num << "\n"
                << std::endl;
      std::cout << "Retry to generate path\n" << std::endl;
    }
  }

  sort(vmtraj_dataset.begin(), vmtraj_dataset.end(), compare_by_second);
  for (auto x : vmtraj_dataset)
    std::cout << "Cost is:" << x.second << std::endl;

  moveit_msgs::RobotTrajectory mrobot_traj;
  mrobot_traj = vmtraj_dataset[0].first;

  new_plan.trajectory_ = mrobot_traj;

  group_.setPlannerId("stomp/STOMPPlanner");

  size_t num_of_points = mrobot_traj.joint_trajectory.points.size();

  moveit_msgs::TrajectoryConstraints initial_traj;
  std::vector<moveit_msgs::Constraints> all_joint_constrains;

  std::string joint_names[6] = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  for (int j = 0; j < num_of_points; j++)
  {
    std::vector<moveit_msgs::JointConstraint> joint_constraints_at_t;
    for (int k = 0; k < 6; k++)
    {
      moveit_msgs::JointConstraint joint_con;
      joint_con.joint_name = joint_names[k];
      joint_con.position = mrobot_traj.joint_trajectory.points[j].positions[k];
      joint_con.weight = 1;
      joint_constraints_at_t.push_back(joint_con);
    }
    moveit_msgs::Constraints con;
    con.joint_constraints = joint_constraints_at_t;
    all_joint_constrains.push_back(con);
  }

  initial_traj.constraints = all_joint_constrains;

  robot_state::RobotState robot_state_start(*group_.getCurrentState());
  group_.setStartState(robot_state_start);
  group_.setTrajectoryConstraints(initial_traj);
  group_.setJointValueTarget(joint_values_);

  bool success2 = (group_.plan(new_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success2)
  {
    ROS_WARN("Successs to make optimized path!");
  }
  else
  {
    ROS_WARN("Could not make optimized path");
  }

  moveit::planning_interface::MoveGroupInterface::Plan scaled_plan = this->scaling_exec_speed(new_plan);

  group_.execute(scaled_plan);
  res.success = true;
  return true;
}
