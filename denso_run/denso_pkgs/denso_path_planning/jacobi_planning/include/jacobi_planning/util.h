#ifndef UTIL_H
#define UTIL_H

#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <algorithm>
#include <iterator>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <time.h>
#include <signal.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/conversions.h>

using namespace std;
using namespace Eigen;

#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl
#define PRINT_MAT2(X, DESC) cout << DESC << ":\n" << X << endl << endl
#define PRINT_FNC cout << "[" << __func__ << "]" << endl

typedef pair<moveit_msgs::RobotTrajectory, double> traj_cost;



namespace utils
{
bool compareBySecond(const traj_cost& a, const traj_cost& b)
{
  if (a.second != b.second)
  {
    return a.second < b.second;
  }
  else
  {
    return false;
  }
}

bool writeTraj2file(const std::string& file_name, const moveit::planning_interface::MoveGroupInterface::Plan plan)
{
  ofstream output_handler;
  output_handler.open(file_name.c_str());
  moveit_msgs::RobotTrajectory m_trajectory;
  m_trajectory = plan.trajectory_;
  vector<std::string> V_joint_name = m_trajectory.joint_trajectory.joint_names;
  vector<trajectory_msgs::JointTrajectoryPoint> V_node = m_trajectory.joint_trajectory.points;
  int num_joints = V_joint_name.size();

  output_handler << "time_from_start.";
  for (size_t i = 0; i < num_joints; ++i)
  {
    output_handler << "j" << std::to_string(i) << "_pos,";
    output_handler << "j" << std::to_string(i) << "_vel,";
    output_handler << "j" << std::to_string(i) << "_acc,";
  }
  output_handler << std::endl;

  for (size_t i = 0; i < V_node.size(); ++i)
  {
    output_handler.precision(10);
    output_handler << V_node[i].time_from_start << ", ";
    for (size_t j = 0; j < num_joints; ++j)
    {
      output_handler << V_node[i].positions[j] << ", ";
      output_handler << V_node[i].velocities[j] << ", ";
      output_handler << V_node[i].accelerations[j] << ", ";
    }
    output_handler << std::endl;
  }
  output_handler.close();

  return true;
}

moveit_msgs::RobotTrajectory readTraj(const std::string& file_name, int joint_num)
{
  std::ifstream input_file;
  std::string line;
  std::string cell;

  bool first_row_read_file = true;
  double first_timestamp;

  input_file.open(file_name.c_str());
  std::getline(input_file, line);
  moveit_msgs::RobotTrajectory m_traj;
  vector<std::string> V_joints;
  vector<trajectory_msgs::JointTrajectoryPoint> V_node;

  int i = 0;
  while (std::getline(input_file, line))
  {
    std::stringstream lineStream(line);
    trajectory_msgs::JointTrajectoryPoint m_point;
    if (!std::getline(lineStream, cell, ','))
    {
      ROS_ERROR_STREAM_NAMED("csv_to_controller", "no time value");
    }

    double timestamp = atof(cell.c_str());
    m_point.time_from_start = ros::Duration(timestamp);

    if (first_row_read_file)
    {
      first_timestamp = timestamp;
      first_row_read_file = false;
    }

    for (int joint_id = 0; joint_id < joint_num; ++joint_id)
    {
      if (!std::getline(lineStream, cell, ','))
      {
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
      }
      m_point.positions.emplace_back(atof(cell.c_str()));

      if (!std::getline(lineStream, cell, ','))
      {
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
      }
      m_point.velocities.emplace_back(atof(cell.c_str()));

      if (!std::getline(lineStream, cell, ','))
      {
        ROS_ERROR_STREAM_NAMED("csv_to_controller", "no joint value");
      }
      m_point.accelerations.emplace_back(atof(cell.c_str()));
    }
    ++i;
    V_node.emplace_back(m_point);
  }

  m_traj.joint_trajectory.points = V_node;
  return m_traj;
}

void SigintHandler(int sig)
{
  ros::shutdown();
}

shape_msgs::Mesh load_mesh(std::string path)
{
    Vector3d b(0.001, 0.001, 0.001);
    shapes::Mesh* m = shapes::createMeshFromResource(path, b);
    ROS_INFO("Mesh Loaded");

    shape_msgs::Mesh mesh;

    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    return mesh;
}

void AddColObjToScene(moveit::planning_interface::PlanningSceneInterface& sc, geometry_msgs::Pose& msg)
{
       ROS_INFO("Adding Target object to planning scene at %f, %f", msg.position.x, msg.position.y);
       // Add a object to the scene to represent the object to be picked
       moveit_msgs::CollisionObject co;
       shape_msgs::Mesh mesh;

       mesh = load_mesh("package://jacobi_planning/config/scene/work1.stl");

       co.id = "target_object";

       co.meshes.resize(1);
       co.mesh_poses.resize(1);
       co.meshes[0] = mesh;

       co.header.frame_id = "J6";

       co.mesh_poses[0].position.x = msg.position.x;
       co.mesh_poses[0].position.y = msg.position.y;
       co.mesh_poses[0].position.z = msg.position.z;
       co.mesh_poses[0].orientation.x = 0;
       co.mesh_poses[0].orientation.y = 0;
       co.mesh_poses[0].orientation.z = 0;
       co.mesh_poses[0].orientation.w = 1;

       co.meshes.push_back(mesh);
       co.mesh_poses.push_back(co.mesh_poses[0]);
       co.operation = co.ADD;

       sc.applyCollisionObject(co);

       // Sleep a little to let the messages flow and be processed
       ros::Duration(1).sleep();
}

moveit_msgs::AttachedCollisionObject AddTargetToScene(moveit::planning_interface::PlanningSceneInterface& sc, geometry_msgs::Pose& msg)
{
       ROS_INFO("Adding Collision Object to planning scene at %f, %f", msg.position.x, msg.position.y);
       // Add a object to the scene to represent the object to be picked
       shape_msgs::Mesh mesh;

       mesh = load_mesh("package://jacobi_planning/config/scene/work1.stl");

       moveit_msgs::CollisionObject co;
       moveit_msgs::AttachedCollisionObject aco;

       co.id = "target_object";

       co.mesh_poses.resize(1);
       co.meshes.resize(1);
       co.meshes[0] = mesh;

       co.header.frame_id = "J6";

       co.mesh_poses[0].position.x = msg.position.x;
       co.mesh_poses[0].position.y = msg.position.y;
       co.mesh_poses[0].position.z = msg.position.z;
       co.mesh_poses[0].orientation.x = msg.orientation.x;
       co.mesh_poses[0].orientation.y = msg.orientation.y;
       co.mesh_poses[0].orientation.z = msg.orientation.z;
       co.mesh_poses[0].orientation.w = msg.orientation.w;

       co.meshes.push_back(mesh);
       co.mesh_poses.push_back(co.mesh_poses[0]);
       co.operation = co.ADD;

       aco.object = co;
       aco.link_name = co.id;
       aco.object.operation = moveit_msgs::CollisionObject::ADD;

       sc.applyCollisionObject(co);
       sc.applyAttachedCollisionObject(aco);
       // Sleep a little to let the messages flow and be processed
       ros::Duration(1).sleep();
       return aco;
}

void RemoveFromScene(moveit::planning_interface::PlanningSceneInterface &sc, std::string object_id)
{
    ROS_INFO("Removing Object from planning scene");
    // Remove the box from the scene
    std::vector<std::string> objs;
    objs.push_back(object_id);
    sc.removeCollisionObjects(objs);
}



bool eef_limit_check(const int eef_joint_index, const double lower_limit, const double upper_limit,
                     const trajectory_msgs::JointTrajectory traj)
{
  for (int i = 0; i < traj.points.size(); i++)
  {
    if ((traj.points[i].positions[eef_joint_index] < lower_limit) ||
        (traj.points[i].positions[eef_joint_index] > upper_limit))
    {
      int dof = 6;
      for (int j = 0; j < dof; j++)
      {
        std::cout << "limit " << j << ": " << traj.points[i].positions[j] << std::endl;
      }
      return false;
    }
  }
  return true;
}




}

#endif //UTIL_H
