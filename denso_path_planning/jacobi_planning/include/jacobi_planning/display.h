#ifndef DISPLAY_H
#define DISPLAY_H

#include <jacobi_planning/util.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;
namespace moveit_visual_tools
{
class Visualizer
{
public:
  explicit Visualizer()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_tools"));
    visual_tools_->loadPlanningSceneMonitor();
    visual_tools_->loadMarkerPub(true);
    visual_tools_->loadRobotStatePub("display_robot_state");
    visual_tools_->setManualSceneUpdating();

    robot_state_ = visual_tools_->getSharedRobotState();

    deleteAllMarkers();
    visual_tools_->loadRemoteControl();
    text_pose1_ = Affine3d::Identity();
    text_pose2_ = Affine3d::Identity();
    text_pose1_.translation().z() = 1.5;
    text_pose2_.translation().z() = 1.4;
    visual_tools_->publishText(text_pose1_, " Generate Path", rvt::WHITE, rvt::XLARGE, false);
    visual_tools_->trigger();
    deleteAllMarkers();
  }

  ~Visualizer()
  {
  }

  void displayPosition(geometry_msgs::Pose pose, std::string& message)
  {
    visual_tools_->publishAxisLabeled(pose, message);
  }

  void buttonController()
  {
    visual_tools_->loadRemoteControl();
    visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  }

  void displayText(std::string& message)
  {
    visual_tools_->publishText(text_pose1_, message, rvt::WHITE, rvt::XLARGE);
  }

  void displayTime(double duration, int num_traj)
  {
    visual_tools_->publishText(text_pose2_, "Time:" + to_string(duration) + " " + "Total Path:" + to_string(num_traj),
                               rvt::GREEN, rvt::XLARGE);
  }

  void displayTrajectory(vector<moveit_msgs::RobotTrajectory>& V_traj,
                         const robot_state::JointModelGroup* joint_model_group, int display_num)
  {
    for (size_t i = 0; i < display_num; i++)
    {
      visual_tools_->publishTrajectoryLine(V_traj[i], joint_model_group, rvt::PINK);
    }
  }

  void displayTrajectory(moveit_msgs::RobotTrajectory& V_traj, const robot_state::JointModelGroup* joint_model_group)
  {
    visual_tools_->publishTrajectoryLine(V_traj, joint_model_group, rvt::RED);
  }
  void displayTrajectory_opt(moveit_msgs::RobotTrajectory& V_traj,
                             const robot_state::JointModelGroup* joint_model_group)
  {
    visual_tools_->publishTrajectoryLine(V_traj, joint_model_group, rvt::GREEN);
  }

  void runRobotStateTests(moveit::core::RobotStatePtr robot_state_,
                          const robot_state::JointModelGroup* joint_model_group)
  {
    for (std::size_t i = 0; i < 5; ++i)
    {
      robot_state_->setToRandomPositions(joint_model_group);
      visual_tools_->publishRobotState(robot_state_, rvt::DEFAULT);
      ros::Duration(0.1).sleep();
    }

    for (std::size_t i = 0; i < 5; ++i)
    {
      robot_state_->setToRandomPositions(joint_model_group);
      visual_tools_->publishRobotState(robot_state_, visual_tools_->getRandColor());
      ros::Duration(0.1).sleep();
    }

    // Hide the robot
    visual_tools_->hideRobot();
    ros::Duration(0.1).sleep();

    // Show the robot
    visual_tools_->publishRobotState(robot_state_, rvt::DEFAULT);
    ros::Duration(0.1).sleep();
  }

  void trigger()
  {
    visual_tools_->trigger();
  }

  void deleteAllMarkers()
  {
    visual_tools_->deleteAllMarkers();
  }

private:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  Affine3d text_pose1_;
  Affine3d text_pose2_;
  moveit::core::RobotStatePtr robot_state_;
};

}  // moveit_visual_tools

#endif  // DISPLAY_H
