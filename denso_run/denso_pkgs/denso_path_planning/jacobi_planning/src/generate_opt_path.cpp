#include <jacobi_planning/jacobi_handler.h>
#include <jacobi_planning/util.h>
#include <jacobi_planning/display.h>
#include <sys/types.h>
#include <map>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

using namespace utils;
using namespace jacobi_handler;
using namespace moveit_visual_tools;
typedef pair<moveit_msgs::RobotTrajectory, double> P;
typedef pair<string, double> PS;

static const std::string PLANNING_GROUP_NAME = "arm";

class OptimizationPath
{
  public:
    explicit OptimizationPath(ros::NodeHandle& nh)
      : nh_{ nh }, move_group_{ PLANNING_GROUP_NAME }, state_{move_group_.getRobotModel()}
    {
      ros::param::param<int>("~batch_size", batch_, 100);
      ros::param::param<std::string>("~target_value", target_value_, "default");
      ros::param::param<bool>("~attached_object", attached_object_, false);
      ros::param::param<std::string>("~robot_name", robot_name_);
      ros::param::param<std::vector<std::string>>("/" + robot_name_ + "/arm_controller/joints", joint_names_);

      client_get_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

      move_group_.setPlannerId("ompl/RRTConnectkConfigDefault");
      move_group_.setJointValueTarget(move_group_.getNamedTargetValues(target_value_));

      if (attached_object_)
      {
        attachObject(target_value_);
        updateState();
      }
      else
      {

        updateState();
      }
   }

    ~OptimizationPath()
    {
      if (attached_object_)
      {
        RemoveFromScene(current_scene_, "target_object");
      }
    }

    void attachObject(std::string target_state)
    {
      geometry_msgs::Pose pose;
      std::string ef_name;
      ef_name = "J6";

      if (target_state == "default")
      {
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0.18;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0.706;
        pose.orientation.w = 0.7073;

        AddTargetToScene(current_scene_, pose);
      }
      else if (target_state == "default_sim")
      {
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0.2;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;

        AddTargetToScene(current_scene_, pose);
      }

      move_group_.attachObject("target_object", ef_name, std::vector<std::string>{ "finger_L", "finger_R"});
    }

    void updateState()
    {
      scene_srv_.request.components.components = scene_srv_.request.components.ROBOT_STATE_ATTACHED_OBJECTS;
      if (!client_get_scene_.call(scene_srv_))
      {
        ROS_WARN("Failed to call service /get_planning_scene");
      }
      else
      {
        ROS_INFO_STREAM("Number of attached bodies according to /get_planning_scene: " << scene_srv_.response.scene.robot_state.attached_collision_objects.size());

        moveit::core::robotStateMsgToRobotState(scene_srv_.response.scene.robot_state, state_);
        robot_state::RobotState RS = state_;
        joint_model_group_ = state_.getRobotModel()->getJointModelGroup(PLANNING_GROUP_NAME);
        std::vector<const robot_state::AttachedBody*> bodies;
        RS.getAttachedBodies(bodies);
        ROS_INFO_STREAM("Number of attached bodies according to MoveGroup interface: " << bodies.size());
      }
    }

    void generateTraj()
    {
      std::string message = "Generate Trajectory";
      vis_.displayText(message);
      vis_.trigger();

      ros::WallTime wall_begin = ros::WallTime::now();
      int data_num = 0;
      while (data_num < batch_ && ros::ok())
      {
        move_group_.setPlanningTime(1);
        bool success = (move_group_.plan(initial_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        bool in_range = eef_limit_check(5, 0, 3.1415, initial_plan_.trajectory_.joint_trajectory);

        if (success && in_range)
        {
          data_num += 1;
          double cost = 0;
          vector<robot_state::RobotState> vRobot_State;
          vector<vector<double> > vvjacobian;

          getRobotState(initial_plan_.trajectory_, state_, vRobot_State);
          getJacobian(vvjacobian, vRobot_State, joint_model_group_);
          cost = calcCost(initial_plan_.trajectory_.joint_trajectory, vvjacobian);
          vmtraj_dataset_.push_back(make_pair(initial_plan_.trajectory_, cost));

          vis_.displayTrajectory(initial_plan_.trajectory_, joint_model_group_);
          ros::WallDuration wall_duration = ros::WallTime::now() - wall_begin;
          vis_.displayTime(wall_duration.toSec(), data_num);
        }
        else
        {
          cout << "Failer: "
            << "%d" << data_num << "\n"
            << endl;
          cout << "Retry to generate path\n" << endl;
        }
        vis_.trigger();
      }


      vis_.deleteAllMarkers();

      sort(vmtraj_dataset_.begin(), vmtraj_dataset_.end(), compareBySecond);
      for (x : vmtraj_dataset_)
        cout << "Cost is:" << x.second << endl;
      mrobot_traj_ = vmtraj_dataset_[0].first;

      new_plan_.trajectory_ = mrobot_traj_;
//      vis_.displayTrajectory(mrobot_traj_, joint_model_group_);
//
//      move_group_.setPlannerId("stomp/STOMPPlanner");
//
//      size_t num_of_points = mrobot_traj_.joint_trajectory.points.size();
//
//      moveit_msgs::TrajectoryConstraints initial_traj;
//      std::vector<moveit_msgs::Constraints> all_joint_constrains;

//      int joint_num = joint_names_.size();
//
//      for (int j = 0; j < num_of_points; j++)
//      {
//        std::vector<moveit_msgs::JointConstraint> joint_constraints_at_t;
//        for (int k = 0; k < joint_num; k++)
//        {
//          moveit_msgs::JointConstraint joint_con;
//          joint_con.joint_name = joint_names_[k];
//          joint_con.position = mrobot_traj_.joint_trajectory.points[j].positions[k];
//          joint_con.weight = 1;
//          joint_constraints_at_t.push_back(joint_con);
//        }
//        moveit_msgs::Constraints con;
//        con.joint_constraints = joint_constraints_at_t;
//        all_joint_constrains.push_back(con);
//      }
//
//      initial_traj.constraints = all_joint_constrains;
//
//      if (attached_object_)
//      {
//        updateState();
//        move_group_.setStartState(state_);
//      }
//
//      move_group_.setStartState(*move_group_.getCurrentState());
//      move_group_.setTrajectoryConstraints(initial_traj);
//
//      move_group_.setJointValueTarget(move_group_.getNamedTargetValues(target_value_));
//      move_group_.setPlanningTime(30);
//
//      bool success2 = (move_group_.plan(new_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//      if (success2)
//      {
//        ROS_WARN("Successs to make optimized path!");
//      }
//      else
//      {
//        ROS_WARN("Could not make optimized path");
//      }
//
      vis_.displayTrajectory_opt(new_plan_.trajectory_, joint_model_group_);
      vis_.trigger();

      message = "Press Next if acceptable this trajectory";

      vis_.displayText(message);
      vis_.trigger();

      vis_.buttonController();
      vis_.trigger();

      if (!move_group_.execute(new_plan_))
      {
        ROS_WARN("Could not move to target pose");
        return;
      }

      message = "Finish moving to default position";
      vis_.displayText(message);
      vis_.trigger();

      sleep(5.0);
      move_group_.clearPathConstraints();
    }

  private:
    int batch_;
    bool attached_object_;
    std::string target_value_;
    std::string robot_name_;
    std::vector<std::string> joint_names_;

    ros::NodeHandle nh_;
    ros::ServiceClient client_get_scene_;
    moveit_msgs::GetPlanningScene scene_srv_;

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan initial_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan new_plan_;
    moveit::planning_interface::PlanningSceneInterface current_scene_;
    moveit_msgs::RobotTrajectory mrobot_traj_;
    std::vector<P> vmtraj_dataset_;

    robot_state::RobotState state_;
    const robot_state::JointModelGroup* joint_model_group_;
    Visualizer vis_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  OptimizationPath move2default(nh);
  move2default.generateTraj();

  ROS_INFO("Finished path_planning");
  ros::waitForShutdown();
  return 0;
}
