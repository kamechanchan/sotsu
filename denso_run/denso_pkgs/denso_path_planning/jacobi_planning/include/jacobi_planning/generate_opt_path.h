#ifndef GENERATE_OPT_PATH_H
#define GENERATE_OPT_PATH_H

#include <jacobi_planning/display.h>
#include <jacobi_planning/util.h>
#include <sys/types.h>
#include <map>

using moveit_visual_tools::Visualizer;
using namespace std;
using namespace Eigen;

typedef pair<moveit_msgs::RobotTrajectory, double> P;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef pair<string, double> PS;
typedef pair<moveit_msgs::RobotTrajectory, double> traj_cost;

namespace generate_opt_path
{
class OptimizationPath
{
    public:
        OptimizationPath(ros::NodeHandle& nh);

        ~OptimizationPath();
        void generateTraj();

    private:
        void getRobotState(const moveit_msgs::RobotTrajectory& m_traj, robot_state::RobotState& state, std::vector<robot_state::RobotState>& vRobotStates);

        void getJacobian(std::vector<std::vector<double>>& vvjacobian, std::vector<robot_state::RobotState> vstates, const robot_state::JointModelGroup* joint_model_group);

        void diffAngle(trajectory_msgs::JointTrajectory& traj, std::vector<double>& vdiff);

        double calcCost(trajectory_msgs::JointTrajectory& m_traj, std::vector<std::vector<double>>& vvjacobian);

        int batch_;
        std::string target_value_;
        std::string robot_name_;
        XmlRpc::XmlRpcValue joints_names_;
        geometry_msgs::Pose pose_;
        ros::NodeHandle nh_;
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

}//namespace generate_opt_path

#endif //GENERATE_OPT_PATH_H
