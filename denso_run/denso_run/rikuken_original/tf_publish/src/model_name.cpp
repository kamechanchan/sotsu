#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model");
    ros::NodeHandle nh;
    boost::shared_ptr<const gazebo_msgs::ModelStates> msg;
    msg = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", nh, ros::Duration(10));
    std::vector<std::string> model_names_;
    std::vector<geometry_msgs::Pose> model_poses_;
    model_names_ = msg->name;
    model_poses_ = msg->pose;
    for (int i = 0; i < model_names_.size(); i++) {
        std::cout << model_names_[i] << "  ";
    }
    for (int i = 0; i < model_poses_.size(); i++) {
        std::cout << model_poses_[i] << "  ";
    }
    std::cout << std::endl;
    return 0;
}