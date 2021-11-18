#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#define PI 3.1415

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory11");
    
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("arm_controller/follow_joint_trajectory", true);
    client.waitForServer();
    ROS_INFO_STREAM("action server get");
     control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = { "shoulder_pan_joint", "shoulder_lift_joint",
                            "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
    goal.trajectory.points.resize(3);
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.points[0].positions = {PI/3, -PI/2, -PI/5, -2*PI/3, PI/2, -PI/2};
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
    goal.trajectory.points[1].positions = {2*PI/3, -PI/2, -PI/5, -2*PI/3, PI/2, -PI/2};
    goal.trajectory.points[1].time_from_start = ros::Duration(1.0);
    goal.trajectory.points[2].positions = {3*PI/3, -PI/2, -PI/5, -2*PI/3, PI/2, -PI/2};
    goal.trajectory.points[2].time_from_start = ros::Duration(1.5);
    ros::Rate loop(0.67);
    while (ros::ok())
    {
       
        client.sendGoal(goal);
        client.waitForResult(ros::Duration(2.0));
        loop.sleep();
    }
    

    
    return 0;
    

}