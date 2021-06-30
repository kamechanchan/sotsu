#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define PI 3.1415

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
    trajectory_msgs::JointTrajectoryPoint joint_value;
    joint_value.positions = {0, -PI/2, -PI/5, -2*PI/3, PI/2, -PI/2};
    

    trajectory_msgs::JointTrajectory joint;
    joint.points.resize(3);
    joint.joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
    joint.header.stamp = ros::Time::now();

    joint.points[0].positions = {PI/3, -PI/2, -PI/5, -2*PI/3, PI/2, -PI/2};
    joint.points[0].time_from_start = ros::Duration(0.5);
    
    joint.points[1].positions = {2*PI/3, -PI/2, -PI/5, -2*PI/3, PI/2, -PI/2};
    joint.points[1].time_from_start = ros::Duration(1);

    joint.points[2].positions = {3*PI/3, -PI/2, -PI/5, -2*PI/3, PI/2, -PI/2};
    joint.points[2].time_from_start = ros::Duration(1.5);

    ros::Rate loop(0.5);
    while (ros::ok())
    {
        pub.publish(joint);
        loop.sleep();
    }
    
    return 0;
    

}