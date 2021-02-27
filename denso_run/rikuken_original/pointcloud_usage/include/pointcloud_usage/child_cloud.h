#pragma once
#include <pointcloud_usage/create_cloud.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>


class Point_and_ROS
{
public:
    Point_and_ROS(int loop_rate_num_1, std::string frame_id_1): frame_id(frame_id_1), loop_rate_num(loop_rate_num_1)
    {};

    template <typename POINT>
    static void convert_1_to_2(pcl::PointCloud<POINT> &pointcloud, sensor_msgs::PointCloud2 &ros_cloud)
    {
        pcl::toROSMsg(pointcloud, ros_cloud);
    }

    template <typename POINT>
    static void convert_1_to_2(sensor_msgs::PointCloud2 &ros_cloud, pcl::PointCloud<POINT> &pointcloud)
    {
        pcl::fromROSMsg(ros_cloud, pointcloud);
    }

   
    void operate(sensor_msgs::PointCloud2 ros_pointcloud, std::string ros_topic_name)
    {
        pub = nh.advertise<sensor_msgs::PointCloud2>(ros_topic_name, 1000);
        ros::Rate loop(loop_rate_num);
        while (ros::ok())
        {
            ros_pointcloud.header.frame_id = frame_id;
            for (int i = 0; i < ros_pointcloud.fields.size(); i++) {
                std::cout << ros_pointcloud.fields[i].name << " ";
            }
            std::cout << std::endl;
            
            pub.publish(ros_pointcloud);
            loop.sleep();
        }
    }
    
    
    template <typename MES_ROS>
    static MES_ROS get_topic_message(std::string topic_name)
    {
        std::cout << "ere" << std::endl;
        boost::shared_ptr<const MES_ROS> mes_ptr;
        std::cout << "ere_1" << std::endl;  
        MES_ROS cp;
        while (1) {
            mes_ptr = ros::topic::waitForMessage<MES_ROS>(topic_name, ros::Duration(10));
            if (mes_ptr != NULL) {
                break;
            }
            std::cout << "ere_2" << std::endl;
        }
        cp = *mes_ptr;
        std::cout << "ere_3" << std::endl;
        return cp;
    }

    static sensor_msgs::PointCloud2 get_topic_message_1(std::string topic_name)
    {
        sensor_msgs::PointCloud2ConstPtr share;
        std::cout << topic_name << std::endl;
        share = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic_name, ros::Duration(10));
        sensor_msgs::PointCloud2 cp;
        if (share != NULL)
        {
            cp = *share;
        }
        return cp;
    }

    static void cloud_transform(pcl::PointCloud<pcl::PointXYZ> &pointcloud, std::string src_frame, std::string child_frame)
    {
        tf::StampedTransform transform_1;
        tf_.lookupTransform(src_frame, child_frame, ros::Time(0), transform_1);
        pcl_ros::transformPointCloud(pointcloud, pointcloud, transform_1);
    }

    template <typename POINT>
    static void cloud_transform(pcl::PointCloud<POINT> &pointcloud, std::string src_frame, std::string child_frame)
    {
        tf::StampedTransform transform_1;
        tf_.lookupTransform(src_frame, child_frame, ros::Time(0), transform_1);
        pcl_ros::transformPointCloud(pointcloud, pointcloud, transform_1);
    
    }



private:
    ros::Publisher pub;
    ros::NodeHandle nh;
    int loop_rate_num;
    std::string frame_id;
    static tf::TransformListener tf_;
};