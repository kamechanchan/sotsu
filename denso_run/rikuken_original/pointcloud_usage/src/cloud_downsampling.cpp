#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>
#include <time.h>



ros::Publisher ros_pub;
float leaf_size = 0.01f;
int point_size;

void callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud, out_cloud;
    pcl::fromROSMsg(*msg, cloud);
    srand((unsigned int)time(NULL));

    // pcl::VoxelGrid<pcl::PointXYZRGB> voxelsample;
    // voxelsample.setInputCloud(cloud.makeShared());
    // voxelsample.setLeafSize(leaf_size, leaf_size, leaf_size);
    // voxelsample.filter(out_cloud);
    for (int i = 0; i < point_size; i++) {
        // if ((rand() / (double)RAND_MAX) <= leaf_size) {
        //     out_cloud.points.push_back(cloud.points[i]);
        // }
        int ii = static_cast<int>((rand() / (double)RAND_MAX) * cloud.points.size());
        out_cloud.points.push_back(cloud.points[ii]);

    }
    sensor_msgs::PointCloud2 ros_msg;
    out_cloud.header.frame_id = msg->header.frame_id;
    // ROS_INFO_STREAM(out_cloud.header.frame_id);
    ROS_INFO_STREAM(cloud.points.size());
    pcl::toROSMsg(out_cloud, ros_msg);
    ros_pub.publish(ros_msg);  
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_init_down");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string input_topic_name, output_topic_name;
    pnh.getParam("input_topic_name", input_topic_name);
    pnh.getParam("output_topic_name", output_topic_name);
    pnh.getParam("leaf_size", leaf_size);
    pnh.getParam("point_size", point_size);
    ros_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic_name, 10);
    ros::Subscriber sub;
    sub = nh.subscribe(input_topic_name, 10, callback);
    ros::spin();
}