#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <color_cloud_bridge/dummy_pcl.h>
#include <color_cloud_bridge/out_segmentation.h>


std::string pcd_dir_name;
std::string pcd_file_name;
std::string receive_topic_name;
std::string output_topic_name;
ros::Publisher pub;
std::string frame_id_;

void callback(const color_cloud_bridge::out_segmentationConstPtr msg)
{
    color_cloud_bridge::out_segmentation out_pcl = *msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    int count = 0;
    for (int i = 0; i < out_pcl.x.size(); i++) {
        pcl::PointXYZRGB xyzrgb;
        xyzrgb.x = out_pcl.x[i];
        xyzrgb.y = out_pcl.y[i];
        xyzrgb.z = out_pcl.z[i];
        if (out_pcl.instance[i] == 0) {
            xyzrgb.r = 255;
            xyzrgb.b = 0;
            xyzrgb.g = 0;
        }
        else if (out_pcl.instance[i] == 1) {
            xyzrgb.r = 0;
            xyzrgb.b = 255;
            xyzrgb.g = 0;
        }
        else if (out_pcl.instance[i] == 2) {
            xyzrgb.r = 0;
            xyzrgb.b = 0;
            xyzrgb.g = 255;
        }
        else if (out_pcl.instance[i] == 3) {
            xyzrgb.r = 255;
            xyzrgb.b = 255;
            xyzrgb.g = 0;
        }
        else if (out_pcl.instance[i] == 4) {
            xyzrgb.r = 0;
            xyzrgb.b = 255;
            xyzrgb.g = 255;
        } 
        else if (out_pcl.instance[i] == 5) {
            xyzrgb.r = 255;
            xyzrgb.b = 0;
            xyzrgb.g = 255;
        }
        else if (out_pcl.instance[i] == 6) {
            xyzrgb.r = 255;
            xyzrgb.b = 100;
            xyzrgb.g = 255;
        }
        else if (out_pcl.instance[i] == 7) {
            xyzrgb.r = 100;
            xyzrgb.b = 255;
            xyzrgb.g = 255;
        }
        else if (out_pcl.instance[i] == 8) {
            xyzrgb.r = 255;
            xyzrgb.b = 255;
            xyzrgb.g = 100;
        }
        else if (out_pcl.instance[i] == 9) {
            xyzrgb.r = 100;
            xyzrgb.b = 100;
            xyzrgb.g = 255;
        }
        else if (out_pcl.instance[i] == 10) {
            xyzrgb.r = 255;
            xyzrgb.b = 100;
            xyzrgb.g = 100;
        }
        else if (out_pcl.instance[i] == 11) {
            xyzrgb.r = 100;
            xyzrgb.b = 255;
            xyzrgb.g = 100;
        }
        else if (out_pcl.instance[i] == 12) {
            xyzrgb.r = 150;
            xyzrgb.b = 100;
            xyzrgb.g = 255;
        }
        else if (out_pcl.instance[i] == 13) {
            xyzrgb.r = 100;
            xyzrgb.b = 150;
            xyzrgb.g = 255;
        }
        else if (out_pcl.instance[i] == 14) {
            xyzrgb.r = 255;
            xyzrgb.b = 100;
            xyzrgb.g = 150;
        }
        else if (out_pcl.instance[i] == 15) {
            xyzrgb.r = 255;
            xyzrgb.b = 150;
            xyzrgb.g = 100;
        }
        else if (out_pcl.instance[i] == 16) {
            xyzrgb.r = 100;
            xyzrgb.b = 255;
            xyzrgb.g = 150;
        }
        else if (out_pcl.instance[i] == 17) {
            xyzrgb.r = 150;
            xyzrgb.b = 255;
            xyzrgb.g = 100;
        }
        else if (out_pcl.instance[i] == 18) {
            xyzrgb.r = 100;
            xyzrgb.b = 150;
            xyzrgb.g = 0;
        }
        else if (out_pcl.instance[i] == 19) {
            xyzrgb.r = 150;
            xyzrgb.b = 100;
            xyzrgb.g = 0;
        }
        else if (out_pcl.instance[i] == 20) {
            xyzrgb.r = 150;
            xyzrgb.b = 0;
            xyzrgb.g = 100;
        }
        else if (out_pcl.instance[i] == 21) {
            xyzrgb.r = 100;
            xyzrgb.b = 0;
            xyzrgb.g = 150;
        }
        else if (out_pcl.instance[i] == 22) {
            xyzrgb.r = 0;
            xyzrgb.b = 100;
            xyzrgb.g = 150;
        }
        else if (out_pcl.instance[i] == 23) {
            xyzrgb.r = 0;
            xyzrgb.b = 150;
            xyzrgb.g = 100;
        }
        else if (out_pcl.instance[i] == 24) {
            xyzrgb.r = 255;
            xyzrgb.b = 150;
            xyzrgb.g = 150;
        }
        else {
            xyzrgb.r = 255;
            xyzrgb.b = 255;
            xyzrgb.g  =255;
            count++;
        }
        
        cloud.push_back(xyzrgb);
    }
    ROS_INFO_STREAM("shiro kazu: " << count);
    ROS_INFO_STREAM("start save");
    ROS_INFO_STREAM("cloud size is " << cloud.points.size());
    ROS_INFO_STREAM("finish save");
    sensor_msgs::PointCloud2 ros_msgs;
    pcl::toROSMsg(cloud, ros_msgs);
    ros_msgs.header.frame_id = frame_id_;
    pub.publish(ros_msgs);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_save_2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParam("receive_topic_name", receive_topic_name);
    pnh.getParam("output_topic_name", output_topic_name);
    pnh.getParam("frame_id", frame_id_);
   
    ros::Subscriber sub;
    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic_name, 10);
    sub = nh.subscribe(receive_topic_name, 10, callback);
    ros::spin();
    
}