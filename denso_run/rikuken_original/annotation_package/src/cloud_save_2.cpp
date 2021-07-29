#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <color_cloud_bridge/dummy_pcl.h>
#include <color_cloud_bridge/out_segmentation.h>

ros::NodeHandle *pnh;
std::string pcd_dir_name;
std::string pcd_file_name;
std::string topic_name;
ros::Publisher pub;

/*void callback(color_cloud_bridge::dummy_pclConstPtr msg)
{
    sensor_msgs::PointCloud2 ros_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (int i = 0; i < msg->x.size(); i++)
    {
        pcl::PointXYZRGB xyzrgb;
        xyzrgb.x = msg->x[i];
        xyzrgb.y = msg->y[i];
        xyzrgb.z = msg->z[i];
        xyzrgb.rgb = msg->rgb[i];
        cloud.push_back(xyzrgb);
    }
    pcl::io::savePCDFile(pcd_dir_name + '/' + pcd_file_name, cloud);
    pcl::toROSMsg(cloud, ros_msg);
    pub.publish(ros_msg);

}*/

template <typename T>
void get_one_message(std::string topic_name, int timeout, ros::NodeHandle nh, 
                    T &msg)
{
    boost::shared_ptr<const T> share;
    share = ros::topic::waitForMessage<T>(topic_name, nh, ros::Duration(timeout));
    if (share != NULL)
    {
        msg = *share;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_save_2");
    ros::NodeHandle nh;
    pnh = new ros::NodeHandle("~");
    pnh->getParam("pcd_dir_name", pcd_dir_name);
    pnh->getParam("pcd_file_name", pcd_file_name);
    pnh->getParam("topic_name", topic_name);
    // color_cloud_bridge::dummy_pcl dum_pcl;
    color_cloud_bridge::out_segmentation out_pcl;
    ROS_INFO_STREAM("start one message");
    ros::Subscriber sub;
    //sub = nh.subscribe(topic_name, 10, callback);
    
    //pub = nh.advertise<sensor_msgs::PointCloud2>("hati", 10);
    // while (dum_pcl.x.size() < 1)
    // {
    //     get_one_message(topic_name, 10, nh, dum_pcl);
    // }
    while (out_pcl.x.size() < 1)
    {
        get_one_message(topic_name, 10, nh, out_pcl);
    }
    ROS_INFO_STREAM("finish one message");
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (int i = 0; i < out_pcl.x.size(); i++) {
        pcl::PointXYZRGB xyzrgb;
        xyzrgb.x = out_pcl.x[i];
        xyzrgb.y = out_pcl.y[i];
        xyzrgb.z = out_pcl.z[i];
        // xyzrgb.rgb = dum_pcl.rgb[i];
        if (out_pcl.instance[i] == 0) {
            xyzrgb.r = 255;
            xyzrgb.g = 0;
            xyzrgb.b  =0;
        }
        else if (out_pcl.instance[i] == 1) {
            xyzrgb.r = 0;
            xyzrgb.g = 255;
            xyzrgb.b  =0;
        }
        else if (out_pcl.instance[i] == 2) {
            xyzrgb.r = 0;
            xyzrgb.g = 0;
            xyzrgb.b  =255;
        }
        else if (out_pcl.instance[i] == 3) {
            xyzrgb.r = 255;
            xyzrgb.g = 255;
            xyzrgb.b  =0;
        }
        else if (out_pcl.instance[i] == 4) {
            xyzrgb.r = 0;
            xyzrgb.g = 255;
            xyzrgb.b  =255;
        }
        else if (out_pcl.instance[i] == 5) {
            xyzrgb.r = 255;
            xyzrgb.g = 0;
            xyzrgb.b  =255;
        }
        else if (out_pcl.instance[i] == 6) {
            xyzrgb.r = 100;
            xyzrgb.g = 255;
            xyzrgb.b  =10;
        }
        else {
            xyzrgb.r = 255;
            xyzrgb.g = 255;
            xyzrgb.b  =255;
        }
        cloud.push_back(xyzrgb);
    }
    ROS_INFO_STREAM("start save");
    ROS_INFO_STREAM("cloud size is " << cloud.points.size());
    pcl::io::savePCDFile(pcd_dir_name + "/" + pcd_file_name, cloud);
    ROS_INFO_STREAM("finish save");
    return 0;

    

}