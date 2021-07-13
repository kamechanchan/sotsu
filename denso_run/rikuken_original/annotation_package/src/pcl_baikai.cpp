#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <color_cloud_bridge/dummy_pcl.h>


ros::NodeHandle *pnh;
std::string sensor_topic_name;
std::string publish_topic_name;
static int count = 0;
ros::Publisher dummy_pub;

void callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    ROS_INFO_STREAM("callback start");
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.points.size() < 1) {
        return;
    }
    ROS_INFO_STREAM("change ros msg to pcl");
    pcl::io::savePCDFile("/home/ericlab/dummy_cloud/hab.pcd", cloud);
    color_cloud_bridge::dummy_pcl dummy;
    for (int i = 0; i < cloud.points.size(); i++) {
        dummy.x.push_back(cloud.points[i].x);
        dummy.y.push_back(cloud.points[i].y);
        dummy.z.push_back(cloud.points[i].z);
        dummy.rgb.push_back(cloud.points[i].rgba);
    }
    for (int i = 0; i < 10; i++) {
        ROS_INFO_STREAM("x: " << dummy.x[i*10] << "  y: " << dummy.y[i*10] << "  z: " << dummy.z[i*10] << "  rgb: " << dummy.rgb[i*10]);
    }
    for (int i = 0; i < 10; i++) {
        ROS_INFO_STREAM("pointcloud  rgb:" << cloud.points[i].rgb << "  rgba" << cloud.points[i].rgba << "  r" << cloud.points[i].r 
                        << "  b" << cloud.points[i].b << "  g" << cloud.points[i].g);
    }
    ROS_INFO_STREAM("publish start");
    dummy_pub.publish(dummy);
    dummy.x.clear();
    dummy.y.clear();
    dummy.z.clear();
    dummy.rgb.clear();
    ROS_INFO_STREAM("finish");


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_bridge");
    ros::NodeHandle nh;
    pnh = new ros::NodeHandle("~");
    pnh->getParam("sensor_topic_name", sensor_topic_name);
    pnh->getParam("publish_topic_name", publish_topic_name);
    dummy_pub = nh.advertise<color_cloud_bridge::dummy_pcl>(publish_topic_name, 10);
    ros::Subscriber sub;
    sub = nh.subscribe(sensor_topic_name, 10, callback);
    ros::spin();
    return 0;
}