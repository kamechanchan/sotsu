#include <pointcloud_usage/child_cloud.h>
#include <pointcloud_usage/create_cloud.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init");
    
    std::string load_file_name;
    std::string output_topic_name;
    std::string parent_link;
    ros::NodeHandle pnh("~");
    pnh.getParam("load_file_name", load_file_name);
    pnh.getParam("output_topic_name", output_topic_name);
    pnh.getParam("parent_link", parent_link);
    Point_and_ROS po_r(10, parent_link);
    pcl::PointCloud<pcl::PointXYZ> load_cloud;
    sensor_msgs::PointCloud2 load_ros;
    PointCloud_class::load_pcd(load_cloud, load_file_name);
    Point_and_ROS::convert_1_to_2(load_cloud, load_ros);
    po_r.operate(load_ros, output_topic_name);
    return 0;
}