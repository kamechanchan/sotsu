#include <pointcloud_usage/create_cloud.h>
#include <pointcloud_usage/child_cloud.h>
#include <pcl/point_types_conversion.h>

static int cout = 0;
ros::Publisher pub;
std::string save_name;
void point_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud, without_segment, segment;
    pcl::ModelCoefficients coefficient;
    sensor_msgs::PointCloud2 sensr;
    /*for (int i = 0; i < cloud_msg->fields.size(); i++) {
        std::cout << cloud_msg->fields[i].name << "  ";
    }
    std::cout << std::endl;*/
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl::PointIndices::Ptr inlier(new pcl::PointIndices());
    PointCloud_class::segment(inlier, cloud, coefficient);
    PointCloud_class::extract(inlier, cloud, segment, without_segment);
    //pcl::PointCloud<pcl::PointXYZI> inten_cloud;
    //pcl::copyPointCloud(without_segment, inten_cloud);
    //pcl::PointCloudXYZRGBtoXYZI(without_segment, inten_cloud);
    PointCloud_class::save_pcd(cloud, save_name);
    //Point_and_ROS::convert_1_to_2(inten_cloud, sensr);
    sensr.header.frame_id = cloud_msg->header.frame_id;
    pub.publish(sensr);
    

    cout++;
    std::cout << cout << std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "niit");
    ros::NodeHandle pnh("~");
    std::string pub_topic_name = "shinya_3";
    std::string topic_name = "/photoneo_center/sensor/points";
    
    pnh.getParam("topic_name", topic_name);
    pnh.getParam("save_name", save_name);
    pnh.getParam("pub_topic_name", pub_topic_name);
    //pcl::PointCloud<pcl::PointXYZI> pointcloud;
   //sensor_msgs::PointCloud2 tr = Point_and_ROS::get_topic_message_1(topic_name);
   ros::Subscriber sub;
   ros::NodeHandle nh;
   pub = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 1);
   sub = nh.subscribe(topic_name, 1, point_callback);
   int count = 0;
   ros::Rate loop_rate(10);
   while (count++ <= 20)
   {
       //std::cout << topic_name << std::endl;
       ros::spinOnce();
       loop_rate.sleep();
   }
    //PointCloud_class::save_pcd(pointcloud, save_name);
    return 0;
}