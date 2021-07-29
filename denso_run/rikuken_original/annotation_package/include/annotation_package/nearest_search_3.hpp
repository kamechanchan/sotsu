#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace nearest_point_extractor
{
    class NearestPointExtractor
    {
    public:
        NearestPointExtractor(ros::NodeHandle &nh, int num);
        void publish();
        void InputCallback(const sensor_msgs::PointCloud2ConstPtr&);
        void mesh_callback(const sensor_msgs::PointCloud2ConstPtr&, int);
        

        void param_register(std::string, std::string, std::string, int);
        void exect();
        void sensor_input(sensor_msgs::PointCloud2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_cloud();
        template <typename T>
        void print_parameter(T para)
        {
            std::cout << para << std::endl;
        }
        pcl::PointCloud<pcl::PointXYZRGB> writing_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_;
        std::string frame_id_;
        std::vector<int> cloud_index_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_nukitori_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>*> mesh_clouds_;
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle *pnh_;
        ros::Publisher cloud_pub_;
        ros::Subscriber cloud_sub_;
        ros::Subscriber mesh_sub_1_;
        ros::Subscriber mesh_sub_2_;
        ros::Subscriber mesh_sub_3_;
        ros::Subscriber mesh_sub_4_;
        ros::Subscriber mesh_sub_5_;
        ros::Subscriber mesh_sub_6_;
        ros::Subscriber mesh_sub_7_;
        ros::Subscriber mesh_sub_8_;
        ros::Subscriber mesh_sub_9_;
        ros::Subscriber mesh_sub_10_;
        ros::Subscriber mesh_sub_11_;
        ros::Subscriber mesh_sub_12_;
        ros::Subscriber mesh_sub_13_;
        ros::Subscriber mesh_sub_14_;
        ros::Subscriber mesh_sub_15_;
        ros::Subscriber mesh_sub_16_;
        ros::Subscriber mesh_sub_17_;
        ros::Subscriber mesh_sub_18_;
        ros::Subscriber mesh_sub_19_;
        ros::Subscriber mesh_sub_20_;
        ros::Subscriber mesh_sub_0_;

        tf::StampedTransform transform_;
        tf::TransformListener listener_;

        
        
        float LEAF_SIZE;
        int num_of_nearest_points_;
        bool flag_;
        int the_number_of_object_;
        std::string mesh_base_topic_name_;
        std::string sensor_topic_name_;
        std::string output_topic_name_;
        std::vector<std::vector<int>> color;
        double radius;

    };
}