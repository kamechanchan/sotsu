#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace nearest_point_extractor
{
    class NearestPointExtractor
    {
    public:
        NearestPointExtractor(ros::NodeHandle &nh);
        void publish();
        void InputCallback(const sensor_msgs::PointCloud2ConstPtr&, const sensor_msgs::PointCloud2ConstPtr&);
        void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
        void param_register(std::string, std::string, std::string, int);
        void color_decide(unsigned char, unsigned char, unsigned char);
        void exect();
        void read_cloud(pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB> write_cloud();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_cloud(pcl::PointCloud<pcl::PointXYZ> sensor_cloud, pcl::PointCloud<pcl::PointXYZ> mesh_cloud, double radius);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> Sensor_Sync_Sub_Type;
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud_;
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle *pnh_;
        ros::Publisher cloud_pub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *sensor_cloud_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *mesh_cloud_sub_;
        message_filters::Synchronizer<Sensor_Sync_Sub_Type> *sensor_sync_;
        tf::StampedTransform transform_;
        tf::TransformListener listener_;

        
        
        float LEAF_SIZE;
        int num_of_nearest_points_;
        bool flag_;
        std::string mesh_topic_name_;
        std::string sensor_topic_name_;
        std::string output_topic_name_;
        int red_;
        int blue_;
        int green_;
        double radius;

    };
}