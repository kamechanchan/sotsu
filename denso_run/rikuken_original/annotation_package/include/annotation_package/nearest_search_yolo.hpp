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
#include <sensor_msgs/PointCloud2.h>
#include <color_cloud_bridge/out_segmentation.h>
#include <color_cloud_bridge/object_kiriwake.h>

namespace nearest_point_extractor
{
    struct mesh_and_instance {
        pcl::PointCloud<pcl::PointXYZ> mesh_pcl_one;
        int instance;
    };
    class NearestPointExtractor
    {
    public:
        NearestPointExtractor(ros::NodeHandle &nh);
        void InputCallback(const color_cloud_bridge::object_kiriwakeConstPtr&);
        void exect();
        color_cloud_bridge::out_segmentation extract_cloud(pcl::PointCloud<pcl::PointXYZ> sensor_cloud, std::vector<mesh_and_instance> mesh_cloud, double radius);
        template <typename T>
        void print_parameter(T para)
        {
            std::cout << para << std::endl;
        }

        template <typename T>
        void get_one_message(T &msg, std::string topic_name, int timeout)
        {
            boost::shared_ptr<const T> share;
            share = ros::topic::waitForMessage<T>(topic_name, nh_, ros::Duration(timeout));
            if (share != NULL) {
                msg = *share;
            }
            share.reset();
        }
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_;
        color_cloud_bridge::out_segmentation output_cloud_;
        std::string frame_id_;
        std::vector<int> cloud_index_;
        sensor_msgs::PointCloud2 sensor_cloud_msgs_;
        std::vector<sensor_msgs::PointCloud2> mesh_clouds_msgs_;

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher dummy_pub_;
        ros::Subscriber mesh_topic_name_sub_;
        tf::StampedTransform transform_;
        tf::TransformListener listener_;
        int num_of_nearest_points_;
        int timeout_;
        bool flag_;
        std::string mesh_topic_name_;
        std::string sensor_topic_name_;
        std::string output_topic_name_;
        double radius_;
        int background_instance_;
    };
}