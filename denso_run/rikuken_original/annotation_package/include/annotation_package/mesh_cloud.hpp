#pragma once
#include <ros/ros.h>
#include <pcl_ros/impl/transforms.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <annotation_package/mesh_sampler.hpp>
namespace mesh_cloud
{
    class MeshCloud
    {
    public:
        MeshCloud(ros::NodeHandle& nh);
        ~MeshCloud(){}
        void getMesh(const std::string dir_path);
        void transformMesh();
        void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ> cloud_filtered);
        void publishCloud();
        

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle *pnh_;
        tf::TransformListener tf_;
        sensor_msgs::PointCloud2 mesh_cloud_ros_;
        ros::Publisher mesh_point_pub_;
        pcl::PointCloud<pcl::PointXYZ> mesh_point_pcl_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_mesh_ptr_pcl_;

        std::vector<pcl::PointCloud<pcl::PointXYZ>> parts_clouds_;
        std::vector<pcl::PolygonMesh> meshes_cluster_;
        std::vector<std::string> link_names_;
        std::vector<std::string> frame_names_;
        std::string frame_;
        std::string link_;
        std::string mesh_path;
        float LEAF_SIZE;
    };
}
