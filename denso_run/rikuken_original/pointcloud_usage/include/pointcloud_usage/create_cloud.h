#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/impl/transforms.hpp>



class PointCloud_class
{
public:
    
    static void initialize(pcl::PointCloud<pcl::PointXYZ> &pointcloud, int width, int height)
    {
        pointcloud.width = width;
        pointcloud.height = height;
        pointcloud.points.resize(width * height);
    }

    template <typename POINT>
    static void initialize(pcl::PointCloud<POINT> &cloud, int width, int height)
    {
        cloud.width = width;
        cloud.height = height;
        cloud.points.resize(cloud.width * cloud.height);
    }

    static void create_cloud(pcl::PointCloud<pcl::PointXYZ> &pointcloud)
    {
        for (size_t i = 0; i < pointcloud.points.size(); ++i)
        {
            pointcloud.points[i].x = 3.0 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].y = 1.0 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].z = 0.5 * rand() / (RAND_MAX + 1.0f);
        }
    }

    static void create_cloud(pcl::PointCloud<pcl::PointXYZI> &pointcloud)
    {
        for (size_t i = 0; i < pointcloud.points.size(); ++i)
        {
            pointcloud.points[i].x = 3.0 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].y = 1.0 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].z = 0.5 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].intensity = 0.2 * rand() / (RAND_MAX + 1.0f);
        }
    }

    static void create_cloud(pcl::PointCloud<pcl::PointXYZINormal> &pointcloud)
    {
        for (size_t i = 0; i < pointcloud.points.size(); ++i)
        {
            pointcloud.points[i].x = 3.0 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].y = 1.0 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].z = 0.5 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].intensity = 0.2 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].normal[0] = 0.5 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].normal[1] = 0.7 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].normal[2] = 0.9 * rand() / (RAND_MAX + 1.0f);
        }
    }

    static void create_cloud(pcl::PointCloud<pcl::PointXYZRGB> &pointcloud)
    {
        for (size_t i = 0; i < pointcloud.points.size(); ++i)
        {
            pointcloud.points[i].x = 3.0 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].y = 1.0 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].z = 0.5 * rand() / (RAND_MAX + 1.0f);
            pointcloud.points[i].r = 134;
            pointcloud.points[i].g = 34;
            pointcloud.points[i].b = 23;
        }
    }
    /*template <typename POINT>
    static void create_cloud(pcl::PointCloud<POINT> &cloud)
    {
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            cloud.points[i].x = 3.0 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].y = 1.0 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].z = 0.5 * rand() / (RAND_MAX + 1.0f);
            
        }
    }*/

    static void load_pcd(pcl::PointCloud<pcl::PointXYZ> &pointcloud, std::string pcd_filename)
    {
        pcl::io::loadPCDFile(pcd_filename, pointcloud);
    }

    template <typename POINT>
    static void load_pcd(pcl::PointCloud<POINT> &pointcloud, std::string pcd_filename)
    {
        pcl::io::loadPCDFile(pcd_filename, pointcloud);
    }

    static void save_pcd(pcl::PointCloud<pcl::PointXYZ> &pointcloud, std::string pcd_savename)
    {
        pcl::io::savePCDFile(pcd_savename, pointcloud);
    }

    static void save_pcd_1(pcl::PointCloud<pcl::PointXYZI> &pointcloud, std::string pcd_savename)
    {
        pcl::io::savePCDFile(pcd_savename, pointcloud);
    }

    template <typename POINT>
    static void save_pcd(pcl::PointCloud<POINT> &pointcloud, std::string pcd_savename)
    {
        pcl::io::savePCDFile(pcd_savename, pointcloud);
    }

    static void filter_cloud(pcl::PointCloud<pcl::PointXYZ> &pointcloud)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
        statFilter.setInputCloud(pointcloud.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(pointcloud);
    }

    template <typename POINT>
    static void filter_cloud(pcl::PointCloud<POINT> &pointcloud)
    {
        pcl::StatisticalOutlierRemoval<POINT> statFilter;
        statFilter.setInputCloud(pointcloud.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(pointcloud);
    }

    static void downsampler_cloud(pcl::PointCloud<pcl::PointXYZ> &pointcloud)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(pointcloud.makeShared());
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(pointcloud);

    }

    
    template <typename POINT>
    static void downsampler_cloud(pcl::PointCloud<POINT> &pointcloud)
    {
        pcl::VoxelGrid<POINT> voxelSampler;
        voxelSampler.setInputCloud(pointcloud.makeShared());
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(pointcloud);
    }

    static void segment(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ> &pointcloud, pcl::ModelCoefficients &coefficient)
    {
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(0.01);
        segmentation.setInputCloud(pointcloud.makeShared());
        segmentation.segment(*inliers, coefficient);
    }

    template <typename POINT>
    static void segment(pcl::PointIndices::Ptr inliers, pcl::PointCloud<POINT> &pointcloud, pcl::ModelCoefficients &coefficient)
    {
        pcl::SACSegmentation<POINT> segmentation;
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(0.005);
        segmentation.setInputCloud(pointcloud.makeShared());
        segmentation.segment(*inliers, coefficient);
    }

    static void extract(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ> &pointcloud, 
                        pcl::PointCloud<pcl::PointXYZ> &segment_cloud, pcl::PointCloud<pcl::PointXYZ> &without_segment)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(pointcloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(segment_cloud);
        extract.setNegative(true);
        extract.filter(without_segment);
    }

    template <typename POINT>
    static void extract(pcl::PointIndices::Ptr inliers, pcl::PointCloud<POINT> &pointcloud, 
                        pcl::PointCloud<POINT> &segment_cloud, pcl::PointCloud<POINT> &without_segment)
    {
        pcl::ExtractIndices<POINT> extract;
        extract.setInputCloud(pointcloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(segment_cloud);
        extract.setNegative(true);
        extract.filter(without_segment);
    }

    



  
};