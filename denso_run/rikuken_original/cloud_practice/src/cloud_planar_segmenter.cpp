#include <cloud_practice/cloud_common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class CloudPlanarSegmenter : public CloudOperator
{
public:
    CloudPlanarSegmenter(ros::NodeHandle &nh) :
        cloud_segmented_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_segmented", 1)),
        cloud_without_segmented_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_without_segmented", 1)),
        indices_pub_(nh.advertise<pcl_msgs::PointIndices>("indices", 1)),
        coefficients_pub_(nh.advertise<pcl_msgs::ModelCoefficients>("coefficients", 1))
    {
        pnh_ = new ros::NodeHandle("~");
        pnh_->getParam("distance_threshold", distance_threshold);
    }

    void operate()
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        segment(inliers);
        extract(inliers);
    }

    void publish()
    {
        cloud_segmented_pub_.publish(cloud_segmented_ros_);
        cloud_without_segmented_pub_.publish(cloud_without_segmented_ros_);
        indices_pub_.publish(indices_ros_);
        coefficients_pub_.publish(coefficients_ros_);
    }
    float distance_threshold;

private:
    void segment(pcl::PointIndices::Ptr inliers)
    {
        pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl_);

        pcl::ModelCoefficients coefficients_pcl;
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;

        segmentation.setModelType(pcl::SACMODEL_PLANE);
      
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(distance_threshold);
        segmentation.setInputCloud(cloud_input_pcl_.makeShared());
        segmentation.segment(*inliers, coefficients_pcl);
        
    }

    void extract(pcl::PointIndices::Ptr inliers)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_segmented_pcl;
        pcl::PointCloud<pcl::PointXYZ> cloud_without_segmented_pcl;
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        extract.setInputCloud(cloud_input_pcl_.makeShared());
        extract.setIndices(inliers);

        extract.setNegative(false);
        extract.filter(cloud_segmented_pcl);

        extract.setNegative(true);
        extract.filter(cloud_without_segmented_pcl);

        pcl::toROSMsg(cloud_segmented_pcl, cloud_segmented_ros_);
        pcl::toROSMsg(cloud_without_segmented_pcl, cloud_without_segmented_ros_);
    }

protected:
    ros::Publisher cloud_segmented_pub_;
    ros::Publisher cloud_without_segmented_pub_;
    ros::Publisher indices_pub_;
    ros::Publisher coefficients_pub_;
    sensor_msgs::PointCloud2 cloud_segmented_ros_;
    sensor_msgs::PointCloud2 cloud_without_segmented_ros_;
    pcl_msgs::PointIndices indices_ros_;
    pcl_msgs::ModelCoefficients coefficients_ros_;
    pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl_;
    ros::NodeHandle *pnh_;
    
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cloud_planar_segmenter");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string pc_topic_name;
    pnh.getParam("input_pc_topic", pc_topic_name);
    CloudOperationHandler handler(nh, new CloudPlanarSegmenter(nh), pc_topic_name);
    ros::spin();
    return 0;
}