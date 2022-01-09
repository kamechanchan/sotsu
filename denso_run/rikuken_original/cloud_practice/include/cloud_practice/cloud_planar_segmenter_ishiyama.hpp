#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <estimator/first_input.h>
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>



template <typename T>
void get_one_message(T &final_message, std::string message_name, ros::NodeHandle nh, int timespan)
{
    boost::shared_ptr<const T> share;
    share = ros::topic::waitForMessage<T>(message_name, nh, ros::Duration(timespan));
    if (share != NULL) {
        final_message = *share;
    }
    share.reset();
}


class CloudPlanarSegmenter
{
private:
    // pcl::ModelCoefficients coefficients_pcl;
    // pcl::SACSegmentation<pcl::PointXYZ> segmentation;

public:
    CloudPlanarSegmenter(ros::NodeHandle&);
    void operate();
    void publish();
    bool callback(estimator::first_input::Request &req, estimator::first_input::Response &res);
    void segment(pcl::PointIndices::Ptr);
    void extract(pcl::PointIndices::Ptr);
    void cloud_callback(const sensor_msgs::PointCloud2&);
    void img_callback(const sensor_msgs::Image&);
    void downSample();
    
    float distance_threshold_;
    sensor_msgs::PointCloud2 cloud_input_ros_;
    sensor_msgs::Image img_input_ros_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceServer ser_;

protected:
    ros::Publisher cloud_segmented_pub_;
    ros::Publisher cloud_without_segmented_pub_;
    ros::Publisher indices_pub_;
    ros::Publisher coefficients_pub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber img_sub_;
    sensor_msgs::PointCloud2 cloud_segmented_ros_;
    sensor_msgs::PointCloud2 cloud_without_segmented_ros_;
    pcl_msgs::PointIndices indices_ros_;
    pcl_msgs::ModelCoefficients coefficients_ros_;
    pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl_;
    std::string cloud_topic_name_;
    std::string img_topic_name_;
    int timespan_;
    std::string service_name_;
    std::vector<float> x_;
    std::vector<float> y_;
    std::vector<float> z_;
    float LEAF_SIZE_;
    bool VoxelGrid_swicth_;
};

