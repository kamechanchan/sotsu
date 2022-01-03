#include <cloud_practice/img_cloud_common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <estimator/first_input.h>

class CloudPlanarSegmenter : public CloudOperator
{
public:
    CloudPlanarSegmenter(ros::NodeHandle &nh) :
        cloud_segmented_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_segmented", 1)),
        cloud_without_segmented_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_without_segmented", 1)),
        indices_pub_(nh.advertise<pcl_msgs::PointIndices>("indices", 1)),
        coefficients_pub_(nh.advertise<pcl_msgs::ModelCoefficients>("coefficients", 1)),
        nh_(nh)
    {
        pnh_ = new ros::NodeHandle("~");
        pnh_->getParam("distance_threshold", distance_threshold);
        pnh_->getParam("service_name", service_name);
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
        ROS_INFO_STREAM("OK");
        cloud_without_segmented_pub_.publish(cloud_without_segmented_ros_);
        ser_ = nh_.advertiseService(service_name, &CloudPlanarSegmenter::callback, this);
        indices_pub_.publish(indices_ros_);
        coefficients_pub_.publish(coefficients_ros_);

    }
    
    bool callback(estimator::first_input::Request &req, estimator::first_input::Response &res){
        res.out_cloud = cloud_without_segmented_ros_;
        res.out_img = img_input_ros_;
        ROS_INFO_STREAM("ok");
    }
    float distance_threshold;
    std::string service_name;
    ros::NodeHandle nh_;

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
    ros::ServiceServer ser_;
    
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cloud_planar_segmenter");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string pc_topic_name;
    std::string img_topic_name;
    int timespan;
    pnh.getParam("input_pc_topic", pc_topic_name);
    pnh.getParam("input_img_topic", img_topic_name);
    pnh.getParam("timespan", timespan);
    CloudOperationHandler handler(nh, new CloudPlanarSegmenter(nh), pc_topic_name, img_topic_name, timespan);
    ros::spin();
    return 0;
}

