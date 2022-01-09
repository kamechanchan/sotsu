#include <cloud_practice/cloud_planar_segmenter_ishiyama.hpp>


CloudPlanarSegmenter::CloudPlanarSegmenter(ros::NodeHandle &nh) :
        cloud_segmented_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_segmented", 1)),
        cloud_without_segmented_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_without_segmented", 1)),
        indices_pub_(nh.advertise<pcl_msgs::PointIndices>("indices", 1)),
        coefficients_pub_(nh.advertise<pcl_msgs::ModelCoefficients>("coefficients", 1)),
        nh_(nh)
{
    pnh_ = ros::NodeHandle("~");
    pnh_.getParam("distance_threshold", distance_threshold_);
    pnh_.getParam("service_name", service_name_);
    pnh_.getParam("input_pc_topic", cloud_topic_name_);
    pnh_.getParam("input_img_topic", img_topic_name_);
    pnh_.getParam("timespan", timespan_);
    pnh_.getParam("VoxelGrid_switch", VoxelGrid_swicth_);
    pnh_.getParam("LEAF_SIZE", LEAF_SIZE_);
    ser_ = nh_.advertiseService(service_name_, &CloudPlanarSegmenter::callback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_name_, 1000, &CloudPlanarSegmenter::cloud_callback, this);
    img_sub_ = nh_.subscribe(img_topic_name_, 1000, &CloudPlanarSegmenter::img_callback, this);
    // publish();
    // operate();
}

void CloudPlanarSegmenter::operate()
{
    pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl_);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    ROS_INFO_STREAM("naaruhodo");
    ROS_INFO_STREAM(VoxelGrid_swicth_);
    if (VoxelGrid_swicth_ == true){
        downSample();
    }
    segment(inliers);
    extract(inliers);
    // publish();
}

void CloudPlanarSegmenter::publish()
{
    cloud_segmented_pub_.publish(cloud_segmented_ros_);
    ROS_INFO_STREAM("publish:OK");
    cloud_without_segmented_pub_.publish(cloud_without_segmented_ros_);
    indices_pub_.publish(indices_ros_);
    coefficients_pub_.publish(coefficients_ros_);
}

bool CloudPlanarSegmenter::callback(estimator::first_input::Request &req, estimator::first_input::Response &res){
    // get_one_message(cloud_input_ros_, cloud_topic_name_, nh_, timespan_);
    // get_one_message(img_input_ros_, img_topic_name_, nh_, timespan_);

    operate();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // res.out_cloud = cloud_segmented_ros_;
    res.x = x_;
    res.y = y_;
    res.z = z_;
    res.out_img = img_input_ros_;
    ROS_INFO_STREAM("service:ok");
    publish();
    // ROS_INFO_STREAM("out_cloud" << res.out_cloud);
    // ROS_INFO_STREAM("out_img" << res.out_img);
    return true;
}


void CloudPlanarSegmenter::segment(pcl::PointIndices::Ptr inliers)
{
    // pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl_);

    pcl::ModelCoefficients coefficients_pcl;
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;

    segmentation.setModelType(pcl::SACMODEL_PLANE);
    
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(1000);
    segmentation.setDistanceThreshold(distance_threshold_);
    segmentation.setInputCloud(cloud_input_pcl_.makeShared());
    segmentation.segment(*inliers, coefficients_pcl);
    
    
}

void CloudPlanarSegmenter::extract(pcl::PointIndices::Ptr inliers)
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

    // float x_min = 0;
    // float x_max = 0;
    // float y_min = 0;
    // float y_max = 0;
    // float z_min = 0;
    // float z_max = 0;
    x_.clear();
    y_.clear();
    z_.clear();

    for (int i=0; i<cloud_without_segmented_pcl.points.size(); i++)
    {
        x_.push_back(cloud_without_segmented_pcl.points[i].x);
        y_.push_back(cloud_without_segmented_pcl.points[i].y);
        z_.push_back(cloud_without_segmented_pcl.points[i].z);
        
        // if (cloud_without_segmented_pcl.points[i].x > x_max){
        //     x_max = cloud_without_segmented_pcl.points[i].x;
        // } 
        // if (cloud_without_segmented_pcl.points[i].x < x_min){
        //     x_min = cloud_without_segmented_pcl.points[i].x;
        // }
        // if (cloud_without_segmented_pcl.points[i].y > y_max){
        //     y_max = cloud_without_segmented_pcl.points[i].y;
        // } 
        // if (cloud_without_segmented_pcl.points[i].y < y_min){
        //     y_min = cloud_without_segmented_pcl.points[i].y;
        // }
        // if (cloud_without_segmented_pcl.points[i].z > z_max){
        //     z_max = cloud_without_segmented_pcl.points[i].z;
        // } 
        // if (cloud_without_segmented_pcl.points[i].z < z_min){
        //     z_min = cloud_without_segmented_pcl.points[i].z;
        // }
    }
    // ROS_INFO_STREAM("x_max;" << x_max << " x_min;" << x_min << " y_max;" << y_max << " y_min;" << y_min << " z_max;" << z_max << " z_min;" << z_min);
    std::cout << "------------------------------------" << std::endl;
    std::cout << "DownSampled cloud point size : " << cloud_without_segmented_pcl.points.size() << std::endl;
}

void CloudPlanarSegmenter::cloud_callback(const sensor_msgs::PointCloud2& cloud)
{
    cloud_input_ros_ = cloud;
    // ROS_INFO_STREAM("pcl_frame_id" << cloud_input_ros_.header.frame_id);
}

void CloudPlanarSegmenter::img_callback(const sensor_msgs::Image& img)
{
    img_input_ros_ = img;
    // ROS_INFO_STREAM("img_frame_id" << img_input_ros_.header.frame_id);
}

void CloudPlanarSegmenter::downSample()
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_input_pcl_.makeShared());
    sor.setLeafSize(LEAF_SIZE_, LEAF_SIZE_, LEAF_SIZE_);
    sor.filter(cloud_input_pcl_);
    ROS_INFO_STREAM("down_sample");
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cloud_planar_segmenter");
    ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");
    CloudPlanarSegmenter tanomuze(nh);
    // tanomuze.publish();
    ros::spin();
    return 0;
}