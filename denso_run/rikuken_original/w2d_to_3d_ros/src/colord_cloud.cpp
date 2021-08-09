#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>


template <typename T>
void get_one_message(T &final_message, std::string message_name, ros::NodeHandle nh, int timespan)
{
    boost::shared_ptr<const T> message;
    message = ros::topic::waitForMessage<T>(message_name, nh, ros::Duration(timespan));
    if (message != NULL) {
        final_message = *message;
    }
}

void tf_get(std::string target_frame, std::string source_frame, tf::StampedTransform &transform)
{
    ros::Time time = ros::Time(0);
    tf::TransformListener tf_listener_;
    try
    {
        tf_listener_.waitForTransform(target_frame, source_frame, time, ros::Duration(4.0));
        tf_listener_.lookupTransform(target_frame, source_frame, time, transform);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
        ros::Duration(4.0).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string cam_info, image_raw, input_pointcloud, output_pointcloud, target_frame, source_frame;
    double upper_val, lower_val;

    pnh.getParam("camera_info", cam_info);
    pnh.getParam("image_raw", image_raw);
    pnh.getParam("input_pointcloud", input_pointcloud);
    pnh.getParam("output_pointcloud", output_pointcloud);
    pnh.getParam("target_frame", target_frame);
    pnh.getParam("source_frame", source_frame);
    pnh.getParam("upper_filter_vals", upper_val);
    pnh.getParam("lower_filter_val", lower_val);

    sensor_msgs::CameraInfo cinfo;
    sensor_msgs::Image image;
    sensor_msgs::PointCloud2 input_cloud;
    tf::StampedTransform transform;
    pcl::PointCloud<pcl::PointXYZ> cloud, trans_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> colored_pointcloud;
    

    get_one_message<sensor_msgs::CameraInfo>(cinfo, cam_info, nh, 10);
    get_one_message<sensor_msgs::Image>(image, image_raw, nh, 10);
    get_one_message<sensor_msgs::PointCloud2>(input_cloud, input_pointcloud, nh, 10);

    std::cout << cinfo.K.elems[0] << " " << cinfo.K.elems[4] << std::endl;
    std::cout << image.height << " " << image.width << std::endl;


    tf_get(target_frame, source_frame, transform);

    tf::Transform tf;
    tf.setOrigin(transform.getOrigin());
    tf.setRotation(transform.getRotation());
    ROS_INFO("transform_pointcloud");
    pcl_ros::transformPointCloud(cloud, trans_cloud, tf);
    
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exeption %s", e.what());
    }

    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;

    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cinfo);
    colored_pointcloud.clear();
    int count = 0;
    int aida = 100;
    double debug_x = 0;
    double debug_y = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = trans_cloud.points.begin(); pt != trans_cloud.points.end(); ++pt)
    {
        count++;
        if (pt->z < 0)
        {
            ROS_INFO("passed point");
            continue;
        }
        cv::Point3d pt_cv(pt->x, pt->y, pt->z);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);
        debug_x += uv.x;
        debug_y += uv.y;
        if (count % aida == 0) {
            ROS_INFO_STREAM("uv.x: " << uv.x << "  uv.y: " << uv.y);
            ROS_INFO_STREAM("image_col / 2: " << rgb_image.cols / 2 << "   image_rows / 2: " << rgb_image.rows / 2);
        }
        if (uv.x > (-rgb_image.cols / 2) && uv.x < (rgb_image.cols / 2) && uv.y > (-rgb_image.rows / 2) &&
        uv.y < (rgb_image.rows / 2))
        {
            ROS_INFO_STREAM("tuuka!!");
            cv::Point2d converted_uv(uv.x + rgb_image.cols / 2, uv.y + rgb_image.rows / 2);
            cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(converted_uv.y, converted_uv.x);
            pcl::PointXYZRGB buffer_point;
            double rgb_ave = (rgb[0] + rgb[1] + rgb[2]) / 3.0;
            if (rgb_ave < upper_val) {
                continue;
            }
            if (rgb_ave > lower_val) {
                continue;
            }
            buffer_point.x = pt->x;
            buffer_point.y = pt->y;
            buffer_point.z = pt->z;
            buffer_point.r = rgb[0];
            buffer_point.g = rgb[1];
            buffer_point.b = rgb[2];
            colored_pointcloud.push_back(buffer_point);
        }
    }

    
    

    return 0;
}