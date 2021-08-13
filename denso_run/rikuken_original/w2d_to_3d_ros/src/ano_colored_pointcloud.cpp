#include <w2d_to_3d_ros/ano_colored_pointcloud.hpp>
#include <mutex>


std::mutex m;
Annotation_yolo::Annotation_yolo(ros::NodeHandle &nh) :
    nh_(nh),
    radious_(0.05),
    save_count_(0),
    work_count_(0)
{
    buffer_ = new tf2_ros::Buffer();
    lister_ = new tf2_ros::TransformListener(*buffer_);
    parameter_set();
    
    
    
    // ROS_INFO_STREAM("wataru");
}

void Annotation_yolo::parameter_set()
{
    pnh_ = new ros::NodeHandle("~");
    pnh_->getParam("camera_topic_name", camera_topic_name_);
    pnh_->getParam("image_topic_name", image_topic_name_);
    pnh_->getParam("source_frame", source_frame_);
    pnh_->getParam("target_frame", target_frame_);
    pnh_->getParam("f_scale", f_scale_);
    pnh_->getParam("cx_scale", cx_scale_);
    pnh_->getParam("cy_scale", cy_scale_);
    pnh_->getParam("radious", radious_);
    pnh_->getParam("save_dir_name", save_dir_name_);
    pnh_->getParam("filebasename", filebasename_);
    pnh_->getParam("work_count", work_count_);
    pnh_->getParam("model_name", model_name_);
    pnh_->getParam("world_frame", world_frame_);
    paramter_set_bara(model_name_, work_count_);
    camera_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_topic_name_, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 10);
    sensor_sync_ = new message_filters::Synchronizer<Sync_Sub_type>(Sync_Sub_type(10), *camera_sub_, *image_sub_);
    sensor_sync_->registerCallback(boost::bind(&Annotation_yolo::InputCallback, this, _1, _2));
}

void Annotation_yolo::InputCallback(sensor_msgs::CameraInfoConstPtr cam_msgs, sensor_msgs::ImageConstPtr image_msgs)
{
    std::lock_guard<std::mutex> lock(m);
    sensor_msgs::CameraInfo cinfo = *cam_msgs;
    sensor_msgs::Image image1 = *image_msgs;
    // geometry_msgs::TransformStamped transform;
    // tf_get(source_frame_, target_frame_, transform);
    // box_get(cinfo, image1, transform, draw_image_);
    geometry_msgs::TransformStamped trans_source;
    tf2::Quaternion q1;
    tf2::convert(trans_source.transform.rotation, q1);
    
    tf_get(world_frame_, source_frame_, trans_source);
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (int i = 0; i < work_count_; i++) {
        geometry_msgs::TransformStamped trans;
        tf_get(world_frame_, target_frames_[i], trans);
        transforms.push_back(trans);
    }
    box_get(cinfo, image1, transforms, draw_image_, work_count_);
    cv::imshow("windoue", draw_image_);
    cv::waitKey(10);
    nh_.getParam("write_is_ok", write_is_ok_);
    ROS_INFO_STREAM("write_is_osk: " << bool(write_is_ok_));
    if (write_is_ok_) {
        nh_.setParam("write_is_ok", false);
        std::string all_path = save_dir_name_ + "/" + filebasename_ + "_" + std::to_string(save_count_) + ".jpg";
        cv::imwrite(all_path, draw_image_);
        save_count_++;
    }
}

void Annotation_yolo::tf_get(std::string source_frame, std::string target_frame, geometry_msgs::TransformStamped &trans)
{
    try
    {
        trans = buffer_->lookupTransform(target_frame, source_frame, ros::Time(0));
        
        
        ROS_INFO_ONCE("I got a transfomr");
    }
    catch (tf2::TransformException &e)
    {
        ROS_WARN_STREAM(e.what());
        ros::Duration(1.0).sleep();
        return;
    }
    
    
}

void Annotation_yolo::box_get(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, geometry_msgs::TransformStamped trans, cv::Mat &draw_img)
{
    ROS_INFO_STREAM("hatayu");
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
        // return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    draw_img = cv_image;
    ROS_INFO_STREAM("x: " << trans.transform.translation.x);
    ROS_INFO_STREAM("y: " << trans.transform.translation.y);
    ROS_INFO_STREAM("z: " << trans.transform.translation.z);
    float x = trans.transform.translation.x;
    float y = trans.transform.translation.y;
    float z = trans.transform.translation.z;
    
    cv::Point3d pt_cv(y, x, z), pt_cv_x1(y - radious_, x - radious_, z), pt_cv_x2(y + radious_, x + radious_, z);
    cv::Point2d uv, uv_x1, uv_x2;
    uv = project3d_to_pixel(pt_cv, cinfo);
    uv_x1 = project3d_to_pixel(pt_cv_x1, cinfo);
    uv_x2 = project3d_to_pixel(pt_cv_x2, cinfo);
    ROS_INFO_STREAM("uv.x: " << uv.x);
    ROS_INFO_STREAM("uv.y: " << uv.y);
    double scale = 1;
    if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
    uv.y < (rgb_image.rows / scale))
    {
        cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(uv.y, uv.x);
        // cv::rotate(draw_img, draw_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::circle(draw_img, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 0, 255), 3, 1);
        cv::rectangle(draw_img, cv::Point(uv_x1.x, uv_x1.y), cv::Point(uv_x2.x, uv_x2.y), cv::Scalar(255, 0, 0), 4);
    }
}

void Annotation_yolo::box_get(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, std::vector<geometry_msgs::TransformStamped> trans_s, cv::Mat &draw_img, int work_count)
{
    ROS_INFO_STREAM("hatayu");
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
        // return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    draw_img = cv_image;
    
    int count = 0;
    int aida = 100;

    for (int i = 0; i < work_count; i++) {
        float x = trans_s[i].transform.translation.y;
        float y = trans_s[i].transform.translation.x;
        float z = trans_s[i].transform.translation.z;
        cv::Point3d pt_cv(y, x, z), pt_cv_x1(y - radious_, x - radious_, z), pt_cv_x2(y + radious_, x + radious_, z);
        cv::Point2d uv, uv_x1, uv_x2;
        uv = project3d_to_pixel(pt_cv, cinfo);
        uv_x1 = project3d_to_pixel(pt_cv_x1, cinfo);
        uv_x2 = project3d_to_pixel(pt_cv_x2, cinfo);
        double scale = 1;
        if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
        uv.y < (rgb_image.rows / scale))
        {
            cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(uv.y, uv.x);
            ROS_INFO_STREAM("image_" << i);
            // cv::rotate(draw_img, draw_img, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::circle(draw_img, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 0, 255), 3, 1);
            cv::rectangle(draw_img, cv::Point(uv_x1.x, uv_x1.y), cv::Point(uv_x2.x, uv_x2.y), cv::Scalar(255, 0, 0), 4);
        }
    }
}

cv::Point2d Annotation_yolo::project3d_to_pixel(cv::Point3d xyz, sensor_msgs::CameraInfo cinfo_msg)
{
    cv::Point2d uv_rect;
    fx_ = cinfo_msg.K[0] * f_scale_;
    tx_ = cinfo_msg.K[1];
    cx_ = cinfo_msg.K[2] * cx_scale_;
    fy_ = cinfo_msg.K[4] * f_scale_;
    ty_ = cinfo_msg.K[3];
    cy_ = cinfo_msg.K[5] * cy_scale_;
    uv_rect.x = (fx_ * xyz.x + tx_) / xyz.z + cx_;
    uv_rect.y = (fy_ * xyz.y + ty_) / xyz.z + cy_;
    return uv_rect;
}

void Annotation_yolo::paramter_set_bara(std::string base_tf_frame, int work_count)
{
    for (int i = 0; i < work_count; i++) {
        target_frames_.push_back(base_tf_frame + "_" + std::to_string(i));
    }
}