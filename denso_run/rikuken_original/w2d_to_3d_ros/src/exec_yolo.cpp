#include <w2d_to_3d_ros/exec_yolo.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>

#define PI 3.141592

std::mutex m;
Exec_yolo::Exec_yolo(ros::NodeHandle &nh) :
    nh_(nh),
    radious_(0.05),
    save_count_(0),
    work_count_(0),
    pnh_("~"),
    buffer_(),
    lister_(buffer_),
    shori_count(0)
{
    parameter_set();
}

void Exec_yolo::parameter_set()
{
    pnh_.getParam("camera_topic_name", camera_topic_name_);
    pnh_.getParam("image_topic_name", image_topic_name_);
    pnh_.getParam("source_frame", source_frame_);
    pnh_.getParam("target_frame", target_frame_);
    pnh_.getParam("f_scale", f_scale_);
    pnh_.getParam("cx_scale", cx_scale_);
    pnh_.getParam("cy_scale", cy_scale_);
    pnh_.getParam("radious", radious_);
    pnh_.getParam("work_count", work_count_);
    pnh_.getParam("model_name", model_name_);
    pnh_.getParam("world_frame", world_frame_);
    pnh_.getParam("the_number_of_data", the_number_of_data);
    pnh_.getParam("inputcloud_topic_name", inputcloud_topic_name_);
    pnh_.getParam("output_topic_name", output_topic_name_);
    pnh_.getParam("dulation", dulation_);
    paramter_set_bara(model_name_, work_count_);
    output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_, 10);
    camera_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_topic_name_, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 10);
    sensor_sync_ = new message_filters::Synchronizer<Sync_Sub_type>(Sync_Sub_type(10), *camera_sub_, *image_sub_);
    sensor_sync_->registerCallback(boost::bind(&Exec_yolo::InputCallback, this, _1, _2));
}

void Exec_yolo::InputCallback(sensor_msgs::CameraInfoConstPtr cam_msgs, sensor_msgs::ImageConstPtr image_msgs)
{
    // ROS_INFO_STREAM("message come");
    // std::lock_guard<std::mutex> lock(m);
    shori_count = shori_count + 1;
    sensor_msgs::CameraInfo cinfo = *cam_msgs;
    sensor_msgs::Image image1 = *image_msgs;
    pcl::PointCloud<pcl::PointXYZ> trans_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    sensor_msgs::PointCloud2 cloud_msgs;
    
    // ROS_INFO_STREAM("message1 come");
    ROS_INFO_STREAM("tsuchida_first: " << shori_count);
    
    
    geometry_msgs::TransformStamped trans_source;
    cv::Mat ori_img;
    
    tf_get(world_frame_, source_frame_, trans_source);
    // ROS_INFO_STREAM("message2 come");
    // ros::WallTime start_w = ros::WallTime::now();
    start_w.push_back(ros::WallTime::now());
    ROS_INFO_STREAM("tsuchida_second: " << shori_count);
    // ROS_INFO_STREAM("start_time " << shori_count << ": " << start_w.toSec());
    get_one_message(cloud_msgs, inputcloud_topic_name_, nh_, 10);
    ROS_INFO_STREAM("tsuchida_third: " << shori_count);
    pcl::fromROSMsg(cloud_msgs, trans_cloud);
    ROS_INFO_STREAM("tsuchida_forth: " << shori_count);
    // ros::WallTime totyu_w = ros::WallTime::now();
    totyu_w.push_back(ros::WallTime::now());
    int i = totyu_w.size() - 1;
    ros::WallDuration take_totyuu_time_w = totyu_w[i] - start_w[i]; 
    // ros::WallDuration take_totyuu_time_w = totyu_w - start_w;
    ROS_INFO_STREAM("totyuu time: " << take_totyuu_time_w.toSec());
    ROS_INFO_STREAM("tsuchida_fifth: " << shori_count);

    std::vector<geometry_msgs::TransformStamped> transforms;
    for (int i = 0; i < work_count_; i++) {
        geometry_msgs::TransformStamped trans;
        tf_get(world_frame_, target_frames_[i], trans);
        transforms.push_back(trans);
    }
    // ROS_INFO_STREAM("message3 come");
    ROS_INFO_STREAM("tsuchida_six: " << shori_count);
    std::vector<cv::Point3d> point_3d;
    ROS_INFO_STREAM("tsuchida_seven" << shori_count);
    rotation_convert(trans_source, transforms, point_3d);
    std::vector<std::vector<cv::Point2d>> uv_points;
    ROS_INFO_STREAM("tsuchida_eight" << shori_count);
    box_get(cinfo, image1, point_3d, draw_image_, uv_points);
    ROS_INFO_STREAM("tsuchida_nine" << shori_count);
    // ROS_INFO_STREAM("message4 come");

    // for (int i = 0; i < draw_image_.rows; i++) {
    //     std::vector<int> iim;
    //     for (int j = 0; j < draw_image_.cols; j++) {
    //         iim.push_back(0);
    //     }
    //     image_instance_.push_back(iim);
    // }
    // ROS_INFO_STREAM("message5 come");
    // write_instance(uv_points, image_instance_);
    image_instance_ = write_instance(uv_points, draw_image_);
    ROS_INFO_STREAM("tsuchida_ten" << shori_count);
    // ROS_INFO_STREAM("message6 come");

    get_original_image(image1, ori_img);
    // ROS_INFO_STREAM("message7 come");

    hurui(trans_cloud, image_instance_, image1, cinfo, color_cloud);
    ROS_INFO_STREAM("tsuchida_eleven" << shori_count);
    // ros::WallTime end_w = ros::WallTime::now();
    end_w.push_back(ros::WallTime::now());
    int j = end_w.size() - 1;
    ros::WallDuration time_2 = end_w[j] - totyu_w[j];
    ROS_INFO_STREAM("2 time: " << time_2.toSec());
    ros::WallDuration take_time_w = end_w[j] - start_w[j];
    ROS_INFO_STREAM("finaltime: " << take_time_w.toSec());
    ROS_INFO_STREAM("");
    // ROS_INFO_STREAM("message8 come");
    pcl::toROSMsg(color_cloud, output_cloud_msgs_);
    output_cloud_msgs_.header.frame_id = cloud_msgs.header.frame_id;
    output_pub_.publish(output_cloud_msgs_);
    cv::resize(draw_image_, draw_image_, cv::Size(), 0.7, 0.7);

    cv::imshow("windoue", draw_image_);
    cv::waitKey(10);
    
}

void Exec_yolo::tf_get(std::string source_frame, std::string target_frame, geometry_msgs::TransformStamped &trans)
{
    try
    {
        trans = buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
        ROS_INFO_ONCE("I got a transform");
    }
    catch (tf2::TransformException &e)
    {
        ROS_WARN_STREAM(e.what());
        buffer_.clear();
        ros::Duration(dulation_).sleep();
        return;
    }
}

void Exec_yolo::box_get(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, std::vector<cv::Point3d> trans_s, cv::Mat &draw_img, std::vector<std::vector<cv::Point2d>> &uv_points)
{
    // ROS_INFO_STREAM("hatayu");
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
    std::cout << "trans s size " << trans_s.size() << std::endl;
    for (int i = 0; i < trans_s.size(); i++) {
        double x = trans_s[i].x;
        double y = trans_s[i].y;
        double z = trans_s[i].z;
        cv::Point3d pt_cv(y, x, z), pt_cv_x1(y - radious_, x - radious_, z), pt_cv_x2(y + radious_, x + radious_, z);
        cv::Point2d uv, uv_x1, uv_x2;
        uv = project3d_to_pixel(pt_cv, cinfo);
        uv_x1 = project3d_to_pixel(pt_cv_x1, cinfo);
        uv_x2 = project3d_to_pixel(pt_cv_x2, cinfo);
        // ROS_INFO_STREAM(uv.x << "  " << uv.y << "   imgasize" << rgb_image.cols << "  " << rgb_image.rows);
        double scale = 1;
        if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
        uv.y < (rgb_image.rows / scale))
        {
            cv::circle(draw_img, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 0, 255), 3, 1);
            
            cv::rectangle(draw_img, cv::Point(uv_x1.x, uv_x2.y), cv::Point(uv_x2.x, uv_x1.y), cv::Scalar(0, 255, 0), 3);
            cv::circle(draw_img, cv::Point(uv_x1.x, uv_x2.y), 8, cv::Scalar(255, 255, 255), 3, 1);
            cv::circle(draw_img, cv::Point(uv_x2.x, uv_x1.y), 8, cv::Scalar(0, 0, 0), 3, 1);
            std::vector<cv::Point2d> uv_s;
            cv::Point2d new_x1(uv_x1.x, uv_x2.y), new_x2(uv_x2.x, uv_x1.y);
            uv_s.push_back(new_x1);
            uv_s.push_back(new_x2);
            // ROS_INFO_STREAM("new.x: " << new_x1.x << " x1.y: " << new_x1.y << "    x2.x: " << new_x2.x << " x2.y: " << new_x2.y);
            uv_points.push_back(uv_s);
        }
    }
    // std::cout << "uv_points size " << uv_points.size() << std::endl;
}

cv::Point2d Exec_yolo::project3d_to_pixel_origin(cv::Point3d xyz, sensor_msgs::CameraInfo cinfo_msg)
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

cv::Point2d Exec_yolo::project3d_to_pixel(cv::Point3d xyz, sensor_msgs::CameraInfo cinfo_msg)
{
    cv::Point2d uv_rect;
    fx_ = cinfo_msg.K[0] * f_scale_;
    tx_ = cinfo_msg.K[1];
    cx_ = cinfo_msg.K[2] * cx_scale_;
    fy_ = cinfo_msg.K[4] * f_scale_;
    ty_ = cinfo_msg.K[3];
    cy_ = cinfo_msg.K[5] * cy_scale_;
    uv_rect.x = (fx_ * xyz.x + tx_) / xyz.z + cx_;
    uv_rect.y = (fy_ * -xyz.y + ty_) / xyz.z + cy_;
    return uv_rect;
}

void Exec_yolo::paramter_set_bara(std::string base_tf_frame, int work_count)
{
    for (int i = 0; i < work_count; i++) {
        target_frames_.push_back(base_tf_frame + "_" + std::to_string(i));
    }
}

void Exec_yolo::rotation_convert(geometry_msgs::TransformStamped source_tf, std::vector<geometry_msgs::TransformStamped> target_tfs, std::vector<cv::Point3d> &output_3d)
{
    geometry_msgs::Vector3 source_trans = source_tf.transform.translation;
    geometry_msgs::Quaternion source_quat = source_tf.transform.rotation;
    tf2::Quaternion q_rot;
    tf2::convert(source_quat, q_rot);
    std::cout << "rotation_convert target tf" << target_tfs.size() << std::endl;

    for (int i = 0; i < target_tfs.size(); i++) {
        geometry_msgs::Vector3 target_trans = target_tfs[i].transform.translation;
        geometry_msgs::Quaternion target_quat = target_tfs[i].transform.rotation;
        tf2::Quaternion target_q_before(target_trans.x, target_trans.y, target_trans.z, 0);
        tf2::Quaternion target_rot;
        tf2::convert(target_quat, target_rot);
        tf2::Quaternion target_after = target_rot.inverse() * target_q_before * target_rot;
        
        double x = -target_after.x() - source_trans.x;
        double y = -target_after.y() - source_trans.y;
        double z = -target_after.z() - source_trans.z;
        tf2::Quaternion q_before(x, y, z, 0);
        tf2::Quaternion q_after = q_rot.inverse() * q_before * q_rot;
        tf2::Quaternion q_rotation;
        q_rotation.setRPY(0, 0, PI / 2);
        q_after = q_rotation * q_after * q_rotation.inverse();
        cv::Point3d out(q_after.x(), q_after.y(), q_after.z());
        output_3d.push_back(out);
    }
}

void Exec_yolo::get_original_image(sensor_msgs::Image image1, cv::Mat &original_img)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
    }
    original_img = cv_img_ptr->image;
}

void Exec_yolo::write_instance(std::vector<std::vector<cv::Point2d>> point_2d, std::vector<std::vector<int>> &instance)
{
    
    std::cout << "pointsize " << point_2d.size() << std::endl;
    for (int i = 0; i < point_2d.size(); i++) {
        int x1 = static_cast<int>(point_2d[i][0].x);
        int x2 = static_cast<int>(point_2d[i][1].x);
        int y1 = static_cast<int>(point_2d[i][0].y);
        int y2 = static_cast<int>(point_2d[i][1].y);
        if (x1 > x2) {
            swap(x1, x2);
        }
        if (y1 > y2) {
            swap(y1, y2);
        }
        int count = 0;
        std::cout << x1 << " " << x2 << " " << y1 << " " << y2 << std::endl;
        for (int k = y1; k <= y2; k++) 
        {
            for (int l = x1; l <= x2; l++) {
                instance[k][l] = 1;
                // std::cout << count << std::endl;
                // count++;
            }
        }

    }
}

std::vector<std::vector<int>> Exec_yolo::write_instance(std::vector<std::vector<cv::Point2d>> point_2d, cv::Mat img_size_img)
{
    std::vector<std::vector<int>> instance_img_sasyo;
    for (int i = 0; i < img_size_img.rows; i++) {
        std::vector<int> iim;
        for (int j = 0; j < img_size_img.cols; j++) {
            iim.push_back(0);
        }
        instance_img_sasyo.push_back(iim);
    }
    std::cout << "pointsize " << point_2d.size() << std::endl;
    for (int i = 0; i < point_2d.size(); i++) {
        int x1 = static_cast<int>(point_2d[i][0].x);
        int x2 = static_cast<int>(point_2d[i][1].x);
        int y1 = static_cast<int>(point_2d[i][0].y);
        int y2 = static_cast<int>(point_2d[i][1].y);
        if (x1 > x2) {
            swap(x1, x2);
        }
        if (y1 > y2) {
            swap(y1, y2);
        }
        int count = 0;
        std::cout << x1 << " " << x2 << " " << y1 << " " << y2 << std::endl;
        for (int k = y1; k <= y2; k++) 
        {
            for (int l = x1; l <= x2; l++) {
                instance_img_sasyo[k][l] = 1;
                // std::cout << count << std::endl;
                // count++;
            }
        }

    }
    return instance_img_sasyo;
}

void Exec_yolo::hurui(pcl::PointCloud<pcl::PointXYZ> input_pcl_cloud, std::vector<std::vector<int>> instance, sensor_msgs::Image image, sensor_msgs::CameraInfo cinfo, pcl::PointCloud<pcl::PointXYZRGB> &outcloud_pcl_cloud)
{
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
    int coun = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = input_pcl_cloud.points.begin(); pt != input_pcl_cloud.points.end(); ++pt)
    {
        if (pt->z < 0)
        {
            ROS_INFO_STREAM("pass point");
            continue;
        }
        cv::Point3d pt_cv(pt->x, pt->y, pt->z);
        cv::Point2d uv;
        uv = project3d_to_pixel_origin(pt_cv, cinfo);
       // cv::Mat rgb_image;
        //cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
        // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        
        double scale = 1;
        if (uv.x > (-rgb_image.cols / scale) && uv.y < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale)
            && uv.y < (rgb_image.rows / scale))
        {
            
            pcl::PointXYZRGB buffer_point;
            int x = static_cast<int>(uv.x);
            int y = static_cast<int>(uv.y);

            if (instance[y][x] == 1) {
                cv::Vec3d rgb = rgb_image.at<cv::Vec3b>(y, x);
                // ROS_INFO_STREAM("x: " << x << "   y: " << y);
                // ROS_INFO_STREAM("r: " << rgb[0] << "  g: " << rgb[1] << "  b: " << rgb[2]);
                buffer_point.x = pt->x;
                buffer_point.y = pt->y;
                buffer_point.z = pt->z;
                buffer_point.r = rgb[0];
                buffer_point.g = rgb[1];
                buffer_point.b = rgb[2];
                outcloud_pcl_cloud.push_back(buffer_point);
            }
        }
    }
}
