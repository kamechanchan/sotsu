#include <2D_to_3D.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <iostream>


YoloBridge::YoloBridge(ros::NodeHandle &nh) :
    nh_(nh),
    radious_(0.05),
    pnh_("~"),
    buffer_(),
    lister_(buffer_)
{
    parameter_set();
}

void YoloBridge::callback(sensor_msgs::CameraInfoConstPtr cam_msgs)
{
    cinfo_ = *cam_msgs;
    get_one_message(cloud_msgs_, inputcloud_topic_name_, nh_, timespan_);
    
    ROS_INFO_STREAM("start" << "kuruze");
    ROS_INFO_STREAM(cinfo_);
    ROS_INFO_STREAM("kokoka");
    get_one_message<color_cloud_bridge::yolo_bridge>(msg_data_, YOLO_topic_name_, nh_, timespan_);
    ROS_INFO_STREAM("iijanai");
    pcl::fromROSMsg(cloud_msgs_, trans_cloud_);
    // ROS_INFO_STREAM(msg_data_.out_data[1]);
    // ROS_INFO_STREAM("koreka: " << size(msg_data_.output_img));
    cv_bridge::CvImageConstPtr cv_img_ptr;
    cv_img_ptr = cv_bridge::toCvCopy(msg_data_.output_img, sensor_msgs::image_encodings::TYPE_8UC3);
    ROS_INFO_STREAM("kitaka");
    for (int i = 0; i < cv_img_ptr->image.rows; i++) {
        std::vector<int> iim;
        for (int j = 0; j < cv_img_ptr->image.cols; j++) {
            iim.push_back(0);
        }
        image_instance_.push_back(iim);
    }
    ROS_INFO_STREAM("majimaji");
    cv::Mat draw_img_(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    draw_img_ = cv_img_ptr->image;
    ROS_INFO_STREAM("douda");
    // float a;
    cv::Point2d points_1;
    cv::Point2d points_2;
    for (int i=0; i<msg_data_.out_data.size(); i++){
        std::vector<cv::Point2d> init_1;
        points_1.x = msg_data_.out_data[i].data[0];
        points_1.y = msg_data_.out_data[i].data[1];
        points_2.x = msg_data_.out_data[i].data[2];
        points_2.y = msg_data_.out_data[i].data[3];
        init_1.push_back(points_1);
        init_1.push_back(points_2);
        uv_points_.push_back(init_1);
    }

    // for (int i=0; i<msg_data_.out_data.size(); i++){
    //     ROS_INFO_STREAM("iissune" << msg_data_.out_data[i].data[0]);
    //     // uv_points_[i][0].x = msg_data_.out_data[i].data[0];
    //     // uv_points_[i][0].y = msg_data_.out_data[i].data[1];
    //     // uv_points_[i][1].x = msg_data_.out_data[i].data[2];
    //     // uv_points_[i][1].y = msg_data_.out_data[i].data[3];
    //     points.x = msg_data_.out_data[i].data[0];
    //     points.y = msg_data_.out_data[i].data[1];
    //     // a = msg_data_.out_data[i].data[0];
    //     // a = msg_data_.out_data[i].data[1];
    //     // a = msg_data_.out_data[i].data[2];
    //     // a = msg_data_.out_data[i].data[3];
    // }

    ROS_INFO_STREAM("warukuhawa");
    image_instance_ = write_instance(uv_points_, draw_img_);
    ROS_INFO_STREAM("majikmajhfijadfjafjk;dafj");
    hurui(trans_cloud_, image_instance_, msg_data_.input_img, cinfo_, color_cloud);
    pcl::toROSMsg(color_cloud, output_cloud_msgs_);
    // pcl::PointCloud<pcl::PointXYZ> ishiyama;
    // pcl::fromROSMsg(output_cloud_msgs_, ishiyama);
    output_cloud_msgs_.header.frame_id = cloud_msgs_.header.frame_id;
    output_pub_.publish(output_cloud_msgs_);
    // ROS_INFO_STREAM("iyagarasedesuka");
    // ROS_INFO_STREAM(ishiyama.size());
    // ROS_INFO_STREAM(color_cloud);
    // ROS_INFO_STREAM(output_cloud_msgs_);
    // client_ = nh_.serviceClient<estimator::bounding>("bounding_box");
    // // srv_data_ = estimator::bounding();
    // srv_data_.request.cloud_in = output_cloud_msgs_;
    // auto segment_data_ = client_.call(srv_data_);
    // {
    //     ROS_INFO_STREAM("success");
    // }
    // else 
    // {
    //     ROS_INFO_STREAM("fail")
    // }

    // cv::Mat cv_image(draw_image_->image.rows, draw_image_->image.cols, draw_image_->image.type());
    // cv_image = draw_image_->image;
    // cv::cvtColor(cv_image, rgb_img, cv::)
    // ROS_INFO_STREAM(cinfo_);
}

void YoloBridge::parameter_set()
{
    pnh_.getParam("camera_topic_name", camera_topic_name_);
    pnh_.getParam("f_scale", f_scale_);
    pnh_.getParam("cx_scale", cx_scale_);
    pnh_.getParam("cy_scale", cy_scale_);
    pnh_.getParam("radious", radious_);
    pnh_.getParam("model_name", model_name_);
    pnh_.getParam("the_number_of_data", the_number_of_data);
    pnh_.getParam("YOLO_topic_name", YOLO_topic_name_);
    pnh_.getParam("inputcloud_topic_name", inputcloud_topic_name_);
    pnh_.getParam("output_topic_name", output_topic_name_);
    pnh_.getParam("timespan", timespan_);
    output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_, 10);
    sub_ = nh_.subscribe(camera_topic_name_, 10, &YoloBridge::callback, this);
    // get_one_message(cinfo_, camera_topic_name_, nh_, timespan_);
    // get_one_message(cloud_msgs_, inputcloud_topic_name_, nh_, timespan_);
}

// void YoloBridge::major()
// {
//     ROS_INFO_STREAM("start" << "kuruze");
//     ROS_INFO_STREAM(cinfo_);
//     ROS_INFO_STREAM("kokoka");
//     get_one_message<color_cloud_bridge::yolo_bridge>(msg_data_, YOLO_topic_name_, nh_, timespan_);
//     ROS_INFO_STREAM("iijanai");
//     pcl::fromROSMsg(cloud_msgs_, trans_cloud_);
//     // ROS_INFO_STREAM(msg_data_.out_data[1]);
//     // ROS_INFO_STREAM("koreka: " << size(msg_data_.output_img));
//     cv_bridge::CvImageConstPtr cv_img_ptr;
//     cv_img_ptr = cv_bridge::toCvCopy(msg_data_.output_img, sensor_msgs::image_encodings::TYPE_8UC3);
//     ROS_INFO_STREAM("kitaka");
//     for (int i = 0; i < cv_img_ptr->image.rows; i++) {
//         std::vector<int> iim;
//         for (int j = 0; j < cv_img_ptr->image.cols; j++) {
//             iim.push_back(0);
//         }
//         image_instance_.push_back(iim);
//     }
//     ROS_INFO_STREAM("majimaji");
//     cv::Mat draw_img_(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
//     draw_img_ = cv_img_ptr->image;
//     ROS_INFO_STREAM("douda");
//     // float a;
//     cv::Point2d points_1;
//     cv::Point2d points_2;
//     for (int i=0; i<msg_data_.out_data.size(); i++){
//         std::vector<cv::Point2d> init_1;
//         points_1.x = msg_data_.out_data[i].data[0];
//         points_1.y = msg_data_.out_data[i].data[1];
//         points_2.x = msg_data_.out_data[i].data[2];
//         points_2.y = msg_data_.out_data[i].data[3];
//         init_1.push_back(points_1);
//         init_1.push_back(points_2);
//         uv_points_.push_back(init_1);
//     }

//     // for (int i=0; i<msg_data_.out_data.size(); i++){
//     //     ROS_INFO_STREAM("iissune" << msg_data_.out_data[i].data[0]);
//     //     // uv_points_[i][0].x = msg_data_.out_data[i].data[0];
//     //     // uv_points_[i][0].y = msg_data_.out_data[i].data[1];
//     //     // uv_points_[i][1].x = msg_data_.out_data[i].data[2];
//     //     // uv_points_[i][1].y = msg_data_.out_data[i].data[3];
//     //     points.x = msg_data_.out_data[i].data[0];
//     //     points.y = msg_data_.out_data[i].data[1];
//     //     // a = msg_data_.out_data[i].data[0];
//     //     // a = msg_data_.out_data[i].data[1];
//     //     // a = msg_data_.out_data[i].data[2];
//     //     // a = msg_data_.out_data[i].data[3];
//     // }

//     ROS_INFO_STREAM("warukuhawa");
//     image_instance_ = write_instance(uv_points_, draw_img_);
//     ROS_INFO_STREAM("majikmajhfijadfjafjk;dafj");
//     hurui(trans_cloud_, image_instance_, msg_data_.input_img, cinfo_, color_cloud);
//     pcl::toROSMsg(color_cloud, output_cloud_msgs_);
//     output_cloud_msgs_.header.frame_id = cloud_msgs_.header.frame_id;
//     output_pub_.publish(output_cloud_msgs_);
//     // ROS_INFO_STREAM(color_cloud);
//     // ROS_INFO_STREAM(output_cloud_msgs_);


//     // cv::Mat cv_image(draw_image_->image.rows, draw_image_->image.cols, draw_image_->image.type());
//     // cv_image = draw_image_->image;
//     // cv::cvtColor(cv_image, rgb_img, cv::)

// }

std::vector<std::vector<int>> YoloBridge::write_instance(std::vector<std::vector<cv::Point2d>> point_2d, cv::Mat img_size_img)
{
    std::vector<std::vector<int>> instance_img_sasyo;
    for (int i = 0; i < img_size_img.rows; i++) {
        std::vector<int> iim;
        for (int j = 0; j < img_size_img.cols; j++) {
            iim.push_back(0);
        }
        instance_img_sasyo.push_back(iim);
    }
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
        for (int k = y1; k <= y2; k++) 
        {
            for (int l = x1; l <= x2; l++) {
                instance_img_sasyo[k][l] = 1;
            }
        }

    }
    return instance_img_sasyo;
}

void YoloBridge::hurui(pcl::PointCloud<pcl::PointXYZ> input_pcl_cloud, std::vector<std::vector<int>> instance, sensor_msgs::Image image, sensor_msgs::CameraInfo cinfo, pcl::PointCloud<pcl::PointXYZ> &outcloud_pcl_cloud)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC3);
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
    srv_ = estimator::bounding();
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = input_pcl_cloud.points.begin(); pt != input_pcl_cloud.points.end(); ++pt)
    {
        // coun++;
        // ROS_INFO_STREAM(coun);
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
            
            pcl::PointXYZ buffer_point;
            int x = static_cast<int>(uv.x);
            int y = static_cast<int>(uv.y);

            if (instance[y][x] == 1) {
                cv::Vec3d rgb = rgb_image.at<cv::Vec3b>(y, x);
                // ROS_INFO_STREAM("x: " << x << "   y: " << y);
                // ROS_INFO_STREAM("r: " << rgb[0] << "  g: " << rgb[1] << "  b: " << rgb[2]);
                buffer_point.x = pt->x;
                buffer_point.y = pt->y;
                buffer_point.z = pt->z;
                srv_.request.x.push_back(pt->x);
                srv_.request.y.push_back(pt->y);
                srv_.request.z.push_back(pt->z);

                
                // buffer_point.r = rgb[0];
                // buffer_point.g = rgb[1];
                // buffer_point.b = rgb[2];
                outcloud_pcl_cloud.push_back(buffer_point);
            }
        }
    }
    client_ = nh_.serviceClient<estimator::bounding>("bounding_box");
    // srv_data_ = estimator::bounding();
    // srv_.request.cloud_in = output_cloud_msgs_;
    auto segment_data_ = client_.call(srv_);
    // ROS_INFO_STREAM("tanomuze");
    // ROS_INFO_STREAM(outcloud_pcl_cloud.size());
}

cv::Point2d YoloBridge::project3d_to_pixel_origin(cv::Point3d xyz, sensor_msgs::CameraInfo cinfo_msg)
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

// bool inputData(estimator::bounding::Request &req, estimator::bounding::Response &res)
// {
//     res.out_img = req.in_img;
//     return true;
// }

int main(int argc, char** argv)
{
    std::cout << "iaipa";
    ros::init(argc, argv, "tuschda_inti");
    ros::NodeHandle nh;
    YoloBridge Ishiyama(nh);
    ros::spin();
}