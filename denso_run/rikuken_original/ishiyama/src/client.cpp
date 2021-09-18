#include <client.hpp>

GetDataNode::GetDataNode()
: sub_from_sensor_(nh_.subscribe("/photoneo_center/sensor/image_color", 1000, &GetDataNode::callback, this))
{
    ROS_INFO_STREAM("start");
}

void GetDataNode::callback(const sensor_msgs::Image& in_img){
    in_img_ = in_img;
    convertImgFromRostoTensor();
}

void GetDataNode::convertImgFromRostoTensor(){
    save_img_path_ = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/ishiyama/result/ishiyama.jpg";
    cv_img_ = cv_bridge::toCvCopy(in_img_, sensor_msgs::image_encodings::BGR8);
    out_img_ = cv_img_->image;
    ROS_INFO_STREAM("img_size:" << out_img_.size());
    cv::imwrite(save_img_path_, out_img_);
    
}