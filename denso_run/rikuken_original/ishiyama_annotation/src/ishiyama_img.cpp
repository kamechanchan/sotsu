#include <ishiyama_annotation/ishiyama_img.h>

GetDataNode::GetDataNode()
{
    ROS_INFO_STREAM("start");
    photoneo_img_sub_ = nh_.subscribe("/photoneo_center/sensor/image_color", 1000, &GetDataNode::callback_, this);
    server_ = nh_.advertiseService("input_img", &GetDataNode::inputData, this);
}

void GetDataNode::callback_(const sensor_msgs::Image& msg){
    data_ = msg;
    ROS_INFO_STREAM("get_photoneo_image");
}

bool GetDataNode::inputData(estimator::input_data::Request &req, estimator::input_data::Response &res){
    // res.out_img = req.in_img;
    res.out_img = data_;
    ROS_INFO_STREAM("success");
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ishinyam_1");
    GetDataNode tanomuze;
    ros::spin();
}
