#include <server_2D.h>

GetDataNode::GetDataNode()
// : sub_from_sensor_(nh_.subscribe("/photoneo_center/sensor/image_color", 1000, &GetDataNode::callback, this))
{
    ROS_INFO_STREAM("start");
    save_img_path_ = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/ishiyama/result/ishiyama.jpg";
    photoneo_img_sub_ = nh_.subscribe("/photoneo_center/sensor/image_color", 1000, &GetDataNode::callback_, this);
    server_ = nh_.advertiseService("ishiyama_input_data", &GetDataNode::inputData, this);
}

// void GetDataNode::callback(const sensor_msgs::Image& in_img){
    // in_img_ = in_img;
    // server_ = nh_.advertiseService("ishiyama_input_data", &inputData);
    // send_data_server();
// }

// void GetDataNode::send_data_server(){
//     save_img_path_ = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/ishiyama/result/ishiyama.jpg";
//     cv_img_ = cv_bridge::toCvCopy(in_img_, sensor_msgs::image_encodings::BGR8);
//     out_img_ = cv_img_->image;
//     ROS_INFO_STREAM("img_size:" << out_img_.size());
//     cv::imwrite(save_img_path_, out_img_);

// }

void GetDataNode::callback_(const sensor_msgs::Image& msg){
    data_ = msg;
    ROS_INFO_STREAM("get_photoneo_image");
}

bool GetDataNode::inputData(estimator::input_data::Request &req, estimator::input_data::Response &res){
    // cv_img_ = cv_bridge::toCvCopy(req.in_img, sensor_msgs::image_encodings::BGR8);
    // res.img_to_box = cv_img_->image;
    // ROS_INFO_STREAM("img_size:" << res.img_to_box.size());
    // cv::imwrite(save_img_path_, out_img_);
    // return true;
    // photoneo_img_sub_ = nh_.subscribe("/photoneo_center/sensor/image_color", 1000, GetDataNode::callback_);
    res.out_img = data_;
    ROS_INFO_STREAM("success!!!!!");
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ishinyam_1");
    GetDataNode tanomuze;
    ros::spin();
}
