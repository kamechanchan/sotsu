#include <tf_publish/model_tf_publisher_bara.h>
#include <algorithm>

using model_tf_publisher_bara::Model_bara;

Model_bara::Model_bara(ros::NodeHandle &nh) : nh_(nh)
{
    pnh_ = new ros::NodeHandle("~");
    pnh_->getParam("src_frame_name", src_frame_);
    pnh_->getParam("object_name", object_name_);
    model_state_sub_ = nh_.subscribe("/gazebo/model_states", 10, &Model_bara::modelstateCallback, this);
    gazebo_model_name_.clear();
    gazebo_model_poses_.clear();
}

void Model_bara::modelstateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    gazebo_model_name_ = msg->name;
    gazebo_model_poses_ = msg->pose;
}