#include <tf_publish/model_tf_publisher_bara.h>
#include <algorithm>

using model_tf_publisher_bara::Model_bara;

Model_bara::Model_bara(ros::NodeHandle &nh) : nh_(nh)
{
    pnh_ = new ros::NodeHandle("~");
    src_frame_ = "world";
    pnh_->getParam("src_frame_name", src_frame_);
    pnh_->getParam("object_name", object_name_);
    pnh_->getParam("object_count", OBJECT_COUNT);
    model_state_sub_ = nh_.subscribe("/gazebo/model_states", 10, &Model_bara::modelstateCallback, this);
    gazebo_model_name_.clear();
    gazebo_model_poses_.clear();
    make_urdf_model_name();
}

void Model_bara::modelstateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    gazebo_model_name_ = msg->name;
    gazebo_model_poses_ = msg->pose;
    //print_para(gazebo_model_name_);
}

int Model_bara::getModelId(std::string urdf_name)
{
    auto iter = std::find(gazebo_model_name_.begin(), gazebo_model_name_.end(), urdf_name);
    int index = std::distance(gazebo_model_name_.begin(), iter);
    if (iter == gazebo_model_name_.end())
    {
        ROS_WARN_STREAM("Not found " << urdf_name << "!!");
        return -1;
    }
    else {
        return index;
    }
}

void Model_bara::make_urdf_model_name()
{
    for (int i = 0; i < OBJECT_COUNT; i++) {
        urdf_model_name_.push_back(object_name_ + "_" + std::to_string(i));
    }
}

void Model_bara::broadcastModelTF()
{
    for (int i = 0; i < OBJECT_COUNT; i++) {
        int index = getModelId(urdf_model_name_[i]);
        if (index < 0)
        {
            ROS_WARN_STREAM(urdf_model_name_[i] << "is not Exsistense");
        }
        else {
            transform_stamp_.header.stamp = ros::Time::now();
            transform_stamp_.header.frame_id = src_frame_;
            transform_stamp_.child_frame_id = gazebo_model_name_[index];
            geometry_msgs::Transform tr;
            tr.translation.x = gazebo_model_poses_[index].position.x;
            tr.translation.y = gazebo_model_poses_[index].position.y;
            tr.translation.z = gazebo_model_poses_[index].position.z;
            tr.rotation.x = gazebo_model_poses_[index].orientation.x;
            tr.rotation.y = gazebo_model_poses_[index].orientation.y;
            tr.rotation.z = gazebo_model_poses_[index].orientation.z;
            tr.rotation.w = gazebo_model_poses_[index].orientation.w;
            transform_stamp_.transform = tr;
            br_.sendTransform(transform_stamp_);
            ROS_INFO_STREAM("Broadcast " << urdf_model_name_[i] << "TF !!");
        }
    }


}