#include <tf_publish/tf_lister.h>

ListenTest::ListenTest() : nh_(), tfBuffer_(), tfListener_(tfBuffer_)
{
    timer_ = nh_.createTimer(ros::Duration(0.5), &ListenTest::timercallback, this);
}

void ListenTest::timercallback(const ros::TimerEvent &e)
{
    geometry_msgs::TransformStamped tfst;
    try {
        tfst = tfBuffer_.lookupTransform("base_link", "dynamic_tf", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_STREAM(ex.what());
        return;
    }
    auto& trans = tfst.transform.translation;
    ROS_INFO_STREAM("world->dynamic_tf: " << trans.x << " " << trans.y << " " << trans.z);
    geometry_msgs::Pose object_d, object_w;
    object_d.position.z = 1.0;
    object_d.orientation.w = 1.0;
    tf2::doTransform(object_d, object_w, tfst);
    ROS_INFO("object_w x: %f, y:%f, z:%f", object_w.position.x, object_w.position.y, object_w.position.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf2_listener");
    ListenTest listen_test;
    ros::spin();
    return 0;
}