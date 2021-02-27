#include <tf_publish/tf_broadcaster.h>

BroadCasterTest::BroadCasterTest() : nh_()
{
    broadcast_static_tf();
    timer_ = nh_.createTimer(ros::Duration(0.1), &BroadCasterTest::timer_callback, this);
}

void BroadCasterTest::timer_callback(const ros::TimerEvent &e)
{
    broadcast_dynamic_tf();
    counter_++;
}

void BroadCasterTest::broadcast_static_tf(void)
{
    geometry_msgs::TransformStamped static_transformSt;
    static_transformSt.header.stamp = ros::Time::now();
    static_transformSt.header.frame_id = "base_link";
    static_transformSt.child_frame_id = "static_tf";
    static_transformSt.transform.translation.x = 0.0;
    static_transformSt.transform.translation.y = 0.0;
    static_transformSt.transform.translation.z = 1;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, M_PI * cos(counter_ * 0.1));
    static_transformSt.transform.rotation.x = quat.x();
    static_transformSt.transform.rotation.y = quat.y();
    static_transformSt.transform.rotation.z = quat.z();
    static_transformSt.transform.rotation.w = quat.w();
    static_br_.sendTransform(static_transformSt);
}

void BroadCasterTest::broadcast_dynamic_tf(void)
{
    geometry_msgs::TransformStamped dyna_trans;
    dyna_trans.header.stamp = ros::Time::now();
    dyna_trans.header.frame_id = "base_link";
    dyna_trans.child_frame_id = "dynamic_tf";
    dyna_trans.transform.translation.x = 1 * cos(counter_ * 0.1);
    dyna_trans.transform.translation.y = 1 * sin(counter_ * 0.1);
    dyna_trans.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, M_PI * cos(counter_ * 0.1), 0);
    dyna_trans.transform.rotation.x = q.x();
    dyna_trans.transform.rotation.y = q.y();
    dyna_trans.transform.rotation.z = q.z();
    dyna_trans.transform.rotation.w = q.w();
    dynamic_br_.sendTransform(dyna_trans);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_static_tf2_broadcaster");
    BroadCasterTest boad;
    ros::spin();
    return 0;
}