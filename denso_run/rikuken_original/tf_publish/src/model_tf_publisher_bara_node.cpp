#include <tf_publish/model_tf_publisher_bara.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_tf_bara");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    model_tf_publisher_bara::Model_bara bara(nh);
    ROS_INFO_STREAM("Publish model tf!");
    ros::Rate loop(10);

    while (ros::ok())
    {
        ros::spinOnce();
        bara.broadcastModelTF();
        loop.sleep();
    }
    return 0;
}