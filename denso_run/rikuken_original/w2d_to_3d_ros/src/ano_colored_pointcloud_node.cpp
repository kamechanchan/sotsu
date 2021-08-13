#include <w2d_to_3d_ros/ano_colored_pointcloud.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "colored");
    // ROS_INFO_STREAM("feei");
    ros::NodeHandle nh;
    Annotation_yolo yolo(nh);
    ros::spin();
}