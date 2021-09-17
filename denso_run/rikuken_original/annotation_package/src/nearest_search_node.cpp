#include <annotation_package/nearest_search.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "nearest_search");
    ros::NodeHandle nh;
    nearest_point_extractor::NearestPointExtractor nes(nh);
    nes.exect();
    ros::Rate loop(1);
    while (ros::ok())
    {
        nes.publish();
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}