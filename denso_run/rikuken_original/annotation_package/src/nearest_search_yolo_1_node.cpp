#include <annotation_package/nearest_search_yolo_1.hpp>
#include <stdlib.h>
#include <time.h>
#include <list>
#include <fstream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bara_nearest");
    ros::NodeHandle nh;
    nearest_point_extractor::NearestPointExtractor loader(nh);

    ros::Rate loop(10);
    while (ros::ok())
    {
        loader.publish();
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
    
}