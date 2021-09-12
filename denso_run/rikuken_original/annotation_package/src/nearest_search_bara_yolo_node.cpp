#include <annotation_package/nearest_search_yolo.hpp>
#include <stdlib.h>
#include <time.h>
#include <list>
#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bara_nearest");
    ros::NodeHandle nh;
    nearest_point_extractor::NearestPointExtractor ne(nh);
    ros::spin();
    return 0;
    
}