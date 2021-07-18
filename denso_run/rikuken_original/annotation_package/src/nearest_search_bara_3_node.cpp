#include <annotation_package/nearest_search_3.hpp>
#include <stdlib.h>
#include <time.h>
#include <list>
#include <fstream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bara_nearest");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // std::vector<nearest_point_extractor::NearestPointExtractor*> loader_;
   
    std::string sensor_topic, mesh_base_topic, output_topic, instance_topic;
    int num_of_object, num_of_nearest_points;
    double radius;
    srand(time(NULL));
    std::ofstream out("/home/ericlab/ja.txt");
    pnh.getParam("num_of_object", num_of_object);
    pnh.getParam("instance_topic_name", instance_topic);
    pnh.getParam("sensor_topic", sensor_topic);
    pnh.getParam("mesh_base_topic", mesh_base_topic);
    pnh.getParam("output_topic_base", output_topic);
    pnh.getParam("radius", radius);
    pnh.getParam("num_of_nearest_points", num_of_nearest_points);
    nearest_point_extractor::NearestPointExtractor loader_(nh, num_of_object);
    loader_.param_register(sensor_topic, mesh_base_topic, output_topic, num_of_nearest_points);
    loader_.exect();

    ros::Rate loop(1);
    while (ros::ok())
    {
        loader_.publish();
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
    
}