#include <annotation_package/nearest_search.hpp>
#include <stdlib.h>
#include <time.h>
#include <color_cloud_bridge/sensor_and_index.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bara_nearest");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::vector<nearest_point_extractor::NearestPointExtractor*> loader_;
    std::string sensor_topic, mesh_base_topic, output_topic, instance_topic;
    int num_of_object, num_of_nearest_points;
    double radius;
    srand(time(NULL));
    unsigned char red, blue, green;
    pnh.getParam("num_of_object", num_of_object);
    pnh.getParam("instance_topic_name", instance_topic);
    pnh.getParam("sensor_topic", sensor_topic);
    pnh.getParam("mesh_base_topic", mesh_base_topic);
    pnh.getParam("output_topic_base", output_topic);
    pnh.getParam("radius", radius);
    pnh.getParam("num_of_nearest_points", num_of_nearest_points);
    for (int i = 0; i < num_of_object; i++) 
    {
        loader_.push_back(new nearest_point_extractor::NearestPointExtractor(nh));
    }
    for (int i = 0; i < num_of_object; i++) {
        loader_[i]->param_register(sensor_topic, mesh_base_topic + "_" + std::to_string(i), output_topic + "_" + std::to_string(i), num_of_nearest_points);
    }
    loader_[0]->color_decide(255, 0, 0);
    loader_[1]->color_decide(0, 255, 0);
    loader_[2]->color_decide(0, 0, 255);
    loader_[3]->color_decide(255, 255, 0);
    loader_[4]->color_decide(0, 255, 255);
    loader_[5]->color_decide(255, 0, 255);
    loader_[6]->color_decide(255, 100, 255);

    pcl::PointCloud<pcl::PointXYZRGB> all_cloud;
    sensor_msgs::PointCloud2 all_msgs;
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/all_cloud", 10);
    ros::Publisher instance_pub = nh.advertise<color_cloud_bridge::sensor_and_index>(instance_topic, 10);
    for (int i = 0; i < num_of_object; i++)
    {
        loader_[i]->exect();
    }
    ros::Rate loop(1);
    while (ros::ok())
    {
        for (int i = 0; i < num_of_object; i++)
        {
            loader_[i]->publish();
            all_cloud += *loader_[i]->output_cloud_;
        }
        all_cloud.header.frame_id = loader_[0]->frame_id_;
        pcl::toROSMsg(all_cloud, all_msgs);
        cloud_pub.publish(all_msgs);
        all_cloud.clear();
        
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
    
}