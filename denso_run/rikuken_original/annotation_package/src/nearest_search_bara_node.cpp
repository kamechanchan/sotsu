#include <annotation_package/nearest_search.hpp>
#include <stdlib.h>
#include <time.h>
#include <list>
#include <fstream>


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
    std::ofstream out("/home/ericlab/ja.txt");
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

    pcl::PointCloud<pcl::PointXYZRGB> all_cloud, nokori_cloud;
    std::vector<int> index_all_cloud;
    sensor_msgs::PointCloud2 all_msgs;
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/all_cloud", 10);
    
    for (int i = 0; i < num_of_object; i++)
    {
        loader_[i]->exect();
    }
    for (int i = 0; i < loader_[0]->sensor_cloud_->points.size(); i++) {
        pcl::PointXYZRGB ten;
        ten.x = loader_[0]->sensor_cloud_->points[i].x;
        ten.y = loader_[0]->sensor_cloud_->points[i].y;
        ten.z = loader_[0]->sensor_cloud_->points[i].z;
        ten.r = 0;
        ten.g = 0;
        ten.b = 0;
        nokori_cloud.push_back(ten);
        
    }
    ros::Rate loop(1);
    while (ros::ok())
    {
        for (int i = 0; i < num_of_object; i++)
        {
            //loader_[i]->read_cloud(nokori_cloud);
            loader_[i]->publish();
            all_cloud += *loader_[i]->output_cloud_;
           // nokori_cloud = loader_[i]->write_cloud();
        }
        
      /*  for (int i = 0; i < index_all_cloud.size(); i++) {
            nokori_index.remove(index_all_cloud[i]);
        }
        std::cout << "kesita" << std::endl;
        for (int i = 0; i < nokori_index.size(); i++) {
            pcl::PointXYZRGB ten;
            ten.x = loader_[0]->sensor_cloud_->points[i].x;
            ten.y = loader_[0]->sensor_cloud_->points[i].y;
            ten.z = loader_[0]->sensor_cloud_->points[i].z;
            ten.r = 0;
            ten.g = 0;
            ten.b = 0;
            nokori_cloud.push_back(ten);
            
        }*/
       // ROS_INFO_STREAM("nokori size is " << nokori_cloud.size());
      //  all_cloud += nokori_cloud;
        all_cloud.header.frame_id = loader_[0]->frame_id_;
        pcl::toROSMsg(all_cloud, all_msgs);
        cloud_pub.publish(all_msgs);
        all_cloud.clear();
        index_all_cloud.clear();
        nokori_cloud.clear();
        
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
    
}