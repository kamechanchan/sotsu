#include <annotation_package/nearest_search_yolo.hpp>

namespace nearest_point_extractor
{
    NearestPointExtractor::NearestPointExtractor(ros::NodeHandle &nh)
    : nh_(nh)
    , flag_(false)
    , pnh_("~")
    {
        pnh_.getParam("sensor_topic_name", sensor_topic_name_);
        ROS_INFO_STREAM(sensor_topic_name_);
        pnh_.getParam("output_topic_name", output_topic_name_);
        ROS_INFO_STREAM(output_topic_name_);
        pnh_.getParam("mesh_topic_name", mesh_topic_name_);
        ROS_INFO_STREAM(mesh_topic_name_);
        pnh_.getParam("radius", radius_);
        ROS_INFO_STREAM(radius_);
        pnh_.getParam("num_of_nearest_points", num_of_nearest_points_);
        ROS_INFO_STREAM(num_of_nearest_points_);
        pnh_.getParam("message_timeout", timeout_);
        pnh_.getParam("background_instance", background_instance_);
        exect();
    }

    /*Refecence the publisher and subscriber*/
    void NearestPointExtractor::exect()
    {
        ROS_INFO_STREAM(output_topic_name_);
        ROS_INFO_STREAM(mesh_topic_name_);
        dummy_pub_ = nh_.advertise<color_cloud_bridge::out_segmentation>(output_topic_name_, 10);
        mesh_topic_name_sub_ = nh_.subscribe(mesh_topic_name_, 10, &NearestPointExtractor::InputCallback, this);
    }



    void NearestPointExtractor::InputCallback(const color_cloud_bridge::object_kiriwakeConstPtr &std_msgs_sub)
    {   
        int size = std_msgs_sub->occuluder_mesh_topic_name.size();
        std::cout << "tsuchida_1" << std::endl;
        get_one_message<sensor_msgs::PointCloud2>(sensor_cloud_msgs_, sensor_topic_name_, timeout_);
        for (int i = 0; i < size; i++) {
            sensor_msgs::PointCloud2 mesh_one;
            std::cout << std_msgs_sub->occuluder_mesh_topic_name[i] << std::endl;
            get_one_message<sensor_msgs::PointCloud2>(mesh_one, std_msgs_sub->occuluder_mesh_topic_name[i], timeout_);
            mesh_clouds_msgs_.push_back(mesh_one);
        }
        pcl::PointCloud<pcl::PointXYZ> sensor_pcl;
        pcl::fromROSMsg(sensor_cloud_msgs_, sensor_pcl);
        std::vector<mesh_and_instance> mesh_closter;
        for (int i = 0; i < mesh_clouds_msgs_.size(); i++) {
            mesh_and_instance mesh_ins_one;
            pcl::fromROSMsg(mesh_clouds_msgs_[i], mesh_ins_one.mesh_pcl_one);
            mesh_ins_one.instance = i;
            mesh_closter.push_back(mesh_ins_one);
        }
        color_cloud_bridge::out_segmentation output;
        output = extract_cloud(sensor_pcl, mesh_closter, radius_);
        output.header.frame_id = sensor_cloud_msgs_.header.frame_id;
        print_parameter(std::to_string(output.x.size()) + "output_point");
        dummy_pub_.publish(output);
    }

    color_cloud_bridge::out_segmentation NearestPointExtractor::extract_cloud(pcl::PointCloud<pcl::PointXYZ> sensor_cloud,
                                                                                std::vector<mesh_and_instance> mesh_cloud, double radisu_arg)
    {
        color_cloud_bridge::out_segmentation out_cloud;
        for (int i = 0; i < sensor_cloud.size(); i++) {
            out_cloud.x.push_back(sensor_cloud.points[i].x);
            out_cloud.y.push_back(sensor_cloud.points[i].y);
            out_cloud.z.push_back(sensor_cloud.points[i].z);
            out_cloud.instance.push_back(background_instance_);
        }
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        
        kdtree.setInputCloud(sensor_cloud.makeShared());
        std::vector<int> pointIndices, list_pointIndices;
        std::vector<float> squaredDistances;
        double c2c_distance = 0.0;
        int point_size = 0;
        
        for (int i = 0; i < mesh_cloud.size(); i++) {
            for (auto mesh : mesh_cloud[i].mesh_pcl_one.points)
            {
                if (kdtree.nearestKSearch(mesh, num_of_nearest_points_, pointIndices, squaredDistances) > 0) {
                    c2c_distance += squaredDistances[0];
                    point_size++;
                    for (int j = 0; j < pointIndices.size(); j++) {
                        out_cloud.instance[j] = mesh_cloud[i].instance;
                    }

                }
                pointIndices.clear();
                squaredDistances.clear();
            }
        }
        // ("sensor all size: " << senROS_INFO_STREAMsor_cloud_->points.size());
        ROS_INFO_STREAM("color point size: " << list_pointIndices.size());
        list_pointIndices.clear();
        return out_cloud;
    }


}