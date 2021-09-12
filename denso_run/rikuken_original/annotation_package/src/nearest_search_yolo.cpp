#include <annotation_package/nearest_search_yolo.hpp>

namespace nearest_point_extractor
{
    NearestPointExtractor::NearestPointExtractor(ros::NodeHandle &nh)
    : nh_(nh)
    , flag_(false)
    , sensor_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , mesh_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        pnh_ = new ros::NodeHandle("~");
        pnh_->getParam("sensor_topic_name", sensor_topic_name_);
        pnh_->getParam("output_topic_name", output_topic_name_);
        pnh_->getParam("mesh_topic_name", mesh_topic_name_);
        pnh_->getParam("LEAF_SIZE", LEAF_SIZE);
        pnh_->getParam("radius", radius);
        pnh_->getParam("num_of_nearest_points", num_of_nearest_points_);
        pnh_->getParam("message_timeout", timeout_);

    }

    /*
    1: sensor_topic,  2: mesh_topic  3:output_topic  4: num_of_near_point
    */
   void NearestPointExtractor::param_register(std::string sensor_topic, std::string mesh_topic, std::string output_topic, int num_of_near_point)
   {
       sensor_topic_name_ = sensor_topic;
       mesh_topic_name_ = mesh_topic;
       output_topic_name_ = output_topic;
       num_of_nearest_points_ = num_of_near_point;
   }

    /*Refecence the publisher and subscriber*/
    void NearestPointExtractor::exect()
    {

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_, 10);
        mesh_topic_name_sub_ = nh_.subscribe(mesh_topic_name_, 10, NearestPointExtractor::InputCallback, this);

    }

    void NearestPointExtractor::publish(void)
    {
        print_parameter(flag_);
        if (!flag_)
            return;
        //print_parameter("extract mae");
        output_cloud_ = extract_cloud(*sensor_cloud_, *mesh_cloud_, radius);
        //print_parameter("extract go");

        sensor_msgs::PointCloud2 cloud_msg;
        output_cloud_.header.frame_id = sensor_cloud_->header.frame_id;
        print_parameter(std::to_string(output_cloud_.x.size()) + "output_point");
        frame_id_ = output_cloud_.header.frame_id;
        // pcl::toROSMsg(*output_cloud_, cloud_msg);
        cloud_pub_.publish(cloud_msg);
    }

    void NearestPointExtractor::InputCallback(const color_cloud_bridge::object_kiriwakeConstPtr &std_msgs_sub)
    {   
        int size = std_msgs_sub->occuluder_mesh_topic_name.size();
        get_one_message<sensor_msgs::PointCloud2>(sensor_cloud_msgs_, sensor_topic_name_, timeout_);
        for (int i = 0; i < size; i++) {
            sensor_msgs::PointCloud2 mesh_one;
            get_one_message<sensor_msgs::PointCloud2>(mesh_one, std_msgs_sub->occuluder_mesh_topic_name[i], timeout_);
            mesh_clouds_msgs_.push_back(mesh_one);
        }
        pcl::PointCloud<pcl::PointXYZ> sensor_pcl;
        pcl::fromROSMsg(sensor_cloud_msgs_, sensor_pcl);
        std::vector<color_cloud_bridge::out_segmentation> outputs;
        for (int i = 0; i < mesh_clouds_msgs_.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> mesh_pcl;
            pcl::fromROSMsg(mesh_clouds_msgs_[i], mesh_pcl);
            color_cloud_bridge::out_segmentation out_one;
            out_one = extract_cloud(sensor_pcl, mesh_pcl, radius);
            outputs.push_back(out_one);
        }
    }

    color_cloud_bridge::out_segmentation NearestPointExtractor::extract_cloud(pcl::PointCloud<pcl::PointXYZ> sensor_cloud,
                                                                                pcl::PointCloud<pcl::PointXYZ> mesh_cloud, double radisu_arg)
    {
        color_cloud_bridge::out_segmentation out_cloud;
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(sensor_cloud_);
        std::vector<int> pointIndices;
        std::vector<float> squaredDistances;
        double c2c_distance = 0.0;
        int point_size = 0;
        print_parameter("mesh size is " + std::to_string(mesh_cloud.points.size()));
        for (auto mesh : mesh_cloud.points)
        {
            if (kdtree.nearestKSearch(mesh, num_of_nearest_points_, pointIndices, squaredDistances) > 0) {
                c2c_distance += squaredDistances[0];
                point_size++;
                for (int j = 0; j < pointIndices.size(); j++) {
                    out_cloud.x.push_back(sensor_cloud.points[pointIndices[j]].x);
                    out_cloud.y.push_back(sensor_cloud.points[pointIndices[j]].y);
                    out_cloud.z.push_back(sensor_cloud.points[pointIndices[j]].z);
                    out_cloud.instance.push_back(1);
                }

            }
            pointIndices.clear();
            squaredDistances.clear();
        }
        ROS_INFO_STREAM("point:size :" << point_size);
        ROS_INFO_STREAM("c2c_distance: " << c2c_distance);
        ROS_INFO_STREAM("c2c_distance(mean): " << c2c_distance / point_size);
        return out_cloud;
    }

    void NearestPointExtractor::sensor_input(sensor_msgs::PointCloud2 msg)
    {
        pcl::fromROSMsg(msg, *sensor_cloud_);
    }
}