#include <annotation_package/nearest_search.hpp>

namespace nearest_point_extractor
{
    NearestPointExtractor::NearestPointExtractor(ros::NodeHandle &nh)
    : nh_(nh)
    , flag_(false)
    , sensor_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , mesh_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , output_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        pnh_ = new ros::NodeHandle("~");
        pnh_->getParam("sensor_topic_name", sensor_topic_name_);
        pnh_->getParam("output_topic_name", output_topic_name_);
        pnh_->getParam("mesh_topic_name", mesh_topic_name_);
        pnh_->getParam("LEAF_SIZE", LEAF_SIZE);
        pnh_->getParam("radius", radius);
        pnh_->getParam("num_of_nearest_points", num_of_nearest_points);
        pnh_->getParam("red", red);
        pnh_->getParam("blue", blue);
        pnh_->getParam("green", green);

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_, 10);
        sensor_cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, sensor_topic_name_, 10);
        mesh_cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, mesh_topic_name_, 10);
        sensor_sync_ = new message_filters::Synchronizer<Sensor_Sync_Sub_Type>(Sensor_Sync_Sub_Type(10), *sensor_cloud_sub_, *mesh_cloud_sub_);
        sensor_sync_->registerCallback(boost::bind(&NearestPointExtractor::InputCallback, this, _1, _2));
        
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
        output_cloud_->header.frame_id = sensor_cloud_->header.frame_id;
        print_parameter(std::to_string(output_cloud_->points.size()) + "output_point");
        pcl::toROSMsg(*output_cloud_, cloud_msg);
        cloud_pub_.publish(cloud_msg);
    }

    void NearestPointExtractor::InputCallback(const sensor_msgs::PointCloud2ConstPtr &sensor_pc_msgs,
                                            const sensor_msgs::PointCloud2ConstPtr &mesh_pc_msgs)
    {
        while (true)
        {
            try 
            {
                listener_.lookupTransform("world", sensor_pc_msgs->header.frame_id, ros::Time(0), transform_);
                ROS_INFO_ONCE("I got a transfomr");
                break;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        sensor_msgs::PointCloud2 msg_transformed;
        //print_parameter(sensor_pc_msgs->header.frame_id);
        //print_parameter(mesh_pc_msgs->header.frame_id);
        //pcl_ros::transformPointCloud("world", transform_, *sensor_pc_msgs, msg_transformed);
        pcl::fromROSMsg(*sensor_pc_msgs, *sensor_cloud_);
        pcl::fromROSMsg(*mesh_pc_msgs, *mesh_cloud_);
        flag_ = true;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr NearestPointExtractor::extract_cloud(pcl::PointCloud<pcl::PointXYZ> sensor_cloud,
                                                                                pcl::PointCloud<pcl::PointXYZ> mesh_cloud, int radisu_arg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(sensor_cloud_);
        std::vector<int> pointIndices(num_of_nearest_points);
        std::vector<float> squaredDistances(num_of_nearest_points);
        //print_parameter("mesh size is " + std::to_string(mesh_cloud.points.size()));
        for (int i = 0; i < 1; i++) {
            
            pcl::PointXYZRGB part_of_extract;
            if (kdtree.nearestKSearch(mesh_cloud[i], radisu_arg, pointIndices, squaredDistances) > 0)
            {
                //print_parameter("Pointindices size is " + std::to_string(pointIndices.size()));
                for (size_t j = 0; j < pointIndices.size(); ++j) 
                {
                    part_of_extract.x = sensor_cloud.points[pointIndices[j]].x;
                    part_of_extract.y = sensor_cloud.points[pointIndices[j]].y;
                    part_of_extract.z = sensor_cloud.points[pointIndices[j]].z;
                    part_of_extract.r = red;
                    part_of_extract.g = green;
                    part_of_extract.b = blue;
                    out_cloud->push_back(part_of_extract);

                }
            }
            pointIndices.clear();
            squaredDistances.clear();
        }
        return out_cloud;
    }
}