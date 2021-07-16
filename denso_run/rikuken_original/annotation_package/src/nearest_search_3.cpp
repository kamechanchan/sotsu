#include <annotation_package/nearest_search_3.hpp>
#include <list>


namespace nearest_point_extractor
{
    NearestPointExtractor::NearestPointExtractor(ros::NodeHandle &nh, int num)
    : nh_(nh)
    , flag_(false)
    , sensor_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , mesh_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , output_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
    , the_number_of_object_(num)
    {
        pnh_ = new ros::NodeHandle("~");
        pnh_->getParam("sensor_topic_name", sensor_topic_name_);
        pnh_->getParam("output_topic_name", output_topic_name_);
        pnh_->getParam("mesh_topic_name", mesh_base_topic_name_);
        pnh_->getParam("LEAF_SIZE", LEAF_SIZE);
        pnh_->getParam("radius", radius);
        pnh_->getParam("num_of_nearest_points", num_of_nearest_points_);
        pnh_->getParam("the_num_of_object", the_number_of_object_);
        
        for (int i = 0; i < the_number_of_object_; i++) {
            mesh_clouds_.push_back(new pcl::PointCloud<pcl::PointXYZ>());
        }
        std::vector<int> v1{255, 0, 0};
        color.push_back(v1);
        std::vector<int> v2{0, 255, 0};
        color.push_back(v2);
        std::vector<int> v3{0, 0, 255};
        color.push_back(v3);
        std::vector<int> v4{255, 255, 0};
        color.push_back(v4);
        std::vector<int> v5{0, 255, 255};
        color.push_back(v5);
        std::vector<int> v6{255, 0, 255};
        color.push_back(v6);
        std::vector<int> v7{255, 100, 255};
        color.push_back(v7);
       
        
    }

    /*
    1: sensor_topic,  2: mesh_topic  3:output_topic  4: num_of_near_point
    */
   void NearestPointExtractor::param_register(std::string sensor_topic, std::string mesh_topic, std::string output_topic, int num_of_near_point)
   {
       sensor_topic_name_ = sensor_topic;
       mesh_base_topic_name_ = mesh_topic;
       output_topic_name_ = output_topic;
       num_of_nearest_points_ = num_of_near_point;
   }

    

    /*Refecence the publisher and subscriber*/
    void NearestPointExtractor::exect()
    {

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_, 10);
        cloud_sub_ = nh_.subscribe(sensor_topic_name_, 10, &NearestPointExtractor::InputCallback, this);
        mesh_sub_0_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_0", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 0));
        mesh_sub_1_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_1", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 1));
        mesh_sub_2_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_2", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 2));
        mesh_sub_3_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_3", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 3));
        mesh_sub_4_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_4", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 4));
        mesh_sub_5_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_5", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 5));
        mesh_sub_6_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_6", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 6));
        mesh_sub_7_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_7", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 7));
        mesh_sub_8_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_8", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 8));
        mesh_sub_9_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_9", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 9));
        mesh_sub_10_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_10", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 10));
    }

    void NearestPointExtractor::publish(void)
    {
        print_parameter(flag_);
        if (!flag_)
            return;
        //print_parameter("extract mae");
        output_cloud_ = extract_cloud();
        //print_parameter("extract go");

        sensor_msgs::PointCloud2 cloud_msg;
        output_cloud_->header.frame_id = sensor_cloud_->header.frame_id;
        print_parameter(std::to_string(output_cloud_->points.size()) + "output_point");
        frame_id_ = output_cloud_->header.frame_id;
        pcl::toROSMsg(*output_cloud_, cloud_msg);
        cloud_pub_.publish(cloud_msg);
    }

    void NearestPointExtractor::InputCallback(const sensor_msgs::PointCloud2ConstPtr &sensor_pc_msgs)
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
        print_parameter(sensor_pc_msgs->header.frame_id);
        pcl_ros::transformPointCloud("world", transform_, *sensor_pc_msgs, msg_transformed);
        pcl::fromROSMsg(msg_transformed, *sensor_cloud_);
        flag_ = true;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr NearestPointExtractor::extract_cloud()
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int i = 0; i < sensor_cloud_->size(); i++) {
            pcl::PointXYZRGB part_of_extract;
            part_of_extract.x = sensor_cloud_->points[i].x;
            part_of_extract.y = sensor_cloud_->points[i].y;
            part_of_extract.z = sensor_cloud_->points[i].z;
            part_of_extract.r = 255;
            part_of_extract.g = 255;
            part_of_extract.b = 255;
            out_cloud->push_back(part_of_extract);
        }
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(sensor_cloud_);
        std::vector<int> pointIndices, list_pointIndices;
        std::vector<float> squaredDistances;
        double c2c_distance = 0.0;
        int point_size = 0;
        
        for (int i = 0; i < the_number_of_object_; i++) {
            for (auto mesh : mesh_clouds_[i]->points)
            {
                if (kdtree.nearestKSearch(mesh, num_of_nearest_points_, pointIndices, squaredDistances) > 0) {
                    c2c_distance += squaredDistances[0];
                    point_size++;
                    for (int j = 0; j < pointIndices.size(); j++) {
                        out_cloud->points[pointIndices[j]].r = color[i][0];
                        out_cloud->points[pointIndices[j]].g = color[i][1];
                        out_cloud->points[pointIndices[j]].b = color[i][2];
                        list_pointIndices.push_back(pointIndices[j]);
                    }

                }
                pointIndices.clear();
                squaredDistances.clear();
            }
        }
        // std::list<int>::iterator itr;
        // for (itr = all_index.begin(); itr != all_index.end(); itr++) {
        //     pcl::PointXYZRGB part_of_extract;
        //     part_of_extract.x = sensor_cloud_->points[*itr].x;
        //     part_of_extract.y = sensor_cloud_->points[*itr].y;
        //     part_of_extract.z = sensor_cloud_->points[*itr].z;
        //     part_of_extract.r = 0;
        //     part_of_extract.g = 0;
        //     part_of_extract.b = 0;
        //     out_cloud->push_back(part_of_extract);
        // }
            
        
        ROS_INFO_STREAM("sensor all size: " << sensor_cloud_->points.size());
        ROS_INFO_STREAM("color point size: " << list_pointIndices.size());
        list_pointIndices.clear();
            
        return out_cloud;
    }


    void NearestPointExtractor::mesh_callback(const sensor_msgs::PointCloud2ConstPtr &msg, int object_num)
    {
        pcl::fromROSMsg(*msg, *mesh_clouds_[object_num]);
    }
}