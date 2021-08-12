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
        pnh_->getParam("num_of_nearest_points", num_of_nearest_points_);
        pnh_->getParam("red", red_);
        pnh_->getParam("blue", blue_);
        pnh_->getParam("green", green_);

        
        
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

    /*
    1: red,   2:blue,  3:green
    */
    void NearestPointExtractor::color_decide(unsigned char red, unsigned char blue, unsigned char green)
    {
        red_ = red;
        blue_ = blue;
        green_ = green;

    }

    /*Refecence the publisher and subscriber*/
    void NearestPointExtractor::exect()
    {

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
        frame_id_ = output_cloud_->header.frame_id;
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
        print_parameter(sensor_pc_msgs->header.frame_id);
        print_parameter(mesh_pc_msgs->header.frame_id);
        pcl_ros::transformPointCloud("world", transform_, *sensor_pc_msgs, msg_transformed);
        pcl::fromROSMsg(msg_transformed, *sensor_cloud_);
        pcl::fromROSMsg(*mesh_pc_msgs, *mesh_cloud_);
        flag_ = true;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr NearestPointExtractor::extract_cloud(pcl::PointCloud<pcl::PointXYZ> sensor_cloud,
                                                                                pcl::PointCloud<pcl::PointXYZ> mesh_cloud, double radisu_arg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(sensor_cloud_);
        std::vector<int> pointIndices;
        std::vector<float> squaredDistances;
        double c2c_distance = 0.0;
        int point_size = 0;

        print_parameter("mesh size is " + std::to_string(mesh_cloud.points.size()));
        //for (int i = 0; i < 1; i++) {
        for (auto mesh : mesh_cloud.points)
        {
            if (kdtree.nearestKSearch(mesh, num_of_nearest_points_, pointIndices, squaredDistances) > 0) {
                c2c_distance += squaredDistances[0];
                point_size++;
                for (int j = 0; j < pointIndices.size(); j++) {
                    pcl::PointXYZRGB part_of_extract;
                    part_of_extract.x = sensor_cloud.points[pointIndices[j]].x;
                    part_of_extract.y = sensor_cloud.points[pointIndices[j]].y;
                    part_of_extract.z = sensor_cloud.points[pointIndices[j]].z;
                    //writing_cloud.points[pointIndices[j]].x = sensor_cloud.points[pointIndices[j]].x;
                   // writing_cloud.points[pointIndices[j]].y = sensor_cloud.points[pointIndices[j]].y;
                    //writing_cloud.points[pointIndices[j]].z = sensor_cloud.points[pointIndices[j]].z;

                    /*part_of_extract.x = mesh.x;
                    part_of_extract.y = mesh.y;
                    part_of_extract.z = mesh.z;*/
                    part_of_extract.r = red_;
                    part_of_extract.g = green_;
                    part_of_extract.b = blue_;
                   //writing_cloud.points[pointIndices[j]].r = red_;
                    //writing_cloud.points[pointIndices[j]].g = green_;
                   // writing_cloud.points[pointIndices[j]].b = blue_;
                    out_cloud->push_back(part_of_extract);
                }

            }
            pointIndices.clear();
            squaredDistances.clear();
        }
        ROS_INFO_STREAM("point:size :" << point_size);
        ROS_INFO_STREAM("c2c_distance: " << c2c_distance);
        ROS_INFO_STREAM("c2c_distance(mean): " << c2c_distance / point_size);
            /*pcl::PointXYZRGB part_of_extract;
            if (kdtree.nearestKSearch(mesh_cloud[i], radisu_arg, pointIndices, squaredDistances) > 0)
            //if (kdtree.radiusSearch(mesh_cloud[i], radisu_arg, pointIndices, squaredDistances) > 0)
            {
                print_parameter("Pointindices size is " + std::to_string(pointIndices.size()));
                for (size_t j = 0; j < pointIndices.size(); ++j) 
                {
                    part_of_extract.x = sensor_cloud.points[pointIndices[j]].x;
                    part_of_extract.y = sensor_cloud.points[pointIndices[j]].y;
                    part_of_extract.z = sensor_cloud.points[pointIndices[j]].z;
                    part_of_extract.r = red_;
                    part_of_extract.g = green_;
                    part_of_extract.b = blue_;
                    out_cloud->push_back(part_of_extract);

                }
            }
            else {
                std::cout << "not foud" << std::endl;
            }
            pointIndices.clear();
            squaredDistances.clear();
        }*/
        return out_cloud;
    }

    void NearestPointExtractor::read_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud)
    {
        writing_cloud = cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB> NearestPointExtractor::write_cloud()
    {
        return writing_cloud;
    }
}