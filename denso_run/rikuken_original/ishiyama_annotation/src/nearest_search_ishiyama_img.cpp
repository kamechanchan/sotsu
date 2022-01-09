#include <ishiyama_annotation/nearest_search_ishiyama_img.hpp>
#include <list>


namespace nearest_point_extractor_ishiyama
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
        pnh_->getParam("photoneo_sensor_img", photoneo_sensor_img_);
        pnh_->getParam("img_service_name", img_service_name_);
        pnh_->getParam("timespan", timespan_);
        pnh_->getParam("main_service_name", main_service_name_);
        pnh_->getParam("photoneo_frame_id", photoneo_frame_id_);

        
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
        std::vector<int> v8{100, 255, 255};
        color.push_back(v8);
        std::vector<int> v9{255, 255, 100};
        color.push_back(v9);
        std::vector<int> v10{100, 100, 255};
        color.push_back(v10);
        std::vector<int> v11{255, 100, 100};
        color.push_back(v11);
        std::vector<int> v12{100, 255, 100};
        color.push_back(v12);
        std::vector<int> v13{150, 100, 255};
        color.push_back(v13);
        std::vector<int> v14{100, 150, 255};
        color.push_back(v14);
        std::vector<int> v15{255, 100, 150};
        color.push_back(v15);
        std::vector<int> v16{255, 150, 100};
        color.push_back(v16);
        std::vector<int> v17{100, 255, 150};
        color.push_back(v17);
        std::vector<int> v18{150, 255, 100};
        color.push_back(v18);
        std::vector<int> v19{100, 150, 0};
        color.push_back(v19);
        std::vector<int> v20{150, 100, 0};
        color.push_back(v20);
        std::vector<int> v21{150, 0, 100};
        color.push_back(v21);
        std::vector<int> v22{100, 0, 150};
        color.push_back(v22);
        std::vector<int> v23{0, 100, 150};
        color.push_back(v23);
        std::vector<int> v24{0, 150, 100};
        color.push_back(v24);
        std::vector<int> v25{255, 150, 150};
        color.push_back(v25);
       
       
        
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
        server_for_clientmain_ = nh_.advertiseService(main_service_name_, &NearestPointExtractor::InputCallback, this);
        ROS_INFO_STREAM("main_service_name" << main_service_name_);
        // cloud_sub_ = nh_.subscribe(sensor_topic_name_, 10, &NearestPointExtractor::InputCallback, this);
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
        mesh_sub_11_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_11", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 11));
        mesh_sub_12_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_12", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 12));
        mesh_sub_13_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_13", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 13));
        mesh_sub_14_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_14", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 14));
        mesh_sub_15_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_15", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 15));
        mesh_sub_16_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_16", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 16));
        mesh_sub_17_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_17", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 17));
        mesh_sub_18_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_18", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 18));
        mesh_sub_19_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_19", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 19));
        mesh_sub_20_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_20", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 20));
        mesh_sub_21_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_21", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 21));
        mesh_sub_22_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_22", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 22));
        mesh_sub_23_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_23", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 23));
        mesh_sub_24_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_24", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 24));

    }

    void NearestPointExtractor::publish(void)
    {
        
    }

    bool NearestPointExtractor::InputCallback(estimator::third_input::Request& req, estimator::third_input::Response& res)
    {   
        // ROS_INFO_STREAM("header_frame_id;" << req.in_cloud.header.frame_id);
        while (true)
        {
            try
            {
                // listener_.lookupTransform("world", req.in_cloud.header.frame_id, ros::Time(0), transform_);
                listener_.lookupTransform("world", photoneo_frame_id_, ros::Time(0), transform_);
                ROS_INFO_ONCE("I got a transfomr");
                break;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }
        }
        tem_cloud_.resize(req.x.size());
        tem_cloud_.header.frame_id = photoneo_frame_id_;
        for (int i=0; i<req.x.size(); i++){
            tem_cloud_.points[i].x = req.x[i];
            tem_cloud_.points[i].y = req.y[i];
            tem_cloud_.points[i].z = req.z[i];
        }
        pcl::toROSMsg(tem_cloud_, fav_cloud_);
        sensor_msgs::PointCloud2 msg_transformed;
        // print_parameter(req.in_cloud.header.frame_id);
        print_parameter(tem_cloud_.header.frame_id);
        pcl_ros::transformPointCloud("world", transform_, fav_cloud_, msg_transformed);
        pcl::fromROSMsg(msg_transformed, *sensor_cloud_);

        float x_min = 0;
        float x_max = 0;
        float y_min = 0;
        float y_max = 0;
        float z_min = 0;
        float z_max = 0;

        for (int i=0; i<sensor_cloud_->points.size(); i++){
            if (sensor_cloud_->points[i].x > x_max){
                x_max = sensor_cloud_->points[i].x;
            } 
            if (sensor_cloud_->points[i].x < x_min){
                x_min = sensor_cloud_->points[i].x;
            }
            if (sensor_cloud_->points[i].y > y_max){
                y_max = sensor_cloud_->points[i].y;
            } 
            if (sensor_cloud_->points[i].y < y_min){
                y_min = sensor_cloud_->points[i].y;
            }
            if (sensor_cloud_->points[i].z > z_max){
                z_max = sensor_cloud_->points[i].z;
            } 
            if (sensor_cloud_->points[i].z < z_min){
                z_min = sensor_cloud_->points[i].z;
            }
        }
        ROS_INFO_STREAM("x_max;" << x_max << " x_min;" << x_min << " y_max;" << y_max << " y_min;" << y_min << " z_max;" << z_max << " z_min;" << z_min);

        flag_ = true;
        print_parameter(flag_);
        if (!flag_)
            return false;
        //print_parameter("extract mae");
        output_cloud_ = extract_cloud();
        //print_parameter("extract go");

        // ROS_INFO_STREAM("kokoda******************");
        // get_one_message(sensor_img_, photoneo_sensor_img_, nh_, timespan_);

        sensor_msgs::PointCloud2 cloud_msg;
        output_cloud_->header.frame_id = sensor_cloud_->header.frame_id;
        print_parameter(std::to_string(output_cloud_->points.size()) + "output_point");
        frame_id_ = output_cloud_->header.frame_id;
        pcl::toROSMsg(*output_cloud_, cloud_msg);
        ROS_INFO_STREAM("************************************");
        cloud_pub_.publish(cloud_msg);
        res.out_cloud = cloud_msg;
        // ROS_INFO_STREAM("itterudeyansu");
        // server_ = nh_.advertiseService(img_service_name_, &NearestPointExtractor::inputData, this);
        return true;
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
        // ROS_INFO_STREAM("toturru");
        
        for (int i = 0; i < the_number_of_object_; i++) {
        // for (int i = 1; i < 2; i++) {
            // i++;
            for (auto mesh : mesh_clouds_[i]->points)
            {
                if (kdtree.nearestKSearch(mesh, num_of_nearest_points_, pointIndices, squaredDistances) > 0) {
                    c2c_distance += squaredDistances[0];
                    point_size++;
                    for (int j = 0; j < pointIndices.size(); j++) {
                        out_cloud->points[pointIndices[j]].r = color[i][0];
                        out_cloud->points[pointIndices[j]].g = color[i][1];
                        out_cloud->points[pointIndices[j]].b = color[i][2];
                        // list_pointIndices.push_back(pointIndices[j]);
                    }

                }
                pointIndices.clear();
                squaredDistances.clear();
            }
        }
        // ROS_INFO_STREAM("shinka");
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
            
        
        // ROS_INFO_STREAM("sensor all size: " << sensor_cloud_->points.size());
        // ROS_INFO_STREAM("color point size: " << list_pointIndices.size());
        // list_pointIndices.clear();
            
        return out_cloud;
    }

    bool NearestPointExtractor::inputData(estimator::input_data::Request &req, estimator::input_data::Response &res){
        res.out_img = sensor_img_;
        ROS_INFO_STREAM("input_img");
        return true;
    }


    void NearestPointExtractor::mesh_callback(const sensor_msgs::PointCloud2ConstPtr &msg, int object_num)
    {
        std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&gogogoogooggo:" << std::endl;
        std::cout << "mesh: " << object_num << std::endl;
        pcl::fromROSMsg(*msg, *mesh_clouds_[object_num]);
    }
}