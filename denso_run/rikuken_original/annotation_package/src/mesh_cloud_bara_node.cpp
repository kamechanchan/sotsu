#include <annotation_package/mesh_kai_cloud.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_init");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    int the_number_of_object;
    std::string object_name, mesh_path;
    pnh.getParam("the_number_of_object", the_number_of_object);
    pnh.getParam("object_name", object_name);
    std::vector<mesh_cloud::MeshCloud*> loader_;
    for (int i = 0; i < the_number_of_object; i++) {
        loader_.push_back(new mesh_cloud::MeshCloud(nh, object_name + "_" + std::to_string(i), "meshcloud_" + std::to_string(i)));
    }
    ros::Rate loop(1.0);
    while (ros::ok())
    {
        for (int i = 0; i < the_number_of_object; i++) {
            loader_[i]->publishCloud(); 
        }
        ros::spinOnce();
        loop.sleep();
        for (int i = 0; i < the_number_of_object; i++) {
            loader_[i]->get_tf();
            loader_[i]->transformMesh();
        }
    }
    return 0;
}