#include <annotation_package/mesh_kai_cloud.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mesh_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string object_name, mesh_topic_name;
    pnh.getParam("object_name", object_name);
    pnh.getParam("mesh_topic_name", mesh_topic_name);
    mesh_cloud::MeshCloud mcp(nh, object_name, mesh_topic_name);
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        
        mcp.publishCloud();
        ros::spinOnce();
        rate.sleep();
        mcp.get_tf();
        mcp.transformMesh();
    }
    return 0;
}