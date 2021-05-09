#include <annotation_package/mesh_cloud.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mesh_publisher");
    ros::NodeHandle nh;

    mesh_cloud::MeshCloud mcp(nh);
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        mcp.publishCloud();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}