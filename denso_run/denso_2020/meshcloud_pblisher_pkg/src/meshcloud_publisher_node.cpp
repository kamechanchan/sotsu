#include <meshcloud_publisher/meshcloud_publisher.h>

using meshcloud_publisher::MeshCloudPublisher;

// Node
int main(int argc, char* argv[])
{
  std::cout << "start" << std::endl;
  ros::init(argc, argv, "meshcloud_publisher");
  ros::NodeHandle nh;

  ROS_INFO("before const");
  MeshCloudPublisher mcp(nh);

  if (!mcp.setMeshCloud())
  {
    return -1;
  }

  ros::Rate rate(1.0);
  while (ros::ok())
  {
    mcp.publishCloud();
    mcp.broadcastTF();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
