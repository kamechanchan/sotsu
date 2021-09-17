#include <meshcloud_publisher/meshcloud_publisher.h>

using meshcloud_publisher::MeshCloudPublisher;

// Node
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "meshcloud_publisher");
  ros::NodeHandle nh;

  MeshCloudPublisher mcp(nh);

  ros::Rate rate(1.0);
  while (ros::ok())
  {
    mcp.publishCloud();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
