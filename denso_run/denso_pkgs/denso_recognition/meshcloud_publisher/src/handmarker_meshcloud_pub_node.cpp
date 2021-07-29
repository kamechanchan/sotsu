#include <meshcloud_publisher/handmarker_meshcloud_pub.h>

using handmarker_meshcloud_pub::HandMarkerMeshCloudPub;

// Node
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hand_meshcloud_publisher");
  ros::NodeHandle nh;

  handmarker_meshcloud_pub::HandMarkerMeshCloudPub mcp(nh);

  ros::Rate rate(1.0);
  while (ros::ok())
  {
    mcp.publishCloud();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
