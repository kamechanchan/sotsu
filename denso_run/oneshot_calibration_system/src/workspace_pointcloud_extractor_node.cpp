#include <oneshot_calibration_system/workspace_pointcloud_extractor.h>

using workspace_pointcloud_extractor::WorkspacePointCloudExtractor;
using workspace_pointcloud_extractor::ExtractorParam;

const int ExtractorParam::LOOP_RATE_TIME = 10;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "workspace_pointcloud_extractor");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  WorkspacePointCloudExtractor extractor(nh, n);

  ros::Rate loop_rate(ExtractorParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    extractor.publishPointCloud();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
