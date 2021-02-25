#include <oneshot_calibration_system/ar_cube_pointcloud_extractor.h>

using ar_cube_pointcloud_extractor::ARCubePointCloudExtractor;
using ar_cube_pointcloud_extractor::ExtractorParam;

const int ExtractorParam::LOOP_RATE_TIME = 10;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_cube_pointcloud_extractor");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  ARCubePointCloudExtractor extractor(nh, n);

  ros::Rate loop_rate(ExtractorParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    extractor.publishPointCloud();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
