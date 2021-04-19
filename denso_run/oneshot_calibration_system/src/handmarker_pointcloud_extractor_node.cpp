#include <oneshot_calibration_system/handmarker_pointcloud_extractor.h>

using handmarker_pointcloud_extractor::HandMarkerPointCloudExtractor;
using handmarker_pointcloud_extractor::ExtractorParam;

const int ExtractorParam::LOOP_RATE_TIME = 10;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "handmarker_pointcloud_extractor");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  HandMarkerPointCloudExtractor extractor(nh, n);

  ros::Rate loop_rate(ExtractorParam::LOOP_RATE_TIME);
  while (ros::ok())
  {
    extractor.publishPointCloud();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
