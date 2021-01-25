#include <image_processing_pkg/histogram_flattening.h>

using histogram_flattening::HistogramFlattening;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "histogram_flattening");
  ros::NodeHandle nh;

  HistogramFlattening histogram(nh);

  ros::spin();

  return 0;
}
