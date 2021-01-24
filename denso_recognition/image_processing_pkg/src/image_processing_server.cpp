#include <image_processing_pkg/contrast_modifier.h>
#include <image_processing_pkg/gamma_modifier.h>
#include <image_processing_pkg/histogram_flattening.h>

using contrast_modifier::ContrastModifier;
using gamma_modifier::GammaModifier;
using histogram_flattening::HistogramFlattening;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_processing_server");
  ros::NodeHandle nh;

  ContrastModifier contrast(nh);
  GammaModifier gamma(nh);
  HistogramFlattening histogram(nh);

  ros::spin();

  return 0;
}
