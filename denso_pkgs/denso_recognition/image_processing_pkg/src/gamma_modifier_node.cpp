#include <image_processing_pkg/gamma_modifier.h>

using gamma_modifier::GammaModifier;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gamma_modifier");
  ros::NodeHandle nh;

  GammaModifier gamma(nh);

  ros::spin();

  return 0;
}
