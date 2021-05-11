#include <image_processing_pkg/contrast_modifier.h>

using contrast_modifier::ContrastModifier;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "contrast_modifier");
  ros::NodeHandle nh;

  ContrastModifier contrast(nh);

  ros::spin();

  return 0;
}
