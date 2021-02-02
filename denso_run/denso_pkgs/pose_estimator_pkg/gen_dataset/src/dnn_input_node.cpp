#include "gen_dataset/dnn_input.hpp"

using dnn_input::DnnInput;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "MakeInputNode");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  DnnInput input(nh, n);
  // crop.run();
  ros::spin();

  return 0;
}
