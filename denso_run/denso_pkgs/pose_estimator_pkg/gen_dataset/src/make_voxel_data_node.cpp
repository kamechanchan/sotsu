#include <gen_dataset/make_voxel_data.h>

using make_voxel_data::MakeVoxelData;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "make_voxel_data");
  ros::NodeHandle nh;

  MakeVoxelData make_voxel_data(nh);

  while (ros::ok())
  {
    if (make_voxel_data.runMakeVoxelData())
    {
      break;
    }
  }

  return 0;
}
