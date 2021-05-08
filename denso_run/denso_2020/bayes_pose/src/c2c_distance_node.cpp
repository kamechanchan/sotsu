#include <bayes_pose/c2c_distance.h>

using bayes_pose::C2CDistance;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "c2c_distance");
  ros::NodeHandle nh;
  std::string compared_pc_topic_name, reference_pc_topic_name;
  ros::param::param<std::string>("compared_pc_topic_name", compared_pc_topic_name, "/cropped_cloud");
  ros::param::param<std::string>("reference_pc_topic_name", reference_pc_topic_name, "/mesh_cloud_id_0");

  C2CDistance c2c_distance(nh, compared_pc_topic_name, reference_pc_topic_name);

  ros::spin();
  return 0;
}
