#include <bayes_pose/icp_registrator.h>

using bayes_pose::ICPRegistrator;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp_registrator");
  ros::NodeHandle nh;

  int loop_rate_hz;
  std::string compared_pc_topic_name, reference_pc_topic_name, registrated_pc_topic_name;
  ros::param::param<int>("~loop_rate", loop_rate_hz, 1);
  ros::param::param<std::string>("~compared_pc_topic_name", compared_pc_topic_name, "/cropped_cloud");
  ros::param::param<std::string>("~reference_pc_topic_name", reference_pc_topic_name, "/mesh_cloud_id_0");
  ros::param::param<std::string>("~registrated_pc_topic_name", registrated_pc_topic_name, "/registrated_cloud");

  ros::Rate loop_rate(loop_rate_hz);

  ICPRegistrator icp_registrator(nh, compared_pc_topic_name, reference_pc_topic_name, registrated_pc_topic_name);

  while (ros::ok())
  {
    icp_registrator.publish();
    icp_registrator.broadcast_tf();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
