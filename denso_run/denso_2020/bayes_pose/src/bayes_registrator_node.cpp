#include <bayes_pose/bayes_registrator.h>
#include <stdexcept>

using bayes_pose::BayesRegistrator;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bayes_registrator");
  ros::NodeHandle nh;

  int loop_rate_hz;
  std::string compared_pc_topic_name, reference_pc_topic_name, registrated_pc_topic_name;
  ros::param::param<int>("~loop_rate", loop_rate_hz, 1);
  ros::param::param<std::string>("~compared_pc_topic_name", compared_pc_topic_name, "/cropped_cloud");
  ros::param::param<std::string>("~reference_pc_topic_name", reference_pc_topic_name, "/mesh_cloud_id_0");
  ros::param::param<std::string>("~registrated_pc_topic_name", registrated_pc_topic_name, "/registrated_cloud");

  ros::Rate loop_rate(loop_rate_hz);

  bayesopt::Parameters pos_param, ori_param, posture_param;
  pos_param = initialize_parameters_to_default();
  // pos_param.n_iterations = 100;
  // pos_param.n_iter_relearn = 10;
  pos_param.n_inner_iterations = 500;
  // pos_param.l_type = L_MCMC;
  // pos_param.l_all = true;
  // pos_param.epsilon = 0.1;
  pos_param.random_seed = 0;
  pos_param.verbose_level = -1;
  // pos_param.noise = 1e-10;

  ori_param = initialize_parameters_to_default();
  // ori_param.n_iterations = 600;
  // ori_param.n_inner_iterations = 1000;
  // ori_param.n_iter_relearn = 1;
  // ori_param.l_type = L_MCMC;
  // ori_param.l_all = true;
  ori_param.epsilon = 0.1;
  ori_param.random_seed = 0;
  ori_param.verbose_level = -1;
  // ori_param.noise = 1e-6;

  posture_param = initialize_parameters_to_default();
  posture_param.n_iterations = 50;
  // ori_param.n_inner_iterations = 500;
  // ori_param.n_iter_relearn = 1;
  posture_param.l_all = true;
  posture_param.random_seed = -1;
  posture_param.verbose_level = -1;
  posture_param.l_type = L_MCMC;
  posture_param.epsilon = 0.3;
  // posture_param.noise = 1e-6;

  BayesRegistrator bayes_registrator(nh, compared_pc_topic_name, reference_pc_topic_name, registrated_pc_topic_name,
                                     pos_param, ori_param, posture_param);

  while (ros::ok())
  {
    try
    {
      bayes_registrator.broadcast_tf();
      ros::spinOnce();
      loop_rate.sleep();
    }
    catch (std::runtime_error e)
    {
      ROS_WARN_STREAM("runtime_error: " << e.what());
      continue;
    }
  }

  return 0;
}
