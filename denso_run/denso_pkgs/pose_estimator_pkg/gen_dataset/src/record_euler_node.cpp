#include <gen_dataset/record_euler.h>

using record_euler::RecordEuler;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "record_euler_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RecordEuler record_euler(nh);

  ros::Rate rate(1);
  while (ros::ok())
  {
    if (!record_euler.runRecord())
    {
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
