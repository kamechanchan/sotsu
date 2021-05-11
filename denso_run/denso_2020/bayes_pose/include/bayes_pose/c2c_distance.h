#ifndef C2C_DISTANCE_H
#define C2C_DISTANCE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;

namespace bayes_pose
{
class C2CDistance
{
public:
  C2CDistance(ros::NodeHandle nh, const std::string& compared_pc_topic_name,
              const std::string& reference_pc_topic_name);
  void calculate(const sensor_msgs::PointCloud2ConstPtr& compared_pc,
                 const sensor_msgs::PointCloud2ConstPtr& reference_pc);

protected:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> compared_pc_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> reference_pc_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
};
}  // namespace bayes_pose

#endif  // C2C_DISTANCE_H
