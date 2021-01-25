#ifndef UTIL_H
#define UTIL_H

#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

typedef std::pair<moveit_msgs::RobotTrajectory, double> traj_cost;

namespace utils
{
bool compare_by_second(const traj_cost& a, const traj_cost& b)
{
  if (a.second != b.second)
  {
    return a.second < b.second;
  }
}

bool eef_limit_check(const int eef_joint_index, const double lower_limit, const double upper_limit,
                     const trajectory_msgs::JointTrajectory traj)
{
  for (int i = 0; i < traj.points.size(); i++)
  {
    if ((traj.points[i].positions[eef_joint_index] < lower_limit) ||
        (traj.points[i].positions[eef_joint_index] > upper_limit))
    {
      int dof = 6;
      for (int j = 0; j < dof; j++)
      {
        std::cout << "limit " << j << ": " << traj.points[i].positions[j] << std::endl;
      }
      return false;
    }
  }
  return true;
}
}

#endif
