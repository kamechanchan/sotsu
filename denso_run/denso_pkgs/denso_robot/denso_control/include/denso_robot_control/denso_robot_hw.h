/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _DENSO_ROBOT_HW_H_
#define _DENSO_ROBOT_HW_H_

#include <ros/ros.h>

// Message (std_msgs)
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std_msgs;

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits_interface.h>

#include "denso_robot_core/denso_robot_core.h"
#include "denso_robot_core/denso_controller.h"
#include "denso_robot_core/denso_robot_rc8.h"
#include "denso_robot_core/denso_variable.h"
#include "denso_robot_core/UserIO.h"
using namespace denso_robot_core;

#include <boost/thread.hpp>

namespace denso_robot_control
{
class DensoRobotHW : public hardware_interface::RobotHW
{
public:
  DensoRobotHW();
  virtual ~DensoRobotHW();

  HRESULT initialize();

  ros::Time getTime() const
  {
    return ros::Time::now();
  }

  ros::Duration getPeriod() const
  {
    return ros::Duration(0.008);
  }

  void read(ros::Time, ros::Duration);
  void write(ros::Time, ros::Duration);

private:
  HRESULT changeModeWithClearError(int mode);
  void callbackChangeMode(const Int32::ConstPtr& msg);

  HRESULT checkRobotType();

  void callbackMiniIO(const Int32::ConstPtr& msg);
  void callbackHandIO(const Int32::ConstPtr& msg);
  void callbackSendUserIO(const UserIO::ConstPtr& msg);
  void callbackRecvUserIO(const UserIO::ConstPtr& msg);

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  std::vector<std::string> joint_names_;
  std::vector<int> joints_type_;
  std::vector<double> joints_cmd_;
  std::vector<double> joints_pos_;
  std::vector<double> joints_vel_;
  std::vector<double> joints_eff_;
  std::vector<double> joints_value_;

  DensoRobotCore_Ptr denso_eng_;
  DensoController_Ptr denso_ctrl_;
  DensoRobotRC8_Ptr denso_rob_;
  DensoVariable_Ptr denso_varerr_;

  std::string robot_name_;
  std::string real_robot_name_;
  int robot_joints_;
  int arm_joints_;
  int gripper_joints_;
  int send_fmt_;
  int recv_fmt_;

  ros::Subscriber change_Mode_sub_;
  ros::Subscriber MiniIO_sub_;
  ros::Subscriber HandIO_sub_;
  ros::Subscriber send_UserIO_sub_;
  ros::Subscriber recv_UserIO_sub_;

  ros::Publisher cur_Mode_pub_;
  ros::Publisher MiniIO_pub_;
  ros::Publisher HandIO_pub_;
  ros::Publisher recv_UserIO_pub_;
  ros::Publisher current_pub_;

  boost::mutex mtx_Mode_;
};
}  // namespace denso_robot_control

#endif  // DENSO_ROBOT_HW_H
