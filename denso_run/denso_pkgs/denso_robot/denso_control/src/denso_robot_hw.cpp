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

#include <string.h>
#include <XmlRpcValue.h>
#include "denso_robot_control/denso_robot_hw.h"

#define RAD2DEG(x) ((x)*180.0 / M_PI)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)

#define M2MM(x) ((x)*1000.0)
#define MM2M(x) ((x) / 1000.0)

namespace denso_robot_control
{
DensoRobotHW::DensoRobotHW() : real_robot_name_(""), arm_joints_(0), gripper_joints_(0), robot_joints_(0)
{
  ros::param::param<std::string>("~robot_name", robot_name_, "vs087");
  joint_names_.clear();
  joints_type_.clear();
  joints_cmd_.clear();
  joints_pos_.clear();
  joints_vel_.clear();
  joints_eff_.clear();
  joints_value_.clear();

  denso_eng_ = boost::make_shared<DensoRobotCore>();
  denso_ctrl_.reset();
  denso_rob_.reset();
  denso_varerr_.reset();

  send_fmt_ = DensoRobotRC8::SENDFMT_MINIIO | DensoRobotRC8::SENDFMT_HANDIO;
  recv_fmt_ = DensoRobotRC8::RECVFMT_POSE_PJ | DensoRobotRC8::RECVFMT_MINIIO | DensoRobotRC8::RECVFMT_HANDIO;
}

DensoRobotHW::~DensoRobotHW()
{
}

HRESULT DensoRobotHW::initialize()
{
  ros::NodeHandle nh;

  if (!nh.getParam("real_robot_name", real_robot_name_))
  {
    ROS_WARN("Failed to get real_robot_name parameter.");
  }

  XmlRpc::XmlRpcValue arm_joints_list;

  if (!nh.getParam("/" + robot_name_ + "/arm_controller/joints", arm_joints_list))
  {
    ROS_WARN("Failed to get robot arm joints parameter.");
  }
  else
  {
    arm_joints_ = arm_joints_list.size();
    for (int i = 0; i < arm_joints_; i++)
    {
      joint_names_.push_back(arm_joints_list[i]);
    }
  }

  XmlRpc::XmlRpcValue gripper_joints_list;

  if (!nh.getParam("/" + robot_name_ + "/gripper_controller/joints", gripper_joints_list))
  {
    ROS_WARN("Failed to get robot gripper joints parameter.");
  }
  else
  {
    gripper_joints_ = gripper_joints_list.size();
    for (int i = 0; i < gripper_joints_; i++)
    {
      joint_names_.push_back(gripper_joints_list[i]);
    }
  }

  robot_joints_ = arm_joints_ + gripper_joints_;

  joints_type_.resize(robot_joints_);
  joints_cmd_.resize(robot_joints_);
  joints_pos_.resize(robot_joints_);
  joints_vel_.resize(robot_joints_);
  joints_eff_.resize(robot_joints_);
  joints_value_.resize(robot_joints_);

  for (int i = 0; i < robot_joints_; i++)
  {
    if (!nh.getParam(joint_names_[i], joints_type_[i]))
    {
      ROS_WARN_STREAM("Failed to get " << joint_names_[i] << " parameter.");
      ROS_WARN("It was assumed revolute type.");
      joints_type_[i] = 1;
    }

    hardware_interface::JointStateHandle state_handle(joint_names_[i], &joints_pos_[i], &joints_vel_[i],
                                                      &joints_eff_[i]);
    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_names_[i]), &joints_cmd_[i]);
    position_joint_interface_.registerHandle(pos_handle);
  }

  int arm_group = 0;
  if (!nh.getParam("arm_group", arm_group))
  {
    ROS_INFO("Use arm group 0.");
    arm_group = 0;
  }

  int gripper_group = 0;
  if (!nh.getParam("gripper_group", gripper_group))
  {
    ROS_INFO("Use gripper group 0.");
    gripper_group = 0;
  }

  int format = 0;
  if (nh.getParam("send_format", format))
  {
    send_fmt_ = format;
  }

  format = 0;
  if (nh.getParam("recv_format", format))
  {
    recv_fmt_ = format;
  }
  switch (recv_fmt_ & DensoRobotRC8::RECVFMT_POSE)
  {
    case DensoRobotRC8::RECVFMT_POSE_J:
    case DensoRobotRC8::RECVFMT_POSE_PJ:
    case DensoRobotRC8::RECVFMT_POSE_TJ:
      break;
    default:
      ROS_WARN("Recieve format has to contain joint.");
      recv_fmt_ = ((recv_fmt_ & ~DensoRobotRC8::RECVFMT_POSE) | DensoRobotRC8::RECVFMT_POSE_J);
      break;
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  HRESULT hr = denso_eng_->Initialize();
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to connect real controller. (%X)", hr);
    return hr;
  }

  denso_ctrl_ = denso_eng_->get_Controller();

  DensoRobot_Ptr pRob;
  hr = denso_ctrl_->get_Robot(DensoBase::SRV_ACT, &pRob);
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to connect real robot. (%X)", hr);
    return hr;
  }

  denso_rob_ = boost::dynamic_pointer_cast<DensoRobotRC8>(pRob);

  hr = checkRobotType();
  if (FAILED(hr))
  {
    ROS_ERROR("Invalid robot type.");
    return hr;
  }

  denso_rob_->ChangeArmGroup(arm_group);

  hr = denso_rob_->ExecCurJnt(joints_value_);
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to get current joint. (%X)", hr);
    return hr;
  }

  hr = denso_ctrl_->AddVariable("@ERROR_CODE");
  if (SUCCEEDED(hr))
  {
    hr = denso_ctrl_->get_Variable("@ERROR_CODE", &denso_varerr_);
  }
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to get @ERROR_CODE object. (%X)", hr);
    return hr;
  }

  {
    // Clear Error
    VARIANT_Ptr vntVal(new VARIANT());
    vntVal->vt = VT_I4;
    vntVal->lVal = 0L;
    hr = denso_varerr_->ExecPutValue(vntVal);
    if (FAILED(hr))
    {
      ROS_ERROR("Failed to clear error. (%X)", hr);
      return hr;
    }
  }

  hr = denso_rob_->AddVariable("@SERVO_ON");
  if (SUCCEEDED(hr))
  {
    DensoVariable_Ptr pVar;
    hr = denso_rob_->get_Variable("@SERVO_ON", &pVar);
    if (SUCCEEDED(hr))
    {
      VARIANT_Ptr vntVal(new VARIANT());
      vntVal->vt = VT_BOOL;
      vntVal->boolVal = VARIANT_TRUE;
      hr = pVar->ExecPutValue(vntVal);
    }
  }
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to motor on. (%X)", hr);
    return hr;
  }

  denso_rob_->put_SendFormat(send_fmt_);
  send_fmt_ = denso_rob_->get_SendFormat();

  denso_rob_->put_RecvFormat(recv_fmt_);
  recv_fmt_ = denso_rob_->get_RecvFormat();

  change_Mode_sub_ = nh.subscribe<Int32>("ChangeMode", 1, &DensoRobotHW::callbackChangeMode, this);
  cur_Mode_pub_ = nh.advertise<Int32>("CurMode", 1);

  hr = changeModeWithClearError(DensoRobotRC8::SLVMODE_SYNC_WAIT | DensoRobotRC8::SLVMODE_POSE_J);
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to change to slave mode. (%X)", hr);
    return hr;
  }

  return S_OK;
}

HRESULT DensoRobotHW::changeModeWithClearError(int mode)
{
  HRESULT hr = denso_eng_->ChangeMode(mode, mode == DensoRobotRC8::SLVMODE_NONE);
  if (FAILED(hr))
  {
    // Clear Error
    VARIANT_Ptr vntVal(new VARIANT());
    vntVal->vt = VT_I4;
    vntVal->lVal = 0L;
    denso_varerr_->ExecPutValue(vntVal);
  }

  Int32 msg;
  msg.data = denso_eng_->get_Mode();
  cur_Mode_pub_.publish(msg);

  if (msg.data == DensoRobotRC8::SLVMODE_NONE)
  {
    MiniIO_sub_.shutdown();
    HandIO_sub_.shutdown();
    send_UserIO_sub_.shutdown();
    recv_UserIO_sub_.shutdown();
    MiniIO_pub_.shutdown();
    HandIO_pub_.shutdown();
    recv_UserIO_pub_.shutdown();
    current_pub_.shutdown();
  }
  else
  {
    ros::NodeHandle nh;

    if (send_fmt_ & DensoRobotRC8::SENDFMT_HANDIO)
    {
      HandIO_sub_ = nh.subscribe<Int32>("Write_HandIO", 1, &DensoRobotHW::callbackHandIO, this);
    }
    if (send_fmt_ & DensoRobotRC8::SENDFMT_MINIIO)
    {
      MiniIO_sub_ = nh.subscribe<Int32>("Write_MiniIO", 1, &DensoRobotHW::callbackMiniIO, this);
    }
    if (send_fmt_ & DensoRobotRC8::SENDFMT_USERIO)
    {
      send_UserIO_sub_ = nh.subscribe<UserIO>("Write_SendUserIO", 1, &DensoRobotHW::callbackSendUserIO, this);
    }
    if (recv_fmt_ & DensoRobotRC8::RECVFMT_HANDIO)
    {
      HandIO_pub_ = nh.advertise<Int32>("Read_HandIO", 1);
    }
    if (recv_fmt_ & DensoRobotRC8::RECVFMT_CURRENT)
    {
      current_pub_ = nh.advertise<Float64MultiArray>("Read_Current", 1);
    }
    if (recv_fmt_ & DensoRobotRC8::RECVFMT_MINIIO)
    {
      MiniIO_pub_ = nh.advertise<Int32>("Read_MiniIO", 1);
    }
    if (recv_fmt_ & DensoRobotRC8::RECVFMT_USERIO)
    {
      recv_UserIO_sub_ = nh.subscribe<UserIO>("Write_RecvUserIO", 1, &DensoRobotHW::callbackRecvUserIO, this);
      recv_UserIO_pub_ = nh.advertise<UserIO>("Read_RecvUserIO", 1);
    }
  }

  return hr;
}

void DensoRobotHW::callbackChangeMode(const Int32::ConstPtr& msg)
{
  boost::mutex::scoped_lock lockMode(mtx_Mode_);

  ROS_INFO("Change to mode %d.", msg->data);
  HRESULT hr = changeModeWithClearError(msg->data);
  if (FAILED(hr))
  {
    ROS_ERROR("Failed to change mode. (%X)", hr);
  }
}

HRESULT DensoRobotHW::checkRobotType()
{
  DensoVariable_Ptr pVar;
  VARIANT_Ptr vntVal(new VARIANT());
  std::string strTypeName = "@TYPE_NAME";

  HRESULT hr = denso_rob_->AddVariable(strTypeName);
  if (SUCCEEDED(hr))
  {
    denso_rob_->get_Variable(strTypeName, &pVar);
    hr = pVar->ExecGetValue(vntVal);
    if (SUCCEEDED(hr))
    {
      strTypeName = DensoBase::ConvertBSTRToString(vntVal->bstrVal);
      if (strncmp(real_robot_name_.c_str(), strTypeName.c_str(), (real_robot_name_.length() < strTypeName.length()) ?
                                                                     real_robot_name_.length() :
                                                                     strTypeName.length()))
      {
        hr = E_FAIL;
      }
    }
  }

  return hr;
}

void DensoRobotHW::read(ros::Time time, ros::Duration period)
{
  boost::mutex::scoped_lock lockMode(mtx_Mode_);

  if (denso_eng_->get_Mode() == DensoRobotRC8::SLVMODE_NONE)
  {
    HRESULT hr = denso_rob_->ExecCurJnt(joints_value_);
    if (FAILED(hr))
    {
      ROS_ERROR("Failed to get current joint. (%X)", hr);
    }
  }

  for (int i = 0; i < arm_joints_; i++)
  {
    switch (joints_type_[i])
    {
      case 0:  // prismatic
        joints_pos_[i] = MM2M(joints_value_[i]);
        break;
      case 1:  // revolute
        joints_pos_[i] = DEG2RAD(joints_value_[i]);
        break;
      case -1:  // fixed
      default:
        joints_pos_[i] = 0.0;
        break;
    }
  }
}

void DensoRobotHW::write(ros::Time time, ros::Duration period)
{
  boost::mutex::scoped_lock lockMode(mtx_Mode_);

  if (denso_eng_->get_Mode() != DensoRobotRC8::SLVMODE_NONE)
  {
    std::vector<double> pose;
    pose.resize(arm_joints_);
    int bits = 0x0000;
    for (int i = 0; i < arm_joints_; i++)
    {
      switch (joints_type_[i])
      {
        case 0:  // prismatic
          pose[i] = M2MM(joints_cmd_[i]);
          break;
        case 1:  // revolute
          pose[i] = RAD2DEG(joints_cmd_[i]);
          break;
        case -1:  // fixed
        default:
          pose[i] = 0.0;
          break;
      }

      bits |= (1 << i);
    }
    pose.push_back(0x400000 | bits);
    HRESULT hr = denso_rob_->ExecSlaveMove(pose, joints_value_);
    if (SUCCEEDED(hr))
    {
      if (recv_fmt_ & DensoRobotRC8::RECVFMT_HANDIO)
      {
        Int32 msg;
        msg.data = denso_rob_->get_HandIO();
        HandIO_pub_.publish(msg);
      }
      if (recv_fmt_ & DensoRobotRC8::RECVFMT_CURRENT)
      {
        Float64MultiArray msg;
        denso_rob_->get_Current(msg.data);
        current_pub_.publish(msg);
      }
      if (recv_fmt_ & DensoRobotRC8::RECVFMT_MINIIO)
      {
        Int32 msg;
        msg.data = denso_rob_->get_MiniIO();
        MiniIO_pub_.publish(msg);
      }
      if (recv_fmt_ & DensoRobotRC8::RECVFMT_USERIO)
      {
        UserIO msg;
        denso_rob_->get_RecvUserIO(msg);
        recv_UserIO_pub_.publish(msg);
      }
    }
    else if (FAILED(hr) && (hr != E_BUF_FULL))
    {
      ROS_ERROR("Failed to write. (%X)", hr);

      VARIANT_Ptr vntVal(new VARIANT());
      hr = denso_varerr_->ExecGetValue(vntVal);
      if (FAILED(hr) || (vntVal->lVal != 0))
      {
        ROS_ERROR("Automatically change to normal mode.");
        changeModeWithClearError(DensoRobotRC8::SLVMODE_NONE);
      }
    }
  }
}

void DensoRobotHW::callbackMiniIO(const Int32::ConstPtr& msg)
{
  denso_rob_->put_MiniIO(msg->data);
}

void DensoRobotHW::callbackHandIO(const Int32::ConstPtr& msg)
{
  denso_rob_->put_HandIO(msg->data);
}

void DensoRobotHW::callbackSendUserIO(const UserIO::ConstPtr& msg)
{
  denso_rob_->put_SendUserIO(*msg.get());
}

void DensoRobotHW::callbackRecvUserIO(const UserIO::ConstPtr& msg)
{
  denso_rob_->put_RecvUserIO(*msg.get());
}
}
