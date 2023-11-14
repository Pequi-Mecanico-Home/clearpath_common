/**
 *
 *  \file
 *  \brief      Class representing W200 hardware
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include "clearpath_platform/w200/hardware.hpp"
#include "clearpath_platform_msgs/msg/feedback.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

static const std::string LEFT_CMD_JOINT_NAME = "front_left_wheel_joint";
static const std::string RIGHT_CMD_JOINT_NAME = "front_right_wheel_joint";
static const std::string LEFT_ALT_JOINT_NAME = "rear_left_wheel_joint";
static const std::string RIGHT_ALT_JOINT_NAME = "rear_right_wheel_joint";

namespace clearpath_platform
{

hardware_interface::CallbackReturn W200Hardware::initHardwareInterface()
{
  node_ = std::make_shared<W200HardwareInterface>("w200_hardware_interface");

  if (node_ == nullptr)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Write commanded velocities to the MCU
 * 
 */
void W200Hardware::writeCommandsToHardware()
{
  double diff_speed_left = hw_commands_[wheel_joints_[LEFT_CMD_JOINT_NAME]];
  double diff_speed_right = hw_commands_[wheel_joints_[RIGHT_CMD_JOINT_NAME]];

  if (std::abs(diff_speed_left) < 0.01 && std::abs(diff_speed_right) < 0.01) {
    diff_speed_left = diff_speed_right = 0.0;
  }

  node_->drive_command(
    static_cast<float>(diff_speed_left),
    static_cast<float>(diff_speed_right),
    1);
}

/**
 * @brief Pull latest speed and travel measurements from MCU, 
 * and store in joint structure for ros_control
 * 
 */
void W200Hardware::updateJointsFromHardware()
{
  rclcpp::spin_some(node_);

  if (node_->has_new_feedback())
  {
    auto left_msg = node_->get_left_feedback();
    auto right_msg = node_->get_right_feedback();
    RCLCPP_DEBUG(
      rclcpp::get_logger(hw_name_),
      "Received linear distance information (L: %f, R: %f)",
      left_msg.data, right_msg.data);
    
    hw_states_velocity_[wheel_joints_[LEFT_CMD_JOINT_NAME]] = left_msg.data;
    hw_states_velocity_[wheel_joints_[RIGHT_CMD_JOINT_NAME]] = right_msg.data;
  }
}


}  // namespace clearpath_platform

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  clearpath_platform::W200Hardware, hardware_interface::SystemInterface)
