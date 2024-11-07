// Copyright (c) 2024, Dyyt587
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "adog_legged_hardware_interface/adog_legged_hardware.hpp"
#include "adog_legged_hardware_interface/usb_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>

namespace adog_legged_hardware_interface
{
hardware_interface::CallbackReturn LegSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joint_torque_command_.assign(12, 0);
  joint_position_command_.assign(12, 0);
  joint_velocities_command_.assign(12, 0);
  joint_kp_command_.assign(12, 0);
  joint_kd_command_.assign(12, 0);

  joint_position_.assign(12, 0);
  joint_velocities_.assign(12, 0);
  joint_effort_.assign(12, 0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() < 5)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("LegSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 5 expected at lest.", joint.name.c_str(),
        joint.command_interfaces.size());
      // return hardware_interface::CallbackReturn::ERROR;
    }
    std::string cmd_name[] = {
      hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_POSITION, "kp", "kd"};
    for (unsigned int i = 0; i < joint.command_interfaces.size(); ++i)
    {
      if (joint.command_interfaces[i].name != cmd_name[i])
      {
        RCLCPP_WARN(
          rclcpp::get_logger("LegSystemHardware"),
          "Joint '%s' have %s command interfaces found(the %ds command). '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(), i + 1, cmd_name[i].c_str());
        // return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("LegSystemHardware"), "Joint '%s' has %zu state interface. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      // return hardware_interface::CallbackReturn::ERROR;
    }
    std::string state_name[] = {

      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT

    };
    for (unsigned int i = 0; i < joint.state_interfaces.size(); ++i)
    {
      if (joint.state_interfaces[i].name != state_name[i])
      {
        RCLCPP_WARN(
          rclcpp::get_logger("LegSystemHardware"),
          "Joint '%s' have %s state interfaces found(the %ds state). '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(), i + 1, state_name[i].c_str());
        // return hardware_interface::CallbackReturn::ERROR;
      }
    }
    for (const auto & interface : joint.state_interfaces)
    {
      joint_state_interfaces[interface.name].push_back(joint.name);
    }
    for (const auto & interface : joint.command_interfaces)
    {
      joint_command_interfaces[interface.name].push_back(joint.name);
    }
  }
  joint_torque_command_.assign(12, 0);
  joint_position_command_.assign(12, 0);
  joint_velocities_command_.assign(12, 0);
  joint_kp_command_.assign(12, 0);
  joint_kd_command_.assign(12, 0);

  joint_position_.assign(12, 0.0);
  joint_velocities_.assign(12, 0.0);
  joint_effort_.assign(12, 0.0);

  imu_states_.assign(10, 0);
  foot_force_.assign(4, 0);

  namespace fs = std::filesystem;

  std::vector<fs::path> tty_devices;
  const fs::path dev_path{"/dev"};  // 设备文件所在的目录

  try
  {
    // 遍历/dev目录下的所有文件
    for (const auto & entry : fs::directory_iterator(dev_path))
    {
      if (entry.is_character_file())
      {  // 检查是否为字符设备文件
        const auto & path = entry.path();
        if (path.filename().string().find("tty") == 0)
        {  // 检查文件名是否以"tty"开头
          tty_devices.push_back(path);
        }
      }
    }
  }
  catch (const fs::filesystem_error & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LegSystemHardware"), "Filesystem error");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 输出找到的TTY设备
  // for (const auto &path : tty_devices)
  // {
  //   // RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Found TTY device: %s ",
  //   path.string().c_str());
  // }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LegSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Configuring ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Successfully configured!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LegSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "12");
  int ind = 0;
  for (const auto & joint_name : joint_state_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_state_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_state_interfaces["effort"])
  {
    state_interfaces.emplace_back(joint_name, "effort", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LegSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "13");

  int ind = 0;
  for (const auto & joint_name : joint_command_interfaces["position"])
  {
    // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "17");

    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_command_interfaces["velocity"])
  {
    // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "18");

    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_command_interfaces["effort"])
  {
    // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "19");

    command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
    ind++;
  }
  ind = 0;
  for (const auto & joint_name : joint_command_interfaces["kp"])
  {
    // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "14");

    command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
    ind++;
  }
  ind = 0;
  for (const auto & joint_name : joint_command_interfaces["kd"])
  {
    // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "15");

    command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
    ind++;
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn LegSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Activating ...please wait...");

  // for (int i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //       rclcpp::get_logger("LegSystemHardware"), "%.1f seconds left...",
  //       hw_start_sec_ - i);
  // }
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  // // command and state should be equal when starting
  // for (uint i = 0; i < hw_states_.size(); i++)
  // {
  //   hw_commands_[i] = hw_states_[i];
  // }

  RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LegSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  static int i = 0;
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  if (i++ % 10000 == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Deactivating ...please wait...");
  }

  // for (int i = 0; i < hw_stop_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //       rclcpp::get_logger("LegSystemHardware"), "%.1f seconds left...",
  //       hw_stop_sec_ - i);
  // }

  if (i++ % 10000 == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Successfully deactivated!");
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LegSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  static int j = 0;
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  if (j++ % 100 == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Reading...");
  }

  if (j++ % 100 == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Joints successfully read!");
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  if (j % 100 == 0)
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("LegSystemHardware"), "Joint position command: "
                                                 << " ");
    for (auto array : joint_position_command_)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), array << " ");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), std::endl);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LegSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  joint_velocities_ = joint_velocities_command_;

  joint_position_ = joint_position_command_;

  joint_effort_ = joint_torque_command_;
  static int j = 0;
  j++;
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  if (j % 100 == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Writing...");
  }

  if (j % 100 == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Joints successfully written!");
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  if (j % 100 == 0)
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("LegSystemHardware"), "Joint position command: "
                                                 << " ");
    for (auto array : joint_position_command_)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), array << " ");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), std::endl);
  }
  if (j % 100 == 0)
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("LegSystemHardware"), "Joint velocity command: "
                                                 << " ");
    for (auto array : joint_velocities_command_)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), array << " ");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), std::endl);
  }
  if (j % 100 == 0)
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("LegSystemHardware"), "Joint torque command: "
                                                 << " ");
    for (auto array : joint_torque_command_)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), array << " ");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), std::endl);
  }
  if (j % 100 == 0)
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("LegSystemHardware"), "Joint kp command: "
                                                 << " ");
    for (auto array : joint_kp_command_)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), array << " ");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("LegSystemHardware"), std::endl);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace adog_legged_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  adog_legged_hardware_interface::LegSystemHardware, hardware_interface::SystemInterface)
