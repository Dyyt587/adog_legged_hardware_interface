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

#ifndef ADOG_LEGGED_HARDWARE_INTERFACE__ADOG_LEGGED_HARDWARE_HPP_
#define ADOG_LEGGED_HARDWARE_INTERFACE__ADOG_LEGGED_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "adog_legged_hardware_interface/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace adog_legged_hardware_interface
{
class LegSystemHardware : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> joint_torque_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_kp_command_;
  std::vector<double> joint_kd_command_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_effort_;

  std::vector<double> imu_states_;
  std::vector<double> foot_force_;

  std::unordered_map<std::string, std::vector<std::string> > joint_command_interfaces = {
    {"position", {}}, {"velocity", {}}, {"effort", {}}, {"kp", {}}, {"kd", {}}};
  std::unordered_map<std::string, std::vector<std::string> > joint_state_interfaces = {
    {"position", {}},
    {"velocity", {}},
    {"effort", {}},

  };
};

}  // namespace adog_legged_hardware_interface

#endif  // ADOG_LEGGED_HARDWARE_INTERFACE__ADOG_LEGGED_HARDWARE_HPP_
