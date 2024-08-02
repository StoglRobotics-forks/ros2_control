// Copyright (c) 2021 PickNik, Inc.
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
//
// Author: Jafar Abdi, Denis Stogl

#ifndef MOCK_COMPONENTS__GENERIC_SYSTEM_HPP_
#define MOCK_COMPONENTS__GENERIC_SYSTEM_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"

using hardware_interface::return_type;

namespace mock_components
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr size_t POSITION_INTERFACE_INDEX = 0;
static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;
static constexpr size_t ACCELERATION_INTERFACE_INDEX = 2;

class HARDWARE_INTERFACE_PUBLIC GenericSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::InterfaceDescription> export_state_interface_descriptions()
    override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

protected:
  /// Use standard interfaces for joints because they are relevant for dynamic behavior
  /**
   * By splitting the standard interfaces from other type, the users are able to inherit this
   * class and simply create small "simulation" with desired dynamic behavior.
   * The advantage over using Gazebo is that enables "quick & dirty" tests of robot's URDF and
   * controllers.
   */
  const std::vector<std::string> standard_interfaces_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};
  // added dynamically during on_init
  std::vector<std::string> non_standard_interfaces_;

  struct MimicJoint
  {
    std::string joint_name;
    std::string mimic_joint_name;
    double multiplier = 1.0;
  };
  std::vector<MimicJoint> mimic_joints_;

  // All the joints that are of type defined by standard_interfaces_ vector -> In {pos, vel, acc,
  // effort}
  std::vector<std::string> std_joint_names_;
  std::unordered_set<std::string> std_joint_command_interface_names_;
  std::unordered_set<std::string> std_joint_state_interface_names_;

  // All the joints that are of not of a type defined by standard_interfaces_ vector -> Not in {pos,
  // vel, acc, effort}
  std::vector<std::string> other_joint_names_;
  std::unordered_set<std::string> other_joint_command_interface_names_;
  std::unordered_set<std::string> other_joint_state_interface_names_;

private:
  void search_and_add_interface_names(
    const std::vector<hardware_interface::ComponentInfo> & components,
    const std::vector<std::string> & interface_list, std::vector<std::string> & vector_to_add);

  bool use_mock_gpio_command_interfaces_;
  bool use_mock_sensor_command_interfaces_;

  double position_state_following_offset_;
  std::string custom_interface_with_following_offset_;
  std::string custom_interface_name_with_following_offset_;

  bool calculate_dynamics_;
  std::unordered_map<std::string, size_t> joint_control_mode_;

  bool command_propagation_disabled_;
};

typedef GenericSystem GenericRobot;

}  // namespace mock_components

#endif  // MOCK_COMPONENTS__GENERIC_SYSTEM_HPP_
