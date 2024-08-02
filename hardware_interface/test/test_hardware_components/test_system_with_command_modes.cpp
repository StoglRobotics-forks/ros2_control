// Copyright 2021 Department of Engineering Cybernetics, NTNU
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

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace test_hardware_components
{
class TestSystemCommandModes : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override
  {
    if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Can only control two joints
    if (info_.joints.size() != 2)
    {
      return CallbackReturn::ERROR;
    }
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      // Can control in position or velocity
      const auto & command_interfaces = joint.command_interfaces;
      if (command_interfaces.size() != 2)
      {
        return CallbackReturn::ERROR;
      }
      for (const auto & command_interface : command_interfaces)
      {
        if (
          command_interface.name != hardware_interface::HW_IF_POSITION &&
          command_interface.name != hardware_interface::HW_IF_VELOCITY)
        {
          return CallbackReturn::ERROR;
        }
      }
      // Can give feedback state for position, velocity, and acceleration
      const auto & state_interfaces = joint.state_interfaces;
      if (state_interfaces.size() != 2)
      {
        return CallbackReturn::ERROR;
      }
      for (const auto & state_interface : state_interfaces)
      {
        if (
          state_interface.name != hardware_interface::HW_IF_POSITION &&
          state_interface.name != hardware_interface::HW_IF_VELOCITY)
        {
          return CallbackReturn::ERROR;
        }
      }
    }

    fprintf(stderr, "TestSystemCommandModes configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::InterfaceDescription> export_state_interface_descriptions()
    override
  {
    std::vector<hardware_interface::InterfaceDescription> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      hardware_interface::InterfaceInfo info;
      info.initial_value = "0.0";

      info.name = hardware_interface::HW_IF_POSITION;
      state_interfaces.push_back(
        hardware_interface::InterfaceDescription(info_.joints[i].name, info));
      info.name = hardware_interface::HW_IF_VELOCITY;
      state_interfaces.push_back(
        hardware_interface::InterfaceDescription(info_.joints[i].name, info));
      info.name = hardware_interface::HW_IF_ACCELERATION;
      state_interfaces.push_back(
        hardware_interface::InterfaceDescription(info_.joints[i].name, info));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::InterfaceDescription> export_command_interface_descriptions()
    override
  {
    std::vector<hardware_interface::InterfaceDescription> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      hardware_interface::InterfaceInfo info;
      info.initial_value = "0.0";

      info.name = hardware_interface::HW_IF_POSITION;
      command_interfaces.push_back(
        hardware_interface::InterfaceDescription(info_.joints[i].name, info));
      info.name = hardware_interface::HW_IF_VELOCITY;
      command_interfaces.push_back(
        hardware_interface::InterfaceDescription(info_.joints[i].name, info));
    }

    return command_interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    const auto first_joint_acc =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const auto old_acc = get_state(first_joint_acc);
    set_state(first_joint_acc, old_acc + 1.0);

    // Starting interfaces
    start_modes_.clear();
    stop_modes_.clear();
    for (const auto & key : start_interfaces)
    {
      for (auto i = 0u; i < info_.joints.size(); i++)
      {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
        {
          start_modes_.push_back(hardware_interface::HW_IF_POSITION);
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
        {
          start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
        }
      }
    }
    // Example Criteria 1 - Starting: All interfaces must be given a new mode at the same time
    if (start_modes_.size() != 0 && start_modes_.size() != info_.joints.size())
    {
      return hardware_interface::return_type::ERROR;
    }

    // Stopping interfaces
    for (const auto & key : stop_interfaces)
    {
      for (auto i = 0u; i < info_.joints.size(); i++)
      {
        if (key.find(info_.joints[i].name) != std::string::npos)
        {
          stop_modes_.push_back(true);
        }
      }
    }
    // Example Criteria 2 - Stopping: All joints must have the same command mode
    if (stop_modes_.size() != 0 && stop_modes_.size() != 2 && stop_modes_[0] != stop_modes_[1])
    {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & /*stop_interfaces*/) override
  {
    const auto first_joint_acc =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const auto old_acc = get_state(first_joint_acc);
    set_state(first_joint_acc, old_acc + 100.0);
    // Test of failure in perform command mode switch
    // Fail if given an empty list.
    // This should never occur in a real system as the same start_interfaces list is sent to both
    // prepare and perform, and an error should be handled in prepare.
    if (start_interfaces.size() == 0)
    {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<std::string> start_modes_ = {"position", "position"};
  std::vector<bool> stop_modes_ = {false, false};
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestSystemCommandModes, hardware_interface::SystemInterface)
