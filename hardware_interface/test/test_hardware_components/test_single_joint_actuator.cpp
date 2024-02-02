// Copyright 2020 ros2_control Development Team
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

#include <memory>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::ActuatorInterface;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

namespace test_hardware_components
{
class TestSingleJointActuator : public ActuatorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & actuator_info) override
  {
    if (ActuatorInterface::on_init(actuator_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // can only control one joint
    if (info_.joints.size() != 1)
    {
      return CallbackReturn::ERROR;
    }
    // can only control in position
    const auto & command_interfaces = info_.joints[0].command_interfaces;
    if (command_interfaces.size() != 1)
    {
      return CallbackReturn::ERROR;
    }
    if (command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      return CallbackReturn::ERROR;
    }
    // can only give feedback state for position and velocity
    const auto & state_interfaces = info_.joints[0].state_interfaces;
    if (state_interfaces.size() < 1)
    {
      return CallbackReturn::ERROR;
    }
    for (const auto & state_interface : state_interfaces)
    {
      if (
        (state_interface.name != hardware_interface::HW_IF_POSITION) &&
        (state_interface.name != hardware_interface::HW_IF_VELOCITY))
      {
        return CallbackReturn::ERROR;
      }
    }
    joint_name_ = info_.joints[0].name;
    joint_pos_ = joint_name_ + "/" + hardware_interface::HW_IF_POSITION;
    joint_vel_ = joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
    fprintf(stderr, "TestSingleJointActuator configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::InterfaceDescription> export_state_interfaces_2() override
  {
    std::vector<hardware_interface::InterfaceDescription> state_interfaces;
    hardware_interface::InterfaceInfo info;
    info.initial_value = "0.0";

    info.name = hardware_interface::HW_IF_POSITION;
    state_interfaces.push_back(hardware_interface::InterfaceDescription(joint_name_, info));
    info.name = hardware_interface::HW_IF_VELOCITY;
    state_interfaces.push_back(hardware_interface::InterfaceDescription(joint_name_, info));

    return state_interfaces;
  }

  std::vector<hardware_interface::InterfaceDescription> export_command_interfaces_2() override
  {
    std::vector<hardware_interface::InterfaceDescription> command_interfaces;
    hardware_interface::InterfaceInfo info;
    info.initial_value = "0.0";

    info.name = hardware_interface::HW_IF_POSITION;
    command_interfaces.push_back(hardware_interface::InterfaceDescription(joint_name_, info));

    return command_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    set_state(joint_vel_, get_command(joint_pos_) - get_state(joint_pos_));
    set_state(joint_pos_, get_command(joint_pos_));
    return return_type::OK;
  }

private:
  std::string joint_name_;
  std::string joint_pos_;
  std::string joint_vel_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestSingleJointActuator, hardware_interface::ActuatorInterface)
