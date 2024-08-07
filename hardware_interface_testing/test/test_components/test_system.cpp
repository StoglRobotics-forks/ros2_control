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

#include <array>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ros2_control_test_assets/test_hardware_interface_constants.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::SystemInterface;

class TestSystem : public SystemInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Simulating initialization error
    if (info_.joints[0].state_interfaces[1].name == "does_not_exist")
    {
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::InterfaceDescription> export_state_interface_descriptions()
    override
  {
    using hardware_interface::InterfaceDescription;
    using hardware_interface::InterfaceInfo;
    std::vector<InterfaceDescription> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(InterfaceDescription(
        info_.joints[i].name, InterfaceInfo(hardware_interface::HW_IF_POSITION)));
      state_interfaces.emplace_back(InterfaceDescription(
        info_.joints[i].name, InterfaceInfo(hardware_interface::HW_IF_VELOCITY)));
      state_interfaces.emplace_back(InterfaceDescription(
        info_.joints[i].name, InterfaceInfo(hardware_interface::HW_IF_ACCELERATION)));
    }

    if (info_.gpios.size() > 0)
    {
      // Add configuration/max_tcp_jerk interface
      state_interfaces.emplace_back(InterfaceDescription(
        info_.gpios[0].name, InterfaceInfo(info_.gpios[0].state_interfaces[0].name)));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::InterfaceDescription> export_command_interface_descriptions()
    override
  {
    using hardware_interface::InterfaceDescription;
    using hardware_interface::InterfaceInfo;
    std::vector<InterfaceDescription> command_interfaces;

    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(InterfaceDescription(
        info_.joints[i].name, InterfaceInfo(hardware_interface::HW_IF_VELOCITY)));
    }
    // Add max_acceleration command interface
    command_interfaces.emplace_back(InterfaceDescription(
      info_.joints[0].name, InterfaceInfo(info_.joints[0].command_interfaces[1].name)));

    if (info_.gpios.size() > 0)
    {
      // Add configuration/max_tcp_jerk interface
      command_interfaces.emplace_back(InterfaceDescription(
        info_.gpios[0].name, InterfaceInfo(info_.gpios[0].command_interfaces[0].name)));
    }

    return command_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // simulate error on read
    if (get_command(info_.joints[0].name + "/velocity") == test_constants::READ_FAIL_VALUE)
    {
      // reset value to get out from error on the next call - simplifies CM
      // tests
      set_command(info_.joints[0].name + "/velocity", 0.0);
      return return_type::ERROR;
    }
    // simulate deactivate on read
    if (get_command(info_.joints[0].name + "/velocity") == test_constants::READ_DEACTIVATE_VALUE)
    {
      return return_type::DEACTIVATE;
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // simulate error on write
    if (get_command(info_.joints[0].name + "/velocity") == test_constants::WRITE_FAIL_VALUE)
    {
      // reset value to get out from error on the next call - simplifies CM
      // tests
      set_command(info_.joints[0].name + "/velocity", 0.0);
      return return_type::ERROR;
    }
    // simulate deactivate on write
    if (get_command(info_.joints[0].name + "/velocity") == test_constants::WRITE_DEACTIVATE_VALUE)
    {
      return return_type::DEACTIVATE;
    }
    return return_type::OK;
  }

private:
  double max_acceleration_command_ = 0.0;
  double configuration_state_ = 0.0;
  double configuration_command_ = 0.0;
};

class TestUninitializableSystem : public TestSystem
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    SystemInterface::on_init(info);
    return CallbackReturn::ERROR;
  }
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(TestUninitializableSystem, hardware_interface::SystemInterface)
