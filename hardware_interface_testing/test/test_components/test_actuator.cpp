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
#include "ros2_control_test_assets/test_hardware_interface_constants.hpp"

using hardware_interface::ActuatorInterface;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

class TestActuator : public ActuatorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    /*
     * a hardware can optional prove for incorrect info here.
     *
     * // can only control one joint
     * if (info_.joints.size() != 1) {return CallbackReturn::ERROR;}
     * // can only control in position
     * if (info_.joints[0].command_interfaces.size() != 1) {return
     * CallbackReturn::ERROR;}
     * // can only give feedback state for position and velocity
     * if (info_.joints[0].state_interfaces.size() != 2) {return
     * CallbackReturn::ERROR;}
     */

    pos_state_ = info_.joints[0].name + "/position";
    vel_state_ = info_.joints[0].name + "/velocity";
    vel_command_ = info_.joints[0].name + "/velocity";

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::InterfaceDescription> export_state_interfaces_2() override
  {
    std::vector<hardware_interface::InterfaceDescription> interfaces;
    hardware_interface::InterfaceInfo info;
    info.name = "some_unlisted_interface";
    hardware_interface::InterfaceDescription unlisted_state_interface(info_.joints[0].name, info);
    interfaces.push_back(unlisted_state_interface);

    return interfaces;
  }

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) override
  {
    set_state(pos_state_, get_state(pos_state_) + 1.0);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) override
  {
    set_state(pos_state_, get_state(pos_state_) + 100.0);
    return hardware_interface::return_type::OK;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // simulate error on read
    if (get_command(vel_command_) == test_constants::READ_FAIL_VALUE)
    {
      // reset value to get out from error on the next call - simplifies CM
      // tests
      set_command(vel_command_, 0.0);
      return return_type::ERROR;
    }
    // simulate deactivate on read
    if (get_command(vel_command_) == test_constants::READ_DEACTIVATE_VALUE)
    {
      return return_type::DEACTIVATE;
    }
    // The next line is for the testing purposes. We need value to be changed to
    // be sure that the feedback from hardware to controllers in the chain is
    // working as it should. This makes value checks clearer and confirms there
    // is no "state = command" line or some other mixture of interfaces
    // somewhere in the test stack.
    set_state(vel_state_, get_command(vel_command_) / 2);
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // simulate error on write
    if (get_command(vel_command_) == test_constants::WRITE_FAIL_VALUE)
    {
      // reset value to get out from error on the next call - simplifies CM
      // tests
      set_command(vel_command_, 0.0);
      return return_type::ERROR;
    }
    // simulate deactivate on write
    if (get_command(vel_command_) == test_constants::WRITE_DEACTIVATE_VALUE)
    {
      return return_type::DEACTIVATE;
    }
    return return_type::OK;
  }

private:
  std::string pos_state_;
  std::string vel_state_;
  std::string vel_command_;
};

class TestUninitializableActuator : public TestActuator
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    ActuatorInterface::on_init(info);
    return CallbackReturn::ERROR;
  }
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestActuator, hardware_interface::ActuatorInterface)
PLUGINLIB_EXPORT_CLASS(TestUninitializableActuator, hardware_interface::ActuatorInterface)
