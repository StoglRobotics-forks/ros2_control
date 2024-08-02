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

#include <algorithm>
#include <cmath>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"

using hardware_interface::InterfaceDescription;
using hardware_interface::return_type;
using hardware_interface::SensorInterface;

namespace test_hardware_components
{
class TestForceTorqueSensor : public SensorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & sensor_info) override
  {
    if (SensorInterface::on_init(sensor_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    const auto & state_interfaces = info_.sensors[0].state_interfaces;
    if (state_interfaces.size() != 6)
    {
      return CallbackReturn::ERROR;
    }
    for (const auto & ft_key : {"fx", "fy", "fz", "tx", "ty", "tz"})
    {
      if (
        std::find_if(
          state_interfaces.begin(), state_interfaces.end(), [&ft_key](const auto & interface_info)
          { return interface_info.name == ft_key; }) == state_interfaces.end())
      {
        return CallbackReturn::ERROR;
      }
    }

    sensor_name_ = info_.sensors[0].name;
    fprintf(stderr, "TestForceTorqueSensor configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::InterfaceDescription> export_state_interface_descriptions()
    override
  {
    std::vector<hardware_interface::InterfaceDescription> state_interfaces;

    hardware_interface::InterfaceInfo info;
    info.initial_value = "0.0";

    for (const auto & interface_name : inteface_names_)
    {
      info.name = interface_name;
      state_interfaces.push_back(hardware_interface::InterfaceDescription(sensor_name_, info));
    }

    return state_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    for (const auto & interface_name : inteface_names_)
    {
      const auto name = sensor_name_ + "/" + interface_name;

      set_state(name, fmod((get_state(name) + 1.0), 10));
    }
    return return_type::OK;
  }

private:
  std::vector<std::string> inteface_names_{"fx", "fy", "fz", "tx", "ty", "tz"};
  std::string sensor_name_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestForceTorqueSensor, hardware_interface::SensorInterface)
