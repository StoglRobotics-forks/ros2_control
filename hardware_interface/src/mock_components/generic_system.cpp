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

#include "mock_components/generic_system.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

namespace mock_components
{

CallbackReturn GenericSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  auto populate_non_standard_interfaces =
    [this](auto interface_list, auto & non_standard_interfaces)
  {
    for (const auto & interface : interface_list)
    {
      // add to list if non-standard interface
      if (
        std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end())
      {
        if (
          std::find(
            non_standard_interfaces.begin(), non_standard_interfaces.end(), interface.name) ==
          non_standard_interfaces.end())
        {
          non_standard_interfaces.emplace_back(interface.name);
        }
      }
    }
  };
  // check if to create mock command interface for sensor
  auto it = info_.hardware_parameters.find("mock_sensor_commands");
  if (it != info_.hardware_parameters.end())
  {
    use_mock_sensor_command_interfaces_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    use_mock_sensor_command_interfaces_ = false;
  }

  // check if to create mock command interface for gpio
  it = info_.hardware_parameters.find("mock_gpio_commands");
  if (it != info_.hardware_parameters.end())
  {
    use_mock_gpio_command_interfaces_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    use_mock_gpio_command_interfaces_ = false;
  }

  // check if there is parameter that disables commands
  // this way we simulate disconnected driver
  it = info_.hardware_parameters.find("disable_commands");
  if (it != info.hardware_parameters.end())
  {
    command_propagation_disabled_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    command_propagation_disabled_ = false;
  }

  // check if there is parameter that enables dynamic calculation
  it = info_.hardware_parameters.find("calculate_dynamics");
  if (it != info.hardware_parameters.end())
  {
    calculate_dynamics_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    calculate_dynamics_ = false;
  }

  // process parameters about state following
  position_state_following_offset_ = 0.0;
  custom_interface_with_following_offset_ = "";

  it = info_.hardware_parameters.find("position_state_following_offset");
  if (it != info_.hardware_parameters.end())
  {
    position_state_following_offset_ = hardware_interface::stod(it->second);
    it = info_.hardware_parameters.find("custom_interface_with_following_offset");
    if (it != info_.hardware_parameters.end())
    {
      custom_interface_with_following_offset_ = it->second;
    }
  }

  // search for non-standard joint interfaces
  for (const auto & joint : info_.joints)
  {
    // populate non-standard command interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.command_interfaces, non_standard_interfaces_);

    // populate non-standard state interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.state_interfaces, non_standard_interfaces_);
  }

  // search for standard joint interfaces and add them to std_joint_names_ and
  // search for non-standard joint interfaces and add them to other_joint_names_
  for (const auto & joint : info.joints)
  {
    for (const auto & state_inteface : joint.state_interfaces)
    {
      if (
        std::find(standard_interfaces_.begin(), standard_interfaces_.end(), state_inteface.name) ==
        standard_interfaces_.end())
      {
        std_joint_names_.push_back(joint.name);
        std_joint_state_interface_names_.insert(joint.name + "/" + state_inteface.name);
      }
      else if (
        std::find(
          non_standard_interfaces_.begin(), non_standard_interfaces_.end(), state_inteface.name) ==
        non_standard_interfaces_.end())
      {
        other_joint_names_.push_back(joint.name);
        other_joint_state_interface_names_.insert(joint.name + "/" + state_inteface.name);
      }
      else
      {
        RCUTILS_LOG_WARN_NAMED(
          "The state_interface: '%s' of the joint: '%s' is neither part of the std_interfaces "
          "(pos, vel, acc, eff), nor of any other use "
          "defined.",
          state_inteface.name.c_str(), joint.name.c_str());
      }
    }

    // Check that for all the available state_interfaces a command_interface exists
    // We don't need to add name of the joint again since it has already been added for the
    // state_interface
    for (const auto & command_interface : joint.command_interfaces)
    {
      if (
        std::find(
          std_joint_state_interface_names_.begin(), std_joint_state_interface_names_.end(),
          command_interface.name) == std_joint_state_interface_names_.end())
      {
        std_joint_command_interface_names_.insert(joint.name + "/" + command_interface.name);
      }
      else if (
        std::find(
          other_joint_state_interface_names_.begin(), other_joint_state_interface_names_.end(),
          command_interface.name) == other_joint_state_interface_names_.end())
      {
        other_joint_command_interface_names_.insert(joint.name + "/" + command_interface.name);
      }
      else
      {
        throw std::runtime_error(
          std::string("For command_interface: '") + command_interface.name + "' of the joint: '" +
          joint.name + "' exists no state_interface");
      }
    }
  }

  // when following offset is used on custom interface then find its index
  custom_interface_name_with_following_offset_ = "";
  if (!custom_interface_with_following_offset_.empty())
  {
    auto if_it = std::find(
      non_standard_interfaces_.begin(), non_standard_interfaces_.end(),
      custom_interface_with_following_offset_);
    if (if_it != non_standard_interfaces_.end())
    {
      custom_interface_name_with_following_offset_ = *if_it;
      RCUTILS_LOG_INFO_NAMED(
        "mock_generic_system", "Custom interface with following offset '%s' found at index: %s.",
        custom_interface_with_following_offset_.c_str(),
        custom_interface_name_with_following_offset_.c_str());
    }
    else
    {
      RCUTILS_LOG_WARN_NAMED(
        "mock_generic_system",
        "Custom interface with following offset '%s' does not exist. Offset will not be applied",
        custom_interface_with_following_offset_.c_str());
    }
  }

  // Search for mimic joints
  for (auto i = 0u; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints.at(i);
    if (joint.parameters.find("mimic") != joint.parameters.cend())
    {
      const auto mimicked_joint_it = std::find_if(
        info_.joints.begin(), info_.joints.end(),
        [&mimicked_joint =
           joint.parameters.at("mimic")](const hardware_interface::ComponentInfo & joint_info)
        { return joint_info.name == mimicked_joint; });
      if (mimicked_joint_it == info_.joints.cend())
      {
        throw std::runtime_error(
          std::string("Mimicked joint '") + joint.parameters.at("mimic") + "' not found");
      }
      MimicJoint mimic_joint;
      mimic_joint.joint_name = joint.name;
      mimic_joint.mimic_joint_name = mimicked_joint_it->name;
      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        mimic_joint.multiplier = hardware_interface::stod(joint.parameters.at("multiplier"));
      }
      mimic_joints_.push_back(mimic_joint);
    }
  }

  // set all values without initial values to 0
  for (auto [name, descr] : joint_state_interfaces_)
  {
    if (std::isnan(get_state(name)))
    {
      set_state(name, 0.0);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::InterfaceDescription> GenericSystem::export_command_interfaces_2()
{
  // we check if we should mock command interfaces or not. After they have been exported we can then
  // use them as we would normally via (set/get)_(state/command)
  std::vector<hardware_interface::InterfaceDescription> descriptions;
  // Mock sensor command interfaces
  if (use_mock_sensor_command_interfaces_)
  {
    for (const auto & sensor : info_.sensors)
    {
      for (const auto & state_interface : sensor.state_interfaces)
      {
        hardware_interface::InterfaceInfo info = state_interface;
        hardware_interface::InterfaceDescription descr(sensor.name, info);
        descriptions.push_back(descr);
      }
    }
  }

  // Mock gpio command interfaces (consider all state interfaces for command interfaces)
  if (use_mock_gpio_command_interfaces_)
  {
    for (const auto & gpio : info_.gpios)
    {
      for (const auto & state_interface : gpio.state_interfaces)
      {
        hardware_interface::InterfaceInfo info = state_interface;
        hardware_interface::InterfaceDescription descr(gpio.name, info);
        descriptions.push_back(descr);
      }
    }
  }
  return descriptions;
}

return_type GenericSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (!calculate_dynamics_)
  {
    return ret_val;
  }

  const size_t FOUND_ONCE_FLAG = 1000000;

  std::vector<size_t> joint_found_in_x_requests;
  joint_found_in_x_requests.resize(info_.joints.size(), 0);

  for (const auto & key : start_interfaces)
  {
    // check if interface is joint
    auto joint_it_found = std::find_if(
      info_.joints.begin(), info_.joints.end(),
      [key](const auto & joint) { return (key.find(joint.name) != std::string::npos); });

    if (joint_it_found != info_.joints.end())
    {
      const size_t joint_index = std::distance(info_.joints.begin(), joint_it_found);
      if (joint_found_in_x_requests[joint_index] == 0)
      {
        joint_found_in_x_requests[joint_index] = FOUND_ONCE_FLAG;
      }

      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        joint_found_in_x_requests[joint_index] += 1;
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        if (!calculate_dynamics_)
        {
          RCUTILS_LOG_WARN_NAMED(
            "mock_generic_system",
            "Requested velocity mode for joint '%s' without dynamics calculation enabled - this "
            "might lead to wrong feedback and unexpected behavior.",
            info_.joints[joint_index].name.c_str());
        }
        joint_found_in_x_requests[joint_index] += 1;
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        if (!calculate_dynamics_)
        {
          RCUTILS_LOG_WARN_NAMED(
            "mock_generic_system",
            "Requested acceleration mode for joint '%s' without dynamics calculation enabled - "
            "this might lead to wrong feedback and unexpected behavior.",
            info_.joints[joint_index].name.c_str());
        }
        joint_found_in_x_requests[joint_index] += 1;
      }
    }
    else
    {
      RCUTILS_LOG_DEBUG_NAMED(
        "mock_generic_system", "Got interface '%s' that is not joint - nothing to do!",
        key.c_str());
    }
  }

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // There has to always be at least one control mode from the above three set
    if (joint_found_in_x_requests[i] == FOUND_ONCE_FLAG)
    {
      RCUTILS_LOG_ERROR_NAMED(
        "mock_generic_system", "Joint '%s' has to have '%s', '%s', or '%s' interface!",
        info_.joints[i].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      ret_val = hardware_interface::return_type::ERROR;
    }

    // Currently we don't support multiple interface request
    if (joint_found_in_x_requests[i] > (FOUND_ONCE_FLAG + 1))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "mock_generic_system",
        "Got multiple (%zu) starting interfaces for joint '%s' - this is not "
        "supported!",
        joint_found_in_x_requests[i] - FOUND_ONCE_FLAG, info_.joints[i].name.c_str());
      ret_val = hardware_interface::return_type::ERROR;
    }
  }

  return ret_val;
}

return_type GenericSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  if (!calculate_dynamics_)
  {
    return hardware_interface::return_type::OK;
  }

  for (const auto & key : start_interfaces)
  {
    // check if interface is joint
    auto joint_it_found = std::find_if(
      info_.joints.begin(), info_.joints.end(),
      [key](const auto & joint) { return (key.find(joint.name) != std::string::npos); });

    if (joint_it_found != info_.joints.end())
    {
      const size_t joint_index = std::distance(info_.joints.begin(), joint_it_found);

      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        joint_control_mode_[key] = POSITION_INTERFACE_INDEX;
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        joint_control_mode_[key] = VELOCITY_INTERFACE_INDEX;
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        joint_control_mode_[key] = ACCELERATION_INTERFACE_INDEX;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

return_type GenericSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (command_propagation_disabled_)
  {
    RCUTILS_LOG_WARN_NAMED(
      "mock_generic_system", "Command propagation is disabled - no values will be returned!");
    return return_type::OK;
  }

  auto mirror_command_to_state = [this](const auto & joint_names, const auto & interface_names)
  {
    for (const auto & joint_name : joint_names)
    {
      for (const auto & interface_name : interface_names)
      {
        const auto & joint_state_name = joint_name + "/" + interface_name;
        if (
          this->std_joint_command_interface_names_.find(joint_state_name) !=
          this->std_joint_command_interface_names_.end())
        {
          this->set_state(joint_state_name, this->get_command(joint_state_name));
        }
      }
    }
  };

  auto mirror_all_available_commands_to_states = [this](const auto & interfaces)
  {
    for (const auto & [name, descr] : interfaces)
    {
      // TODO(Manuel) should this be checked if interface exists???
      this->set_state(name, this->get_command(name));
    }
  };

  if (calculate_dynamics_)
  {
    for (const auto joint_name : std_joint_names_)
    {
      const auto joint_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
      const auto joint_vel = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
      const auto joint_acc = joint_name + "/" + hardware_interface::HW_IF_ACCELERATION;
      const auto joint_eff = joint_name + "/" + hardware_interface::HW_IF_EFFORT;

      switch (joint_control_mode_.at(joint_name))
      {
        case ACCELERATION_INTERFACE_INDEX:
        {
          // currently we do backward integration
          // apply offset to positions only
          const auto old_pos = get_state(joint_pos);
          const auto new_pos =
            get_state(joint_vel) * period.seconds() +
            (custom_interface_with_following_offset_.empty() ? position_state_following_offset_
                                                             : 0.0);
          set_state(joint_pos, old_pos + new_pos);

          const auto old_vel = get_state(joint_vel);
          const auto new_vel = get_state(joint_acc) * period.seconds();
          set_state(joint_vel, old_vel + new_vel);

          if (
            std_joint_command_interface_names_.find(joint_acc) !=
            std_joint_command_interface_names_.end())
          {
            set_state(joint_acc, get_command(joint_acc));
          }
          break;
        }
        case VELOCITY_INTERFACE_INDEX:
        {
          // currently we do backward integration
          const auto old_pos = get_state(joint_pos);
          const auto new_pos =
            get_state(joint_vel) * period.seconds() +
            (custom_interface_with_following_offset_.empty() ? position_state_following_offset_
                                                             : 0.0);
          set_state(joint_pos, old_pos + new_pos);

          if (
            std_joint_command_interface_names_.find(joint_vel) !=
            std_joint_command_interface_names_.end())
          {
            const auto old_vel = get_state(joint_vel);
            set_state(joint_vel, get_command(joint_vel));
            set_state(joint_acc, (get_state(joint_vel) - old_vel) / period.seconds());
          }
          break;
        }
        case POSITION_INTERFACE_INDEX:
        {
          if (
            std_joint_command_interface_names_.find(joint_pos) !=
            std_joint_command_interface_names_.end())
          {
            const double old_pos = get_state(joint_pos);
            const double old_vel = get_state(joint_vel);

            set_state(
              joint_pos, get_command(joint_pos) + (custom_interface_with_following_offset_.empty()
                                                     ? position_state_following_offset_
                                                     : 0.0));
            set_state(joint_vel, (get_state(joint_pos) - old_pos) / period.seconds());
            set_state(joint_acc, (get_state(joint_vel) - old_vel) / period.seconds());
          }
          break;
        }
      }
    }
  }
  else
  {
    for (const auto joint_name : std_joint_names_)
    {
      const auto joint_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
      if (
        std_joint_command_interface_names_.find(joint_pos) !=
        std_joint_command_interface_names_.end())
      {
        set_state(
          joint_pos, get_command(joint_pos) + (custom_interface_with_following_offset_.empty()
                                                 ? position_state_following_offset_
                                                 : 0.0));
      }
    }
  }

  // do loopback on all other interfaces - starts from 1 or 3 because 0, 1, 3 are position,
  // velocity, and acceleration interface
  if (calculate_dynamics_)
  {
    std::vector<std::string> interfaces(
      standard_interfaces_.begin() + 3, standard_interfaces_.end());
    mirror_command_to_state(std_joint_names_, interfaces);
  }
  else
  {
    std::vector<std::string> interfaces(
      standard_interfaces_.begin() + 1, standard_interfaces_.end());
    mirror_command_to_state(std_joint_names_, interfaces);
  }

  for (const auto & mimic_joint : mimic_joints_)
  {
    set_state(
      mimic_joint.joint_name, get_state(mimic_joint.mimic_joint_name) * mimic_joint.multiplier);
  }

  for (const auto joint_name : other_joint_names_)
  {
    for (const auto interface_name : non_standard_interfaces_)
    {
      const auto joint_inteface = joint_name + "/" + interface_name;
      const auto joint_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
      if (
        interface_name == custom_interface_name_with_following_offset_ &&
        (std_joint_command_interface_names_.find(joint_pos) !=
         std_joint_command_interface_names_.end()))
      {
        set_state(joint_inteface, get_command(joint_pos) + position_state_following_offset_);
      }
      else if (
        other_joint_command_interface_names_.find(joint_inteface) !=
        other_joint_command_interface_names_.end())
      {
        set_state(joint_inteface, get_command(joint_inteface));
      }
    }
  }

  // do loopback on all sensor interfaces
  if (use_mock_sensor_command_interfaces_)
  {
    mirror_all_available_commands_to_states(sensor_state_interfaces_);
  }

  // do loopback on all gpio interfaces
  mirror_all_available_commands_to_states(gpio_state_interfaces_);

  return return_type::OK;
}

}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mock_components::GenericSystem, hardware_interface::SystemInterface)
