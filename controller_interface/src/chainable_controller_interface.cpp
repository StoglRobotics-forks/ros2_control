// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "controller_interface/chainable_controller_interface.hpp"

#include <vector>

#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
ChainableControllerInterface::ChainableControllerInterface() : ControllerInterfaceBase() {}

bool ChainableControllerInterface::is_chainable() const { return true; }

return_type ChainableControllerInterface::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return_type ret = return_type::ERROR;

  if (!is_in_chained_mode())
  {
    ret = update_reference_from_subscribers(time, period);
    if (ret != return_type::OK)
    {
      return ret;
    }
  }

  ret = update_and_write_commands(time, period);

  return ret;
}

std::vector<std::shared_ptr<hardware_interface::StateInterface>>
ChainableControllerInterface::export_state_interfaces()
{
  auto state_interfaces_descr = export_state_interface_descriptions();
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_interfaces_ptrs_vec;
  state_interfaces_ptrs_vec.reserve(state_interfaces_descr.size());

  // check if the names of the controller state interfaces begin with the controller's name
  for (auto & descr : state_interfaces_descr)
  {
    if (descr.prefix_name != get_node()->get_name())
    {
      std::string error_msg =
        "The prefix of the interface description'" + descr.prefix_name +
        "' does not equal the controller's name '" + get_node()->get_name() +
        "'. This is mandatory for state interfaces. No state interface will be exported. Please "
        "correct and recompile the controller with name '" +
        get_node()->get_name() + "' and try again.";
      throw std::runtime_error(error_msg);
    }

    auto state_interface = std::make_shared<hardware_interface::StateInterface>(descr);
    const auto inteface_name = state_interface->get_name();
    // check the exported interface name is unique
    auto [it, succ] = exported_state_interfaces_.insert({inteface_name, state_interface});
    // either we have name duplicate which we want to avoid under all circumstances since interfaces
    // need to be uniquely identify able or something else really went wrong. In any case abort and
    // inform cm by throwing exception
    if (!succ)
    {
      std::string error_msg =
        "Could not insert StateInterface<" + inteface_name +
        "> into exported_state_interfaces_ map. Check if you export duplicates. The "
        "map returned iterator with interface_name<" +
        it->second->get_name() +
        ">. If its a duplicate adjust exportation of InterfacesDescription so that all the "
        "interface names are unique.";
      exported_state_interfaces_.clear();
      exported_state_interface_names_.clear();
      state_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    exported_state_interface_names_.push_back(inteface_name);
    state_interfaces_ptrs_vec.push_back(state_interface);
  }

  if (exported_state_interfaces_.size() != state_interfaces_descr.size())
  {
    std::string error_msg =
      "The internal storage for reference ptrs 'exported_state_interfaces_' variable has size '" +
      std::to_string(exported_state_interfaces_.size()) +
      "', but it is expected to have the size '" + std::to_string(state_interfaces_descr.size()) +
      "' equal to the number of exported reference interfaces. Please correct and recompile the "
      "controller with name '" +
      get_node()->get_name() + "' and try again.";
    throw std::runtime_error(error_msg);
  }

  return state_interfaces_ptrs_vec;
}

std::vector<std::shared_ptr<hardware_interface::CommandInterface>>
ChainableControllerInterface::export_reference_interfaces()
{
  auto reference_interface_descr = export_reference_interface_descriptions();
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> reference_interfaces_ptrs_vec;
  reference_interfaces_ptrs_vec.reserve(reference_interface_descr.size());

  // check if the names of the reference interfaces begin with the controller's name
  for (auto & descr : reference_interface_descr)
  {
    if (descr.prefix_name != get_node()->get_name())
    {
      std::string error_msg = "The name of the interface descr " + descr.prefix_name +
                              " does not begin with the controller's name. This is mandatory for "
                              "reference interfaces. Please "
                              "correct and recompile the controller with name " +
                              get_node()->get_name() + " and try again.";
      throw std::runtime_error(error_msg);
    }

    auto reference_interface = std::make_shared<hardware_interface::CommandInterface>(descr);
    const auto inteface_name = reference_interface->get_name();
    // check the exported interface name is unique
    auto [it, succ] = reference_interfaces_.insert({inteface_name, reference_interface});
    // either we have name duplicate which we want to avoid under all circumstances since interfaces
    // need to be uniquely identify able or something else really went wrong. In any case abort and
    // inform cm by throwing exception
    if (!succ)
    {
      std::string error_msg =
        "Could not insert Reference interface<" + inteface_name +
        "> into reference_interfaces_ map. Check if you export duplicates. The "
        "map returned iterator with interface_name<" +
        it->second->get_name() +
        ">. If its a duplicate adjust exportation of InterfacesDescription so that all the "
        "interface names are unique.";
      reference_interfaces_.clear();
      exported_reference_interface_names_.clear();
      reference_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    exported_reference_interface_names_.push_back(inteface_name);
    reference_interfaces_ptrs_vec.push_back(reference_interface);
  }

  if (reference_interfaces_.size() != reference_interface_descr.size())
  {
    std::string error_msg =
      "The internal storage for reference ptrs 'reference_interfaces_' variable has size '" +
      std::to_string(reference_interfaces_.size()) + "', but it is expected to have the size '" +
      std::to_string(reference_interface_descr.size()) +
      "' equal to the number of exported reference interfaces. Please correct and recompile the "
      "controller with name '" +
      get_node()->get_name() + "' and try again.";
    throw std::runtime_error(error_msg);
  }

  return reference_interfaces_ptrs_vec;
}

bool ChainableControllerInterface::set_chained_mode(bool chained_mode)
{
  bool result = false;

  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    result = on_set_chained_mode(chained_mode);

    if (result)
    {
      in_chained_mode_ = chained_mode;
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Can not change controller's chained mode because it is no in '%s' state. "
      "Current state is '%s'.",
      hardware_interface::lifecycle_state_names::UNCONFIGURED, get_state().label().c_str());
  }

  return result;
}

bool ChainableControllerInterface::is_in_chained_mode() const { return in_chained_mode_; }

bool ChainableControllerInterface::on_set_chained_mode(bool /*chained_mode*/) { return true; }

}  // namespace controller_interface
