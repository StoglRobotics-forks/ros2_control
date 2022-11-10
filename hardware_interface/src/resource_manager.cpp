// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "hardware_interface/resource_manager.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_component_info.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace hardware_interface
{
auto trigger_and_print_hardware_state_transition =
  [](
    auto transition, const std::string transition_name, const std::string & hardware_name,
    const lifecycle_msgs::msg::State::_id_type & target_state)
{
  RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "'%s' hardware '%s' ", transition_name.c_str(), hardware_name.c_str());

  const rclcpp_lifecycle::State new_state = transition();

  bool result = new_state.id() == target_state;

  if (result)
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Successful '%s' of hardware '%s'", transition_name.c_str(),
      hardware_name.c_str());
  }
  else
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Failed to '%s' hardware '%s'", transition_name.c_str(),
      hardware_name.c_str());
  }
  return result;
};

class ResourceStorage
{
  static constexpr const char * pkg_name = "hardware_interface";

  static constexpr const char * actuator_interface_name = "hardware_interface::ActuatorInterface";
  static constexpr const char * sensor_interface_name = "hardware_interface::SensorInterface";
  static constexpr const char * system_interface_name = "hardware_interface::SystemInterface";

public:
  ResourceStorage()
  : actuator_loader_(pkg_name, actuator_interface_name),
    sensor_loader_(pkg_name, sensor_interface_name),
    system_loader_(pkg_name, system_interface_name)
  {
  }

  template <class HardwareT, class HardwareInterfaceT>
  void load_hardware(
    const HardwareInfo & hardware_info, pluginlib::ClassLoader<HardwareInterfaceT> & loader,
    std::vector<HardwareT> & container)
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Loading hardware '%s' ", hardware_info.name.c_str());
    // hardware_plugin_name has to match class name in plugin xml description
    auto interface = std::unique_ptr<HardwareInterfaceT>(
      loader.createUnmanagedInstance(hardware_info.hardware_plugin_name));
    HardwareT hardware(std::move(interface));
    container.emplace_back(std::move(hardware));
    // initialize static data about hardware component to reduce later calls
    HardwareComponentInfo component_info;
    component_info.name = hardware_info.name;
    component_info.type = hardware_info.type;
    component_info.plugin_name = hardware_info.hardware_plugin_name;
    component_info.is_async = hardware_info.is_async;

    hardware_info_map_.insert(std::make_pair(component_info.name, component_info));
    hardware_used_by_controllers_.insert(
      std::make_pair(component_info.name, std::vector<std::string>()));
  }

  template <class HardwareT>
  bool initialize_hardware(const HardwareInfo & hardware_info, HardwareT & hardware)
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Initialize hardware '%s' ", hardware_info.name.c_str());

    const rclcpp_lifecycle::State new_state = hardware.initialize(hardware_info);

    bool result = new_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

    if (result)
    {
      RCUTILS_LOG_INFO_NAMED(
        "resource_manager", "Successful initialization of hardware '%s'",
        hardware_info.name.c_str());
    }
    else
    {
      RCUTILS_LOG_INFO_NAMED(
        "resource_manager", "Failed to initialize hardware '%s'", hardware_info.name.c_str());
    }
    return result;
  }

  template <class HardwareT>
  bool configure_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::configure, &hardware), "configure", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    if (result)
    {
      // TODO(destogl): is it better to check here if previous state was unconfigured instead of
      // checking if each state already exists? Or we should somehow know that transition has
      // happened and only then trigger this part of the code?
      // On the other side this part of the code should never be executed in real-time critical
      // thread, so it could be also OK as it is...
      for (const auto & interface : hardware_info_map_[hardware.get_name()].state_interfaces)
      {
        // add all state interfaces to available list
        auto found_it = std::find(
          available_state_interfaces_.begin(), available_state_interfaces_.end(), interface);

        if (found_it == available_state_interfaces_.end())
        {
          available_state_interfaces_.emplace_back(interface);
          RCUTILS_LOG_DEBUG_NAMED(
            "resource_manager", "(hardware '%s'): '%s' state interface added into available list",
            hardware.get_name().c_str(), interface.c_str());
        }
        else
        {
          // TODO(destogl): do here error management if interfaces are only partially added into
          // "available" list - this should never be the case!
          RCUTILS_LOG_WARN_NAMED(
            "resource_manager",
            "(hardware '%s'): '%s' state interface already in available list."
            " This can happen due to multiple calls to 'configure'",
            hardware.get_name().c_str(), interface.c_str());
        }
      }

      // add command interfaces to available list
      for (const auto & interface : hardware_info_map_[hardware.get_name()].command_interfaces)
      {
        // TODO(destogl): check if interface should be available on configure
        auto found_it = std::find(
          available_command_interfaces_.begin(), available_command_interfaces_.end(), interface);

        if (found_it == available_command_interfaces_.end())
        {
          available_command_interfaces_.emplace_back(interface);
          RCUTILS_LOG_DEBUG_NAMED(
            "resource_manager", "(hardware '%s'): '%s' command interface added into available list",
            hardware.get_name().c_str(), interface.c_str());
        }
        else
        {
          // TODO(destogl): do here error management if interfaces are only partially added into
          // "available" list - this should never be the case!
          RCUTILS_LOG_WARN_NAMED(
            "resource_manager",
            "(hardware '%s'): '%s' command interface already in available list."
            " This can happen due to multiple calls to 'configure'",
            hardware.get_name().c_str(), interface.c_str());
        }
      }
    }
    return result;
  }

  void remove_all_hardware_interfaces_from_available_list(const std::string & hardware_name)
  {
    // remove all command interfaces from available list
    for (const auto & interface : hardware_info_map_[hardware_name].command_interfaces)
    {
      auto found_it = std::find(
        available_command_interfaces_.begin(), available_command_interfaces_.end(), interface);

      if (found_it != available_command_interfaces_.end())
      {
        available_command_interfaces_.erase(found_it);
        RCUTILS_LOG_DEBUG_NAMED(
          "resource_manager", "(hardware '%s'): '%s' command interface removed from available list",
          hardware_name.c_str(), interface.c_str());
      }
      else
      {
        // TODO(destogl): do here error management if interfaces are only partially added into
        // "available" list - this should never be the case!
        RCUTILS_LOG_WARN_NAMED(
          "resource_manager",
          "(hardware '%s'): '%s' command interface not in available list. "
          "This should not happen (hint: multiple cleanup calls).",
          hardware_name.c_str(), interface.c_str());
      }
    }
    // remove all state interfaces from available list
    for (const auto & interface : hardware_info_map_[hardware_name].state_interfaces)
    {
      auto found_it = std::find(
        available_state_interfaces_.begin(), available_state_interfaces_.end(), interface);

      if (found_it != available_state_interfaces_.end())
      {
        available_state_interfaces_.erase(found_it);
        RCUTILS_LOG_DEBUG_NAMED(
          "resource_manager", "(hardware '%s'): '%s' state interface removed from available list",
          hardware_name.c_str(), interface.c_str());
      }
      else
      {
        // TODO(destogl): do here error management if interfaces are only partially added into
        // "available" list - this should never be the case!
        RCUTILS_LOG_WARN_NAMED(
          "resource_manager",
          "(hardware '%s'): '%s' state interface not in available list. "
          "This should not happen (hint: multiple cleanup calls).",
          hardware_name.c_str(), interface.c_str());
      }
    }
  }

  template <class HardwareT>
  bool cleanup_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::cleanup, &hardware), "cleanup", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    if (result)
    {
      remove_all_hardware_interfaces_from_available_list(hardware.get_name());
    }
    return result;
  }

  template <class HardwareT>
  bool shutdown_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::shutdown, &hardware), "shutdown", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

    if (result)
    {
      // TODO(destogl): change this - deimport all things if there is there are interfaces there
      // deimport_non_movement_command_interfaces(hardware);
      // deimport_state_interfaces(hardware);
      // use remove_command_interfaces(hardware);
    }
    return result;
  }

  template <class HardwareT>
  bool activate_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::activate, &hardware), "activate", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    if (result)
    {
      // TODO(destogl): make all command interfaces available (currently are all available)
    }

    return result;
  }

  template <class HardwareT>
  bool deactivate_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::deactivate, &hardware), "deactivate", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    if (result)
    {
      // TODO(destogl): make all command interfaces unavailable that should be present only
      // when active (currently are all available) also at inactive
    }
    return result;
  }

  template <class HardwareT>
  bool set_component_state(HardwareT & hardware, const rclcpp_lifecycle::State & target_state)
  {
    using lifecycle_msgs::msg::State;

    bool result = false;

    switch (target_state.id())
    {
      case State::PRIMARY_STATE_UNCONFIGURED:
        switch (hardware.get_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = true;
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = cleanup_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = deactivate_hardware(hardware);
            if (result)
            {
              result = cleanup_hardware(hardware);
            }
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCUTILS_LOG_WARN_NAMED(
              "resource_manager", "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_INACTIVE:
        switch (hardware.get_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = configure_hardware(hardware);
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = true;
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = deactivate_hardware(hardware);
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCUTILS_LOG_WARN_NAMED(
              "resource_manager", "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_ACTIVE:
        switch (hardware.get_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = configure_hardware(hardware);
            if (result)
            {
              result = activate_hardware(hardware);
            }
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = activate_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = true;
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCUTILS_LOG_WARN_NAMED(
              "resource_manager", "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_FINALIZED:
        switch (hardware.get_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = true;
            break;
        }
        break;
    }

    return result;
  }

  void add_state_interface(std::shared_ptr<ReadOnlyHandle> state_interface)
  {
    const auto [it, success] =
      state_interface_map_.insert(std::make_pair(state_interface->get_name(), state_interface));
    if (!success)
    {
      std::string msg(
        "ResourceStorage: Tried to insert StateInterface with already existing key. Insert[" +
        state_interface->get_name() + "]");
      throw std::runtime_error(msg);
    }
  }

  void add_state_interface(const StateInterface & state_interface)
  {
    const auto [it, success] = state_interface_map_.emplace(std::make_pair(
      state_interface.get_name(), std::make_shared<StateInterface>(state_interface)));
    if (!success)
    {
      std::string msg(
        "ResourceStorage: Tried to insert StateInterface with already existing key. Insert[" +
        state_interface.get_name() + "]");
      throw std::runtime_error(msg);
    }
  }

  template <class HardwareT>
  void import_state_interfaces(HardwareT & hardware)
  {
    auto interfaces = hardware.export_state_interfaces();
    std::vector<std::string> interface_names;
    interface_names.reserve(interfaces.size());
    for (auto const & interface : interfaces)
    {
      add_state_interface(interface);
      interface_names.push_back(interface.get_name());
    }
    hardware_info_map_[hardware.get_name()].state_interfaces = interface_names;
    available_state_interfaces_.reserve(
      available_state_interfaces_.capacity() + interface_names.size());
  }

  template <class HardwareT>
  void import_command_interfaces(HardwareT & hardware)
  {
    auto interfaces = hardware.export_command_interfaces();
    hardware_info_map_[hardware.get_name()].command_interfaces = add_command_interfaces(interfaces);
  }

  void add_command_interface(std::shared_ptr<ReadWriteHandle> command_interface)
  {
    const auto [it, success] = command_interface_map_.insert(
      std::make_pair(command_interface->get_name(), command_interface));
    if (!success)
    {
      std::string msg(
        "ResourceStorage: Tried to insert CommandInterface with already existing key. Insert[" +
        command_interface->get_name() + "]");
      throw std::runtime_error(msg);
    }
  }

  void add_command_interface(CommandInterface && command_interface)
  {
    std::string key = command_interface.get_name();
    const auto [it, success] = command_interface_map_.emplace(
      std::make_pair(key, std::make_shared<CommandInterface>(std::move(command_interface))));
    if (!success)
    {
      std::string msg(
        "ResourceStorage: Tried to insert CommandInterface with already existing key. Insert[" +
        key + "]");
      throw std::runtime_error(msg);
    }
  }

  /// Adds exported command interfaces into internal storage.
  /**
   * Add command interfaces to the internal storage. Command interfaces exported from hardware or
   * chainable controllers are moved to the map with name-interface pairs, the interface names are
   * added to the claimed map and available list's size is increased to reserve storage when
   * interface change theirs status in real-time control loop.
   *
   * \param[interfaces] list of command interface to add into storage.
   * \returns list of interface names that are added into internal storage. The output is used to
   * avoid additional iterations to cache interface names, e.g., for initializing info structures.
   */
  std::vector<std::string> add_command_interfaces(std::vector<CommandInterface> & interfaces)
  {
    std::vector<std::string> interface_names;
    interface_names.reserve(interfaces.size());
    for (auto & interface : interfaces)
    {
      auto key = interface.get_name();
      add_command_interface(std::move(interface));
      claimed_command_interface_map_.emplace(std::make_pair(key, false));
      interface_names.push_back(key);
    }
    available_command_interfaces_.reserve(
      available_command_interfaces_.capacity() + interface_names.size());

    return interface_names;
  }

  /// Removes command interfaces from internal storage.
  /**
   * Command interface are removed from the maps with theirs storage and their claimed status.
   *
   * \param[interface_names] list of command interface names to remove from storage.
   */
  void remove_command_interfaces(const std::vector<std::string> & interface_names)
  {
    for (const auto & interface : interface_names)
    {
      command_interface_map_.erase(interface);
      claimed_command_interface_map_.erase(interface);
    }
  }

  void add_state_publisher(std::shared_ptr<distributed_control::StatePublisher> state_publisher)
  {
    const auto [it, success] = state_interface_state_publisher_map_.insert(
      std::pair{state_publisher->state_interface_name(), state_publisher});
    if (!success)
    {
      std::string msg(
        "ResourceStorage: Tried to insert StatePublisher with already existing key. Insert[" +
        state_publisher->state_interface_name() + "]");
      throw std::runtime_error(msg);
    }
  }

  void add_command_forwarder(
    std::shared_ptr<distributed_control::CommandForwarder> command_forwarder)
  {
    const auto [it, success] = command_interface_command_forwarder_map_.insert(
      std::pair{command_forwarder->command_interface_name(), command_forwarder});
    if (!success)
    {
      std::string msg(
        "ResourceStorage: Tried to insert CommandForwarder with already existing key. Insert[" +
        command_forwarder->command_interface_name() + "]");
      throw std::runtime_error(msg);
    }
  }

  std::shared_ptr<distributed_control::StatePublisher> get_state_publisher(
    std::string state_interface_name) const
  {
    return state_interface_state_publisher_map_.at(state_interface_name);
  }

  std::vector<std::shared_ptr<distributed_control::StatePublisher>> get_state_publishers() const
  {
    std::vector<std::shared_ptr<distributed_control::StatePublisher>> state_publishers_vec;
    state_publishers_vec.reserve(state_interface_state_publisher_map_.size());

    for (auto state_publisher : state_interface_state_publisher_map_)
    {
      state_publishers_vec.push_back(state_publisher.second);
    }
    return state_publishers_vec;
  }

  std::vector<std::shared_ptr<distributed_control::CommandForwarder>> get_command_forwarders() const
  {
    std::vector<std::shared_ptr<distributed_control::CommandForwarder>> command_forwarders_vec;
    command_forwarders_vec.reserve(command_interface_command_forwarder_map_.size());

    for (auto command_forwarder : command_interface_command_forwarder_map_)
    {
      command_forwarders_vec.push_back(command_forwarder.second);
    }
    return command_forwarders_vec;
  }

  std::pair<bool, std::shared_ptr<distributed_control::CommandForwarder>> find_command_forwarder(
    const std::string & key)
  {
    auto command_forwarder = command_interface_command_forwarder_map_.find(key);
    // we could not find a command forwarder for the provided key
    if (command_forwarder == command_interface_command_forwarder_map_.end())
    {
      return std::make_pair(false, nullptr);
    }
    return std::make_pair(true, command_forwarder->second);
  }

  void check_for_duplicates(const HardwareInfo & hardware_info)
  {
    // Check for identical names
    if (hardware_info_map_.find(hardware_info.name) != hardware_info_map_.end())
    {
      throw std::runtime_error(
        "Hardware name " + hardware_info.name +
        " is duplicated. Please provide a unique 'name' in the URDF.");
    }
  }

  // TODO(destogl): Propagate "false" up, if happens in initialize_hardware
  void load_and_initialize_actuator(const HardwareInfo & hardware_info)
  {
    check_for_duplicates(hardware_info);
    load_hardware<Actuator, ActuatorInterface>(hardware_info, actuator_loader_, actuators_);
    initialize_hardware(hardware_info, actuators_.back());
    import_state_interfaces(actuators_.back());
    import_command_interfaces(actuators_.back());
  }

  void load_and_initialize_sensor(const HardwareInfo & hardware_info)
  {
    check_for_duplicates(hardware_info);
    load_hardware<Sensor, SensorInterface>(hardware_info, sensor_loader_, sensors_);
    initialize_hardware(hardware_info, sensors_.back());
    import_state_interfaces(sensors_.back());
  }

  void load_and_initialize_system(const HardwareInfo & hardware_info)
  {
    check_for_duplicates(hardware_info);
    load_hardware<System, SystemInterface>(hardware_info, system_loader_, systems_);
    initialize_hardware(hardware_info, systems_.back());
    import_state_interfaces(systems_.back());
    import_command_interfaces(systems_.back());
  }

  void initialize_actuator(
    std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info)
  {
    this->actuators_.emplace_back(Actuator(std::move(actuator)));
    initialize_hardware(hardware_info, actuators_.back());
    import_state_interfaces(actuators_.back());
    import_command_interfaces(actuators_.back());
  }

  void initialize_sensor(
    std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info)
  {
    this->sensors_.emplace_back(Sensor(std::move(sensor)));
    initialize_hardware(hardware_info, sensors_.back());
    import_state_interfaces(sensors_.back());
  }

  void initialize_system(
    std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info)
  {
    this->systems_.emplace_back(System(std::move(system)));
    initialize_hardware(hardware_info, systems_.back());
    import_state_interfaces(systems_.back());
    import_command_interfaces(systems_.back());
  }

  void add_sub_controller_manager(
    std::shared_ptr<distributed_control::SubControllerManagerWrapper> sub_controller_manager)
  {
    const auto [it, success] = sub_controller_manager_map_.insert(
      std::pair{sub_controller_manager->get_name(), sub_controller_manager});
    if (!success)
    {
      std::string msg(
        "ResourceStorage: Tried to add sub ControllerManager with already existing key. Insert[" +
        sub_controller_manager->get_name() + "]");
      throw std::runtime_error(msg);
    }
  }

  std::vector<std::shared_ptr<DistributedReadOnlyHandle>> import_distributed_state_interfaces(
    std::shared_ptr<distributed_control::SubControllerManagerWrapper> sub_controller_manager)
  {
    std::vector<std::shared_ptr<DistributedReadOnlyHandle>> distributed_state_interfaces;
    distributed_state_interfaces.reserve(sub_controller_manager->get_state_publisher_count());
    std::vector<std::string> interface_names;
    interface_names.reserve(sub_controller_manager->get_state_publisher_count());

    for (const auto & state_publisher_description :
         sub_controller_manager->get_state_publisher_descriptions())
    {
      // create StateInterface from the Description and store in ResourceStorage.
      auto state_interface =
        std::make_shared<DistributedReadOnlyHandle>(state_publisher_description);

      add_state_interface(state_interface);
      // add to return vector, node needs to added to executor.
      distributed_state_interfaces.push_back(state_interface);
      interface_names.push_back(state_interface->get_name());
    }
    // TODO(Manuel) this should be handled at one point DRY (adding, hardware_info_map, make available...), key should be made explicit
    hardware_info_map_[sub_controller_manager->get_name()].state_interfaces = interface_names;
    available_state_interfaces_.reserve(
      available_state_interfaces_.capacity() + interface_names.size());

    for (const auto & interface :
         hardware_info_map_[sub_controller_manager->get_name()].state_interfaces)
    {
      // add all state interfaces to available list
      auto found_it = std::find(
        available_state_interfaces_.begin(), available_state_interfaces_.end(), interface);

      if (found_it == available_state_interfaces_.end())
      {
        available_state_interfaces_.emplace_back(interface);
        RCUTILS_LOG_DEBUG_NAMED(
          "resource_manager", "(hardware '%s'): '%s' state interface added into available list",
          sub_controller_manager->get_name().c_str(), interface.c_str());
      }
      else
      {
        // TODO(destogl): do here error management if interfaces are only partially added into
        // "available" list - this should never be the case!
        RCUTILS_LOG_WARN_NAMED(
          "resource_manager",
          "(hardware '%s'): '%s' state interface already in available list."
          " This can happen due to multiple calls to 'configure'",
          sub_controller_manager->get_name().c_str(), interface.c_str());
      }
    }
    return distributed_state_interfaces;
  }

  std::vector<std::shared_ptr<DistributedReadWriteHandle>> import_distributed_command_interfaces(
    std::shared_ptr<distributed_control::SubControllerManagerWrapper> sub_controller_manager)
  {
    std::vector<std::shared_ptr<DistributedReadWriteHandle>> distributed_command_interfaces;
    distributed_command_interfaces.reserve(sub_controller_manager->get_command_forwarder_count());
    std::vector<std::string> interface_names;
    interface_names.reserve(sub_controller_manager->get_command_forwarder_count());

    for (const auto & command_forwarder_description :
         sub_controller_manager->get_command_forwarder_descriptions())
    {
      // create StateInterface from the Description and store in ResourceStorage.
      auto command_interface =
        std::make_shared<DistributedReadWriteHandle>(command_forwarder_description);
      add_command_interface(command_interface);
      // add to return vector, node needs to added to executor.
      distributed_command_interfaces.push_back(command_interface);
      // TODO(Manuel) this should be handled at one point DRY (adding, claimed ....), key should be made explicit
      claimed_command_interface_map_.emplace(std::make_pair(command_interface->get_name(), false));
      interface_names.push_back(command_interface->get_name());
    }
    // TODO(Manuel) this should be handled at one point DRY(adding, claimed,make available....), key should be made explicit
    available_command_interfaces_.reserve(
      available_command_interfaces_.capacity() + interface_names.size());
    hardware_info_map_[sub_controller_manager->get_name()].command_interfaces = interface_names;

    for (const auto & interface :
         hardware_info_map_[sub_controller_manager->get_name()].command_interfaces)
    {
      // TODO(destogl): check if interface should be available on configure
      auto found_it = std::find(
        available_command_interfaces_.begin(), available_command_interfaces_.end(), interface);

      if (found_it == available_command_interfaces_.end())
      {
        available_command_interfaces_.emplace_back(interface);
        RCUTILS_LOG_DEBUG_NAMED(
          "resource_manager", "(hardware '%s'): '%s' command interface added into available list",
          sub_controller_manager->get_name().c_str(), interface.c_str());
      }
      else
      {
        // TODO(destogl): do here error management if interfaces are only partially added into
        // "available" list - this should never be the case!
        RCUTILS_LOG_WARN_NAMED(
          "resource_manager",
          "(hardware '%s'): '%s' command interface already in available list."
          " This can happen due to multiple calls to 'configure'",
          sub_controller_manager->get_name().c_str(), interface.c_str());
      }
    }

    return distributed_command_interfaces;
  }

  // hardware plugins
  pluginlib::ClassLoader<ActuatorInterface> actuator_loader_;
  pluginlib::ClassLoader<SensorInterface> sensor_loader_;
  pluginlib::ClassLoader<SystemInterface> system_loader_;

  std::vector<Actuator> actuators_;
  std::vector<Sensor> sensors_;
  std::vector<System> systems_;

  std::unordered_map<std::string, HardwareComponentInfo> hardware_info_map_;

  /// Mapping between hardware and controllers that are using it (accessing data from it)
  std::unordered_map<std::string, std::vector<std::string>> hardware_used_by_controllers_;

  /// Mapping between controllers and list of reference interfaces they are using
  std::unordered_map<std::string, std::vector<std::string>> controllers_reference_interfaces_map_;

  /// Storage of all available state interfaces
  std::map<std::string, std::shared_ptr<ReadOnlyHandle>> state_interface_map_;
  /// Storage of all available command interfaces
  std::map<std::string, std::shared_ptr<ReadWriteHandle>> command_interface_map_;

  /// Vectors with interfaces available to controllers (depending on hardware component state)
  std::vector<std::string> available_state_interfaces_;
  std::vector<std::string> available_command_interfaces_;

  /// List of all claimed command interfaces
  std::unordered_map<std::string, bool> claimed_command_interface_map_;

private:
  std::map<std::string, std::shared_ptr<distributed_control::StatePublisher>>
    state_interface_state_publisher_map_;

  std::map<std::string, std::shared_ptr<distributed_control::CommandForwarder>>
    command_interface_command_forwarder_map_;

  std::map<std::string, std::shared_ptr<distributed_control::SubControllerManagerWrapper>>
    sub_controller_manager_map_;
};

ResourceManager::ResourceManager() : resource_storage_(std::make_unique<ResourceStorage>()) {}

ResourceManager::~ResourceManager() = default;

ResourceManager::ResourceManager(
  const std::string & urdf, bool validate_interfaces, bool activate_all)
: resource_storage_(std::make_unique<ResourceStorage>())
{
  load_urdf(urdf, validate_interfaces);

  if (activate_all)
  {
    for (auto const & hw_info : resource_storage_->hardware_info_map_)
    {
      using lifecycle_msgs::msg::State;
      rclcpp_lifecycle::State state(State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
      set_component_state(hw_info.first, state);
    }
  }
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::load_urdf(const std::string & urdf, bool validate_interfaces)
{
  load_urdf_called_ = true;
  const std::string system_type = "system";
  const std::string sensor_type = "sensor";
  const std::string actuator_type = "actuator";

  const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);
  for (const auto & individual_hardware_info : hardware_info)
  {
    if (individual_hardware_info.type == actuator_type)
    {
      std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
      std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
      resource_storage_->load_and_initialize_actuator(individual_hardware_info);
    }
    if (individual_hardware_info.type == sensor_type)
    {
      std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
      resource_storage_->load_and_initialize_sensor(individual_hardware_info);
    }
    if (individual_hardware_info.type == system_type)
    {
      std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
      std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
      resource_storage_->load_and_initialize_system(individual_hardware_info);
    }
  }

  // throw on missing state and command interfaces, not specified keys are being ignored
  if (validate_interfaces)
  {
    validate_storage(hardware_info);
  }

  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  read_write_status.failed_hardware_names.reserve(
    resource_storage_->actuators_.size() + resource_storage_->sensors_.size() +
    resource_storage_->systems_.size());
}

bool ResourceManager::load_urdf_called() const { return load_urdf_called_; }

// CM API: Called in "update"-thread
LoanedStateInterface ResourceManager::claim_state_interface(const std::string & key)
{
  if (!state_interface_is_available(key))
  {
    throw std::runtime_error(std::string("State interface with key '") + key + "' does not exist");
  }

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return LoanedStateInterface(resource_storage_->state_interface_map_.at(key));
}

// CM API: Called in "callback/slow"-thread
std::vector<std::string> ResourceManager::state_interface_keys() const
{
  std::vector<std::string> keys;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & item : resource_storage_->state_interface_map_)
  {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

// CM API: Called in "update"-thread
std::vector<std::string> ResourceManager::available_state_interfaces() const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->available_state_interfaces_;
}

// CM API: Called in "update"-thread (indirectly through `claim_state_interface`)
bool ResourceManager::state_interface_is_available(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return std::find(
           resource_storage_->available_state_interfaces_.begin(),
           resource_storage_->available_state_interfaces_.end(),
           name) != resource_storage_->available_state_interfaces_.end();
}

void ResourceManager::register_sub_controller_manager(
  std::shared_ptr<distributed_control::SubControllerManagerWrapper> sub_controller_manager)
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  resource_storage_->add_sub_controller_manager(sub_controller_manager);
}

std::vector<std::shared_ptr<DistributedReadOnlyHandle>>
ResourceManager::import_state_interfaces_of_sub_controller_manager(
  std::shared_ptr<distributed_control::SubControllerManagerWrapper> sub_controller_manager)
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->import_distributed_state_interfaces(sub_controller_manager);
}

std::vector<std::shared_ptr<DistributedReadWriteHandle>>
ResourceManager::import_command_interfaces_of_sub_controller_manager(
  std::shared_ptr<distributed_control::SubControllerManagerWrapper> sub_controller_manager)
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->import_distributed_command_interfaces(sub_controller_manager);
}

std::vector<std::shared_ptr<distributed_control::StatePublisher>>
ResourceManager::create_hardware_state_publishers(const std::string & ns)
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  std::vector<std::shared_ptr<distributed_control::StatePublisher>> state_publishers_vec;
  state_publishers_vec.reserve(available_state_interfaces().size());

  for (const auto & state_interface : available_state_interfaces())
  {
    auto state_publisher = std::make_shared<distributed_control::StatePublisher>(
      std::move(std::make_unique<hardware_interface::LoanedStateInterface>(
        claim_state_interface(state_interface))),
      ns);

    resource_storage_->add_state_publisher(state_publisher);
    state_publishers_vec.push_back(state_publisher);
  }

  return state_publishers_vec;
}

std::vector<std::shared_ptr<distributed_control::CommandForwarder>>
ResourceManager::create_hardware_command_forwarders(const std::string & ns)
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  std::vector<std::shared_ptr<distributed_control::CommandForwarder>> command_forwarders_vec;
  command_forwarders_vec.reserve(available_command_interfaces().size());

  for (const auto & command_interface : available_command_interfaces())
  {
    auto command_forwarder = std::make_shared<distributed_control::CommandForwarder>(
      std::move(std::make_unique<hardware_interface::LoanedCommandInterface>(
        claim_command_interface(command_interface))),
      ns);

    resource_storage_->add_command_forwarder(command_forwarder);
    command_forwarders_vec.push_back(command_forwarder);
  }

  return command_forwarders_vec;
}

std::vector<std::shared_ptr<distributed_control::StatePublisher>>
ResourceManager::get_state_publishers() const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->get_state_publishers();
}

std::vector<std::shared_ptr<distributed_control::CommandForwarder>>
ResourceManager::get_command_forwarders() const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->get_command_forwarders();
}

std::pair<bool, std::shared_ptr<distributed_control::CommandForwarder>>
ResourceManager::find_command_forwarder(const std::string & key)
{
  return resource_storage_->find_command_forwarder(key);
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::import_controller_reference_interfaces(
  const std::string & controller_name, std::vector<CommandInterface> & interfaces)
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  auto interface_names = resource_storage_->add_command_interfaces(interfaces);
  resource_storage_->controllers_reference_interfaces_map_[controller_name] = interface_names;
}

// CM API: Called in "callback/slow"-thread
std::vector<std::string> ResourceManager::get_controller_reference_interface_names(
  const std::string & controller_name)
{
  return resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
}

// CM API: Called in "update"-thread
void ResourceManager::make_controller_reference_interfaces_available(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  resource_storage_->available_command_interfaces_.insert(
    resource_storage_->available_command_interfaces_.end(), interface_names.begin(),
    interface_names.end());
}

// CM API: Called in "update"-thread
void ResourceManager::make_controller_reference_interfaces_unavailable(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & interface : interface_names)
  {
    auto found_it = std::find(
      resource_storage_->available_command_interfaces_.begin(),
      resource_storage_->available_command_interfaces_.end(), interface);
    if (found_it != resource_storage_->available_command_interfaces_.end())
    {
      resource_storage_->available_command_interfaces_.erase(found_it);
      RCUTILS_LOG_DEBUG_NAMED(
        "resource_manager", "'%s' command interface removed from available list",
        interface.c_str());
    }
  }
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::remove_controller_reference_interfaces(const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
  resource_storage_->controllers_reference_interfaces_map_.erase(controller_name);

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  resource_storage_->remove_command_interfaces(interface_names);
}

// CM API: Called in "callback/slow"-thread
void ResourceManager::cache_controller_to_hardware(
  const std::string & controller_name, const std::vector<std::string> & interfaces)
{
  for (const auto & interface : interfaces)
  {
    bool found = false;
    for (const auto & [hw_name, hw_info] : resource_storage_->hardware_info_map_)
    {
      auto cmd_itf_it =
        std::find(hw_info.command_interfaces.begin(), hw_info.command_interfaces.end(), interface);
      if (cmd_itf_it != hw_info.command_interfaces.end())
      {
        found = true;
      }
      auto state_itf_it =
        std::find(hw_info.state_interfaces.begin(), hw_info.state_interfaces.end(), interface);
      if (state_itf_it != hw_info.state_interfaces.end())
      {
        found = true;
      }

      if (found)
      {
        // check if controller exist already in the list and if not add it
        auto controllers = resource_storage_->hardware_used_by_controllers_[hw_name];
        auto ctrl_it = std::find(controllers.begin(), controllers.end(), controller_name);
        if (ctrl_it == controllers.end())
        {
          // add because it does not exist
          controllers.reserve(controllers.size() + 1);
          controllers.push_back(controller_name);
        }
        resource_storage_->hardware_used_by_controllers_[hw_name] = controllers;
        break;
      }
    }
  }
}

// CM API: Called in "update"-thread
std::vector<std::string> ResourceManager::get_cached_controllers_to_hardware(
  const std::string & hardware_name)
{
  return resource_storage_->hardware_used_by_controllers_[hardware_name];
}

// CM API: Called in "update"-thread
bool ResourceManager::command_interface_is_claimed(const std::string & key) const
{
  if (!command_interface_is_available(key))
  {
    return false;
  }

  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  return resource_storage_->claimed_command_interface_map_.at(key);
}

// CM API: Called in "update"-thread
LoanedCommandInterface ResourceManager::claim_command_interface(const std::string & key)
{
  if (!command_interface_is_available(key))
  {
    throw std::runtime_error(std::string("Command interface with '") + key + "' does not exist");
  }

  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  if (command_interface_is_claimed(key))
  {
    throw std::runtime_error(
      std::string("Command interface with '") + key + "' is already claimed");
  }

  resource_storage_->claimed_command_interface_map_[key] = true;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return LoanedCommandInterface(
    resource_storage_->command_interface_map_.at(key),
    std::bind(&ResourceManager::release_command_interface, this, key));
}

// CM API: Called in "update"-thread
void ResourceManager::release_command_interface(const std::string & key)
{
  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  resource_storage_->claimed_command_interface_map_[key] = false;
}

// CM API: Called in "callback/slow"-thread
std::vector<std::string> ResourceManager::command_interface_keys() const
{
  std::vector<std::string> keys;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & item : resource_storage_->command_interface_map_)
  {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

// CM API: Called in "update"-thread
std::vector<std::string> ResourceManager::available_command_interfaces() const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->available_command_interfaces_;
}

// CM API: Called in "callback/slow"-thread
bool ResourceManager::command_interface_is_available(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return std::find(
           resource_storage_->available_command_interfaces_.begin(),
           resource_storage_->available_command_interfaces_.end(),
           name) != resource_storage_->available_command_interfaces_.end();
}

void ResourceManager::import_component(
  std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info)
{
  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  resource_storage_->initialize_actuator(std::move(actuator), hardware_info);
  read_write_status.failed_hardware_names.reserve(
    resource_storage_->actuators_.size() + resource_storage_->sensors_.size() +
    resource_storage_->systems_.size());
}

void ResourceManager::import_component(
  std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info)
{
  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  resource_storage_->initialize_sensor(std::move(sensor), hardware_info);
  read_write_status.failed_hardware_names.reserve(
    resource_storage_->actuators_.size() + resource_storage_->sensors_.size() +
    resource_storage_->systems_.size());
}

void ResourceManager::import_component(
  std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info)
{
  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  resource_storage_->initialize_system(std::move(system), hardware_info);
  read_write_status.failed_hardware_names.reserve(
    resource_storage_->actuators_.size() + resource_storage_->sensors_.size() +
    resource_storage_->systems_.size());
}

// CM API: Called in "callback/slow"-thread
std::unordered_map<std::string, HardwareComponentInfo> ResourceManager::get_components_status()
{
  for (auto & component : resource_storage_->actuators_)
  {
    resource_storage_->hardware_info_map_[component.get_name()].state = component.get_state();
  }
  for (auto & component : resource_storage_->sensors_)
  {
    resource_storage_->hardware_info_map_[component.get_name()].state = component.get_state();
  }
  for (auto & component : resource_storage_->systems_)
  {
    resource_storage_->hardware_info_map_[component.get_name()].state = component.get_state();
  }

  return resource_storage_->hardware_info_map_;
}

// CM API: Called in "callback/slow"-thread
bool ResourceManager::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto interfaces_to_string = [&]()
  {
    std::stringstream ss;
    ss << "Start interfaces: " << std::endl << "[" << std::endl;
    for (const auto & start_if : start_interfaces)
    {
      ss << "  " << start_if << std::endl;
    }
    ss << "]" << std::endl;
    ss << "Stop interfaces: " << std::endl << "[" << std::endl;
    for (const auto & stop_if : stop_interfaces)
    {
      ss << "  " << stop_if << std::endl;
    }
    ss << "]" << std::endl;
    return ss.str();
  };

  for (auto & component : resource_storage_->actuators_)
  {
    if (return_type::OK != component.prepare_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' did not accept new command resource combination: \n %s",
        component.get_name().c_str(), interfaces_to_string().c_str());
      return false;
    }
  }
  for (auto & component : resource_storage_->systems_)
  {
    if (return_type::OK != component.prepare_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' did not accept new command resource combination: \n %s",
        component.get_name().c_str(), interfaces_to_string().c_str());
      return false;
    }
  }
  return true;
}

// CM API: Called in "update"-thread
bool ResourceManager::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (auto & component : resource_storage_->actuators_)
  {
    if (return_type::OK != component.perform_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' could not perform switch",
        component.get_name().c_str());
      return false;
    }
  }
  for (auto & component : resource_storage_->systems_)
  {
    if (return_type::OK != component.perform_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' could not perform switch",
        component.get_name().c_str());
      return false;
    }
  }
  return true;
}

// CM API: Called in "callback/slow"-thread
return_type ResourceManager::set_component_state(
  const std::string & component_name, rclcpp_lifecycle::State & target_state)
{
  using lifecycle_msgs::msg::State;
  using std::placeholders::_1;
  using std::placeholders::_2;

  auto found_it = resource_storage_->hardware_info_map_.find(component_name);

  if (found_it == resource_storage_->hardware_info_map_.end())
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Hardware Component with name '%s' does not exists",
      component_name.c_str());
    return return_type::ERROR;
  }

  return_type result = return_type::OK;

  if (target_state.id() == 0)
  {
    if (target_state.label() == lifecycle_state_names::UNCONFIGURED)
    {
      target_state = rclcpp_lifecycle::State(
        State::PRIMARY_STATE_UNCONFIGURED, lifecycle_state_names::UNCONFIGURED);
    }
    if (target_state.label() == lifecycle_state_names::INACTIVE)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
    }
    if (target_state.label() == lifecycle_state_names::ACTIVE)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
    }
    if (target_state.label() == lifecycle_state_names::FINALIZED)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED);
    }
  }

  auto find_set_component_state = [&](auto action, auto & components)
  {
    auto found_component_it = std::find_if(
      components.begin(), components.end(),
      [&](const auto & component) { return component.get_name() == component_name; });

    if (found_component_it != components.end())
    {
      if (action(*found_component_it, target_state))
      {
        result = return_type::OK;
      }
      else
      {
        result = return_type::ERROR;
      }
      return true;
    }
    return false;
  };

  bool found = find_set_component_state(
    std::bind(&ResourceStorage::set_component_state<Actuator>, resource_storage_.get(), _1, _2),
    resource_storage_->actuators_);
  if (!found)
  {
    found = find_set_component_state(
      std::bind(&ResourceStorage::set_component_state<Sensor>, resource_storage_.get(), _1, _2),
      resource_storage_->sensors_);
  }
  if (!found)
  {
    found = find_set_component_state(
      std::bind(&ResourceStorage::set_component_state<System>, resource_storage_.get(), _1, _2),
      resource_storage_->systems_);
  }

  return result;
}

// CM API: Called in "update"-thread
HardwareReadWriteStatus ResourceManager::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  read_write_status.ok = true;
  read_write_status.failed_hardware_names.clear();

  auto read_components = [&](auto & components)
  {
    for (auto & component : components)
    {
      if (component.read(time, period) != return_type::OK)
      {
        read_write_status.ok = false;
        read_write_status.failed_hardware_names.push_back(component.get_name());
        resource_storage_->remove_all_hardware_interfaces_from_available_list(component.get_name());
      }
    }
  };

  read_components(resource_storage_->actuators_);
  read_components(resource_storage_->sensors_);
  read_components(resource_storage_->systems_);

  return read_write_status;
}

// CM API: Called in "update"-thread
HardwareReadWriteStatus ResourceManager::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::lock_guard<std::recursive_mutex> guard(resources_lock_);
  read_write_status.ok = true;
  read_write_status.failed_hardware_names.clear();

  auto write_components = [&](auto & components)
  {
    for (auto & component : components)
    {
      if (component.write(time, period) != return_type::OK)
      {
        read_write_status.ok = false;
        read_write_status.failed_hardware_names.push_back(component.get_name());
        resource_storage_->remove_all_hardware_interfaces_from_available_list(component.get_name());
      }
    }
  };

  write_components(resource_storage_->actuators_);
  write_components(resource_storage_->systems_);

  return read_write_status;
}

// BEGIN: "used only in tests and locally"
size_t ResourceManager::actuator_components_size() const
{
  return resource_storage_->actuators_.size();
}

size_t ResourceManager::sensor_components_size() const
{
  return resource_storage_->sensors_.size();
}

size_t ResourceManager::system_components_size() const
{
  return resource_storage_->systems_.size();
}

bool ResourceManager::command_interface_exists(const std::string & key) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->command_interface_map_.find(key) !=
         resource_storage_->command_interface_map_.end();
}

bool ResourceManager::state_interface_exists(const std::string & key) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->state_interface_map_.find(key) !=
         resource_storage_->state_interface_map_.end();
}
// END: "used only in tests and locally"

// BEGIN: private methods

void ResourceManager::validate_storage(
  const std::vector<hardware_interface::HardwareInfo> & hardware_info) const
{
  std::vector<std::string> missing_state_keys = {};
  std::vector<std::string> missing_command_keys = {};

  for (const auto & hardware : hardware_info)
  {
    for (const auto & joint : hardware.joints)
    {
      for (const auto & state_interface : joint.state_interfaces)
      {
        if (!state_interface_exists(joint.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(joint.name + "/" + state_interface.name);
        }
      }
      for (const auto & command_interface : joint.command_interfaces)
      {
        if (!command_interface_exists(joint.name + "/" + command_interface.name))
        {
          missing_command_keys.emplace_back(joint.name + "/" + command_interface.name);
        }
      }
    }
    for (const auto & sensor : hardware.sensors)
    {
      for (const auto & state_interface : sensor.state_interfaces)
      {
        if (!state_interface_exists(sensor.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(sensor.name + "/" + state_interface.name);
        }
      }
    }
    for (const auto & gpio : hardware.gpios)
    {
      for (const auto & state_interface : gpio.state_interfaces)
      {
        if (!state_interface_exists(gpio.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(gpio.name + "/" + state_interface.name);
        }
      }
      for (const auto & command_interface : gpio.command_interfaces)
      {
        if (!command_interface_exists(gpio.name + "/" + command_interface.name))
        {
          missing_command_keys.emplace_back(gpio.name + "/" + command_interface.name);
        }
      }
    }
  }

  if (!missing_state_keys.empty() || !missing_command_keys.empty())
  {
    std::string err_msg = "Wrong state or command interface configuration.\n";
    err_msg += "missing state interfaces:\n";
    for (const auto & missing_key : missing_state_keys)
    {
      err_msg += "' " + missing_key + " '" + "\t";
    }
    err_msg += "\nmissing command interfaces:\n";
    for (const auto & missing_key : missing_command_keys)
    {
      err_msg += "' " + missing_key + " '" + "\t";
    }

    throw std::runtime_error(err_msg);
  }
}

// END: private methods

// Temporary method to keep old interface and reduce framework changes in the PRs
void ResourceManager::activate_all_components()
{
  using lifecycle_msgs::msg::State;
  rclcpp_lifecycle::State active_state(
    State::PRIMARY_STATE_ACTIVE, hardware_interface::lifecycle_state_names::ACTIVE);

  for (auto & component : resource_storage_->actuators_)
  {
    set_component_state(component.get_name(), active_state);
  }
  for (auto & component : resource_storage_->sensors_)
  {
    set_component_state(component.get_name(), active_state);
  }
  for (auto & component : resource_storage_->systems_)
  {
    set_component_state(component.get_name(), active_state);
  }
}

}  // namespace hardware_interface
