// Copyright 2020 PAL Robotics S.L.
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

#ifndef HARDWARE_INTERFACE__HANDLE_HPP_
#define HARDWARE_INTERFACE__HANDLE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/macros.hpp"
#include "hardware_interface/types/handle_datatype.hpp"

namespace hardware_interface
{

/// A handle used to get and set a value on a given interface.
class Handle
{
public:
  [[deprecated("Use InterfaceDescription for initializing the Interface")]]

  Handle(const std::string & prefix_name, const std::string & interface_name)
  : prefix_name_(prefix_name), interface_name_(interface_name)
  {
    // init to default value defined by init_handle_value()
    InterfaceInfo info;
    init_handle_value(info);
  }

  explicit Handle(const InterfaceDescription & interface_description)
  : prefix_name_(interface_description.prefix_name),
    interface_name_(interface_description.interface_info.name)
  {
    init_handle_value(interface_description.interface_info);
  }

  Handle(const Handle & other) = default;

  Handle(Handle && other) = default;

  Handle & operator=(const Handle & other) = default;

  Handle & operator=(Handle && other) = default;

  virtual ~Handle() = default;

  /// Returns true if handle references a value.
  inline operator bool() const { return value_ptr_ != nullptr; }

  const std::string get_name() const { return prefix_name_ + "/" + interface_name_; }

  const std::string & get_interface_name() const { return interface_name_; }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return get_name();
  }

  const std::string & get_prefix_name() const { return prefix_name_; }

  template <typename T, typename std::enable_if<HANDLE_DATATYPE_TYPES<T>::value, int>::type = 0>
  T get_value() const
  {
    return std::get<T>(value_);
  }

  template <typename T, typename std::enable_if<HANDLE_DATATYPE_TYPES<T>::value, int>::type = 0>
  void set_value(T value)
  {
    value_ = value;
  }

protected:
  // used for the
  bool correct_vector_size(const size_t & expected, const size_t & actual)
  {
    return expected == actual;
  }

  void init_handle_value(const InterfaceInfo & interface_info)
  {
    if (interface_info.data_type == "bool")
    {
      value_ = interface_info.initial_value.empty() ? false
                                                    : (interface_info.initial_value == "true" ||
                                                       interface_info.initial_value == "True");
    }
    else if (interface_info.data_type == "vector<int8_t>")
    {
      if (
        interface_info.size != 0 && hardware_interface::warning_signal_count != interface_info.size)
      {
        throw std::runtime_error(
          "The size:{" + std::to_string(interface_info.size) + "} for data_type{" +
          interface_info.data_type + "} for the InterfaceInfo with name:{" + interface_info.name +
          "} does not equal the expected size:{" +
          std::to_string(hardware_interface::warning_signal_count) + "}.");
      }
      value_ = std::vector<int8_t>(hardware_interface::warning_signal_count, 0);
    }
    else if (interface_info.data_type == "vector<uint8_t>")
    {
      if (interface_info.size != 0 && hardware_interface::error_signal_count != interface_info.size)
      {
        throw std::runtime_error(
          "The size:{" + std::to_string(interface_info.size) + "} for data_type{" +
          interface_info.data_type + "} for the InterfaceInfo with name:{" + interface_info.name +
          "} does not equal the expected size:{" +
          std::to_string(hardware_interface::error_signal_count) + "}.");
      }

      value_ = std::vector<uint8_t>(hardware_interface::error_signal_count, 0);
    }
    else if (interface_info.data_type == "vector<string>")
    {
      if (
        interface_info.size != 0 && hardware_interface::warning_signal_count != interface_info.size)
      {
        throw std::runtime_error(
          "The size:{" + std::to_string(interface_info.size) + "} for data_type{" +
          interface_info.data_type + "} for the InterfaceInfo with name:{" + interface_info.name +
          "} does not equal the expected size:{" +
          std::to_string(hardware_interface::warning_signal_count) + "}.");
      }

      value_ = std::vector<std::string>(hardware_interface::warning_signal_count, "");
    }
    // Default for empty is double
    else if (interface_info.data_type.empty() || interface_info.data_type == "double")
    {
      value_ = interface_info.initial_value.empty() ? std::numeric_limits<double>::quiet_NaN()
                                                    : std::stod(interface_info.initial_value);
    }
    // If not empty and it belongs to none of the above types, we still want to throw as there might
    // be a typo in the data_type like "bol" or user wants some unsupported type
    else
    {
      throw std::runtime_error(
        "The data_type:{" + interface_info.data_type + "} for the InterfaceInfo with name:{" +
        interface_info.name +
        "} is not supported for Handles. Supported data_types are: bool, double, vector<int8_t>, "
        "vector<uint8_t> and vector<string>.");
    }
  }

  std::string prefix_name_;
  std::string interface_name_;
  HANDLE_DATATYPE value_;
  double * value_ptr_;
};

class StateInterface : public Handle
{
public:
  explicit StateInterface(const InterfaceDescription & interface_description)
  : Handle(interface_description)
  {
  }

  StateInterface(const StateInterface & other) = default;

  StateInterface(StateInterface && other) = default;

  using Handle::Handle;
};

class CommandInterface : public Handle
{
public:
  explicit CommandInterface(const InterfaceDescription & interface_description)
  : Handle(interface_description)
  {
  }
  /// CommandInterface copy constructor is actively deleted.
  /**
   * Command interfaces are having a unique ownership and thus
   * can't be copied in order to avoid simultaneous writes to
   * the same resource.
   */
  CommandInterface(const CommandInterface & other) = delete;

  CommandInterface(CommandInterface && other) = default;

  using Handle::Handle;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
