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

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/macros.hpp"
#include "hardware_interface/types/hardware_interface_error_signals.hpp"
#include "hardware_interface/types/hardware_interface_warning_signals.hpp"
#include "hardware_interface/visibility_control.h"
namespace hardware_interface
{

using HANDLE_DATATYPE = std::variant<
  bool, double, hardware_interface::WARNING_SIGNALS, hardware_interface::ERROR_SIGNALS,
  hardware_interface::WARNING_MESSAGES>;

// Define a type trait for allowed types
template <typename T>
struct HANDLE_DATATYPE_TYPES : std::disjunction<
                                 std::is_same<T, bool>, std::is_same<T, double>,
                                 std::is_same<T, hardware_interface::WARNING_SIGNALS>,
                                 std::is_same<T, hardware_interface::ERROR_SIGNALS>,
                                 std::is_same<T, hardware_interface::WARNING_MESSAGES>>
{
};

/// A handle used to get and set a value on a given interface.
class Handle
{
public:
  [[deprecated("Use InterfaceDescription for initializing the Interface")]]

  Handle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : prefix_name_(prefix_name), interface_name_(interface_name), value_ptr_(value_ptr)
  {
  }

  explicit Handle(const InterfaceDescription & interface_description)
  : prefix_name_(interface_description.prefix_name),
    interface_name_(interface_description.interface_info.name)
  {
    // TODO(Manuel): implement this.
    // As soon as multiple datatypes are used in HANDLE_DATATYPE
    // we need to initialize according the type passed in interface description
    // value_ = std::numeric_limits<double>::quiet_NaN();
  }

  [[deprecated("Use InterfaceDescription for initializing the Interface")]]

  explicit Handle(const std::string & interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
  {
  }

  [[deprecated("Use InterfaceDescription for initializing the Interface")]]

  explicit Handle(const char * interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
  {
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
