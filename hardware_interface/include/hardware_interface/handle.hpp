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
#include <string>
#include <utility>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/macros.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
/// A handle used to get and set a value on a given interface.
class Handle
{
public:
  [[deprecated("Use InterfaceDescription for initializing the Command-/StateIntefaces.")]] Handle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : prefix_name_(prefix_name), interface_name_(interface_name), value_ptr_(value_ptr)
  {
  }

  explicit Handle(const InterfaceDescription & interface_description)
  : interface_description_(interface_description),
    prefix_name_(interface_description.get_prefix_name()),
    interface_name_(interface_description.get_interface_type()),
    value_(std::numeric_limits<double>::quiet_NaN()),
    value_ptr_(&value_)
  {
  }

  [[deprecated("Use InterfaceDescription for initializing the Command-/StateIntefaces.")]]

  explicit Handle(const std::string & interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
  {
  }

  [[deprecated("Use InterfaceDescription for initializing the Command-/StateIntefaces.")]]

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

  const std::string & get_component_type() const
  {
    return interface_description_.get_component_type();
  }

  const std::string get_name() const
  {
    // deprecated is going to be replaced by interface_description_.get_name()
    return prefix_name_ + "/" + interface_name_;
  }

  const std::string & get_interface_name() const { return interface_name_; }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return get_name();
  }

  // Only used for hw side. LoanedStateInterface does not expose this to controllers
  void set_value(const double & value) { *value_ptr_ = value; }

  const std::string & get_prefix_name() const
  {
    // deprecated is going to be replaced by interface_description_.get_prefix_name()
    return prefix_name_;
  }

  double get_value() const
  {
    THROW_ON_NULLPTR(value_ptr_);
    return *value_ptr_;
  }

protected:
  InterfaceDescription interface_description_;
  // deprecated is going to be replaced by InterfaceDescription
  std::string prefix_name_;
  // deprecated is going to be replaced by InterfaceDescription
  std::string interface_name_;
  double value_;
  // deprecated is going to be replaced by value_
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

  CommandInterface & operator=(const Handle & other) = delete;

  CommandInterface(CommandInterface && other) = default;

  using Handle::Handle;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
