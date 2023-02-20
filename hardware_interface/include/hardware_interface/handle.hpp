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
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
/// A handle used to get and set a value on a given interface.
class Handle
{
public:
  Handle(const std::string & prefix_name, const std::string & interface_name)
  : prefix_name_(prefix_name),
    interface_name_(interface_name),
    has_new_data_(false),
    value_(HandleValue(
      std::numeric_limits<double>::quiet_NaN(), hardware_interface::return_type::INVALID))
  {
  }

  explicit Handle(const std::string & interface_name)
  : interface_name_(interface_name),
    has_new_data_(false),
    value_(HandleValue(
      std::numeric_limits<double>::quiet_NaN(), hardware_interface::return_type::INVALID))
  {
  }

  explicit Handle(const char * interface_name)
  : interface_name_(interface_name),
    has_new_data_(false),
    value_(HandleValue(
      std::numeric_limits<double>::quiet_NaN(), hardware_interface::return_type::INVALID))
  {
  }

  // Handle should be unique
  Handle(const Handle & other) = delete;

  Handle(Handle && other) = default;

  Handle & operator=(const Handle & other) = default;

  Handle & operator=(Handle && other) = default;

  virtual ~Handle() = default;

  const std::string get_name() const { return prefix_name_ + "/" + interface_name_; }

  const std::string & get_interface_name() const { return interface_name_; }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return get_name();
  }

  const std::string & get_prefix_name() const { return prefix_name_; }

  /**
   * @brief Set the new value of the handle and mark the Handle as "has_new_data_ = true".
   * This indicates that new data has been set since last read access.
   *
   * @param value current stored value in the handle.
   */
  virtual void set_value(const HandleValue & value)
  {
    value_ = value;
    has_new_data_ = true;
  }

  /**
   * @brief Get the value of the handle an mark the handle as "has_new_data_ = false"
   * since the value has been read and not be changed since last read access.
   *
   * @return HandleValue is the current stored value of the handle.
   */
  virtual HandleValue get_value()
  {
    has_new_data_ = false;
    return value_;
  }

  /**
   * @brief Indicates if new value has been stored in the handle since the last
   * read access.
   *
   * @return true => new value has been stored since last read access to the handle.
   * @return false => no new value has been stored since last read access to the handle.
   */
  virtual bool has_new_data() const { return has_new_data_; }

protected:
  std::string prefix_name_;
  std::string interface_name_;
  // marks if data is new or has already been read
  bool has_new_data_;
  // the current stored value of the handle
  HandleValue value_;
};

class StateInterface : public Handle
{
public:
  explicit StateInterface(const InterfaceDescription & interface_description)
  : Handle(interface_description.prefix_name, interface_description.interface_info.name)
  {
  }

  StateInterface(const StateInterface & other) = delete;

  StateInterface(StateInterface && other) = default;

  using Handle::Handle;
};

class CommandInterface : public Handle
{
public:
  explicit CommandInterface(const InterfaceDescription & interface_description)
  : Handle(interface_description.prefix_name, interface_description.interface_info.name)
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
