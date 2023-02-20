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

#ifndef HARDWARE_INTERFACE__LOANED_HW_COMMAND_INTERFACE_HPP_
#define HARDWARE_INTERFACE__LOANED_HW_COMMAND_INTERFACE_HPP_

#include <functional>
#include <limits>
#include <string>
#include <utility>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace hardware_interface
{
class LoanedHwCommandInterface
{
public:
  using Deleter = std::function<void(void)>;

  explicit LoanedHwCommandInterface(CommandInterface & command_interface)
  : LoanedHwCommandInterface(command_interface, nullptr)
  {
  }

  LoanedHwCommandInterface(CommandInterface & command_interface, Deleter && deleter)
  : command_interface_(command_interface), deleter_(std::forward<Deleter>(deleter))
  {
  }

  LoanedHwCommandInterface(const LoanedHwCommandInterface & other) = delete;

  LoanedHwCommandInterface(LoanedHwCommandInterface && other) = default;

  virtual ~LoanedHwCommandInterface()
  {
    if (deleter_)
    {
      deleter_();
    }
  }

  const std::string get_name() const { return command_interface_.get_name(); }

  const std::string & get_interface_name() const { return command_interface_.get_interface_name(); }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return command_interface_.get_name();
  }

  const std::string & get_prefix_name() const { return command_interface_.get_prefix_name(); }

  hardware_interface::HandleValue get_command() const { return command_interface_.get_value(); }

  // Should we provide this?
  double get_plain_command() const { return command_interface_.get_value().value(); }

  bool has_new_data() const { return command_interface_.has_new_data(); }

  void reset_command()
  {
    command_interface_.set_value(hardware_interface::HandleValue(
      std::numeric_limits<double>::quiet_NaN(), hardware_interface::return_type::INVALID));
  }

  // Should we provide this?
  void reset_command(const double & value)
  {
    command_interface_.set_value(
      hardware_interface::HandleValue(value, hardware_interface::return_type::INVALID));
  }

  // Should we provide this?
  void reset_command(const hardware_interface::HandleValue & value)
  {
    command_interface_.set_value(value);
  }

protected:
  CommandInterface & command_interface_;
  Deleter deleter_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__LOANED_HW_COMMAND_INTERFACE_HPP_
