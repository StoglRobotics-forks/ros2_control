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

#ifndef HARDWARE_INTERFACE__LOANED_COMMAND_INTERFACE_HPP_
#define HARDWARE_INTERFACE__LOANED_COMMAND_INTERFACE_HPP_

#include <functional>
#include <string>
#include <utility>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace hardware_interface
{
class LoanedCommandInterface
{
public:
  using Deleter = std::function<void(void)>;

  explicit LoanedCommandInterface(std::shared_ptr<ReadWriteHandle> command_interface)
  : LoanedCommandInterface(command_interface, nullptr)
  {
  }

  LoanedCommandInterface(std::shared_ptr<ReadWriteHandle> command_interface, Deleter && deleter)
  : command_interface_(command_interface), deleter_(std::forward<Deleter>(deleter))
  {
  }

  LoanedCommandInterface(const LoanedCommandInterface & other) = delete;

  LoanedCommandInterface(LoanedCommandInterface && other) = default;

  virtual ~LoanedCommandInterface()
  {
    if (deleter_)
    {
      deleter_();
    }
  }

  std::string get_name() const { return command_interface_->get_name(); }

  std::string get_interface_name() const { return command_interface_->get_interface_name(); }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return command_interface_->get_name();
  }

  std::string get_prefix_name() const { return command_interface_->get_prefix_name(); }

  double get_value() const { return command_interface_->get_value(); }

  std::string get_underscore_separated_name() const
  {
    return command_interface_->get_underscore_separated_name();
  }

  bool has_new_value() const { return command_interface_->has_new_value(); }

  void set_behavior(std::shared_ptr<SetValueBehavior> behavior)
  {
    command_interface_->set_behavior(behavior);
  }

  void set_value(double value) { command_interface_->set_value(value); }

  void set_value_on_receive(double value) { command_interface_->set_value_on_receive(value); }

  bool value_is_valid() const { return command_interface_->value_is_valid(); }

protected:
  std::shared_ptr<ReadWriteHandle> command_interface_;
  Deleter deleter_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__LOANED_COMMAND_INTERFACE_HPP_
