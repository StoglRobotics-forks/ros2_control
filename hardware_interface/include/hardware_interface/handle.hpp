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

#include <string>
#include <utility>

#include "hardware_interface/macros.hpp"
#include "hardware_interface/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
namespace hardware_interface
{
/// A handle used to get and set a value on a given interface.
class ReadHandleInterface
{
public:
  virtual double get_value() const = 0;
};

class WriteHandleInterface
{
public:
  virtual void set_value(double value) = 0;
};

class HandleInterface
{
public:
  HandleInterface(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr)
  : prefix_name_(prefix_name), interface_name_(interface_name), value_ptr_(value_ptr), node_(node)
  {
  }

  explicit HandleInterface(const std::string & interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr), node_(nullptr)
  {
  }

  explicit HandleInterface(const char * interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr), node_(nullptr)
  {
  }

  HandleInterface(const HandleInterface & other) = default;

  HandleInterface(HandleInterface && other) = default;

  HandleInterface & operator=(const HandleInterface & other) = default;

  HandleInterface & operator=(HandleInterface && other) = default;

  virtual ~HandleInterface() = default;

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

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const
  {
    THROW_ON_NULLPTR(node_);
    if (!node_.get())
    {
      throw std::runtime_error("DistributedReadOnlyHandle: Node not initialized!");
    }
    return node_;
  }

  const std::string & get_prefix_name() const { return prefix_name_; }

protected:
  std::string prefix_name_;
  std::string interface_name_;
  double * value_ptr_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

class ReadOnlyHandle : public HandleInterface, ReadHandleInterface
{
public:
  ReadOnlyHandle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : HandleInterface(prefix_name, interface_name, value_ptr)
  {
  }

  explicit ReadOnlyHandle(const std::string & interface_name) : HandleInterface(interface_name) {}

  explicit ReadOnlyHandle(const char * interface_name) : HandleInterface(interface_name) {}

  ReadOnlyHandle(const ReadOnlyHandle & other) = default;

  ReadOnlyHandle(ReadOnlyHandle && other) = default;

  ReadOnlyHandle & operator=(const ReadOnlyHandle & other) = default;

  ReadOnlyHandle & operator=(ReadOnlyHandle && other) = default;

  virtual ~ReadOnlyHandle() = default;

  double get_value() const override
  {
    THROW_ON_NULLPTR(value_ptr_);
    return *value_ptr_;
  }
};

class ReadWriteHandle : public HandleInterface, ReadHandleInterface, WriteHandleInterface
{
public:
  ReadWriteHandle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : HandleInterface(prefix_name, interface_name, value_ptr)
  {
  }

  explicit ReadWriteHandle(const std::string & interface_name) : HandleInterface(interface_name) {}

  explicit ReadWriteHandle(const char * interface_name) : HandleInterface(interface_name) {}

  ReadWriteHandle(const ReadWriteHandle & other) = default;

  ReadWriteHandle(ReadWriteHandle && other) = default;

  ReadWriteHandle & operator=(const ReadWriteHandle & other) = default;

  ReadWriteHandle & operator=(ReadWriteHandle && other) = default;

  virtual ~ReadWriteHandle() = default;

  double get_value() const override
  {
    THROW_ON_NULLPTR(value_ptr_);
    return *value_ptr_;
  }

  void set_value(double value) override
  {
    THROW_ON_NULLPTR(this->value_ptr_);
    *this->value_ptr_ = value;
  }
};

class DistributedReadOnlyHandle : public ReadOnlyHandle
{
public:
  // TODO(Manuel): We should pass the initial value via service call, so that the value_ of ReadOnlyHandle
  // is initialized with a feasible value.
  DistributedReadOnlyHandle(
    const std::string & prefix_name, const std::string & interface_name,
    const std::string & topic_name)
  : ReadOnlyHandle(prefix_name, interface_name, &value_), topic_name_(topic_name)
  {
    rclcpp::NodeOptions node_options;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      get_interface_name() + "_node", get_prefix_name(), node_options, false);
    state_value_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
      topic_name, 10,
      std::bind(&DistributedReadOnlyHandle::set_value_cb, this, std::placeholders::_1));
  }

  explicit DistributedReadOnlyHandle(const std::string & interface_name)
  : ReadOnlyHandle(interface_name)
  {
  }

  explicit DistributedReadOnlyHandle(const char * interface_name) : ReadOnlyHandle(interface_name)
  {
  }

  DistributedReadOnlyHandle(const DistributedReadOnlyHandle & other) = default;

  DistributedReadOnlyHandle(DistributedReadOnlyHandle && other) = default;

  DistributedReadOnlyHandle & operator=(const DistributedReadOnlyHandle & other) = default;

  DistributedReadOnlyHandle & operator=(DistributedReadOnlyHandle && other) = default;

  virtual ~DistributedReadOnlyHandle() = default;

protected:
  void set_value_cb(const std_msgs::msg::Float64 & msg) { value_ = msg.data; }

  std::string topic_name_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_value_subscription_;
  double value_;
};

class StateInterface : public ReadOnlyHandle
{
public:
  StateInterface(const StateInterface & other) = default;

  StateInterface(StateInterface && other) = default;

  using ReadOnlyHandle::ReadOnlyHandle;
};

class DistributedStateInterface : public DistributedReadOnlyHandle
{
  DistributedStateInterface(const DistributedStateInterface & other) = default;

  DistributedStateInterface(DistributedStateInterface && other) = default;

  using DistributedReadOnlyHandle::DistributedReadOnlyHandle;
};

class CommandInterface : public ReadWriteHandle
{
public:
  /// CommandInterface copy constructor is actively deleted.
  /**
   * Command interfaces are having a unique ownership and thus
   * can't be copied in order to avoid simultaneous writes to
   * the same resource.
   */
  CommandInterface(const CommandInterface & other) = delete;

  CommandInterface(CommandInterface && other) = default;

  using ReadWriteHandle::ReadWriteHandle;
};
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
