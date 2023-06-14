
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

#include "hardware_interface/distributed_control_interface/evaluation_helper.hpp"
#include "hardware_interface/distributed_control_interface/publisher_description.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/macros.hpp"
#include "hardware_interface/visibility_control.h"

#include "controller_manager_msgs/msg/interface_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
namespace hardware_interface
{
/// A handle used to get and set a value on a given interface.

class SetValueBehavior
{
public:
  virtual double set_value(double value) = 0;
};

class Identity : public SetValueBehavior
{
public:
  double set_value(double value) override { return value; }
};

class PublishBehavior : public SetValueBehavior
{
public:
  explicit PublishBehavior(
    const std::string & ns, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & topic_name, rclcpp::QoS qos_profile)
  : namespace_(ns), node_(node), topic_name_(topic_name)
  {
    state_value_pub_ = node_->create_publisher<controller_manager_msgs::msg::InterfaceData>(
      topic_name_, qos_profile);
  }

  double set_value(double value) override
  {
    auto msg = std::make_unique<controller_manager_msgs::msg::InterfaceData>();
    msg->data = value;
    RCLCPP_DEBUG(node_->get_logger(), "Publishing: '%.7lf'", msg->data);
    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    msg->header.seq = seq_number_;
    ++seq_number_;
    state_value_pub_->publish(std::move(msg));
    return value;
  }

protected:
  const std::string namespace_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  const std::string topic_name_;
  rclcpp::Publisher<controller_manager_msgs::msg::InterfaceData>::SharedPtr state_value_pub_;
  uint32_t seq_number_ = 0;
};

class HandleInterface
{
public:
  HandleInterface(
    const std::string & prefix_name, const std::string & interface_name,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr)
  : prefix_name_(prefix_name),
    interface_name_(interface_name),
    has_new_value_(false),
    is_valid_(false),
    value_(std::numeric_limits<double>::quiet_NaN()),
    node_(node)
  {
    behavior_ = std::make_shared<Identity>();
  }

  explicit HandleInterface(const std::string & interface_name)
  : interface_name_(interface_name),
    has_new_value_(false),
    is_valid_(false),
    value_(std::numeric_limits<double>::quiet_NaN()),
    node_(nullptr)
  {
    behavior_ = std::make_shared<Identity>();
  }

  explicit HandleInterface(const char * interface_name)
  : interface_name_(interface_name),
    has_new_value_(false),
    is_valid_(false),
    value_(std::numeric_limits<double>::quiet_NaN()),
    node_(nullptr)
  {
    behavior_ = std::make_shared<Identity>();
  }

  HandleInterface(const HandleInterface & other) = default;

  HandleInterface(HandleInterface && other) = default;

  HandleInterface & operator=(const HandleInterface & other) = default;

  HandleInterface & operator=(HandleInterface && other) = default;

  virtual ~HandleInterface() = default;

  // /// Returns true if handle references a value.
  // inline operator bool() const { return value_ptr_ != nullptr; }

  std::string get_interface_name() const { return interface_name_; }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return get_name();
  }

  void set_behavior(std::shared_ptr<SetValueBehavior> behavior) { behavior_ = behavior; }

  virtual std::string get_name() const { return prefix_name_ + "/" + interface_name_; }

  // TODO(Manuel): Maybe not the best place to put...
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const
  {
    THROW_ON_NULLPTR(node_);
    if (!node_.get())
    {
      throw std::runtime_error("Node not initialized!");
    }
    return node_;
  }

  std::string get_prefix_name() const { return prefix_name_; }

  /**
   * @brief Create the full name consisting of prefix and interface name separated by an underscore.
   * Used for e.g. name generation of nodes, where "/" are not allowed.
   *
   * @return std::string prefix_name + _ + interface_name.
   */
  virtual std::string get_underscore_separated_name() const
  {
    return append_char(get_prefix_name(), '_') + get_interface_name();
  }

  /**
   * @brief Get the value of the handle an mark the handle as "has_new_value_ = false"
   * since the value has been read and not be changed since last read access.
   *
   * @return HandleValue is the current stored value of the handle.
   */
  virtual double get_value()
  {
    has_new_value_ = false;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Handle"), "get_value value[" << value_ << "]");
    return value_;
  }

  /**
   * @brief Set the new value of the handle and mark the Handle as "has_new_value_ = true".
   * This indicates that new data has been set since last read access.
   *
   * @param value current stored value in the handle.
   */
  virtual void set_value(double value)
  {
    value_ = behavior_->set_value(value);
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("Handle"),
      "set_value with value[" << value << "] and after set: value[" << value_ << "]");
    has_new_value_ = true;
  }

  /**
   * @brief Indicates if new value has been stored in the handle since the last
   * read access.
   *
   * @return true => new value has been stored since last read access to the handle.
   * @return false => no new value has been stored since last read access to the handle.
   */
  virtual bool has_new_value() const { return has_new_value_; }

  /**
   * @brief Indicates if the value stored inside the handle is valid
   *
   * @return true => stored value is valid and can be used.
   * @return false => false stored value is not valid and should not be used.
   */
  virtual bool value_is_valid() const
  {
    if (value_ == std::numeric_limits<double>::quiet_NaN())
    {
      return false;
    }
    return true;
  }

protected:
  std::string append_char(std::string str, const char & char_to_append) const
  {
    if (!str.empty())
    {
      return str + char_to_append;
    }
    return str;
  }

  std::string erase_slash_at_start(std::string str) const
  {
    if (!str.empty())
    {
      if (str.at(0) == '/')
      {
        return str.erase(0, 1);
      }
    }
    return str;
  }

  std::string replace_all_chars_from_string(
    std::string str, const char & char_to_replace, const char & replace_with_char) const
  {
    std::replace(str.begin(), str.end(), char_to_replace, replace_with_char);
    return str;
  }

  std::string prefix_name_;
  std::string interface_name_;
  // marks if value is new or has already been read
  bool has_new_value_;
  // stores if the stored value is valid
  bool is_valid_;
  // the current stored value of the handle
  double value_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<SetValueBehavior> behavior_;
};

class ReadOnlyHandle : public HandleInterface
{
public:
  ReadOnlyHandle(
    const std::string & prefix_name, const std::string & interface_name,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr)
  : HandleInterface(prefix_name, interface_name, node)
  {
  }

  explicit ReadOnlyHandle(const std::string & interface_name) : HandleInterface(interface_name) {}

  explicit ReadOnlyHandle(const char * interface_name) : HandleInterface(interface_name) {}

  ReadOnlyHandle(const ReadOnlyHandle & other) = default;

  ReadOnlyHandle(ReadOnlyHandle && other) = default;

  ReadOnlyHandle & operator=(const ReadOnlyHandle & other) = default;

  ReadOnlyHandle & operator=(ReadOnlyHandle && other) = default;

  virtual ~ReadOnlyHandle() = default;
};

class StateInterface : public ReadOnlyHandle
{
public:
  explicit StateInterface(const InterfaceDescription & interface_description)
  : ReadOnlyHandle(interface_description.prefix_name, interface_description.interface_info.name)
  {
  }
  StateInterface(const StateInterface & other) = default;

  StateInterface(StateInterface && other) = default;

  using ReadOnlyHandle::ReadOnlyHandle;
};

class DistributedReadOnlyHandle : public ReadOnlyHandle
{
public:
  DistributedReadOnlyHandle(
    const distributed_control::PublisherDescription & description, const std::string & ns,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
  : ReadOnlyHandle(description.prefix_name(), description.interface_name(), node),
    get_value_topic_name_(description.topic_name()),
    namespace_(ns),
    interface_namespace_(description.get_namespace())
  {
    // if no node has been passed
    // create node for subscribing to StatePublisher described in StatePublisherDescription
    if (!node_.get())
    {
      rclcpp::NodeOptions node_options;
      node_options.clock_type(rcl_clock_type_t::RCL_STEADY_TIME);
      node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        get_underscore_separated_name() + "_state_interface_subscriber", namespace_, node_options,
        false);
    }

    auto evaluation_helper = evaluation_helper::Evaluation_Helper::get_instance();
    rclcpp::QoS qos_profile(
      rclcpp::QoSInitialization::from_rmw(evaluation_helper->get_qos_profile()));
    // subscribe to topic provided by StatePublisher
    state_value_sub_ = node_->create_subscription<controller_manager_msgs::msg::InterfaceData>(
      get_value_topic_name_, qos_profile,
      std::bind(&DistributedReadOnlyHandle::receive_value_cb, this, std::placeholders::_1));
  }

  explicit DistributedReadOnlyHandle(const std::string & interface_name)
  : ReadOnlyHandle(interface_name)
  {
  }

  explicit DistributedReadOnlyHandle(const char * interface_name) : ReadOnlyHandle(interface_name)
  {
  }

  DistributedReadOnlyHandle() = delete;

  DistributedReadOnlyHandle(const DistributedReadOnlyHandle & other) = delete;

  DistributedReadOnlyHandle(DistributedReadOnlyHandle && other) = default;

  DistributedReadOnlyHandle & operator=(const DistributedReadOnlyHandle & other) = default;

  DistributedReadOnlyHandle & operator=(DistributedReadOnlyHandle && other) = default;

  virtual ~DistributedReadOnlyHandle() = default;

  virtual std::string get_name() const override
  {
    // concatenate: interface_namespace/prefix_name/interface_name to obtain
    // a unique name.
    return append_char(interface_namespace_, '/') + append_char(get_prefix_name(), '/') +
           get_interface_name();
  }

  virtual std::string get_underscore_separated_name() const override
  {
    // remove first "/" from namespace and replace all follow occurrences of "/" with "_"
    std::string ns =
      replace_all_chars_from_string(erase_slash_at_start(interface_namespace_), '/', '_');
    // concatenate: interface_namespace + _ + namespace_prefix + _ + name_interface_name
    return append_char(ns, '_') + append_char(get_prefix_name(), '_') + get_interface_name();
  }

protected:
  void receive_value_cb(const controller_manager_msgs::msg::InterfaceData & msg)
  {
    set_value(msg.data);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Receiving:[" << value_ << "].");
  }

  std::string get_value_topic_name_;
  // the current namespace we are in. Needed to create the node in the correct namespace
  std::string namespace_;
  // the namespace the actual StateInterface we subscribe to is in.
  // We need this to create unique names for the StateInterface.
  std::string interface_namespace_;
  rclcpp::Subscription<controller_manager_msgs::msg::InterfaceData>::SharedPtr state_value_sub_;
};

class DistributedStateInterface : public DistributedReadOnlyHandle
{
public:
  DistributedStateInterface(const DistributedStateInterface & other) = default;

  DistributedStateInterface(DistributedStateInterface && other) = default;

  using DistributedReadOnlyHandle::DistributedReadOnlyHandle;
};

class ReadWriteHandle : public HandleInterface
{
public:
  ReadWriteHandle(
    const std::string & prefix_name, const std::string & interface_name,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr)
  : HandleInterface(prefix_name, interface_name, node)
  {
  }

  explicit ReadWriteHandle(const std::string & interface_name) : HandleInterface(interface_name) {}

  explicit ReadWriteHandle(const char * interface_name) : HandleInterface(interface_name) {}

  ReadWriteHandle(const ReadWriteHandle & other) = default;

  ReadWriteHandle(ReadWriteHandle && other) = default;

  ReadWriteHandle & operator=(const ReadWriteHandle & other) = default;

  ReadWriteHandle & operator=(ReadWriteHandle && other) = default;

  virtual void set_value_on_receive(double value)
  {
    has_new_value_ = true;
    value_ = value;
  }

  virtual ~ReadWriteHandle() = default;
};

class CommandInterface : public ReadWriteHandle
{
public:
  explicit CommandInterface(const InterfaceDescription & interface_description)
  : ReadWriteHandle(interface_description.prefix_name, interface_description.interface_info.name)
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

  using ReadWriteHandle::ReadWriteHandle;
};

class DistributedReadWriteHandle : public ReadWriteHandle
{
public:
  DistributedReadWriteHandle(
    const distributed_control::PublisherDescription & description, const std::string & ns,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
  : ReadWriteHandle(description.prefix_name(), description.interface_name(), node),
    get_value_topic_name_(description.topic_name()),
    namespace_(ns),
    interface_namespace_(description.get_namespace()),
    forward_command_topic_name_(get_underscore_separated_name() + "_command_forwarding")
  {
    // if no node has been passed
    // create node for subscribing to CommandForwarder described in CommandForwarderDescription
    if (!node_.get())
    {
      rclcpp::NodeOptions node_options;
      node_options.clock_type(rcl_clock_type_t::RCL_STEADY_TIME);
      node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        get_underscore_separated_name() + "_distributed_command_interface", namespace_,
        node_options, false);
    }

    auto evaluation_helper = evaluation_helper::Evaluation_Helper::get_instance();
    rclcpp::QoS qos_profile(
      rclcpp::QoSInitialization::from_rmw(evaluation_helper->get_qos_profile()));
    // subscribe to topic provided by CommandForwarder
    command_value_sub_ = node_->create_subscription<controller_manager_msgs::msg::InterfaceData>(
      get_value_topic_name_, qos_profile,
      std::bind(&DistributedReadWriteHandle::receive_value_cb, this, std::placeholders::_1));

    // create publisher so that we can forward the commands
    command_value_pub_ = node_->create_publisher<controller_manager_msgs::msg::InterfaceData>(
      forward_command_topic_name_, qos_profile);
  }

  explicit DistributedReadWriteHandle(const std::string & interface_name)
  : ReadWriteHandle(interface_name)
  {
  }

  explicit DistributedReadWriteHandle(const char * interface_name) : ReadWriteHandle(interface_name)
  {
  }

  DistributedReadWriteHandle(const DistributedReadWriteHandle & other) = default;

  DistributedReadWriteHandle(DistributedReadWriteHandle && other) = default;

  DistributedReadWriteHandle & operator=(const DistributedReadWriteHandle & other) = default;

  DistributedReadWriteHandle & operator=(DistributedReadWriteHandle && other) = default;

  virtual ~DistributedReadWriteHandle() = default;

  virtual std::string get_name() const override
  {
    // concatenate: interface_namespace/prefix_name/interface_name to obtain
    // a unique name.
    return append_char(interface_namespace_, '/') + append_char(get_prefix_name(), '/') +
           get_interface_name();
  }

  virtual std::string get_underscore_separated_name() const override
  {
    // remove first "/" from namespace and replace all follow occurrences with "_"
    std::string ns =
      replace_all_chars_from_string(erase_slash_at_start(interface_namespace_), '/', '_');
    // concatenate: interface_namespace + _ + namespace_prefix + _ + name_interface_name
    return append_char(ns, '_') + append_char(get_prefix_name(), '_') + get_interface_name();
  }

  void set_value(double value) override
  {
    value_ = value;
    has_new_value_ = true;

    auto msg = std::make_unique<controller_manager_msgs::msg::InterfaceData>();
    msg->data = get_value();
    msg->header.seq = seq_number_;
    ++seq_number_;

    RCLCPP_DEBUG(node_->get_logger(), "DistributedCommandInterface Publishing: '%.7lf'", msg->data);
    command_value_pub_->publish(std::move(msg));
  }

  std::string forward_command_topic_name() const { return forward_command_topic_name_; }

protected:
  void receive_value_cb(const controller_manager_msgs::msg::InterfaceData & msg)
  {
    value_ = msg.data;
    has_new_value_ = true;
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(), "DistributedCommandInterface Receiving:[" << value_ << "].");
  }

  std::string get_value_topic_name_;
  // the current namespace we are in. Needed to create the node in the correct namespace
  std::string namespace_;
  // the namespace the actual CommandInterface we subscribe to is in.
  // We need this to create unique names for the CommandInterface.
  std::string interface_namespace_;
  rclcpp::Subscription<controller_manager_msgs::msg::InterfaceData>::SharedPtr command_value_sub_;
  std::string forward_command_topic_name_;
  rclcpp::Publisher<controller_manager_msgs::msg::InterfaceData>::SharedPtr command_value_pub_;
  double value_;
  uint_fast32_t seq_number_ = 0;
};

class DistributedCommandInterface : public DistributedReadWriteHandle
{
public:
  /// CommandInterface copy constructor is actively deleted.
  /**
   * Command interfaces are having a unique ownership and thus
   * can't be copied in order to avoid simultaneous writes to
   * the same resource.
   */
  DistributedCommandInterface(const DistributedCommandInterface & other) = delete;

  DistributedCommandInterface(DistributedCommandInterface && other) = default;

  using DistributedReadWriteHandle::DistributedReadWriteHandle;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
