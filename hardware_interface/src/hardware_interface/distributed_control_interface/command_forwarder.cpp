#include "hardware_interface/distributed_control_interface/command_forwarder.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <stdexcept>

using namespace std::chrono_literals;

namespace distributed_control
{

CommandForwarder::CommandForwarder(
  std::unique_ptr<hardware_interface::LoanedCommandInterface> loaned_command_interface_ptr,
  const std::string & ns, std::chrono::milliseconds period_in_ms,
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
: loaned_command_interface_ptr_(std::move(loaned_command_interface_ptr)),
  namespace_(ns),
  period_in_ms_(period_in_ms),
  node_(node),
  topic_name_(loaned_command_interface_ptr_->get_underscore_separated_name() + "_command_state")
{
  // if we did not get a node passed, we create one ourselves
  if (!node_.get())
  {
    rclcpp::NodeOptions node_options;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      loaned_command_interface_ptr_->get_underscore_separated_name() + "_command_forwarder",
      namespace_, node_options, false);
  }

  state_value_pub_ = node_->create_publisher<std_msgs::msg::Float64>(topic_name_, 10);
  // TODO(Manuel): We should check if we cannot detect changes to LoanedStateInterface's value and only publish then
  timer_ = node_->create_wall_timer(
    period_in_ms_, std::bind(&CommandForwarder::publish_value_on_timer, this));
  RCLCPP_INFO(node_->get_logger(), "Creating CommandForwarder<%s>.", topic_name_.c_str());
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> CommandForwarder::get_node() const
{
  if (!node_.get())
  {
    std::string msg(
      "CommandForwarder<" + command_interface_name() + ">: Node hasn't been configured yet!");
    throw std::runtime_error(msg);
  }
  return node_;
}

std::string CommandForwarder::get_namespace() const { return namespace_; }

std::string CommandForwarder::topic_name() const { return topic_name_; }

std::string CommandForwarder::topic_name_relative_to_namespace() const
{
  return get_namespace() + "/" + topic_name();
}

std::string CommandForwarder::command_interface_name() const
{
  return loaned_command_interface_ptr_->get_name();
}

std::string CommandForwarder::command_interface_prefix_name() const
{
  return loaned_command_interface_ptr_->get_prefix_name();
}

std::string CommandForwarder::command_interface_interface_name() const
{
  return loaned_command_interface_ptr_->get_interface_name();
}

controller_manager_msgs::msg::PublisherDescription
CommandForwarder::create_publisher_description_msg() const
{
  auto msg = controller_manager_msgs::msg::PublisherDescription();

  msg.ns = get_namespace();
  msg.name.prefix_name = command_interface_prefix_name();
  msg.name.interface_name = command_interface_interface_name();
  msg.publisher_topic = topic_name_relative_to_namespace();

  return msg;
}

void CommandForwarder::subscribe_to_command_publisher(const std::string & topic_name)
{
  subscription_topic_name_ = topic_name;
  command_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
    subscription_topic_name_, 10,
    std::bind(&CommandForwarder::forward_command, this, std::placeholders::_1));
}

void CommandForwarder::publish_value_on_timer()
{
  // Todo(Manuel) create custom msg and return success or failure not just nan.
  auto msg = std::make_unique<std_msgs::msg::Float64>();
  try
  {
    msg->data = loaned_command_interface_ptr_->get_value();
  }
  catch (const std::runtime_error & e)
  {
    msg->data = std::numeric_limits<double>::quiet_NaN();
  }
  RCLCPP_WARN(node_->get_logger(), "Publishing: '%.7lf'", msg->data);
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  state_value_pub_->publish(std::move(msg));
}

void CommandForwarder::forward_command(const std_msgs::msg::Float64 & msg)
{
  loaned_command_interface_ptr_->set_value(msg.data);
}

}  // namespace distributed_control