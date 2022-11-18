#include "hardware_interface/distributed_control_interface/state_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <stdexcept>

using namespace std::chrono_literals;

namespace distributed_control
{

StatePublisher::StatePublisher(
  const std::string & ns,
  std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr)
: namespace_(ns),
  loaned_state_interface_ptr_(std::move(loaned_state_interface_ptr)),
  topic_name_(loaned_state_interface_ptr_->get_underscore_separated_name())
{
  rclcpp::NodeOptions node_options;
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    loaned_state_interface_ptr_->get_underscore_separated_name() + "_node", namespace_,
    node_options, false);
  state_value_pub_ = node_->create_publisher<std_msgs::msg::Float64>(topic_name_, 10);
  timer_ = node_->create_wall_timer(50ms, std::bind(&StatePublisher::publish_value_on_timer, this));
  RCLCPP_INFO(node_->get_logger(), "Creating StatePublisher<%s>.", topic_name_.c_str());
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> StatePublisher::get_node() const
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been configured yet!");
  }
  return node_;
}

std::string StatePublisher::get_topic_name() const { return topic_name_; }

std::string StatePublisher::get_state_interface_name() const
{
  return loaned_state_interface_ptr_->get_name();
}

void StatePublisher::publish_value_on_timer()
{
  // Todo(Manuel) create custom msg and return success or failure not just nan.
  auto msg = std::make_unique<std_msgs::msg::Float64>();
  try
  {
    msg->data = loaned_state_interface_ptr_->get_value();
  }
  catch (const std::runtime_error & e)
  {
    msg->data = std::numeric_limits<double>::quiet_NaN();
  }
  RCLCPP_INFO(node_->get_logger(), "Publishing: '%.7lf'", msg->data);
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  state_value_pub_->publish(std::move(msg));
}

}  // namespace distributed_control