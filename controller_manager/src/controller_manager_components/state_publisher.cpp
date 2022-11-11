#include "controller_manager/controller_manager_components/state_publisher.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;

namespace controller_manager_components
{

StatePublisher::StatePublisher(const std::string & ns, std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr)
: namespace_(ns), loaned_state_interface_ptr_(std::move(loaned_state_interface_ptr))
{
    rclcpp::NodeOptions node_options;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(loaned_state_interface_ptr_->get_underscore_separated_name() + "_node", namespace_, node_options, false);
    RCLCPP_INFO(node_->get_logger(), "Creating StatePublisher<%s>.", loaned_state_interface_ptr_->get_underscore_separated_name().c_str());
    state_value_pub_ = node_->create_publisher<std_msgs::msg::Float64>(loaned_state_interface_ptr_->get_underscore_separated_name(), 10);
    timer_ = node_->create_wall_timer(1s, std::bind(&StatePublisher::on_timer, this));
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> StatePublisher::get_node() 
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return node_;
}

void StatePublisher::on_timer()
{
  // Todo(Manuel) create custom msg and return success or failure not just nan.
  auto msg = std::make_unique<std_msgs::msg::Float64>();
  try
  {
    msg->data = loaned_state_interface_ptr_->get_value();
  } 
  catch ( const std::runtime_error & e)
  {
    msg->data = std::numeric_limits<double>::quiet_NaN();
  }
  RCLCPP_INFO(node_->get_logger(), "Publishing: '%.7lf'", msg->data);
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  state_value_pub_->publish(std::move(msg));
}

} // namespace controller_manager_components