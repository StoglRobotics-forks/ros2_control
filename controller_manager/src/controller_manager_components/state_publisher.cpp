#include "controller_manager/controller_manager_components/state_publisher.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace controller_manager_components
{

StatePublisher::StatePublisher(const rclcpp::NodeOptions & options, std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr)
: Node(loaned_state_interface_ptr->get_underscore_separated_name() + "_node", options), loaned_state_interface_ptr_(std::move(loaned_state_interface_ptr))
{
    assert(loaned_state_interface_ptr_);
    RCLCPP_INFO(this->get_logger(), "Creating StatePublisher<%s>.", loaned_state_interface_ptr->get_underscore_separated_name().c_str());
    state_value_pub_ = create_publisher<std_msgs::msg::Float64>(loaned_state_interface_ptr->get_underscore_separated_name(), 10);
    timer_ = create_wall_timer(1s, std::bind(&StatePublisher::on_timer, this));
}

void StatePublisher::on_timer()
{
  auto msg = std::make_unique<std_msgs::msg::Float64>();
  msg->data = loaned_state_interface_ptr_->get_value();
  RCLCPP_INFO(this->get_logger(), "Publishing: '%.7lf'", msg->data);
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  state_value_pub_->publish(std::move(msg));
}

} // namespace controller_manager_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(controller_manager_components::StatePublisher)