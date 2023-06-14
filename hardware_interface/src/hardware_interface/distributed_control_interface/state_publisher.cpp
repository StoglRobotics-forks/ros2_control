#include "hardware_interface/distributed_control_interface/state_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace distributed_control
{

StatePublisher::StatePublisher(
  std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr,
  const std::string & ns, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
: loaned_state_interface_ptr_(std::move(loaned_state_interface_ptr)),
  namespace_(ns),
  node_(node),
  node_name_(loaned_state_interface_ptr_->get_underscore_separated_name() + "_state_publisher"),
  topic_name_(loaned_state_interface_ptr_->get_underscore_separated_name() + "_state")
{
  // if we did not get a node passed, we create one ourselves
  if (!node_.get())
  {
    rclcpp::NodeOptions node_options;
    node_options.clock_type(rcl_clock_type_t::RCL_STEADY_TIME);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      node_name_, namespace_, node_options, false);
  }

  auto evaluation_helper = evaluation_helper::Evaluation_Helper::get_instance();
  rclcpp::QoS qos_profile(
    rclcpp::QoSInitialization::from_rmw(evaluation_helper->get_qos_profile()));
  std::shared_ptr<hardware_interface::SetValueBehavior> distributed_behavior =
    std::make_shared<hardware_interface::PublishBehavior>(
      namespace_, node_, topic_name_, qos_profile);

  loaned_state_interface_ptr->set_behavior(distributed_behavior);
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> StatePublisher::get_node() const
{
  if (!node_.get())
  {
    std::string msg(
      "StatePublisher<" + state_interface_name() + ">: Node hasn't been configured yet!");
    throw std::runtime_error(msg);
  }
  return node_;
}

std::string StatePublisher::get_namespace() const { return namespace_; }

std::string StatePublisher::topic_name() const { return topic_name_; }

std::string StatePublisher::topic_name_relative_to_namespace() const
{
  return get_namespace() + "/" + topic_name();
}

std::string StatePublisher::state_interface_name() const
{
  return loaned_state_interface_ptr_->get_name();
}

std::string StatePublisher::state_interface_prefix_name() const
{
  return loaned_state_interface_ptr_->get_prefix_name();
}

std::string StatePublisher::state_interface_interface_name() const
{
  return loaned_state_interface_ptr_->get_interface_name();
}

controller_manager_msgs::msg::PublisherDescription
StatePublisher::create_publisher_description_msg() const
{
  auto msg = controller_manager_msgs::msg::PublisherDescription();
  msg.ns = get_namespace();
  msg.name.prefix_name = state_interface_prefix_name();
  msg.name.interface_name = state_interface_interface_name();
  msg.publisher_topic = topic_name_relative_to_namespace();

  return msg;
}

}  // namespace distributed_control