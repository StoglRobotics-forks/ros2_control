#ifndef DISTRIBUTED_CONTROL__COMMAND_FORWARDER_HPP_
#define DISTRIBUTED_CONTROL__COMMAND_FORWARDER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/distributed_control_interface/publisher_description.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

#include "controller_manager_msgs/msg/publisher_description.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace distributed_control
{

class CommandForwarder final
{
public:
  explicit CommandForwarder(
    std::unique_ptr<hardware_interface::LoanedCommandInterface> loaned_command_interface_ptr_,
    const std::string & ns = "");

  CommandForwarder() = delete;

  ~CommandForwarder() {}

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

  std::string get_namespace() const;

  std::string topic_name() const;

  std::string topic_name_relative_to_namespace() const;

  std::string command_interface_name() const;

  std::string command_interface_prefix_name() const;

  std::string command_interface_interface_name() const;

  controller_manager_msgs::msg::PublisherDescription create_publisher_description_msg() const;

  void subscribe_to_command_publisher(const std::string & topic_name);

private:
  void publish_value_on_timer();
  void forward_command(const std_msgs::msg::Float64 & msg);

  std::unique_ptr<hardware_interface::LoanedCommandInterface> loaned_command_interface_ptr_;
  const std::string namespace_;
  const std::string topic_name_;
  std::string subscription_topic_name_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_value_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace distributed_control

#endif  // DISTRIBUTED_CONTROL__COMMAND_FORWARDER_HPP_