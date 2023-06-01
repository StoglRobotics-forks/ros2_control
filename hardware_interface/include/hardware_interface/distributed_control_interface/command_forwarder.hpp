#ifndef DISTRIBUTED_CONTROL__COMMAND_FORWARDER_HPP_
#define DISTRIBUTED_CONTROL__COMMAND_FORWARDER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/distributed_control_interface/evaluation_helper.hpp"
#include "hardware_interface/distributed_control_interface/publisher_description.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

#include "controller_manager_msgs/msg/publisher_description.hpp"

#include "controller_manager_msgs/msg/evaluation.hpp"
#include "controller_manager_msgs/msg/interface_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace distributed_control
{

class CommandForwarder final
{
public:
  explicit CommandForwarder(
    std::unique_ptr<hardware_interface::LoanedCommandInterface> loaned_command_interface_ptr,
    const std::string & ns, std::chrono::milliseconds period_in_ms,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

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
  void forward_command(const controller_manager_msgs::msg::InterfaceData & msg);

  std::unique_ptr<hardware_interface::LoanedCommandInterface> loaned_command_interface_ptr_;
  const std::string namespace_;
  const std::chrono::milliseconds period_in_ms_;

  const std::string topic_name_;
  const std::string evaluation_topic_name_;
  std::string subscription_topic_name_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Publisher<controller_manager_msgs::msg::InterfaceData>::SharedPtr state_value_pub_;
  rclcpp::Subscription<controller_manager_msgs::msg::InterfaceData>::SharedPtr
    command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> evaluation_node_;
  rclcpp::Publisher<controller_manager_msgs::msg::Evaluation>::SharedPtr evaluation_pub_;
  const std::string evaluation_type_ = "commandInterface";
  std::string evaluation_identifier_;
};

}  // namespace distributed_control

#endif  // DISTRIBUTED_CONTROL__COMMAND_FORWARDER_HPP_