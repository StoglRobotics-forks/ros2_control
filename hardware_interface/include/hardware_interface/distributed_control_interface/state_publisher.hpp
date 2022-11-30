#ifndef DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_
#define DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "visibility_control.h"

#include "hardware_interface/loaned_state_interface.hpp"

#include "controller_manager_msgs/msg/state_publisher_description.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace distributed_control
{

class StatePublisher final 
{
public:
  explicit StatePublisher(
    const std::string & ns = "",
    std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr = nullptr);

  ~StatePublisher() {}

  STATE_PUBLISHER_PUBLIC
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

  STATE_PUBLISHER_PUBLIC
  std::string get_namespace() const;

  STATE_PUBLISHER_PUBLIC
  std::string topic_name() const;

  STATE_PUBLISHER_PUBLIC
  std::string topic_name_relative_to_namespace() const;

  STATE_PUBLISHER_PUBLIC
  std::string state_interface_name() const;

  STATE_PUBLISHER_PUBLIC
  std::string state_interface_prefix_name() const;

  STATE_PUBLISHER_PUBLIC
  std::string state_interface_interface_name() const;

  STATE_PUBLISHER_PUBLIC
  controller_manager_msgs::msg::StatePublisherDescription create_description_msg() const;

private:
  void publish_value_on_timer();

  const std::string namespace_;
  std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr_;
  const std::string topic_name_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_value_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace distributed_control

#endif  // DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_