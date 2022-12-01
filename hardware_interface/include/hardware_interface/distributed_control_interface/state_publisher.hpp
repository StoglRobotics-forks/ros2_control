#ifndef DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_
#define DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"

#include "controller_manager_msgs/msg/publisher_description.hpp"

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
    std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr,
    const std::string & ns = "");

  StatePublisher() = delete;

  ~StatePublisher() {}

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

  std::string get_namespace() const;

  std::string topic_name() const;

  std::string topic_name_relative_to_namespace() const;

  std::string state_interface_name() const;

  std::string state_interface_prefix_name() const;

  std::string state_interface_interface_name() const;

  controller_manager_msgs::msg::PublisherDescription create_publisher_description_msg() const;

private:
  void publish_value_on_timer();

  std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr_;
  const std::string namespace_;
  const std::string topic_name_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_value_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace distributed_control

#endif  // DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_