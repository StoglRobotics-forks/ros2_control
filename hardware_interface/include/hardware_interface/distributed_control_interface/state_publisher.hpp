#ifndef DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_
#define DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/distributed_control_interface/evaluation_helper.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "controller_manager_msgs/msg/publisher_description.hpp"

#include "controller_manager_msgs/msg/interface_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace distributed_control
{

class StatePublisher final
{
public:
  explicit StatePublisher(
    std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr,
    const std::string & ns, std::chrono::milliseconds period_in_ms,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
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
  const std::chrono::milliseconds period_in_ms_;

  const std::string topic_name_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Publisher<controller_manager_msgs::msg::InterfaceData>::SharedPtr state_value_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t seq_number_ = 0;
};

}  // namespace distributed_control

#endif  // DISTRIBUTED_CONTROL__STATE_PUBLISHER_HPP_