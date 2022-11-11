#ifndef DISTRIBUTED_CONTROL__STATE_PUBLISHER
#define DISTRIBUTED_CONTROL__STATE_PUBLISHER

#include <memory>

#include "hardware_interface/loaned_state_interface.hpp"
#include "visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

namespace distributed_control
{

class StatePublisher final : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
    explicit StatePublisher(const std::string & ns = "" , std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr = nullptr);
    
    ~StatePublisher(){}
    
    STATE_PUBLISHER_PUBLIC
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node();
    
private:
    void on_timer();

    const std::string & namespace_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_value_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace distributed_control

#endif // DISTRIBUTED_CONTROL__STATE_PUBLISHER