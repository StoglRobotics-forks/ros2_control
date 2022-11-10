#ifndef CONTROLLER_MAMAGER_COMPONENTS__STATE_PUBLISHER
#define CONTROLLER_MAMAGER_COMPONENTS__STATE_PUBLISHER

#include <memory>

#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace controller_manager_components
{

class StatePublisher final : public rclcpp::Node
{
public:
    explicit StatePublisher(const rclcpp::NodeOptions & options, std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr = nullptr);
    
    ~StatePublisher(){}

private:
    void on_timer();

    std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_interface_ptr_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_value_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace controller_manager_components

#endif // CONTROLLER_MAMAGER_COMPONENTS__STATE_PUBLISHER