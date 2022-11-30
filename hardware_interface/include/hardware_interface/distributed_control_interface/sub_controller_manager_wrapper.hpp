#ifndef DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_
#define DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "controller_manager_msgs/msg/publisher_description.hpp"

#include "hardware_interface/distributed_control_interface/publisher_description.hpp"

namespace distributed_control
{

class SubControllerManagerWrapper final
{
public:
  explicit SubControllerManagerWrapper(
    const std::string & ns, const std::string & name,
    const std::vector<controller_manager_msgs::msg::PublisherDescription> & state_publishers)
  : NAMESPACE_(ns),
    NAME_(name),
    state_publisher_descriptions_({state_publishers.begin(), state_publishers.end()})
  {
  }

  SubControllerManagerWrapper(const SubControllerManagerWrapper & other) = default;

  SubControllerManagerWrapper(SubControllerManagerWrapper && other) = default;

  ~SubControllerManagerWrapper() {}

  std::string get_namespace() const { return NAMESPACE_; }

  std::string get_controller_manager_name() const { return NAME_; }

  std::string get_name() const { return get_namespace() + "/" + get_controller_manager_name(); }

  std::vector<PublisherDescription> get_state_publisher_descriptions() const
  {
    return state_publisher_descriptions_;
  }

  size_t get_state_publisher_count() const { return state_publisher_descriptions_.size(); }

private:
  const std::string NAMESPACE_;
  const std::string NAME_;
  std::vector<PublisherDescription> state_publisher_descriptions_;
};

}  // namespace distributed_control

#endif  // DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_