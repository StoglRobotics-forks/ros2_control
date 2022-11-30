#ifndef DISTRIBUTED_CONTROL__STATE_PUBLISHER_DESCRIPTION_HPP_
#define DISTRIBUTED_CONTROL__STATE_PUBLISHER_DESCRIPTION_HPP_

#include "controller_manager_msgs/msg/publisher_description.hpp"

#include "rclcpp/rclcpp.hpp"

namespace distributed_control
{

class PublisherDescription final
{
public:
  explicit PublisherDescription(
    const controller_manager_msgs::msg::PublisherDescription & description)
  : namespace_(description.ns),
    prefix_name_(description.name.prefix_name),
    interface_name_(description.name.interface_name),
    topic_name_(description.publisher_topic)
  {
  }

  PublisherDescription() = delete;

  PublisherDescription(const PublisherDescription & other) = default;

  PublisherDescription(PublisherDescription && other) = default;

  ~PublisherDescription() {}

  std::string get_namespace() const { return namespace_; }

  std::string prefix_name() const { return prefix_name_; }

  std::string interface_name() const { return interface_name_; }

  std::string topic_name() const { return topic_name_; }

private:
  std::string namespace_;
  std::string prefix_name_;
  std::string interface_name_;
  std::string topic_name_;
};

}  // namespace distributed_control
#endif  // DISTRIBUTED_CONTROL__STATE_PUBLISHER_DESCRIPTION_HPP_