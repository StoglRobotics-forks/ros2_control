#ifndef DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_
#define DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace distributed_control
{

class SubControllerManagerWrapper final
{
public:
  explicit SubControllerManagerWrapper(
    const std::string & ns, const std::string & name,
    const std::vector<std::string> state_interface_names, std::vector<std::string> topic_names)
  : NAMESPACE_(ns),
    NAME_(name),
    state_interface_names_(std::move(state_interface_names)),
    topic_names_(std::move(topic_names))
  {
    if (state_interface_names_.size() != topic_names_.size())
    {
      throw std::logic_error(
        "SubControllerManagerWrapper: state_interfaces_names.size() != topic_names.size(). They "
        "must be equal.");
    }
    for (size_t i = 0; i < state_interface_names_.size(); ++i)
    {
      const auto [it, success] = state_interface_topic_name_map_.insert(
        std::pair<std::string, std::string>{state_interface_names_.at(i), topic_names_.at(i)});
      if (!success)
      {
        throw std::logic_error(
          "SubControllerManagerWrapper: Duplicate of given state_interface_name:'" + it->first +
          "'. They must be unique.");
      }
    }
  }

  ~SubControllerManagerWrapper() {}

  std::string get_namespace() const { return NAMESPACE_; }

  std::string get_name() const { return NAME_; }

  std::string get_full_qualified_name() const { return get_namespace() + "/" + get_name(); }

  std::vector<std::string> get_state_interface_names() const { return state_interface_names_; }

  std::vector<std::string> get_topic_names() const { return topic_names_; }

  std::string get_topic_for_state_interface(const std::string & state_interface_name)
  {
    if (auto element = state_interface_topic_name_map_.find(state_interface_name);
        element != state_interface_topic_name_map_.end())
    {
      return element->second;
    }
    throw std::runtime_error("SubControllerManagerWrapper: No state_interface found.");
  }

private:
  const std::string NAMESPACE_;
  const std::string NAME_;
  std::vector<std::string> state_interface_names_;
  std::vector<std::string> topic_names_;
  std::map<std::string, std::string> state_interface_topic_name_map_;
};

}  // namespace distributed_control

#endif  // DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_