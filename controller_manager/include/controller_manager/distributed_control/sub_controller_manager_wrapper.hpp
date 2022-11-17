#ifndef DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_
#define DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_

#include <map>
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
  : NAMESPACE_(ns), NAME_(name)
  {
    if (state_interface_names.size() != topic_names.size())
    {
      throw std::logic_error(
        "SubControllerManagerWrapper: state_interfaces_names.size() != topic_names.size(). They "
        "must be equal.");
    }
    for (size_t i = 0; i < state_interface_names.size(); ++i)
    {
      state_interface_topic_name_map_.insert(
        std::pair<std::string, std::string>{state_interface_names.at(i), topic_names.at(i)});
    }
  }

  ~SubControllerManagerWrapper() {}

  std::string get_namespace() const { return NAMESPACE_; }

  std::string get_name() const { return NAME_; }

  std::string get_full_qualified_name() const { return "/" + get_namespace() + "/" + get_name(); }

  std::vector<std::string> get_topic_names() const {}

private:
  const std::string NAMESPACE_;
  const std::string NAME_;
  std::map<std::string, std::string> state_interface_topic_name_map_;
};

}  // namespace distributed_control

#endif  // DISTRIBUTED_CONTROL__SUB_CONTROLLER_MANAGER_WRAPPER_HPP_