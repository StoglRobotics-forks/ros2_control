#include "hardware_interface/distributed_control_interface/evaluation_helper.hpp"

std::shared_ptr<evaluation_helper::Evaluation_Helper>
  evaluation_helper::Evaluation_Helper::qos_profile_instance_{nullptr};
std::mutex evaluation_helper::Evaluation_Helper::mutex_;