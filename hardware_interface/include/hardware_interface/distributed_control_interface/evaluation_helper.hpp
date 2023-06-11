// Copyright 2020 - 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DISTRIBUTED_CONTROL__QOS_PROFILES_HELPER_HPP_
#define DISTRIBUTED_CONTROL__QOS_PROFILES_HELPER_HPP_

#include <memory>
#include <mutex>

#include "rmw/qos_profiles.h"
#include "rmw/types.h"

namespace evaluation_helper
{

// All profiles used for evaluation, copy them to make sure that exactly those params are used.
static const rmw_qos_profile_t rmw_qos_profile_sensor_data = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

static const rmw_qos_profile_t rmw_qos_profile_sensor_data_1 = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

static const rmw_qos_profile_t rmw_qos_profile_sensor_data_100 = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  100,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

static const rmw_qos_profile_t rmw_qos_profile_reliable = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

static const rmw_qos_profile_t rmw_qos_profile_reliable_100 = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  100,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

static const rmw_qos_profile_t rmw_qos_profile_system_default = {
  RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
  RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT,
  RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
  RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

class Evaluation_Helper
{
protected:
  Evaluation_Helper(
    const rmw_qos_profile_t & qos_profile, const bool & publish_evaluation_msg,
    const rmw_qos_profile_t & evaluation_qos_profile)
  : qos_profile_(qos_profile),
    publish_evaluation_msg_(publish_evaluation_msg),
    evaluation_qos_profile_(evaluation_qos_profile)
  {
  }

public:
  /**
  * Evaluation_Helpers should not be cloneable.
  */
  Evaluation_Helper(Evaluation_Helper & other) = delete;
  /**
  * Evaluation_Helpers should not be assignable.
  */
  void operator=(const Evaluation_Helper &) = delete;

  static std::shared_ptr<Evaluation_Helper> create_instance(
    const rmw_qos_profile_t & qos_profile, const bool & publish_evaluation_msg,
    const rmw_qos_profile_t & evaluation_qos_profile)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (qos_profile_instance_ == nullptr)
    {
      qos_profile_instance_ = std::shared_ptr<Evaluation_Helper>(
        new Evaluation_Helper(qos_profile, publish_evaluation_msg, evaluation_qos_profile));
    }
    return qos_profile_instance_;
  }

  static std::shared_ptr<Evaluation_Helper> get_instance()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (qos_profile_instance_ == nullptr)
    {
      throw std::runtime_error("Evaluation_Helper not initialized!");
    }
    return qos_profile_instance_;
  }

  rmw_qos_profile_t get_qos_profile() const { return qos_profile_; }

  rmw_qos_profile_t get_evaluation_qos_profile() const { return evaluation_qos_profile_; }

  bool publish_evaluation_msg() const { return publish_evaluation_msg_; }

protected:
  static std::shared_ptr<Evaluation_Helper> qos_profile_instance_;
  static std::mutex mutex_;

  const rmw_qos_profile_t qos_profile_;
  const bool publish_evaluation_msg_;
  const rmw_qos_profile_t evaluation_qos_profile_;
};

}  // namespace evaluation_helper

#endif  // DISTRIBUTED_CONTROL__EVALUATION_HELPER_HPP_