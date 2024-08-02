// Copyright 2023 PAL Robotics SL.
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

/*
 * Author: Sai Kishor Kothakota
 */

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"

using hardware_interface::return_type;
using hardware_interface::SensorInterface;
using hardware_interface::StateInterface;

namespace test_hardware_components
{
class TestIMUSensor : public SensorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & sensor_info) override
  {
    if (SensorInterface::on_init(sensor_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    const auto & state_interfaces = info_.sensors[0].state_interfaces;
    if (state_interfaces.size() != 10)
    {
      return CallbackReturn::ERROR;
    }
    std::vector<std::string> imu_keys;
    imu_keys.insert(imu_keys.end(), quat_.begin(), quat_.end());
    imu_keys.insert(imu_keys.end(), ang_vel_.begin(), ang_vel_.end());
    imu_keys.insert(imu_keys.end(), lin_acc_.begin(), lin_acc_.end());
    for (const auto & imu_key : imu_keys)
    {
      if (
        std::find_if(
          state_interfaces.begin(), state_interfaces.end(), [&imu_key](const auto & interface_info)
          { return interface_info.name == imu_key; }) == state_interfaces.end())
      {
        return CallbackReturn::ERROR;
      }
    }

    fprintf(stderr, "TestIMUSensor configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::InterfaceDescription> export_state_interface_descriptions()
    override
  {
    using hardware_interface::InterfaceInfo;
    std::vector<hardware_interface::InterfaceDescription> state_interfaces;

    const std::string & sensor_name = info_.sensors[0].name;

    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(quat_.x, quat_.data_type, quat_.initial_x));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(quat_.y, quat_.data_type, quat_.initial_y));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(quat_.z, quat_.data_type, quat_.initial_z));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(quat_.w, quat_.data_type, quat_.initial_w));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(ang_vel_.x, ang_vel_.data_type, ang_vel_.initial_x));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(ang_vel_.y, ang_vel_.data_type, ang_vel_.initial_y));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(ang_vel_.z, ang_vel_.data_type, ang_vel_.initial_z));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(lin_acc_.x, lin_acc_.data_type, lin_acc_.initial_x));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(lin_acc_.y, lin_acc_.data_type, lin_acc_.initial_y));
    state_interfaces.emplace_back(
      sensor_name, InterfaceInfo(lin_acc_.z, lin_acc_.data_type, lin_acc_.initial_z));

    return state_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // generate a random distribution of the quaternion
    std::uniform_real_distribution<double> distribution_1(0.0, 1.0);
    const double u1 = distribution_1(generator_);
    const double u2 = distribution_1(generator_);
    const double u3 = distribution_1(generator_);
    set_state(quat_.w, std::sqrt(1. - u1) * std::sin(2 * M_PI * u2));
    set_state(quat_.x, std::sqrt(1. - u1) * std::cos(2 * M_PI * u2));
    set_state(quat_.y, std::sqrt(u1) * std::sin(2 * M_PI * u3));
    set_state(quat_.z, std::sqrt(u1) * std::cos(2 * M_PI * u3));

    // generate random angular velocities and linear accelerations
    std::uniform_real_distribution<double> distribution_2(0.0, 0.1);
    set_state(ang_vel_.x, distribution_2(generator_));
    set_state(ang_vel_.y, distribution_2(generator_));
    set_state(ang_vel_.z, distribution_2(generator_));

    set_state(lin_acc_.x, distribution_2(generator_));
    set_state(lin_acc_.y, distribution_2(generator_));
    set_state(lin_acc_.z, distribution_2(generator_));
    return return_type::OK;
  }

private:
  struct Quaternion
  {
    const std::string x = "orientation.x";
    const std::string y = "orientation.y";
    const std::string z = "orientation.z";
    const std::string w = "orientation.w";

    const std::string data_type = "double";

    const std::string initial_x = "0.0";
    const std::string initial_y = "0.0";
    const std::string initial_z = "0.0";
    const std::string initial_w = "1.0";

    std::vector<std::string>::const_iterator begin() const { return component_names_.begin(); }
    std::vector<std::string>::const_iterator end() const { return component_names_.end(); }

  private:
    const std::vector<std::string> component_names_{x, y, z, w};
  };

  struct AngularVel
  {
    const std::string x = "angular_velocity.x";
    const std::string y = "angular_velocity.y";
    const std::string z = "angular_velocity.z";

    const std::string data_type = "double";

    const std::string initial_x = "0.0";
    const std::string initial_y = "0.0";
    const std::string initial_z = "0.0";

    std::vector<std::string>::const_iterator begin() const { return component_names_.begin(); }
    std::vector<std::string>::const_iterator end() const { return component_names_.end(); }

  private:
    const std::vector<std::string> component_names_{x, y, z};
  };

  struct LinearAccel
  {
    const std::string x = "linear_acceleration.x";
    const std::string y = "linear_acceleration.y";
    const std::string z = "linear_acceleration.z";

    const std::string data_type = "double";

    const std::string initial_x = "0.0";
    const std::string initial_y = "0.0";
    const std::string initial_z = "0.0";

    std::vector<std::string>::const_iterator begin() const { return component_names_.begin(); }
    std::vector<std::string>::const_iterator end() const { return component_names_.end(); }

  private:
    const std::vector<std::string> component_names_{x, y, z};
  };

  std::default_random_engine generator_;
  Quaternion quat_;
  AngularVel ang_vel_;
  LinearAccel lin_acc_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(test_hardware_components::TestIMUSensor, hardware_interface::SensorInterface)
