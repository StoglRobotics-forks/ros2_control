// Copyright 2021 PAL Robotics SL.
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
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#include "test_imu_sensor.hpp"

#include <memory>
#include <string>
#include <vector>

void IMUSensorTest::TearDown() { imu_sensor_.reset(nullptr); }

TEST_F(IMUSensorTest, validate_all)
{
  // create the IMU sensor
  imu_sensor_ = std::make_unique<TestableIMUSensor>(sensor_name_);

  // validate the component name
  ASSERT_EQ(imu_sensor_->name_, sensor_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(imu_sensor_->interface_names_.size(), size_);
  ASSERT_EQ(imu_sensor_->state_interfaces_.capacity(), size_);

  // validate the default interface_names_
  ASSERT_TRUE(std::equal(
    imu_sensor_->interface_names_.begin(), imu_sensor_->interface_names_.end(),
    full_interface_names_.begin(), full_interface_names_.end()));

  // get the interface names
  std::vector<std::string> interface_names = imu_sensor_->get_state_interface_names();

  // assign values to orientation
  hardware_interface::InterfaceInfo orientation_x_info;
  orientation_x_info.name = imu_interface_names_[0];
  orientation_x_info.initial_value = std::to_string(orientation_values_[0]);
  hardware_interface::InterfaceDescription orientation_x_descr(sensor_name_, orientation_x_info);
  hardware_interface::StateInterface orientation_x{orientation_x_descr};

  hardware_interface::InterfaceInfo orientation_y_info;
  orientation_y_info.name = imu_interface_names_[1];
  orientation_y_info.initial_value = std::to_string(orientation_values_[1]);
  hardware_interface::InterfaceDescription orientation_y_descr(sensor_name_, orientation_y_info);
  hardware_interface::StateInterface orientation_y{orientation_y_descr};

  hardware_interface::InterfaceInfo orientation_z_info;
  orientation_z_info.name = imu_interface_names_[2];
  orientation_z_info.initial_value = std::to_string(orientation_values_[2]);
  hardware_interface::InterfaceDescription orientation_z_descr(sensor_name_, orientation_z_info);
  hardware_interface::StateInterface orientation_z{orientation_z_descr};

  hardware_interface::InterfaceInfo orientation_w_info;
  orientation_w_info.name = imu_interface_names_[3];
  orientation_w_info.initial_value = std::to_string(orientation_values_[3]);
  hardware_interface::InterfaceDescription orientation_w_descr(sensor_name_, orientation_w_info);
  hardware_interface::StateInterface orientation_w{orientation_w_descr};

  // assign values to angular velocity
  hardware_interface::InterfaceInfo angular_x_info;
  angular_x_info.name = imu_interface_names_[4];
  angular_x_info.initial_value = std::to_string(angular_velocity_values_[0]);
  hardware_interface::InterfaceDescription angular_x_descr(sensor_name_, angular_x_info);
  hardware_interface::StateInterface angular_velocity_x{angular_x_descr};

  hardware_interface::InterfaceInfo angular_y_info;
  angular_y_info.name = imu_interface_names_[5];
  angular_y_info.initial_value = std::to_string(angular_velocity_values_[1]);
  hardware_interface::InterfaceDescription angular_y_descr(sensor_name_, angular_y_info);
  hardware_interface::StateInterface angular_velocity_y{angular_y_descr};

  hardware_interface::InterfaceInfo angular_z_info;
  angular_z_info.name = imu_interface_names_[6];
  angular_z_info.initial_value = std::to_string(angular_velocity_values_[2]);
  hardware_interface::InterfaceDescription angular_z_descr(sensor_name_, angular_z_info);
  hardware_interface::StateInterface angular_velocity_z{angular_z_descr};

  // assign values to linear acceleration
  hardware_interface::InterfaceInfo linear_x_info;
  linear_x_info.name = imu_interface_names_[7];
  linear_x_info.initial_value = std::to_string(linear_acceleration_values_[0]);
  hardware_interface::InterfaceDescription linear_x_descr(sensor_name_, linear_x_info);
  hardware_interface::StateInterface linear_acceleration_x{linear_x_descr};

  hardware_interface::InterfaceInfo linear_y_info;
  linear_y_info.name = imu_interface_names_[8];
  linear_y_info.initial_value = std::to_string(linear_acceleration_values_[1]);
  hardware_interface::InterfaceDescription linear_y_descr(sensor_name_, linear_y_info);
  hardware_interface::StateInterface linear_acceleration_y{linear_y_descr};

  hardware_interface::InterfaceInfo linear_z_info;
  linear_z_info.name = imu_interface_names_[9];
  linear_z_info.initial_value = std::to_string(linear_acceleration_values_[2]);
  hardware_interface::InterfaceDescription linear_z_descr(sensor_name_, linear_z_info);
  hardware_interface::StateInterface linear_acceleration_z{linear_z_descr};

  // create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(10);

  // insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(angular_velocity_y);
  temp_state_interfaces.emplace_back(orientation_y);
  temp_state_interfaces.emplace_back(linear_acceleration_y);
  temp_state_interfaces.emplace_back(orientation_x);
  temp_state_interfaces.emplace_back(linear_acceleration_z);
  temp_state_interfaces.emplace_back(angular_velocity_z);
  temp_state_interfaces.emplace_back(orientation_z);
  temp_state_interfaces.emplace_back(orientation_w);
  temp_state_interfaces.emplace_back(angular_velocity_x);
  temp_state_interfaces.emplace_back(linear_acceleration_x);

  // now call the function to make them in order like interface_names
  imu_sensor_->assign_loaned_state_interfaces(temp_state_interfaces);

  // validate the count of state_interfaces_
  ASSERT_EQ(imu_sensor_->state_interfaces_.size(), size_);

  // validate the orientation values
  std::array<double, 4> temp_orientation_values = imu_sensor_->get_orientation();
  ASSERT_EQ(temp_orientation_values, orientation_values_);

  // validate the angular_velocity values
  std::array<double, 3> temp_angular_velocity_values = imu_sensor_->get_angular_velocity();
  ASSERT_EQ(temp_angular_velocity_values, angular_velocity_values_);

  // validate the linear_acceleration values
  std::array<double, 3> temp_linear_acceleration_values = imu_sensor_->get_linear_acceleration();
  ASSERT_EQ(temp_linear_acceleration_values, linear_acceleration_values_);

  // validate get_values_as_message
  sensor_msgs::msg::Imu temp_message;
  ASSERT_TRUE(imu_sensor_->get_values_as_message(temp_message));
  ASSERT_EQ(temp_message.orientation.x, orientation_values_[0]);
  ASSERT_EQ(temp_message.orientation.y, orientation_values_[1]);
  ASSERT_EQ(temp_message.orientation.z, orientation_values_[2]);
  ASSERT_EQ(temp_message.orientation.w, orientation_values_[3]);
  ASSERT_EQ(temp_message.angular_velocity.x, angular_velocity_values_[0]);
  ASSERT_EQ(temp_message.angular_velocity.y, angular_velocity_values_[1]);
  ASSERT_EQ(temp_message.angular_velocity.z, angular_velocity_values_[2]);
  ASSERT_EQ(temp_message.linear_acceleration.x, linear_acceleration_values_[0]);
  ASSERT_EQ(temp_message.linear_acceleration.y, linear_acceleration_values_[1]);
  ASSERT_EQ(temp_message.linear_acceleration.z, linear_acceleration_values_[2]);

  // release the state_interfaces_
  imu_sensor_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(imu_sensor_->state_interfaces_.size(), 0u);
}
