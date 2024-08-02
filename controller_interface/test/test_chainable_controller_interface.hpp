// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_
#define TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

constexpr char TEST_CONTROLLER_NAME[] = "testable_chainable_controller";
constexpr double INTERFACE_VALUE = 1989.0;
constexpr double INTERFACE_VALUE_SUBSCRIBER_ERROR = 12345.0;
constexpr double INTERFACE_VALUE_UPDATE_ERROR = 67890.0;
constexpr double INTERFACE_VALUE_INITIAL_REF = 1984.0;
constexpr double EXPORTED_STATE_INTERFACE_VALUE = 21833.0;
constexpr double EXPORTED_STATE_INTERFACE_VALUE_IN_CHAINMODE = 82802.0;
constexpr double EXPORTED_REF_INTERFACE_VALUE_IN_CHAINMODE = 8280.0;
constexpr int DEFAULT_INIT_POS = 1;

using hardware_interface::InterfaceDescription;
using hardware_interface::InterfaceInfo;

class TestableChainableControllerInterface
: public controller_interface::ChainableControllerInterface
{
public:
  FRIEND_TEST(ChainableControllerInterfaceTest, interfaces_storage_not_correct_size);
  FRIEND_TEST(ChainableControllerInterfaceTest, test_update_logic);

  TestableChainableControllerInterface()
  : ref_itf_names_({}), state_itf_names_({}), val_for_default_init_(-1)
  {
  }

  TestableChainableControllerInterface(
    const std::vector<std::string> & ref_itf_names,
    const std::vector<std::string> & state_itf_names)
  : ref_itf_names_(ref_itf_names),
    state_itf_names_(state_itf_names),
    val_for_default_init_(DEFAULT_INIT_POS)
  {
  }

  controller_interface::CallbackReturn on_init() override
  {
    // set default value
    prefix_of_interfaces_ = get_node()->get_name();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  // Implementation of ChainableController virtual methods
  std::vector<hardware_interface::InterfaceDescription> export_state_interface_descriptions()
    override
  {
    using hardware_interface::InterfaceDescription;
    using hardware_interface::InterfaceInfo;
    std::vector<InterfaceDescription> exported_state_interfaces_descr;

    int i = 0;
    for (const auto & state_itf_name : state_itf_names_)
    {
      if (i == val_for_default_init_)
      {
        exported_state_interfaces_descr.emplace_back(
          prefix_of_interfaces_, InterfaceInfo(state_itf_name, "double"));
      }
      else
      {
        exported_state_interfaces_descr.emplace_back(
          prefix_of_interfaces_,
          InterfaceInfo(state_itf_name, "double", std::to_string(EXPORTED_STATE_INTERFACE_VALUE)));
      }
      ++i;
    }

    return exported_state_interfaces_descr;
  }

  // Implementation of ChainableController virtual methods
  std::vector<hardware_interface::InterfaceDescription> export_reference_interface_descriptions()
    override
  {
    using hardware_interface::InterfaceDescription;
    using hardware_interface::InterfaceInfo;
    std::vector<InterfaceDescription> reference_interface_descr;

    int i = 0;
    for (const auto & ref_itf_name : ref_itf_names_)
    {
      if (i == val_for_default_init_)
      {
        reference_interface_descr.emplace_back(
          prefix_of_interfaces_, InterfaceInfo(ref_itf_name, "double"));
      }
      else
      {
        reference_interface_descr.emplace_back(
          prefix_of_interfaces_,
          InterfaceInfo(ref_itf_name, "double", std::to_string(INTERFACE_VALUE)));
      }
      ++i;
    }

    return reference_interface_descr;
  }

  bool on_set_chained_mode(bool /*chained_mode*/) override
  {
    if (reference_interfaces_[exported_reference_interface_names_[0]]->get_value<double>() == 0.0)
    {
      reference_interfaces_[exported_reference_interface_names_[1]]->set_value(
        EXPORTED_REF_INTERFACE_VALUE_IN_CHAINMODE);
      exported_state_interfaces_[exported_state_interface_names_[0]]->set_value(
        EXPORTED_STATE_INTERFACE_VALUE_IN_CHAINMODE);

      return true;
    }
    else
    {
      return false;
    }
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (
      reference_interfaces_[exported_reference_interface_names_[0]]->get_value<double>() ==
      INTERFACE_VALUE_SUBSCRIBER_ERROR)
    {
      return controller_interface::return_type::ERROR;
    }

    reference_interfaces_[exported_reference_interface_names_[0]]->set_value(
      INTERFACE_VALUE_INITIAL_REF);
    exported_state_interfaces_[exported_reference_interface_names_[0]]->set_value(
      INTERFACE_VALUE_INITIAL_REF);
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (
      reference_interfaces_[exported_reference_interface_names_[0]]->get_value<double>() ==
      INTERFACE_VALUE_UPDATE_ERROR)
    {
      return controller_interface::return_type::ERROR;
    }

    reference_interfaces_[exported_reference_interface_names_[0]]->operator-=(1);
    exported_state_interfaces_[exported_state_interface_names_[0]]->operator+=(1);

    return controller_interface::return_type::OK;
  }

  void set_name_prefix_of_reference_interfaces(const std::string & prefix)
  {
    prefix_of_interfaces_ = prefix;
  }

  std::string prefix_of_interfaces_;
  const std::vector<std::string> ref_itf_names_;
  const std::vector<std::string> state_itf_names_;
  const int val_for_default_init_;
};

class ChainableControllerInterfaceTest : public ::testing::Test
{
public:
  static const std::vector<std::string> ref_itf_names;
  static const std::vector<std::string> state_itf_names;
  static const std::vector<std::string> ref_itf_names_duplicate;
  static const std::vector<std::string> state_itf_names_duplicate;
  static const std::string full_ref_itf_name_1;
  static const std::string full_ref_itf_name_2;
  static const std::string full_state_itf_name_1;
  static const std::string full_state_itf_name_2;

  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

#endif  // TEST_CHAINABLE_CONTROLLER_INTERFACE_HPP_
