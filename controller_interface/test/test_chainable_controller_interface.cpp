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

#include "test_chainable_controller_interface.hpp"

#include <gmock/gmock.h>
#include <memory>

using ::testing::IsEmpty;
using ::testing::SizeIs;

const std::vector<std::string> ChainableControllerInterfaceTest::ref_itf_names{
  "test_ref_1", "test_ref_2"};
const std::vector<std::string> ChainableControllerInterfaceTest::state_itf_names{
  "test_state_1", "test_state_2"};

// I think in this case std::string should be ok. Its only used for testing and there should be no
// static initialization order fiasco risk
const std::string ChainableControllerInterfaceTest::full_ref_itf_name_1 =    // NOLINT
  std::string(TEST_CONTROLLER_NAME) + "/" + ref_itf_names[0];                // NOLINT
const std::string ChainableControllerInterfaceTest::full_ref_itf_name_2 =    // NOLINT
  std::string(TEST_CONTROLLER_NAME) + "/" + ref_itf_names[1];                // NOLINT
const std::string ChainableControllerInterfaceTest::full_state_itf_name_1 =  // NOLINT
  std::string(TEST_CONTROLLER_NAME) + "/" + state_itf_names[0];              // NOLINT
const std::string ChainableControllerInterfaceTest::full_state_itf_name_2 =  // NOLINT
  std::string(TEST_CONTROLLER_NAME) + "/" + state_itf_names[1];              // NOLINT
const std::vector<std::string> ChainableControllerInterfaceTest::ref_itf_names_duplicate{
  "test_ref_1", "test_ref_1"};
const std::vector<std::string> ChainableControllerInterfaceTest::state_itf_names_duplicate{
  "test_state_1", "test_state_1"};

TEST_F(ChainableControllerInterfaceTest, default_returns)
{
  TestableChainableControllerInterface controller(ref_itf_names, state_itf_names);
  // initialize, create node
  const auto node_options = controller.define_custom_node_options();
  ASSERT_EQ(
    controller.init(TEST_CONTROLLER_NAME, "", 50.0, "", node_options),
    controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  EXPECT_TRUE(controller.is_chainable());
  EXPECT_FALSE(controller.is_in_chained_mode());
}

TEST_F(ChainableControllerInterfaceTest, export_empty_ref_and_state_interfaces)
{
  TestableChainableControllerInterface controller;
  // initialize, create node
  const auto node_options = controller.define_custom_node_options();
  ASSERT_EQ(
    controller.init(TEST_CONTROLLER_NAME, "", 50.0, "", node_options),
    controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto exported_state_interfaces = controller.export_state_interfaces();
  ASSERT_THAT(exported_state_interfaces, SizeIs(0));
  auto exported_reference_interfaces = controller.export_state_interfaces();
  ASSERT_THAT(exported_reference_interfaces, SizeIs(0));
}

TEST_F(ChainableControllerInterfaceTest, export_state_interfaces)
{
  TestableChainableControllerInterface controller(ref_itf_names, state_itf_names);
  // initialize, create node
  const auto node_options = controller.define_custom_node_options();
  ASSERT_EQ(
    controller.init(TEST_CONTROLLER_NAME, "", 50.0, "", node_options),
    controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto exported_state_interfaces = controller.export_state_interfaces();

  ASSERT_THAT(exported_state_interfaces, SizeIs(2));
  EXPECT_EQ(exported_state_interfaces[0]->get_prefix_name(), TEST_CONTROLLER_NAME);
  EXPECT_EQ(exported_state_interfaces[0]->get_interface_name(), state_itf_names[0]);
  EXPECT_EQ(exported_state_interfaces[1]->get_prefix_name(), TEST_CONTROLLER_NAME);
  EXPECT_EQ(exported_state_interfaces[1]->get_interface_name(), state_itf_names[1]);

  EXPECT_EQ(exported_state_interfaces[0]->get_value<double>(), EXPORTED_STATE_INTERFACE_VALUE);
  EXPECT_EQ(exported_state_interfaces[DEFAULT_INIT_POS]->get_value<double>(), 0.0);
}

TEST_F(ChainableControllerInterfaceTest, export_reference_interfaces)
{
  TestableChainableControllerInterface controller(ref_itf_names, state_itf_names);
  // initialize, create node
  const auto node_options = controller.define_custom_node_options();
  ASSERT_EQ(
    controller.init(TEST_CONTROLLER_NAME, "", 50.0, "", node_options),
    controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto reference_interfaces = controller.export_reference_interfaces();

  ASSERT_THAT(reference_interfaces, SizeIs(2));
  EXPECT_EQ(reference_interfaces[0]->get_prefix_name(), TEST_CONTROLLER_NAME);
  EXPECT_EQ(reference_interfaces[0]->get_interface_name(), ref_itf_names[0]);
  EXPECT_EQ(reference_interfaces[1]->get_prefix_name(), TEST_CONTROLLER_NAME);
  EXPECT_EQ(reference_interfaces[1]->get_interface_name(), ref_itf_names[1]);

  EXPECT_EQ(reference_interfaces[0]->get_value<double>(), INTERFACE_VALUE);
  EXPECT_EQ(reference_interfaces[DEFAULT_INIT_POS]->get_value<double>(), 0.0);
}

TEST_F(ChainableControllerInterfaceTest, interfaces_prefix_is_not_node_name)
{
  TestableChainableControllerInterface controller(ref_itf_names, state_itf_names);
  // initialize, create node
  const auto node_options = controller.define_custom_node_options();
  ASSERT_EQ(
    controller.init(TEST_CONTROLLER_NAME, "", 50.0, "", node_options),
    controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  controller.set_name_prefix_of_reference_interfaces("some_not_correct_interface_prefix");

  // expect empty return because interface prefix is not equal to the node name
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> exported_reference_interfaces;
  EXPECT_THROW(
    { exported_reference_interfaces = controller.export_reference_interfaces(); },
    std::runtime_error);
  ASSERT_THAT(exported_reference_interfaces, IsEmpty());
  // expect empty return because interface prefix is not equal to the node name
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> exported_state_interfaces;
  EXPECT_THROW(
    { exported_state_interfaces = controller.export_state_interfaces(); }, std::runtime_error);
  ASSERT_THAT(exported_state_interfaces, IsEmpty());
}

TEST_F(ChainableControllerInterfaceTest, export_duplicate_iterface_names)
{
  TestableChainableControllerInterface controller(
    ref_itf_names_duplicate, state_itf_names_duplicate);
  // initialize, create node
  const auto node_options = controller.define_custom_node_options();
  ASSERT_EQ(
    controller.init(TEST_CONTROLLER_NAME, "", 50.0, "", node_options),
    controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  // expect empty return because interface prefix is not equal to the node name
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> exported_reference_interfaces;
  EXPECT_THROW(
    { exported_reference_interfaces = controller.export_reference_interfaces(); },
    std::runtime_error);
  ASSERT_THAT(exported_reference_interfaces, IsEmpty());
  // expect empty return because interface prefix is not equal to the node name
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> exported_state_interfaces;
  EXPECT_THROW(
    { exported_state_interfaces = controller.export_state_interfaces(); }, std::runtime_error);
  ASSERT_THAT(exported_state_interfaces, IsEmpty());
}

TEST_F(ChainableControllerInterfaceTest, setting_chained_mode)
{
  TestableChainableControllerInterface controller(ref_itf_names, state_itf_names);
  // initialize, create node
  const auto node_options = controller.define_custom_node_options();
  ASSERT_EQ(
    controller.init(TEST_CONTROLLER_NAME, "", 50.0, "", node_options),
    controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto reference_interfaces = controller.export_reference_interfaces();
  ASSERT_THAT(reference_interfaces, SizeIs(2));
  auto exported_state_interfaces = controller.export_state_interfaces();
  ASSERT_THAT(exported_state_interfaces, SizeIs(2));

  EXPECT_FALSE(controller.is_in_chained_mode());

  // Fail setting chained mode
  EXPECT_EQ(reference_interfaces[0]->get_value<double>(), INTERFACE_VALUE);
  EXPECT_EQ(reference_interfaces[1]->get_value<double>(), 0.0);
  EXPECT_EQ(exported_state_interfaces[0]->get_value<double>(), EXPORTED_STATE_INTERFACE_VALUE);
  EXPECT_EQ(exported_state_interfaces[1]->get_value<double>(), 0.0);

  EXPECT_FALSE(controller.set_chained_mode(true));
  EXPECT_FALSE(controller.is_in_chained_mode());

  EXPECT_FALSE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());

  // Success setting chained mode
  reference_interfaces[0]->set_value(0.0);

  EXPECT_TRUE(controller.set_chained_mode(true));
  EXPECT_TRUE(controller.is_in_chained_mode());
  EXPECT_EQ(
    reference_interfaces[0]->get_value<double>(), EXPORTED_REF_INTERFACE_VALUE_IN_CHAINMODE);
  EXPECT_EQ(reference_interfaces[1]->get_value<double>(), 0.0);
  EXPECT_EQ(
    exported_state_interfaces[0]->get_value<double>(), EXPORTED_STATE_INTERFACE_VALUE_IN_CHAINMODE);
  EXPECT_EQ(exported_state_interfaces[1]->get_value<double>(), 0.0);

  controller.configure();
  EXPECT_TRUE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());

  controller.get_node()->activate();
  // Can not change chained mode until in "ACTIVE" state
  EXPECT_FALSE(controller.set_chained_mode(true));
  EXPECT_FALSE(controller.is_in_chained_mode());

  controller.get_node()->deactivate();
  EXPECT_TRUE(controller.set_chained_mode(true));
  EXPECT_TRUE(controller.is_in_chained_mode());

  // Can change 'chained' mode only in "UNCONFIGURED" state
  controller.get_node()->cleanup();
  EXPECT_TRUE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());
}

TEST_F(ChainableControllerInterfaceTest, test_update_logic)
{
  TestableChainableControllerInterface controller(ref_itf_names, state_itf_names);
  // initialize, create node
  const auto node_options = controller.define_custom_node_options();
  ASSERT_EQ(
    controller.init(TEST_CONTROLLER_NAME, "", 50.0, "", node_options),
    controller_interface::return_type::OK);
  ASSERT_NO_THROW(controller.get_node());

  auto reference_interfaces = controller.export_reference_interfaces();
  ASSERT_THAT(reference_interfaces, SizeIs(2));
  auto exported_state_interfaces = controller.export_state_interfaces();
  ASSERT_THAT(exported_state_interfaces, SizeIs(2));

  EXPECT_FALSE(controller.set_chained_mode(false));
  EXPECT_FALSE(controller.is_in_chained_mode());

  // call update and update it from subscriber because not in chained mode
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller.reference_interfaces_[full_ref_itf_name_1]->get_value<double>(),
    INTERFACE_VALUE_INITIAL_REF - 1);
  ASSERT_EQ(controller.reference_interfaces_[full_ref_itf_name_2]->get_value<double>(), 0.0);
  ASSERT_EQ(
    controller.exported_state_interfaces_[full_state_itf_name_1]->get_value<double>(),
    EXPORTED_STATE_INTERFACE_VALUE + 1);
  ASSERT_EQ(controller.exported_state_interfaces_[full_state_itf_name_2]->get_value<double>(), 0.0);

  // Provoke error in update from subscribers - return ERROR and update_and_write_commands not exec.
  reference_interfaces[0]->set_value(INTERFACE_VALUE_SUBSCRIBER_ERROR);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);
  ASSERT_EQ(
    controller.reference_interfaces_[full_ref_itf_name_1]->get_value<double>(),
    INTERFACE_VALUE_INITIAL_REF - 1);
  ASSERT_EQ(controller.reference_interfaces_[full_ref_itf_name_2]->get_value<double>(), 0.0);
  ASSERT_EQ(
    controller.exported_state_interfaces_[full_state_itf_name_1]->get_value<double>(),
    EXPORTED_STATE_INTERFACE_VALUE + 1);
  ASSERT_EQ(controller.exported_state_interfaces_[full_state_itf_name_2]->get_value<double>(), 0.0);

  // Provoke error from update - return ERROR, but reference interface is updated and not reduced
  reference_interfaces[0]->set_value(INTERFACE_VALUE_UPDATE_ERROR);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);
  ASSERT_EQ(controller.reference_interfaces_[0]->get_value<double>(), INTERFACE_VALUE_UPDATE_ERROR);
  ASSERT_EQ(
    controller.exported_state_interfaces_[0]->get_value<double>(),
    EXPORTED_STATE_INTERFACE_VALUE + 1);

  reference_interfaces[0]->set_value(0.0);

  EXPECT_TRUE(controller.set_chained_mode(true));
  EXPECT_TRUE(controller.is_in_chained_mode());

  // Provoke error in update from subscribers - return OK because update of subscribers is not used
  // reference interface is not updated (updated directly because in chained mode)
  reference_interfaces[0]->set_value(INTERFACE_VALUE_SUBSCRIBER_ERROR);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(controller.reference_interfaces_[full_ref_itf_name_1]->get_value<double>(), -1.0);
  ASSERT_EQ(controller.reference_interfaces_[full_ref_itf_name_2]->get_value<double>(), 0.0);
  ASSERT_EQ(
    controller.exported_state_interfaces_[full_state_itf_name_1]->get_value<double>(),
    EXPORTED_STATE_INTERFACE_VALUE_IN_CHAINMODE + 1);
  ASSERT_EQ(controller.exported_state_interfaces_[full_state_itf_name_2]->get_value<double>(), 0.0);

  // Provoke error from update - return ERROR, but reference interface is updated directly
  controller.reference_interfaces_[full_ref_itf_name_1]->set_value(INTERFACE_VALUE_UPDATE_ERROR);
  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);
  ASSERT_EQ(controller.reference_interfaces_[0]->get_value<double>(), INTERFACE_VALUE_UPDATE_ERROR);
  ASSERT_EQ(controller.reference_interfaces_[full_ref_itf_name_2]->get_value<double>(), 0.0);
  ASSERT_EQ(
    controller.exported_state_interfaces_[full_state_itf_name_1]->get_value<double>(),
    EXPORTED_STATE_INTERFACE_VALUE_IN_CHAINMODE + 1);
  ASSERT_EQ(controller.exported_state_interfaces_[full_state_itf_name_2]->get_value<double>(), 0.0);
}
