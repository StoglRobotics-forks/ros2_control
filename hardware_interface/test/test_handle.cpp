// Copyright 2020 PAL Robotics S.L.
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

#include <gmock/gmock.h>
#include <stdexcept>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_error_signals.hpp"
#include "hardware_interface/types/hardware_interface_warning_signals.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::InterfaceDescription;
using hardware_interface::InterfaceInfo;
using hardware_interface::StateInterface;

namespace
{
constexpr auto JOINT_NAME = "joint_1";
constexpr auto FOO_INTERFACE = "FooInterface";
}  // namespace

TEST(TestHandle, ci_empty_handle_value)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, true);
}

TEST(TestHandle, si_empty_handle_value)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, true);
}

TEST(TestHandle, ci_empty_handle_prefix_empty_value)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  InterfaceDescription descr("", info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, si_empty_handle_prefix_empty_value)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  InterfaceDescription descr("", info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, ci_empty_handle_inteface_name_empty_value)
{
  InterfaceInfo info;
  info.name = "";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, si_empty_handle_inteface_name_empty_value)
{
  InterfaceInfo info;
  info.name = "";
  info.data_type = "bool";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
}

TEST(TestHandle, ci_empty_handle_prefix_and_inteface_name)
{
  InterfaceInfo info;
  info.name = "";
  info.data_type = "bool";
  InterfaceDescription descr("", info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, si_empty_handle_prefix_and_inteface_name)
{
  InterfaceInfo info;
  info.name = "";
  info.data_type = "bool";
  InterfaceDescription descr("", info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, ci_empty_handle_prefix_and_inteface_name_value)
{
  InterfaceInfo info;
  info.name = "";
  InterfaceDescription descr("", info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, si_empty_handle_prefix_and_inteface_name_value)
{
  InterfaceInfo info;
  info.name = "";
  InterfaceDescription descr("", info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, ci_empty_handle_prefix)
{
  InterfaceInfo info;
  info.name = "";
  info.data_type = "bool";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, si_empty_handle_prefix)
{
  InterfaceInfo info;
  info.name = "";
  info.data_type = "bool";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, ci_empty_handle_interface_name)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "bool";
  InterfaceDescription descr("", info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, si_empty_handle_interface_name)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "bool";
  InterfaceDescription descr("", info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_EQ(interface, false);
}

TEST(TestHandle, ci_empty_bool_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "bool";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<bool>(), false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
}

TEST(TestHandle, ci_true_bool_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "bool";
  info.initial_value = "true";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_NO_THROW(interface.set_value(false));
  EXPECT_EQ(interface.get_value<bool>(), false);
}

TEST(TestHandle, ci_empty_double_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_TRUE(std::isnan(interface.get_value<double>()));
  EXPECT_NO_THROW(interface.set_value(1.5));
  EXPECT_EQ(interface.get_value<double>(), 1.5);
}

TEST(TestHandle, ci_pos_double_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  info.initial_value = "1.5";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<double>(), 1.5);
  EXPECT_NO_THROW(interface.set_value(0.0));
  EXPECT_EQ(interface.get_value<double>(), 0.0);
}

TEST(TestHandle, ci_negative_double_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  info.initial_value = "-1.5";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<double>(), -1.5);
  EXPECT_NO_THROW(interface.set_value(0.0));
  EXPECT_EQ(interface.get_value<double>(), 0.0);
}

TEST(TestHandle, ci_invalid_double_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  info.initial_value = "abc";
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(CommandInterface interface{descr}, std::invalid_argument);
}

TEST(TestHandle, ci_int8_t_vector_default_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<int8_t>";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};

  std::vector<int8_t> zero_vector(hardware_interface::warning_signal_count, 0);

  EXPECT_EQ(interface.get_value<std::vector<int8_t>>(), zero_vector);
}

TEST(TestHandle, ci_int8_t_vector_wrong_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<int8_t>";
  info.size = 5;
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(CommandInterface interface{descr}, std::runtime_error);
}

TEST(TestHandle, ci_int8_t_vector_correct_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<int8_t>";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};

  std::vector<int8_t> zero_vector(hardware_interface::warning_signal_count, 0);

  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<std::vector<int8_t>>(), zero_vector);
  zero_vector[1] = 1;
  zero_vector[2] = 2;
  zero_vector[4] = 4;
  zero_vector[8] = 8;
  EXPECT_NO_THROW(interface.set_value(zero_vector));
  EXPECT_EQ(interface.get_value<std::vector<int8_t>>(), zero_vector);
}

TEST(TestHandle, ci_uint8_t_vector_default_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<uint8_t>";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};

  std::vector<uint8_t> zero_vector(hardware_interface::warning_signal_count, 0);

  EXPECT_EQ(interface.get_value<std::vector<uint8_t>>(), zero_vector);
}

TEST(TestHandle, ci_uint8_t_vector_wrong_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<uint8_t>";
  info.size = 5;
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(CommandInterface interface{descr}, std::runtime_error);
}

TEST(TestHandle, ci_uint8_t_vector_correct_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<uint8_t>";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};

  std::vector<uint8_t> zero_vector(hardware_interface::warning_signal_count, 0);

  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<std::vector<uint8_t>>(), zero_vector);
  zero_vector[1] = 1;
  zero_vector[2] = 2;
  zero_vector[4] = 4;
  zero_vector[8] = 8;
  EXPECT_NO_THROW(interface.set_value(zero_vector));
  EXPECT_EQ(interface.get_value<std::vector<uint8_t>>(), zero_vector);
}

TEST(TestHandle, ci_string_vector_default_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<string>";
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};

  std::vector<std::string> zero_vector(hardware_interface::warning_signal_count, "");

  EXPECT_EQ(interface.get_value<std::vector<std::string>>(), zero_vector);
}

TEST(TestHandle, ci_string_vector_wrong_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<string>";
  info.size = 5;
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(CommandInterface interface{descr}, std::runtime_error);
}

TEST(TestHandle, ci_string_vector_correct_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<string>";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};

  std::vector<std::string> empty_str_vector(hardware_interface::warning_signal_count, "");

  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<std::vector<std::string>>(), empty_str_vector);
  empty_str_vector[1] = "Warning message number 1";
  empty_str_vector[2] = "Warn msg no. 2";
  empty_str_vector[4] = "message no. 3";
  empty_str_vector[8] = "Warning message no. 4";

  EXPECT_NO_THROW(interface.set_value(empty_str_vector));
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<std::vector<std::string>>(), empty_str_vector);
}

TEST(TestHandle, ci_throw_on_get_wrong_type_bool_from_int8_vec)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<int8_t>";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};

  EXPECT_THROW(interface.get_value<bool>(), std::bad_variant_access);
}

TEST(TestHandle, ci_throw_on_not_know_type)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "banana";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(CommandInterface interface{descr};, std::runtime_error);
}

TEST(TestHandle, ci_throw_on_get_double_from_monostate)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  CommandInterface interface{descr};
  EXPECT_EQ(interface, false);
  EXPECT_THROW(interface.get_value<double>(), std::bad_variant_access);
}

TEST(TestHandle, si_empty_bool_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "bool";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<bool>(), false);
  EXPECT_NO_THROW(interface.set_value(true));
  EXPECT_EQ(interface.get_value<bool>(), true);
}

TEST(TestHandle, si_true_bool_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "bool";
  info.initial_value = "true";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<bool>(), true);
  EXPECT_NO_THROW(interface.set_value(false));
  EXPECT_EQ(interface.get_value<bool>(), false);
}

TEST(TestHandle, si_empty_double_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_TRUE(std::isnan(interface.get_value<double>()));
  EXPECT_NO_THROW(interface.set_value(1.5));
  EXPECT_EQ(interface.get_value<double>(), 1.5);
}

TEST(TestHandle, si_pos_double_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  info.initial_value = "1.5";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<double>(), 1.5);
  EXPECT_NO_THROW(interface.set_value(0.0));
  EXPECT_EQ(interface.get_value<double>(), 0.0);
}

TEST(TestHandle, si_negative_double_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  info.initial_value = "-1.5";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<double>(), -1.5);
  EXPECT_NO_THROW(interface.set_value(0.0));
  EXPECT_EQ(interface.get_value<double>(), 0.0);
}

TEST(TestHandle, si_invalid_double_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  info.initial_value = "abc";
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(StateInterface interface{descr}, std::invalid_argument);
}

TEST(TestHandle, si_int8_t_vector_default_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<int8_t>";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};

  std::vector<int8_t> zero_vector(hardware_interface::warning_signal_count, 0);

  EXPECT_EQ(interface.get_value<std::vector<int8_t>>(), zero_vector);
}

TEST(TestHandle, si_int8_t_vector_wrong_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<int8_t>";
  info.size = 5;
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(StateInterface interface{descr}, std::runtime_error);
}

TEST(TestHandle, si_int8_t_vector_correct_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<int8_t>";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};

  std::vector<int8_t> zero_vector(hardware_interface::warning_signal_count, 0);

  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<std::vector<int8_t>>(), zero_vector);
  zero_vector[1] = 1;
  zero_vector[2] = 2;
  zero_vector[4] = 4;
  zero_vector[8] = 8;
  EXPECT_NO_THROW(interface.set_value(zero_vector));
  EXPECT_EQ(interface.get_value<std::vector<int8_t>>(), zero_vector);
}

TEST(TestHandle, si_uint8_t_vector_default_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<uint8_t>";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};

  std::vector<uint8_t> zero_vector(hardware_interface::warning_signal_count, 0);

  EXPECT_EQ(interface.get_value<std::vector<uint8_t>>(), zero_vector);
}

TEST(TestHandle, si_uint8_t_vector_wrong_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<uint8_t>";
  info.size = 5;
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(StateInterface interface{descr}, std::runtime_error);
}

TEST(TestHandle, si_uint8_t_vector_correct_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<uint8_t>";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};

  std::vector<uint8_t> zero_vector(hardware_interface::warning_signal_count, 0);

  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<std::vector<uint8_t>>(), zero_vector);
  zero_vector[1] = 1;
  zero_vector[2] = 2;
  zero_vector[4] = 4;
  zero_vector[8] = 8;
  EXPECT_NO_THROW(interface.set_value(zero_vector));
  EXPECT_EQ(interface.get_value<std::vector<uint8_t>>(), zero_vector);
}

TEST(TestHandle, si_string_vector_default_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<string>";
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};

  std::vector<std::string> zero_vector(hardware_interface::warning_signal_count, "");

  EXPECT_EQ(interface.get_value<std::vector<std::string>>(), zero_vector);
}

TEST(TestHandle, si_string_vector_wrong_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<string>";
  info.size = 5;
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(StateInterface interface{descr}, std::runtime_error);
}

TEST(TestHandle, si_string_vector_correct_size_initialization)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<string>";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};

  std::vector<std::string> empty_str_vector(hardware_interface::warning_signal_count, "");

  EXPECT_EQ(interface, true);
  EXPECT_EQ(interface.get_value<std::vector<std::string>>(), empty_str_vector);
  empty_str_vector[1] = "Warning message number 1";
  empty_str_vector[2] = "Warn msg no. 2";
  empty_str_vector[4] = "message no. 3";
  empty_str_vector[8] = "Warning message no. 4";

  EXPECT_NO_THROW(interface.set_value(empty_str_vector));
  EXPECT_EQ(interface.get_value<std::vector<std::string>>(), empty_str_vector);
}

TEST(TestHandle, si_throw_on_get_wrong_type)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "vector<int8_t>";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};

  EXPECT_THROW(interface.get_value<bool>(), std::bad_variant_access);
}

TEST(TestHandle, si_throw_on_not_know_type)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "banana";
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  EXPECT_THROW(StateInterface interface{descr};, std::runtime_error);
}

TEST(TestHandle, si_throw_on_empty_type)
{
  InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.size = hardware_interface::warning_signal_count;
  InterfaceDescription descr(JOINT_NAME, info);

  StateInterface interface{descr};
  EXPECT_TRUE(std::isnan(interface.get_value<double>()));
}

TEST(TestHandle, name_getters_work)
{
  hardware_interface::InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  hardware_interface::InterfaceDescription descr(JOINT_NAME, info);
  StateInterface handle{descr};

  EXPECT_EQ(handle, false);
  EXPECT_EQ(handle.get_name(), std::string(JOINT_NAME) + "/" + std::string(FOO_INTERFACE));
  EXPECT_EQ(handle.get_interface_name(), FOO_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME);
}

TEST(TestHandle, ci_value_methods)
{
  hardware_interface::InterfaceInfo info;
  info.name = FOO_INTERFACE;
  info.data_type = "double";
  hardware_interface::InterfaceDescription descr(JOINT_NAME, info);
  CommandInterface handle{descr};

  EXPECT_EQ(handle, true);
  EXPECT_NO_THROW(handle.get_value<double>());
  EXPECT_TRUE(std::isnan(handle.get_value<double>()));
  EXPECT_NO_THROW(handle.set_value(0.0));
  EXPECT_EQ(handle.get_value<double>(), 0.0);
}

TEST(TestHandle, si_value_methods)
{
  hardware_interface::InterfaceInfo info;
  info.name = FOO_INTERFACE;
  hardware_interface::InterfaceDescription descr(JOINT_NAME, info);
  StateInterface handle{descr};

  EXPECT_EQ(handle, true);
  EXPECT_NO_THROW(handle.get_value<double>());
  EXPECT_TRUE(std::isnan(handle.get_value<double>()));
  EXPECT_NO_THROW(handle.set_value(0.0));
  EXPECT_EQ(handle.get_value<double>(), 0.0);
}

TEST(TestHandle, interface_description_state_interface_name_getters_work)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  InterfaceInfo info;
  info.name = POSITION_INTERFACE;
  info.data_type = "double";
  InterfaceDescription interface_descr(JOINT_NAME_1, info);
  StateInterface handle{interface_descr};

  EXPECT_EQ(handle, true);
  EXPECT_EQ(handle.get_name(), JOINT_NAME_1 + "/" + POSITION_INTERFACE);
  EXPECT_EQ(handle.get_interface_name(), POSITION_INTERFACE);
  (handle.get_prefix_name(), JOINT_NAME_1);
}

TEST(TestHandle, interface_description_command_interface_name_getters_work)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  InterfaceInfo info;
  info.data_type = "double";
  info.name = POSITION_INTERFACE;
  InterfaceDescription interface_descr(JOINT_NAME_1, info);
  CommandInterface handle{interface_descr};

  EXPECT_EQ(handle, true);
  EXPECT_EQ(handle.get_name(), JOINT_NAME_1 + "/" + POSITION_INTERFACE);
  EXPECT_EQ(handle.get_interface_name(), POSITION_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME_1);
}
