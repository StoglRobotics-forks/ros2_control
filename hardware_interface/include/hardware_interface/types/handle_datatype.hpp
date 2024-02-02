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

#ifndef HARDWARE_INTERFACE__TYPES__HANDLE_DATATYPE_HPP_
#define HARDWARE_INTERFACE__TYPES__HANDLE_DATATYPE_HPP_

#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "hardware_interface/types/hardware_interface_error_signals.hpp"
#include "hardware_interface/types/hardware_interface_warning_signals.hpp"

namespace hardware_interface
{

using HANDLE_DATATYPE =
  std::variant<bool, double, std::vector<int8_t>, std::vector<uint8_t>, std::vector<std::string>>;

// Define a type trait for allowed types
template <typename T>
struct HANDLE_DATATYPE_TYPES
: std::disjunction<
    std::is_same<T, bool>, std::is_same<T, double>, std::is_same<T, std::vector<int8_t>>,
    std::is_same<T, std::vector<uint8_t>>, std::is_same<T, std::vector<std::string>>>
{
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HANDLE_DATATYPE_HPP_
