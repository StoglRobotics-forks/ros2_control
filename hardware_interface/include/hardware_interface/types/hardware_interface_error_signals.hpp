// Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_ERROR_SIGNALS_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_ERROR_SIGNALS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace hardware_interface
{

// Count of how many different error signals there are that can be reported.
const size_t error_signal_count = 32;
using ERROR_SIGNALS = std::vector<uint8_t>;

constexpr char ERROR_SIGNAL_INTERFACE_NAME[] = "ERROR_SIGNAL";

using ERROR_MESSAGES = std::vector<std::string>;
constexpr char ERROR_SIGNAL_MESSAGE_INTERFACE_NAME[] = "ERROR_SIGNAL_MESSAGE";

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_ERROR_SIGNALS_HPP_
