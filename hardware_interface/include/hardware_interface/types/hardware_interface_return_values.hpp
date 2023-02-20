// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_RETURN_VALUES_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_RETURN_VALUES_HPP_

#include <cstdint>

namespace hardware_interface
{
enum class return_type : std::uint8_t
{
  OK = 0,
  ERROR = 1,
  INVALID = 2
};

// (TODO Manuel) Not sure if we should put here...
/**
 * @brief This class acts as a return value datatype for the Handles.
 * This way instead of returning a plain value (double) additional functionality is provided.
 * The data can be marked as e.g. return_type::OK or return_type::INVALID informing the reader
 * that the provided data is either ok to use or not.
 */
class HandleValue
{
public:
  explicit HandleValue(const double & value, const return_type & type = return_type::OK)
  : value_(value), type_(type)
  {
  }

  /**
   * @brief Returns the stored plain value
   *
   * @return double the value of the handle.
   */
  double value() const { return value_; }

  // (TODO Manuel) Not sure if we can/should use return_typ or just use a bool valid?
  // Maybe a additional enum class would be better?
  /**
   * @brief Indicates if the data is OK or e.g INVALID
   *
   * @return return_type enum class indicating the return_type of the data (e.g. OK or INVALID)
   */
  return_type type() const { return type_; }

  /**
   * @brief Indicates if the provided data of the handle is valid and can be used or not and
   * should not be used.
   *
   * @return true => data is valid and can be used
   * @return false => data is not valid and should not be used.
   */
  bool valid() const
  {
    if (type_ == return_type::OK)
    {
      return true;
    }
    return false;
  }

protected:
  double value_;
  return_type type_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_RETURN_VALUES_HPP_
