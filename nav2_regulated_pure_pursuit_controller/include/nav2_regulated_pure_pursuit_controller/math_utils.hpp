// Copyright (c) 2022 Samsung Research America
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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__MATH_UTILS_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__MATH_UTILS_HPP_

#include <geometry_msgs/msg/point.hpp>

namespace nav2_regulated_pure_pursuit_controller {

/**
 * @brief Plus operator for geometry_msgs::msg::Point
 * @param lhs The left-hand-side argument
 * @param rhs The right-hand-side argument
 * @return The result (lhs + rhs)
 */
geometry_msgs::msg::Point operator+(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs);

/**
 * @brief Minus operator for geometry_msgs::msg::Point
 * @param lhs The left-hand-side argument
 * @param rhs The right-hand-side argument
 * @return The result (lhs - rhs)
 */
geometry_msgs::msg::Point operator-(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs);

/**
 * @brief Multiply operator between scalar and vector as geometry_msgs::msg::Point
 * @param lhs The scalar
 * @param rhs The vector
 * @return The result lhs * rhs
 */
geometry_msgs::msg::Point operator*(double lhs, const geometry_msgs::msg::Point & rhs);

/**
 * @brief Dot product between two vectors as geometry_msgs::msg::Point
 * @param a The first vector
 * @param b The second vector
 * @return The dot product between the two vectors
 */
double dotProduct(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

/**
 * @brief Returns the squared l2 norm of a vector as a geometry_msgs::msg::Point
 * @param a The vector
 * @return The squared l2 norm
 */
double normSquared(const geometry_msgs::msg::Point & a);

} // namespace nav2_regulated_pure_pursuit_controller

#endif
