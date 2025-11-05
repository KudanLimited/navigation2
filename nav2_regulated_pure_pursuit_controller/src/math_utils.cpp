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

#include "nav2_regulated_pure_pursuit_controller/math_utils.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @brief Plus operator for geometry_msgs::msg::Point
 * @param lhs The left-hand-side argument
 * @param rhs The right-hand-side argument
 * @return The result (lhs + rhs)
 */
geometry_msgs::msg::Point operator+(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  geometry_msgs::msg::Point result;
  result.x = lhs.x + rhs.x;
  result.y = lhs.y + rhs.y;
  result.z = lhs.z + rhs.z;
  return result;
}

/**
 * @brief Minus operator for geometry_msgs::msg::Point
 * @param lhs The left-hand-side argument
 * @param rhs The right-hand-side argument
 * @return The result (lhs - rhs)
 */
geometry_msgs::msg::Point operator-(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  geometry_msgs::msg::Point result;
  result.x = lhs.x - rhs.x;
  result.y = lhs.y - rhs.y;
  result.z = lhs.z - rhs.z;
  return result;
}

/**
 * @brief Multiply operator between scalar and vector as geometry_msgs::msg::Point
 * @param lhs The scalar
 * @param rhs The vector
 * @return The result lhs * rhs
 */
geometry_msgs::msg::Point operator*(double lhs, const geometry_msgs::msg::Point & rhs)
{
  geometry_msgs::msg::Point result;
  result.x = lhs * rhs.x;
  result.y = lhs * rhs.y;
  result.z = lhs * rhs.z;
  return result;
}

/**
 * @brief Dot product between two vectors as geometry_msgs::msg::Point
 * @param a The first vector
 * @param b The second vector
 * @return The dot product between the two vectors
 */
double dotProduct(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * @brief Returns the squared l2 norm of a vector as a geometry_msgs::msg::Point
 * @param a The vector
 * @return The squared l2 norm
 */
double normSquared(const geometry_msgs::msg::Point & a)
{
  return a.x * a.x + a.y * a.y + a.z * a.z;
}

}  // namespace nav2_regulated_pure_pursuit_controller
