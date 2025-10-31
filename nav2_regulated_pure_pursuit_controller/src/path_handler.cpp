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

#include <Eigen/Geometry>
#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_regulated_pure_pursuit_controller/path_handler.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
  tf2::Duration transform_tolerance,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: transform_tolerance_(transform_tolerance), tf_(tf), costmap_ros_(costmap_ros),
  global_plan_reset_(false)
{
}

double PathHandler::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

nav_msgs::msg::Path PathHandler::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  double max_robot_pose_search_dist,
  bool reject_unit_path)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  if (reject_unit_path && global_plan_.poses.size() == 1) {
    throw nav2_core::InvalidPath("Received plan with length of one");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  if (global_plan_reset_) {
    // If the global plan has been updated, find the closest pose and erase all preceding poses

    auto closest_pose_upper_bound =
      nav2_util::geometry_utils::first_after_integrated_distance(
      global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist);

    auto closest_pose = nav2_util::geometry_utils::min_by(
      global_plan_.poses.begin(), closest_pose_upper_bound,
      [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
        return euclidean_distance(robot_pose, ps);
      });

    if (reject_unit_path && std::next(closest_pose) == global_plan_.poses.end()) {
      // If reject_unit_path == true, guaranteed that there are at least 2 points, so there is a
      // point before the final point in the path, can safely decrement closest_pose
      closest_pose--;
    }

    global_plan_.poses.erase(global_plan_.poses.begin(), closest_pose);
    global_plan_reset_ = false;
  }

  // Prune the first point on the path whenever the corresponding segment is passed
  // If already at the final point (size = 1), then no pruning required
  if (global_plan_.poses.size() >= 2) {
    const auto & a_msg = global_plan_.poses[0].pose.position;
    const auto & b_msg = global_plan_.poses[1].pose.position;
    const auto & x_msg = robot_pose.pose.position;
    const Eigen::Vector3d a(a_msg.x, a_msg.y, a_msg.z);
    const Eigen::Vector3d b(b_msg.x, b_msg.y, b_msg.z);
    const Eigen::Vector3d x(x_msg.x, x_msg.y, x_msg.z);

    Eigen::Vector3d ab = (b - a);
    if (ab.dot(x - a) > ab.dot(ab)) {
      global_plan_.poses.erase(global_plan_.poses.begin());
    }
  }

  // We'll discard points on the plan that are outside the local costmap
  // ensuring at least 2 points
  const double max_costmap_extent = getCostmapMaxExtent();
  auto transformation_end = std::find_if(
    global_plan_.poses.begin(),
    global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
        throw nav2_core::ControllerTFError("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    global_plan_.poses.begin(), transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace nav2_regulated_pure_pursuit_controller
