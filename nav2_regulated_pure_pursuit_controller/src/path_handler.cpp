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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <Eigen/Geometry>

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
: transform_tolerance_(transform_tolerance), tf_(tf), costmap_ros_(costmap_ros), global_plan_reset_(false), prev_track_segments_(false)
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
  bool track_segments,
  double /* segment_switch_proportion */,
  bool reject_unit_path)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  if (reject_unit_path && global_plan_.poses.size() == 1) {
    throw nav2_core::InvalidPath("Received plan with length of one");
  }

#if 0
  // Since the track_segments parameter may change, cache the previous value
  // and reset the segment_index_ if it wasn't previously being used
  if (!prev_track_segments_ && track_segments) {
    segment_index_ = std::nullopt;
  }
  prev_track_segments_ = track_segments;
#endif

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist);

  std::vector<geometry_msgs::msg::PoseStamped>::iterator transformation_begin;

  if (global_plan_reset_ || !track_segments) {
    std::cout << "Plan reset" << std::endl;
    transformation_begin =
      nav2_util::geometry_utils::min_by(
        global_plan_.poses.begin(), closest_pose_upper_bound,
        [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
          return euclidean_distance(robot_pose, ps);
        });
  } else {
    bool increment_pose = false;

    bool at_cusp = false;
    if (global_plan_.poses.size() >= 3) {
      const auto & a_p_msg = global_plan_.poses[0].pose.position;
      const auto & b_p_msg = global_plan_.poses[1].pose.position;
      const auto & c_p_msg = global_plan_.poses[2].pose.position;
      const auto & b_q_msg = global_plan_.poses[1].pose.orientation;

      const Eigen::Vector3d a_p(a_p_msg.x, a_p_msg.y, a_p_msg.z);
      const Eigen::Vector3d b_p(b_p_msg.x, b_p_msg.y, b_p_msg.z);
      const Eigen::Vector3d c_p(c_p_msg.x, c_p_msg.y, c_p_msg.z);
      const Eigen::Quaterniond b_q(b_q_msg.w, b_q_msg.x, b_p_msg.y, b_p_msg.z);

      const auto & x_p_msg = robot_pose.pose.position;
      const Eigen::Vector3d x_p(x_p_msg.x, x_p_msg.y, x_p_msg.z);

      Eigen::Vector3d ab_b = b_q.inverse() * (b_p - a_p);
      Eigen::Vector3d bc_b = b_q.inverse() * (c_p - b_p);
      Eigen::Vector3d xb_b = b_q.inverse() * (b_p - x_p);
      at_cusp = ab_b.dot(bc_b) < 0;

      if (at_cusp) {
        std::cout << "AT CUSP" << std::endl;
      }
      std::cout << "bx_a: " << xb_b.transpose() << std::endl;
      std::cout << "ab_a: " << ab_b.transpose() << std::endl;

      if (at_cusp && xb_b.dot(ab_b) < 0) {
        increment_pose = true;
        std::cout << "INCREMENT AT CUSP" << std::endl;
      }
      // if at_custp && bx_a.dot(ab_a) < 0, then remain at cusp (increment_pose = false)
    }

    if (!at_cusp && euclidean_distance(robot_pose, global_plan_.poses[1]) < euclidean_distance(robot_pose, global_plan_.poses[0])) {
      increment_pose = true;
      std::cout << "INCREMENT NORMAL" << std::endl;
    }

    if (increment_pose) {
      transformation_begin = global_plan_.poses.begin() + 1;
    } else {
      transformation_begin = global_plan_.poses.begin();
    }
  }
  global_plan_reset_ = false;

#if 0
  if (track_segments) {
    if (!segment_index_) {
      std::size_t closest_pose_index = transformation_begin - global_plan_.poses.begin();
      segment_index_ = closest_pose_index;
      std::cout << "Segment index: " << *segment_index_ << std::endl;
    }

    if (*segment_index_ + 1 < global_plan_.poses.size()) {
      const auto &a_point = global_plan_.poses[*segment_index_].pose.position;
      Eigen::Vector3d a(a_point.x, a_point.y, a_point.z);

      const auto &b_point = global_plan_.poses[*segment_index_+1].pose.position;
      Eigen::Vector3d b(b_point.x, b_point.y, b_point.z);

      const auto& x_point = robot_pose.pose.position;
      Eigen::Vector3d x(x_point.x, x_point.y, x_point.z);

      double proportion = (b - a).dot(x - a) / (b - a).squaredNorm();;
      if (proportion >= segment_switch_proportion) {
        (*segment_index_)++;
        std::cout << "Increment segment index: " << *segment_index_ << std::endl;
      }
    }

    if (*segment_index_ + 1 < global_plan_.poses.size()) {
      transformation_begin = global_plan_.poses.begin() + (*segment_index_ + 1);
    } else {
      transformation_begin = global_plan_.poses.begin() + *segment_index_;
    }
    std::cout << "Transformation begin: " << (transformation_begin - global_plan_.poses.begin()) << " / " << global_plan_.poses.size() << std::endl;
  }
#endif

#if 0
  // Make sure we always have at least 2 points on the transformed plan and that we don't prune
  // the global plan below 2 points in order to have always enough point to interpolate the
  // end of path direction
  if (global_plan_.poses.begin() != closest_pose_upper_bound && global_plan_.poses.size() > 1 &&
    transformation_begin == std::prev(closest_pose_upper_bound))
  {
    transformation_begin = std::prev(std::prev(closest_pose_upper_bound));
  }

  // We'll discard points on the plan that are outside the local costmap
  const double max_costmap_extent = getCostmapMaxExtent();
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
    });
#else
  auto transformation_end = global_plan_.poses.end();
#endif

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
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

#if 1
  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  if (transformation_begin != global_plan_.poses.begin()) {
#if 0
    if (segment_index_) {
      std::size_t to_remove = (transformation_begin - global_plan_.poses.begin());
      std::cout << "To remove: " << to_remove << ", from " << *segment_index_ << std::endl;
      *segment_index_ -= to_remove;
    }
#endif
    std::cout << "ERASING " << (transformation_begin - global_plan_.poses.begin()) << " POINTS" << std::endl;
    global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);
  }
#endif

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
