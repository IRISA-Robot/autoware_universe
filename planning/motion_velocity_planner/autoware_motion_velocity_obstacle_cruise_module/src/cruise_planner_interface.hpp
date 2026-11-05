// Copyright 2025 TIER IV, Inc.
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

#ifndef CRUISE_PLANNER_INTERFACE_HPP_
#define CRUISE_PLANNER_INTERFACE_HPP_

#include "autoware/motion_velocity_planner_common/planner_data.hpp"
#include "parameters.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{
class CruisePlannerInterface
{
public:
  CruisePlannerInterface(
    rclcpp::Node & node, const CommonParam & common_param,
    const CruisePlanningParam & cruise_planning_param)
  {
    clock_ = node.get_clock();
    logger_ = node.get_logger();
    common_param_ = common_param;
    cruise_planning_param_ = cruise_planning_param;
  }

  virtual ~CruisePlannerInterface() = default;

  virtual std::vector<TrajectoryPoint> plan_cruise(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<CruiseObstacle> & obstacles, std::shared_ptr<DebugData> debug_data_ptr,
    std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface> &
      planning_factor_interface,
    std::optional<VelocityLimit> & velocity_limit) = 0;

  virtual void update_parameters(const std::vector<rclcpp::Parameter> & parameters) = 0;

  virtual Float32MultiArrayStamped get_cruise_planning_debug_message(
    [[maybe_unused]] const rclcpp::Time & current_time) const
  {
    return Float32MultiArrayStamped{};
  }

protected:
  rclcpp::Clock::SharedPtr clock_{};
  rclcpp::Logger logger_{rclcpp::get_logger("")};
  CommonParam common_param_;
  CruisePlanningParam cruise_planning_param_;

  // [REVERSE-CRUISE 2026-11-05] reverse_lane_follow paths carry yaw = direction of travel and a
  // negative velocity, so planner_data->is_driving_forward (decided from the yaw) is true while
  // the robot backs up. Everything measured along such a path (arc lengths, obstacle velocity
  // relative to the trajectory) is already in the direction of travel; only the ego velocity
  // (signed in body frame, negative) and the leading edge (the rear bumper) need the real
  // direction, which the velocity sign of the plan at ego gives.
  static bool is_reversing(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::shared_ptr<const PlannerData> planner_data)
  {
    if (!traj_points.empty()) {
      const size_t ego_idx =
        planner_data->find_index(traj_points, planner_data->current_odometry.pose.pose);
      for (size_t i = ego_idx; i < traj_points.size(); ++i) {
        const double v = traj_points.at(i).longitudinal_velocity_mps;
        if (std::abs(v) > 1e-3) {
          return v < 0.0;
        }
      }
    }
    return planner_data->current_odometry.twist.twist.linear.x < 0.0;
  }

  // Ego speed in its direction of travel: positive while it moves the way the plan drives.
  static double calc_ego_velocity_along_travel(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::shared_ptr<const PlannerData> planner_data)
  {
    const double vx = planner_data->current_odometry.twist.twist.linear.x;
    return is_reversing(traj_points, planner_data) ? -vx : vx;
  }

  // base_link to the bumper that leads in the direction of travel.
  static double calc_leading_edge_offset(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::shared_ptr<const PlannerData> planner_data)
  {
    const bool rear_leads =
      !planner_data->is_driving_forward || is_reversing(traj_points, planner_data);
    return rear_leads ? std::abs(planner_data->vehicle_info_.min_longitudinal_offset_m)
                      : std::abs(planner_data->vehicle_info_.max_longitudinal_offset_m);
  }

  static double calc_distance_to_collisionPoint(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::shared_ptr<const PlannerData> planner_data,
    const geometry_msgs::msg::Point & collision_point)
  {
    const double offset = calc_leading_edge_offset(traj_points, planner_data);

    const size_t ego_segment_idx =
      planner_data->find_segment_index(traj_points, planner_data->current_odometry.pose.pose);

    const size_t collision_segment_idx =
      autoware::motion_utils::findNearestSegmentIndex(traj_points, collision_point);

    // `calcSignedArcLength` is anchored to the trajectory's own fixed point
    // ordering, NOT to ego's actual direction of travel: a positive result only
    // means "collision_point is at a higher index than ego", which is "ahead of
    // ego" when driving forward but "behind ego" when reversing. Flip the sign
    // when reversing so the result is always positive when the collision point
    // is in ego's actual direction of travel (same idiom as
    // `calc_distance_to_front_object()` in autoware_motion_velocity_planner_common).
    const auto raw_dist_to_collision_point = autoware::motion_utils::calcSignedArcLength(
      traj_points, planner_data->current_odometry.pose.pose.position, ego_segment_idx,
      collision_point, collision_segment_idx);
    const auto dist_to_collision_point =
      planner_data->is_driving_forward ? raw_dist_to_collision_point : -raw_dist_to_collision_point;

    return dist_to_collision_point - offset;
  }

  double calc_rss_distance(
    const double ego_vel, const double obstacle_vel, const double margin = 0.0) const
  {
    const double rss_dist_with_margin =
      ego_vel * cruise_planning_param_.idling_time +
      std::pow(ego_vel, 2) * 0.5 / std::abs(cruise_planning_param_.min_ego_accel_for_rss) -
      std::pow(obstacle_vel, 2) * 0.5 / std::abs(cruise_planning_param_.min_object_accel_for_rss) +
      margin;
    return rss_dist_with_margin;
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // CRUISE_PLANNER_INTERFACE_HPP_
