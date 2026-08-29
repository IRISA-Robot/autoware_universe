// Copyright 2022 TIER IV, Inc.
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

#ifndef MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
#define MISSION_PLANNER__ARRIVAL_CHECKER_HPP_

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace autoware::mission_planner_universe
{

class ArrivalChecker
{
public:
  using PoseWithUuidStamped = autoware_planning_msgs::msg::PoseWithUuidStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  explicit ArrivalChecker(rclcpp::Node * node);
  void set_goal();
  // `is_reversed_goal`: whether the route's final (goal) segment is traversed in the inverted
  // (reverse) direction of its lanelet (bidirectional-driving support, see
  // LaneletSegment::is_reversed / RouteHandler::createMapSegments()). When true, is_arrived()
  // bypasses the yaw-alignment gate entirely -- backing into a goal does not produce any fixed,
  // predictable relationship to the stored goal orientation (depends on approach curvature), so a
  // full bypass is used rather than comparing against a computed yaw_goal + pi. Defaults to false
  // so ordinary (non-reversed) goals are completely unaffected.
  void set_goal(const PoseWithUuidStamped & goal, bool is_reversed_goal = false);
  bool is_arrived(const PoseStamped & pose) const;

private:
  double angle_;
  double duration_;
  double arrival_check_lateral_distance_;
  double arrival_check_longitudinal_undershoot_distance_;
  double arrival_check_longitudinal_overshoot_distance_;
  std::optional<PoseWithUuidStamped> goal_with_uuid_;
  bool is_reversed_goal_{false};
  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr sub_goal_;
  autoware::motion_utils::VehicleStopChecker vehicle_stop_checker_;
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
