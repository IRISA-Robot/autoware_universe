// Copyright 2026 Autoware Planning Team
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

#ifndef AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__DATA_STRUCTS_HPP_

#include <lanelet2_core/primitives/Lanelet.h>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

// Tunables for Phase 1 "retrace last N meters" scaffold. All declared with in-code defaults (see
// manager.cpp) -- once tuning-engineer wires config/reverse_lane_follow.param.yaml these are the
// keys under the `reverse_lane_follow.` namespace to expose.
struct ReverseLaneFollowParameters
{
  // Default distance to retrace when a request does not specify its own distance
  // (request value <= 0 falls back to this).
  double default_retrace_distance_m{10.0};

  // Minimum meaningful retrace request; requests below this are ignored.
  double min_retrace_distance_m{1.0};

  // Magnitude (m/s) of the commanded reverse speed. Sign is applied separately using the
  // `(is_back ? -1 : 1) * velocity` convention from autoware_freespace_planner's utils.cpp.
  double retrace_velocity_mps{1.0};

  // Ego-to-path-end distance below which the retrace is considered complete.
  double goal_reach_tolerance_m{0.5};

  // Bidirectional-driving support (Task E / §4d): forward/backward lookahead (meters) used when
  // this module activates because ego is on a route segment RouteHandler flags as reversed,
  // independent of any explicit retrace request. "forward" here means ahead of ego in the actual
  // direction of travel (i.e. along the inverted lanelet sequence), not map/centerline order.
  double route_reversed_forward_distance_m{30.0};
  double route_reversed_backward_distance_m{5.0};
};

// This module's own "am I currently retracing" status flag -- the reverse-lane-follow analogue of
// autoware_behavior_path_start_planner_module's `status_.driving_forward` /
// `isDrivingForward()` pattern.
struct ReverseLaneFollowStatus
{
  bool is_retracing{false};
  double requested_distance_m{0.0};
  PathWithLaneId retrace_path{};
  // Lanelet sequence the retrace_path was built from -- kept around so plan() can regenerate a
  // drivable area without re-querying RouteHandler every cycle.
  lanelet::ConstLanelets retrace_lanelets{};

  // Bidirectional-driving support (Task E / §4d): true when this module is active because ego is
  // on a route segment RouteHandler flags as reversed, rather than because of an explicit
  // retrace request. route_reversed_path/route_reversed_lanelets mirror retrace_path/
  // retrace_lanelets for that trigger.
  bool is_route_reversed_active{false};
  PathWithLaneId route_reversed_path{};
  lanelet::ConstLanelets route_reversed_lanelets{};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__DATA_STRUCTS_HPP_
