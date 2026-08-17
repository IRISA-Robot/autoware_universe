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

#include "autoware/behavior_path_reverse_lane_follow_module/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::behavior_path_planner::reverse_lane_follow_utils
{

std::optional<TraveledTail> extractTraveledTailPath(
  const std::shared_ptr<RouteHandler> & route_handler, const Pose & ego_pose,
  const double retrace_distance_m)
{
  if (!route_handler) {
    return std::nullopt;
  }

  lanelet::ConstLanelet current_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lanelet)) {
    return std::nullopt;
  }

  // Lanelet sequence covering (at least) retrace_distance_m behind ego and nothing ahead of ego.
  // This is purely a lookup into the already-computed, already-legal route -- no routing-graph
  // invert / new route message involved.
  const auto lanelet_sequence =
    route_handler->getLaneletSequence(current_lanelet, ego_pose, retrace_distance_m, 0.0);
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  auto tail_path =
    route_handler->getCenterLinePath(lanelet_sequence, 0.0, std::numeric_limits<double>::max());
  if (tail_path.points.empty()) {
    return std::nullopt;
  }

  // getLaneletSequence(..., forward_distance = 0.0) still returns whole lanelets, so the
  // resulting centerline can extend slightly past ego. Trim so the tail truly ends at ego's
  // current pose -- that is the point the retrace path starts from once reversed.
  const size_t ego_idx =
    autoware::motion_utils::findNearestIndex(tail_path.points, ego_pose.position);
  tail_path.points.resize(ego_idx + 1);

  return TraveledTail{tail_path, lanelet_sequence};
}

PathWithLaneId reversePathForRetrace(
  const PathWithLaneId & forward_tail_path, const double retrace_velocity_mps)
{
  auto reversed = forward_tail_path;
  std::reverse(reversed.points.begin(), reversed.points.end());

  const auto speed_magnitude_mps =
    static_cast<float>(std::abs(retrace_velocity_mps));

  for (auto & path_point : reversed.points) {
    constexpr bool is_back = true;  // this module only ever emits retrace (reverse) motion
    // Exact sign convention proven in autoware_freespace_planner's utils.cpp (~line 135):
    // `(is_back ? -1 : 1) * point.longitudinal_velocity_mps`.
    path_point.point.longitudinal_velocity_mps = (is_back ? -1.0F : 1.0F) * speed_magnitude_mps;
  }

  // Bring the retrace path to a stop at its end (the point ego started retracing from), same
  // idiom autoware_freespace_planner uses for its trajectory tail.
  if (!reversed.points.empty()) {
    reversed.points.back().point.longitudinal_velocity_mps = 0.0F;
  }

  return reversed;
}

std::optional<RouteReversedFollow> buildRouteReversedFollowPath(
  const std::shared_ptr<RouteHandler> & route_handler, const Pose & ego_pose,
  const double backward_distance_m, const double forward_distance_m)
{
  if (!route_handler) {
    return std::nullopt;
  }

  lanelet::ConstLanelet current_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lanelet)) {
    return std::nullopt;
  }

  if (!route_handler->isLaneletInvertedInRoute(current_lanelet)) {
    // Not on a reversed route segment -- this activation trigger does not apply; the caller
    // should fall back to the (still-supported standalone) retrace-request trigger.
    return std::nullopt;
  }

  // following()/previous() on routing_graph_ptr_ already resolve correctly for an inverted
  // ConstLanelet (verified by the Phase-0 spike), so "forward"/"backward" here already mean
  // ahead-of/behind-ego in the actual direction of travel, not map/centerline-index order.
  const auto lanelet_sequence = route_handler->getLaneletSequence(
    current_lanelet, ego_pose, backward_distance_m, forward_distance_m);
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  auto path =
    route_handler->getCenterLinePath(lanelet_sequence, 0.0, std::numeric_limits<double>::max());
  if (path.points.empty()) {
    return std::nullopt;
  }

  for (auto & path_point : path.points) {
    path_point.point.longitudinal_velocity_mps =
      -std::abs(path_point.point.longitudinal_velocity_mps);
  }

  return RouteReversedFollow{path, lanelet_sequence};
}

}  // namespace autoware::behavior_path_planner::reverse_lane_follow_utils
