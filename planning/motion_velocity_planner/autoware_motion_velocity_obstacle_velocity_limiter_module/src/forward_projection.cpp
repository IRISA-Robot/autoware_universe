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

#include "forward_projection.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/strategies/agnostic/buffer_distance_asymmetric.hpp>
#include <boost/geometry/strategies/agnostic/buffer_distance_symmetric.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/strategies/cartesian/buffer_end_flat.hpp>
#include <boost/geometry/strategies/cartesian/buffer_join_miter.hpp>
#include <boost/geometry/strategies/cartesian/buffer_point_circle.hpp>
#include <boost/geometry/strategies/cartesian/buffer_side_straight.hpp>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{

segment_t forwardSimulatedSegment(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params)
{
  // `params.heading` is the trajectory point's pose orientation, i.e. the nominal path
  // direction, not necessarily ego's actual direction of travel: when reversing, ego moves
  // towards -heading. `params.velocity` is treated as a speed magnitude (not assumed to already
  // carry a reversing sign, matching this module's other trajectory-point reads, e.g.
  // trajectory_preprocessing.cpp's `longitudinal_velocity_mps > 0.0` checks) and `extra_length`
  // is the (always-positive) margin from the trajectory point to ego's currently-leading bumper
  // (front when forward, rear when reversing -- selected by the caller). Both terms must be
  // flipped together so the projected segment sweeps towards ego's real leading edge.
  const auto direction_sign = params.is_driving_forward ? 1.0 : -1.0;
  const auto length =
    direction_sign * (std::abs(params.velocity) * params.duration + params.extra_length);
  const auto from = point_t{origin.x, origin.y};
  const auto heading = params.heading;
  const auto to =
    point_t{from.x() + std::cos(heading) * length, from.y() + std::sin(heading) * length};
  return segment_t{from, to};
}

multi_linestring_t bicycleProjectionLines(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params)
{
  multi_linestring_t lines;
  if (params.steering_angle_offset == 0.0)
    lines.push_back(bicycleProjectionLine(origin, params, params.steering_angle));
  else
    for (const auto offset : {params.steering_angle_offset, 0.0, -params.steering_angle_offset})
      lines.push_back(bicycleProjectionLine(origin, params, params.steering_angle + offset));
  return lines;
}

linestring_t bicycleProjectionLine(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params,
  const double steering_angle)
{
  linestring_t line;
  line.reserve(params.points_per_projection);
  line.emplace_back(origin.x, origin.y);
  const auto dt = params.duration / (params.points_per_projection - 1);
  // Same direction-of-travel caveat as forwardSimulatedSegment(): the bicycle model's yaw rate
  // (v/L * tan(steer)) and the swept length both depend on the *signed* velocity in ego's actual
  // direction of travel, not the nominal path heading direction. Rebuild that signed velocity
  // from the (assumed-magnitude) `params.velocity` and the direction flag so both the curvature
  // and the sweep distance reverse together when backing up.
  const auto direction_sign = params.is_driving_forward ? 1.0 : -1.0;
  const auto signed_velocity = direction_sign * std::abs(params.velocity);
  const auto rotation_rate = signed_velocity * std::tan(steering_angle) / params.wheel_base;
  for (auto i = 1; i < params.points_per_projection; ++i) {
    const auto t = i * dt;
    const auto heading = params.heading + rotation_rate * t;
    const auto length = signed_velocity * t + direction_sign * params.extra_length;
    line.emplace_back(origin.x + length * std::cos(heading), origin.y + length * std::sin(heading));
  }
  return line;
}

polygon_t generateFootprint(const multi_linestring_t & lines, const double lateral_offset)
{
  polygon_t footprint;
  if (lines.size() == 1) {
    footprint = generateFootprint(lines.front(), lateral_offset);
  } else if (lines.size() >= 3) {
    // assumes 3 lines ordered from left to right
    footprint = generateFootprint(lines[0], lines[1], lines[2], lateral_offset);
  }
  return footprint;
}

polygon_t generateFootprint(const segment_t & segment, const double lateral_offset)
{
  return generateFootprint(linestring_t{segment.first, segment.second}, lateral_offset);
}

polygon_t generateFootprint(const linestring_t & linestring, const double lateral_offset)
{
  namespace bg = boost::geometry;
  multi_polygon_t footprint;
  namespace strategy = bg::strategy::buffer;
  bg::buffer(
    linestring, footprint, strategy::distance_symmetric<double>(lateral_offset),
    strategy::side_straight(), strategy::join_miter(), strategy::end_flat(),
    strategy::point_square());
  if (footprint.empty()) return {};
  return footprint[0];
}

polygon_t generateFootprint(
  const linestring_t & left, const linestring_t & middle, const linestring_t & right,
  const double lateral_offset)
{
  polygon_t footprint;
  // calculate normal unit vector from the end of a line to generate its left/right points
  constexpr auto perpendicular_point =
    [](const point_t & a, const point_t & b, const double offset) {
      const auto vector = (b - a).normalized();
      const auto normal_vector = point_t{-vector.y(), vector.x()};
      const auto point = b + (normal_vector * offset);
      return point_t{point.x(), point.y()};
    };

  footprint.outer().push_back(perpendicular_point(left[1], left[0], -lateral_offset));
  for (auto it = left.begin(); it != std::prev(left.end()); ++it)
    footprint.outer().push_back(perpendicular_point(*it, *std::next(it), lateral_offset));
  // only use the left/right points at the end of the center line
  {
    footprint.outer().push_back(
      perpendicular_point(middle[middle.size() - 2], middle.back(), lateral_offset));
    footprint.outer().push_back(
      perpendicular_point(middle[middle.size() - 2], middle.back(), -lateral_offset));
  }
  // points to the right of the right line are added in reverse
  footprint.outer().push_back(
    perpendicular_point(right[right.size() - 2], right.back(), -lateral_offset));
  for (auto it = right.rbegin(); it != std::prev(right.rend()); ++it)
    footprint.outer().push_back(perpendicular_point(*it, *std::next(it), lateral_offset));
  footprint.outer().push_back(footprint.outer().front());  // close the polygon
  return footprint;
}
}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
