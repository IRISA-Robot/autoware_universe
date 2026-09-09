// Copyright 2026 azzamwildan462
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

#include "autoware/stuck_recovery_supervisor/corridor_checker.hpp"

// tf2/utils.h only *declares* tf2::fromMsg for geometry_msgs types; the definition
// lives in tf2_geometry_msgs.  Without this include the library still links (shared
// objects tolerate undefined symbols) and only dies at the first getYaw() call --
// i.e. the moment the supervisor first sweeps a corridor.
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::stuck_recovery_supervisor
{

namespace
{

/// Occupied, or off-grid, or unknown -- all three mean "do not drive here".
/// Treating off-grid as blocked is deliberate: the recovery grid is only 20 m wide,
/// so a ray leaving it has already travelled further than we are willing to trust.
bool is_blocked(const nav_msgs::msg::OccupancyGrid & grid, double x, double y, int8_t threshold)
{
  const double res = grid.info.resolution;
  if (res <= 0.0) {
    return true;
  }

  // The recovery costmap is published axis-aligned in the map frame, but honour a
  // rotated origin anyway so this stays correct if that ever changes.
  const double yaw = tf2::getYaw(grid.info.origin.orientation);
  const double dx = x - grid.info.origin.position.x;
  const double dy = y - grid.info.origin.position.y;
  const double cos_yaw = std::cos(-yaw);
  const double sin_yaw = std::sin(-yaw);
  const double local_x = dx * cos_yaw - dy * sin_yaw;
  const double local_y = dx * sin_yaw + dy * cos_yaw;

  const int col = static_cast<int>(std::floor(local_x / res));
  const int row = static_cast<int>(std::floor(local_y / res));
  if (
    col < 0 || row < 0 || col >= static_cast<int>(grid.info.width) ||
    row >= static_cast<int>(grid.info.height)) {
    return true;
  }

  const auto value =
    grid.data[static_cast<size_t>(row) * grid.info.width + static_cast<size_t>(col)];
  if (value < 0) {
    return true;  // unknown
  }
  return value >= threshold;
}

/// A maximal run of free cells along the perpendicular, described by its signed
/// lateral offsets (positive = left of the direction of travel).
struct FreeSpan
{
  double from{0.0};
  double to{0.0};
  double width() const { return to - from; }
  double centre() const { return 0.5 * (from + to); }
  bool contains(double offset) const { return offset >= from && offset <= to; }
};

/// Cut the perpendicular line through (x0,y0) into its maximal free spans.
std::vector<FreeSpan> scan_profile(
  const nav_msgs::msg::OccupancyGrid & grid, double x0, double y0, double ux, double uy,
  double max_distance, double step, int8_t threshold)
{
  std::vector<FreeSpan> spans;
  bool in_span = false;
  double span_start = 0.0;
  double offset = -max_distance;
  double last_free = -max_distance;

  for (; offset <= max_distance; offset += step) {
    const bool blocked = is_blocked(grid, x0 + ux * offset, y0 + uy * offset, threshold);
    if (!blocked) {
      if (!in_span) {
        in_span = true;
        span_start = offset;
      }
      last_free = offset;
    } else if (in_span) {
      spans.push_back(FreeSpan{span_start, last_free});
      in_span = false;
    }
  }
  if (in_span) {
    spans.push_back(FreeSpan{span_start, last_free});
  }
  return spans;
}

double arc_distance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  return std::hypot(dx, dy);
}

}  // namespace

std::optional<size_t> find_nearest_index(
  const std::vector<geometry_msgs::msg::Pose> & path, const geometry_msgs::msg::Pose & pose)
{
  if (path.empty()) {
    return std::nullopt;
  }

  size_t best = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); ++i) {
    const double distance = arc_distance(path[i], pose);
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

CorridorResult sweep_corridor(
  const nav_msgs::msg::OccupancyGrid & grid, const std::vector<geometry_msgs::msg::Pose> & path,
  size_t start_index, const CorridorParams & params)
{
  CorridorResult result;
  result.required_width_m = params.vehicle_width_m + 2.0 * params.lateral_margin_m;

  if (grid.info.resolution <= 0.0 || grid.data.empty() || start_index >= path.size()) {
    return result;
  }

  const double step = std::max(grid.info.resolution * 0.5, 0.02);
  const double half_footprint = params.vehicle_width_m * 0.5 + params.lateral_margin_m;

  // Per-side minima along the corridor.  Aggregating each side independently is the
  // whole point: the robot has to stay on ONE side for the entire stretch.
  double min_left = std::numeric_limits<double>::max();
  double min_right = std::numeric_limits<double>::max();
  double left_offset_at_min = 0.0;
  double right_offset_at_min = 0.0;
  geometry_msgs::msg::Pose left_pose_at_min{};
  geometry_msgs::msg::Pose right_pose_at_min{};

  bool clear = true;
  bool first_blocked_recorded = false;

  // Walk the path by arc length rather than by index, so the sampling density does
  // not depend on however finely the centerline happened to be discretised.
  double since_last_sample = params.step_m;  // force a sample at the very first pose
  double arc_from_start = 0.0;
  for (size_t i = start_index; i < path.size(); ++i) {
    if (i > start_index) {
      const double d = arc_distance(path[i - 1], path[i]);
      since_last_sample += d;
      arc_from_start += d;
    }
    if (arc_from_start > params.max_arc_m) {
      break;
    }
    if (since_last_sample < params.step_m) {
      continue;
    }
    since_last_sample = 0.0;

    const auto & pose = path[i];
    const double yaw = tf2::getYaw(pose.orientation);
    // +1 when the pose yaw already is the vehicle heading, -1 on a reversed route
    // where the pose yaw is the travel direction and the vehicle faces the other way.
    const double mirror = params.mirror_lateral ? -1.0 : 1.0;
    const double nx = mirror * -std::sin(yaw);  // vehicle-left normal
    const double ny = mirror * std::cos(yaw);
    const double px = pose.position.x;
    const double py = pose.position.y;

    ++result.stations;

    const auto spans = scan_profile(
      grid, px, py, nx, ny, params.max_scan_half_width_m, step, params.occupancy_threshold);

    // Where is the blockage on this station?  If something sits on the path, the
    // split is the blocked interval around offset 0; spans beyond its left edge are
    // LEFT gaps, beyond its right edge are RIGHT gaps.  If the path itself is free,
    // the span straddling 0 is the corridor we are already in, and it is credited to
    // both sides because the robot can reach it without committing to either.
    double station_left = 0.0;
    double station_right = 0.0;
    double station_left_offset = 0.0;
    double station_right_offset = 0.0;

    for (const auto & span : spans) {
      const double width = span.width();
      if (width <= 0.0) {
        continue;
      }
      if (span.contains(0.0)) {
        // Straddles the path: usable from either side.
        if (width > station_left) {
          station_left = width;
          station_left_offset = span.centre();
        }
        if (width > station_right) {
          station_right = width;
          station_right_offset = span.centre();
        }
      } else if (span.centre() > 0.0) {
        if (width > station_left) {
          station_left = width;
          station_left_offset = span.centre();
        }
      } else {
        if (width > station_right) {
          station_right = width;
          station_right_offset = span.centre();
        }
      }
    }

    if (station_left < min_left) {
      min_left = station_left;
      left_offset_at_min = station_left_offset;
      left_pose_at_min = pose;
    }
    if (station_right < min_right) {
      min_right = station_right;
      right_offset_at_min = station_right_offset;
      right_pose_at_min = pose;
    }

    // EXIT question, deliberately a different test: the robot must fit *centred on
    // the path*, without shifting, because that is what lane driving is about to do.
    const bool centre_free = !is_blocked(grid, px, py, params.occupancy_threshold);
    bool station_clear = centre_free;
    if (centre_free) {
      for (const auto & span : spans) {
        if (span.contains(0.0)) {
          station_clear = (0.0 - span.from) >= half_footprint && (span.to - 0.0) >= half_footprint;
          break;
        }
      }
    }
    if (!station_clear) {
      if (!first_blocked_recorded) {
        result.first_blocked_station = result.stations - 1;
        result.first_blocked_arc_m = arc_from_start;
        result.has_blockage = true;
        first_blocked_recorded = true;
      }
      clear = false;
    }
  }

  if (result.stations == 0) {
    return result;
  }

  result.valid = true;
  result.free_left_m = (min_left == std::numeric_limits<double>::max()) ? 0.0 : min_left;
  result.free_right_m = (min_right == std::numeric_limits<double>::max()) ? 0.0 : min_right;
  result.clear = clear;

  // Commit to the better single side -- never the sum, never a per-station mix.
  if (result.free_left_m >= result.free_right_m) {
    result.free_width_m = result.free_left_m;
    result.best_side = result.free_left_m > 0.0 ? Side::LEFT : Side::NONE;
    result.best_offset_m = left_offset_at_min;
    result.narrowest_pose = left_pose_at_min;
  } else {
    result.free_width_m = result.free_right_m;
    result.best_side = Side::RIGHT;
    result.best_offset_m = right_offset_at_min;
    result.narrowest_pose = right_pose_at_min;
  }
  return result;
}

}  // namespace autoware::stuck_recovery_supervisor
