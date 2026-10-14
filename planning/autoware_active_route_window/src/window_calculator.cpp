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

#include "autoware/active_route_window/window_calculator.hpp"

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::active_route_window
{
namespace
{
constexpr double kSampleStepM = 0.5;
// Where across the lanelet to sample, as a fraction of the way from the left to the right bound.
// 0.1/0.9 keep the samples ~10 % of the width inside the bounds, so a shared bound is not overlap.
constexpr double kAcross[] = {0.1, 0.5, 0.9};

lanelet::BasicPoint2d point_at(const lanelet::ConstLineString2d & line, const double fraction)
{
  const double length = lanelet::geometry::length(line);
  return lanelet::geometry::interpolatedPointAtDistance(line, fraction * length);
}

std::vector<lanelet::BasicPoint2d> inner_samples(const lanelet::ConstLanelet & llt)
{
  const auto left = llt.leftBound2d();
  const auto right = llt.rightBound2d();
  const double longest =
    std::max(lanelet::geometry::length(left), lanelet::geometry::length(right));
  const auto steps = static_cast<size_t>(std::max(2.0, std::ceil(longest / kSampleStepM)));
  std::vector<lanelet::BasicPoint2d> samples;
  samples.reserve((steps + 1) * std::size(kAcross));
  for (size_t k = 0; k <= steps; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(steps);
    const auto l = point_at(left, t);
    const auto r = point_at(right, t);
    for (const double f : kAcross) {
      samples.emplace_back(l + f * (r - l));
    }
  }
  return samples;
}

// Even-odd rule: works on self-intersecting polygons too.
bool inside(const lanelet::BasicPoint2d & p, const lanelet::BasicPolygon2d & polygon)
{
  bool in = false;
  const size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const auto & a = polygon.at(i);
    const auto & b = polygon.at(j);
    if ((a.y() > p.y()) != (b.y() > p.y())) {
      const double x = a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
      if (p.x() < x) {
        in = !in;
      }
    }
  }
  return in;
}

bool any_inside(const lanelet::ConstLanelet & from, const lanelet::ConstLanelet & into)
{
  const auto polygon = into.polygon2d().basicPolygon();
  if (polygon.size() < 3) {
    return false;
  }
  const auto samples = inner_samples(from);
  return std::any_of(samples.begin(), samples.end(), [&](const auto & p) {
    return inside(p, polygon);
  });
}
}  // namespace

bool lanelets_overlap(const lanelet::ConstLanelet & a, const lanelet::ConstLanelet & b)
{
  return any_inside(a, b) || any_inside(b, a);
}

ConflictMatrix compute_conflicts(const std::vector<lanelet::ConstLanelet> & segment_lanelets)
{
  const size_t n = segment_lanelets.size();
  ConflictMatrix conflicts(n, std::vector<bool>(n, false));
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 2; j < n; ++j) {
      const bool overlap = lanelets_overlap(segment_lanelets.at(i), segment_lanelets.at(j));
      conflicts.at(i).at(j) = overlap;
      conflicts.at(j).at(i) = overlap;
    }
  }
  return conflicts;
}

std::optional<size_t> select_ego_segment(
  const std::vector<double> & distances_m, const std::optional<size_t> & previous,
  const double tie_tolerance_m, const double max_drift_m, const double max_distance_m)
{
  if (distances_m.empty()) {
    return std::nullopt;
  }
  const size_t n = distances_m.size();

  if (previous && *previous < n) {
    const size_t p = *previous;
    const size_t begin = p > 0 ? p - 1 : 0;
    const size_t end = std::min(n, p + 3);
    double local_min = std::numeric_limits<double>::max();
    for (size_t i = begin; i < end; ++i) {
      local_min = std::min(local_min, distances_m.at(i));
    }
    if (local_min <= max_drift_m) {
      // The furthest along the route that ego is on: hand over at a seam as soon as ego reaches it.
      for (size_t i = end; i-- > p;) {
        if (distances_m.at(i) <= local_min + tie_tolerance_m) {
          return i;
        }
      }
      return begin;  // only the one behind is near: ego went backwards along the route
    }
    // Ego is off all of them (relocalised): search the whole route below.
  }

  const double min_distance = *std::min_element(distances_m.begin(), distances_m.end());
  if (min_distance > max_distance_m) {
    return std::nullopt;
  }
  const auto is_candidate = [&](const size_t i) {
    return distances_m.at(i) <= min_distance + tie_tolerance_m;
  };
  if (previous) {
    for (size_t i = std::min(*previous, n); i < n; ++i) {
      if (is_candidate(i)) {
        return i;
      }
    }
  }
  for (size_t i = 0; i < n; ++i) {
    if (is_candidate(i)) {
      return i;
    }
  }
  return std::nullopt;
}

Window compute_window(
  const size_t ego_segment, const double ego_arc_m, const std::vector<double> & segment_lengths_m,
  const ConflictMatrix & conflicts, const double backward_m, const double forward_m)
{
  const size_t n = segment_lengths_m.size();
  Window window{ego_segment, ego_segment};
  if (ego_segment >= n) {
    return window;
  }
  const auto conflict = [&](const size_t i, const size_t j) {
    return i < conflicts.size() && j < conflicts.at(i).size() && conflicts.at(i).at(j);
  };

  // Ahead: stop before a segment that lies over one between ego's and it.
  double ahead = std::max(0.0, segment_lengths_m.at(ego_segment) - ego_arc_m);
  for (size_t j = ego_segment + 1; j < n && ahead < forward_m; ++j) {
    bool overlaps_earlier = false;
    for (size_t i = ego_segment; i + 2 <= j; ++i) {
      if (conflict(i, j)) {
        overlaps_earlier = true;
        break;
      }
    }
    if (overlaps_earlier) {
      break;
    }
    window.end = j;
    ahead += segment_lengths_m.at(j);
  }

  // Behind: stop at a segment that lies under any later one in the window.
  double behind = std::max(0.0, ego_arc_m);
  for (size_t i = ego_segment; i-- > 0 && behind < backward_m;) {
    bool overlaps_later = false;
    for (size_t j = i + 2; j <= window.end; ++j) {
      if (conflict(i, j)) {
        overlaps_later = true;
        break;
      }
    }
    if (overlaps_later) {
      break;
    }
    window.begin = i;
    behind += segment_lengths_m.at(i);
  }
  return window;
}

}  // namespace autoware::active_route_window
