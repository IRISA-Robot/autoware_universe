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

#ifndef AUTOWARE__ACTIVE_ROUTE_WINDOW__WINDOW_CALCULATOR_HPP_
#define AUTOWARE__ACTIVE_ROUTE_WINDOW__WINDOW_CALCULATOR_HPP_

#include <lanelet2_core/primitives/Lanelet.h>

#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::active_route_window
{

/// conflicts[i][j]: route segments i and j lie over each other (never set for i == j or for
/// neighbours, which share a seam by construction).
using ConflictMatrix = std::vector<std::vector<bool>>;

struct Window
{
  size_t begin{0};  ///< first segment, inclusive
  size_t end{0};    ///< last segment, inclusive
};

/**
 * @brief Whether two lanelets overlap: a point well inside one (sampled along it, inset from its
 *        bounds) lies inside the other.
 * @details Insetting keeps lanelets that only share a bound (lanes side by side) from counting.
 *          Point-in-polygon by the even-odd rule also works on the self-intersecting polygons a
 *          tight U-turn lanelet can have, where boost polygon intersection does not.
 */
bool lanelets_overlap(const lanelet::ConstLanelet & a, const lanelet::ConstLanelet & b);

/// Conflicts between every pair of non-neighbouring segments, given their preferred lanelets.
ConflictMatrix compute_conflicts(const std::vector<lanelet::ConstLanelet> & segment_lanelets);

/**
 * @brief The segment ego is on, by route order rather than by which lanelet happens to be nearest.
 * @details With a previous pick: look only at it and its route neighbours (one back, two ahead)
 *          and take the furthest along the route within tie_tolerance_m of the nearest of those --
 *          a later pass lying under ego never wins.  When ego is farther than max_drift_m from all
 *          of them (relocalised) the whole route is searched: the first candidate at or after the
 *          previous pick, else the earliest.  With no previous pick, the earliest candidate.
 * @param distances_m distance from ego to each segment's lanelet, in route order
 * @return nullopt if every segment is farther than max_distance_m
 */
std::optional<size_t> select_ego_segment(
  const std::vector<double> & distances_m, const std::optional<size_t> & previous,
  double tie_tolerance_m, double max_drift_m, double max_distance_m);

/**
 * @brief The window of segments around ego that contains no two overlapping segments.
 * @details Ahead: segments are added until forward_m is covered, stopping before the first one
 *          that overlaps a segment between ego's and it.  Behind: segments are added until
 *          backward_m is covered, stopping at the first one that overlaps any later segment in the
 *          window.  Ego's own segment is always in the window.
 * @param ego_arc_m how far along its segment ego is
 */
Window compute_window(
  size_t ego_segment, double ego_arc_m, const std::vector<double> & segment_lengths_m,
  const ConflictMatrix & conflicts, double backward_m, double forward_m);

}  // namespace autoware::active_route_window

#endif  // AUTOWARE__ACTIVE_ROUTE_WINDOW__WINDOW_CALCULATOR_HPP_
