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

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace
{
using autoware::active_route_window::compute_conflicts;
using autoware::active_route_window::compute_window;
using autoware::active_route_window::ConflictMatrix;
using autoware::active_route_window::lanelets_overlap;
using autoware::active_route_window::select_ego_segment;

// hibikino_kanan out-and-back route, measured on the map:
//   0=382 1=373 2=347 3=458(U-turn) 4=195 5=188 6=303 7=294
// 195 lies over 382 and 373, 188 over 382; 347 runs 2.5 m beside 195 without overlapping.
const std::vector<double> kLengths{91.7, 31.0, 20.9, 5.1, 91.0, 55.5, 4.1, 4.8};
ConflictMatrix hibikino_conflicts()
{
  ConflictMatrix c(8, std::vector<bool>(8, false));
  const auto set = [&](size_t i, size_t j) { c[i][j] = c[j][i] = true; };
  set(0, 4);
  set(0, 5);
  set(1, 4);
  return c;
}

struct Expect
{
  size_t ego;
  double arc;
  size_t begin;
  size_t end;
};

TEST(ComputeWindow, OutAndBackRouteNeverHoldsBothPasses)
{
  // Expected windows, cross-checked against a Python prototype on the real map geometry.
  const std::vector<Expect> cases{
    {0, 0.0, 0, 3},  {0, 91.7, 0, 3}, {1, 0.0, 0, 3},  {1, 15.5, 1, 3}, {2, 0.0, 1, 3},
    {2, 20.9, 2, 7}, {3, 2.5, 2, 7},  {4, 0.0, 2, 7},  {4, 45.5, 4, 7}, {5, 27.7, 5, 7},
    {7, 4.8, 5, 7}};
  const auto conflicts = hibikino_conflicts();
  for (const auto & c : cases) {
    const auto w = compute_window(c.ego, c.arc, kLengths, conflicts, 10.0, 300.0);
    EXPECT_EQ(w.begin, c.begin) << "ego segment " << c.ego << " arc " << c.arc;
    EXPECT_EQ(w.end, c.end) << "ego segment " << c.ego << " arc " << c.arc;
    for (size_t i = w.begin; i <= w.end; ++i) {
      for (size_t j = i + 1; j <= w.end; ++j) {
        EXPECT_FALSE(conflicts[i][j]) << "window holds overlapping segments " << i << "/" << j;
      }
    }
    EXPECT_LE(w.begin, c.ego);
    EXPECT_GE(w.end, c.ego);
  }
}

TEST(ComputeWindow, KeepsTheLaneletJustLeftAndThePathEndClearOfEgo)
{
  // Forward over the same route: 0=294 1=303 2=188 3=195 4=458 5=347 6=373 7=382.
  const std::vector<double> lengths{4.8, 4.1, 55.5, 91.0, 5.1, 20.9, 31.0, 91.7};
  ConflictMatrix c(8, std::vector<bool>(8, false));
  const auto set = [&](size_t i, size_t j) { c[i][j] = c[j][i] = true; };
  set(2, 7);
  set(3, 6);
  set(3, 7);
  // Just inside the U-turn: 195 (just left) stays for the drivable area, the cut is at 373.
  auto w = compute_window(4, 0.5, lengths, c, 10.0, 300.0);
  EXPECT_EQ(w.begin, 3u);
  EXPECT_EQ(w.end, 5u);
  // Walking the route in 0.5 m steps, whenever the window is cut short its end stays well ahead.
  double closest_cut_end = 1e9;
  for (size_t k = 0; k < lengths.size(); ++k) {
    for (double arc = 0.0; arc <= lengths[k]; arc += 0.5) {
      w = compute_window(k, arc, lengths, c, 10.0, 300.0);
      if (w.end + 1 < lengths.size()) {
        double ahead = lengths[k] - arc;
        for (size_t j = k + 1; j <= w.end; ++j) ahead += lengths[j];
        closest_cut_end = std::min(closest_cut_end, ahead);
      }
      for (size_t i = w.begin; i <= w.end; ++i) {
        for (size_t j = i + 1; j <= w.end; ++j) {
          EXPECT_FALSE(c[i][j]) << "segments " << i << "/" << j << " ego " << k << " arc " << arc;
        }
      }
    }
  }
  EXPECT_GT(closest_cut_end, 15.0);
}

TEST(ComputeWindow, NothingOverlapsMeansTheWholeRouteAhead)
{
  const ConflictMatrix none(8, std::vector<bool>(8, false));
  // 5 m into segment 2: the 10 m behind reaches into segment 1
  auto w = compute_window(2, 5.0, kLengths, none, 10.0, 300.0);
  EXPECT_EQ(w.begin, 1u);
  EXPECT_EQ(w.end, 7u);
  // 10 m into it: already covered by segment 2 itself
  w = compute_window(2, 10.0, kLengths, none, 10.0, 300.0);
  EXPECT_EQ(w.begin, 2u);
}

TEST(ComputeWindow, ForwardDistanceLimitsTheWindow)
{
  const ConflictMatrix none(8, std::vector<bool>(8, false));
  const auto w = compute_window(0, 0.0, kLengths, none, 10.0, 100.0);
  EXPECT_EQ(w.end, 1u);  // 91.7 m of segment 0, then segment 1 starts within 100 m
}

TEST(SelectEgoSegment, OutboundLegKeptWhileInsideTheReturnLegToo)
{
  const std::vector<double> d{0.0, 30.0, 50.0, 60.0, 8.0, 0.0};
  EXPECT_EQ(select_ego_segment(d, std::nullopt, 0.5, 1.0, 3.0), std::optional<size_t>(0));
  EXPECT_EQ(select_ego_segment(d, 0, 0.5, 1.0, 3.0), std::optional<size_t>(0));
}

TEST(SelectEgoSegment, DriftingOffTheOutboundLaneletDoesNotJumpToTheReturnLeg)
{
  const std::vector<double> d{0.6, 40.0, 70.0, 90.0, 5.0, 0.0};
  EXPECT_EQ(select_ego_segment(d, 0, 0.5, 1.0, 3.0), std::optional<size_t>(0));
}

TEST(SelectEgoSegment, HandsOverAtTheSeam)
{
  EXPECT_EQ(select_ego_segment({0.0, 0.4, 20.0}, 0, 0.5, 1.0, 3.0), std::optional<size_t>(1));
  EXPECT_EQ(select_ego_segment({0.0, 0.8, 20.0}, 0, 0.5, 1.0, 3.0), std::optional<size_t>(0));
  // stopped at the U-turn seam: 0.02 m from 347, inside 458
  const std::vector<double> d{50.0, 20.0, 0.02, 0.0, 1.38, 88.0};
  EXPECT_EQ(select_ego_segment(d, 2, 0.5, 1.0, 3.0), std::optional<size_t>(3));
}

TEST(SelectEgoSegment, NeverSkipsTheUTurnOntoTheParallelReturnLeg)
{
  // Reversing along 347 (index 2), drifted to 0.4 m from 195 (index 4, two on, beside it);
  // 458 (index 3) is 3 m ahead.  Ego stays on 347.
  const std::vector<double> d{50.0, 15.0, 0.0, 3.0, 0.4, 70.0};
  EXPECT_EQ(select_ego_segment(d, 2, 0.5, 1.0, 3.0), std::optional<size_t>(2));
}

TEST(SelectEgoSegment, ReturnLegKeptOverTheOutboundLeg)
{
  const std::vector<double> d{0.0, 30.0, 50.0, 60.0, 0.9, 0.0};
  EXPECT_EQ(select_ego_segment(d, 4, 0.5, 1.0, 3.0), std::optional<size_t>(5));
  EXPECT_EQ(select_ego_segment(d, 5, 0.5, 1.0, 3.0), std::optional<size_t>(5));
}

TEST(SelectEgoSegment, RelocalisedSearchesTheWholeRouteAndFarAwayIsRefused)
{
  const std::vector<double> d{50.0, 20.0, 1.6, 1.8, 0.0, 88.0};
  EXPECT_EQ(select_ego_segment(d, 0, 0.5, 1.0, 3.0), std::optional<size_t>(4));
  EXPECT_EQ(select_ego_segment({3.5, 4.0}, 0, 0.5, 1.0, 3.0), std::nullopt);
}

lanelet::Id next_id()
{
  static lanelet::Id id = 1000;
  return ++id;
}

lanelet::ConstLanelet strip(double y_left, double y_right, double x0, double x1, lanelet::Id id)
{
  lanelet::LineString3d left(
    next_id(), {lanelet::Point3d(next_id(), x0, y_left, 0),
                              lanelet::Point3d(next_id(), x1, y_left, 0)});
  lanelet::LineString3d right(
    next_id(), {lanelet::Point3d(next_id(), x0, y_right, 0),
                              lanelet::Point3d(next_id(), x1, y_right, 0)});
  return lanelet::Lanelet(id, left, right);
}

TEST(LaneletsOverlap, OverlapCountsSharedBoundOrGapDoesNot)
{
  const auto a = strip(1.75, -1.75, 0.0, 30.0, 1);
  const auto overlapping = strip(2.85, -0.65, 0.0, 30.0, 2).invert();  // 1.1 m strip, other way
  const auto side_by_side = strip(5.25, 1.75, 0.0, 30.0, 3).invert();  // shares a bound
  const auto apart = strip(6.0, 2.5, 0.0, 30.0, 4).invert();           // 0.75 m gap
  EXPECT_TRUE(lanelets_overlap(a, overlapping));
  EXPECT_FALSE(lanelets_overlap(a, side_by_side));
  EXPECT_FALSE(lanelets_overlap(a, apart));

  const auto conflicts = compute_conflicts({a, side_by_side, overlapping});
  EXPECT_TRUE(conflicts[0][2]);
  EXPECT_FALSE(conflicts[0][1]);  // neighbours are never checked
}
}  // namespace
