// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Reverse driving must give the FSM the same answers as forward driving.  Every geometric case
// is run four ways -- {forward, reverse gear} x {lanelet drawn along, against the travel} --
// and the answers compared with the plain forward one.

#include "autoware/road_crossing_fsm/direction_utils.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <random>
#include <vector>

namespace d = autoware::road_crossing_fsm::direction;

namespace
{
constexpr double kFront = 0.9;  // GAMA: wheel_base 0.6 + front_overhang 0.3
constexpr double kRear = 0.3;   // rear_overhang

// A crossing-like polyline in travel order: 12 m with a bend, placed somewhere in the map.
d::Polyline travelLine()
{
  return {{100.0, 50.0}, {104.0, 51.0}, {108.0, 53.0}, {112.0, 56.0}};
}

d::Polyline reversed(const d::Polyline & l)
{
  return d::Polyline(l.rbegin(), l.rend());
}

// A point t metres along the travel direction from the entry (negative: before it, beyond the
// length: past the exit), `lat` metres to the left.
d::Point2 alongTravel(const d::Polyline & line, double t, double lat)
{
  // walk the polyline (extrapolate before the first / after the last segment)
  double acc = 0.0;
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const auto & a = line[i];
    const auto & b = line[i + 1];
    const double len = d::distance(a, b);
    const bool last = i + 2 == line.size();
    if (t <= acc + len || last || (i == 0 && t < 0.0)) {
      const double u = (t - acc) / len;
      const double ux = (b.x - a.x) / len;
      const double uy = (b.y - a.y) / len;
      return {a.x + u * (b.x - a.x) - lat * uy, a.y + u * (b.y - a.y) + lat * ux};
    }
    acc += len;
  }
  return line.back();
}

// What the FSM feeds the helpers: the lanelet's centerline as the route handler returns it
// (getLaneletsFromId(): ConstLanelet::invert() where the route takes the lanelet against its
// drawing direction).  Built through lanelet2 so the inversion is the real one.
d::Polyline routedViaLanelet2(const d::Polyline & travel, const bool drawn_against_travel)
{
  // One physical lanelet around the travel polyline (bounds 1 m to each side).  Drawn against
  // the travel = the same bounds in reverse point order with left and right swapped; the route
  // then takes it inverted.
  std::vector<d::Point2> l;
  std::vector<d::Point2> r;
  for (size_t i = 0; i < travel.size(); ++i) {
    const auto & a = travel[std::min(i, travel.size() - 2)];
    const auto & b = travel[std::min(i, travel.size() - 2) + 1];
    const double len = d::distance(a, b);
    const double nx = -(b.y - a.y) / len;
    const double ny = (b.x - a.x) / len;
    l.push_back({travel[i].x + nx, travel[i].y + ny});
    r.push_back({travel[i].x - nx, travel[i].y - ny});
  }
  if (drawn_against_travel) {
    std::reverse(l.begin(), l.end());
    std::reverse(r.begin(), r.end());
    std::swap(l, r);
  }
  lanelet::Id id = 1000;
  lanelet::LineString3d left(id++);
  lanelet::LineString3d right(id++);
  for (const auto & p : l) left.push_back(lanelet::Point3d(id++, p.x, p.y, 0.0));
  for (const auto & p : r) right.push_back(lanelet::Point3d(id++, p.x, p.y, 0.0));
  lanelet::ConstLanelet llt(lanelet::Lanelet(id++, left, right));
  if (drawn_against_travel) llt = llt.invert();
  d::Polyline out;
  for (const auto & p : llt.centerline2d()) out.push_back({p.x(), p.y()});
  return out;
}

struct Case
{
  const char * name;
  bool drawn_against_travel;  // map drawn against the travel -> the route inverts it
};
// The gear does not enter the geometry at all (that is the point): a reversed route is drawn
// against the travel whatever bumper leads, and the 18(inv) -> 291(fwd) join drives a lanelet
// drawn along the travel in reverse gear.
const Case kCases[] = {{"drawn along the travel", false}, {"drawn against the travel", true}};
}  // namespace

TEST(Projection, MatchesLanelet2ToArcCoordinates)
{
  const auto line = travelLine();
  lanelet::Id id = 1;
  lanelet::LineString3d ls(id++);
  for (const auto & p : line) ls.push_back(lanelet::Point3d(id++, p.x, p.y, 0.0));
  const lanelet::ConstLineString2d ls2d = lanelet::utils::to2D(ls);
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> ux(90.0, 122.0);
  std::uniform_real_distribution<double> uy(40.0, 66.0);
  for (int i = 0; i < 2000; ++i) {
    const d::Point2 p{ux(rng), uy(rng)};
    const double expected =
      lanelet::geometry::toArcCoordinates(ls2d, lanelet::BasicPoint2d{p.x, p.y}).length;
    const double got = d::project(line, p).arc;
    // Equidistant from two segments: either answer is a valid nearest projection.
    if (std::abs(got - expected) > 1e-6) {
      const auto proj = d::project(line, p);
      EXPECT_NEAR(proj.offset, std::abs(lanelet::geometry::toArcCoordinates(
                                 ls2d, lanelet::BasicPoint2d{p.x, p.y}).distance), 1e-6)
        << "p=(" << p.x << "," << p.y << ")";
    }
  }
}

TEST(Geometry, RoutedCenterlineIsInTravelOrder)
{
  // Drawn against the travel and inverted by the route = the same line as drawn along it.
  const auto travel = travelLine();
  const auto along = routedViaLanelet2(travel, false);
  const auto against = routedViaLanelet2(travel, true);
  ASSERT_GE(along.size(), 2u);
  ASSERT_EQ(along.size(), against.size());
  for (size_t i = 0; i < along.size(); ++i) {
    EXPECT_NEAR(along[i].x, against[i].x, 1e-6) << "i=" << i;
    EXPECT_NEAR(along[i].y, against[i].y, 1e-6) << "i=" << i;
  }
  // and it starts at the travel entry
  EXPECT_NEAR(d::distance(along.front(), travel.front()), 0.0, 1e-6);
  EXPECT_NEAR(d::distance(along.back(), travel.back()), 0.0, 1e-6);
}

TEST(Geometry, EntryUpstreamExitIdenticalForBothDrawingDirections)
{
  const auto ref = travelLine();
  const auto along = routedViaLanelet2(ref, false);
  const auto against = routedViaLanelet2(ref, true);
  for (double t = -6.0; t <= 18.0; t += 0.1) {
    for (const double lat : {-0.4, 0.0, 0.6}) {
      const auto p = alongTravel(ref, t, lat);
      EXPECT_NEAR(d::distanceToEntry(against, p), d::distanceToEntry(along, p), 1e-6) << "t=" << t;
      EXPECT_EQ(d::upstreamOfEntry(against, p, 0.3), d::upstreamOfEntry(along, p, 0.3))
        << "t=" << t;
      EXPECT_EQ(d::pastExit(against, p), d::pastExit(along, p)) << "t=" << t;
    }
  }
}

TEST(Geometry, SameAnswersAsAPlainPolylineOfTheTravel)
{
  // The helpers on a hand-made travel polyline (independent of lanelet2's centerline).
  const auto ref = travelLine();
  const auto rev = reversed(reversed(ref));
  for (double t = -6.0; t <= 18.0; t += 0.1) {
    const auto p = alongTravel(ref, t, 0.2);
    EXPECT_NEAR(d::distanceToEntry(rev, p), d::distanceToEntry(ref, p), 1e-9);
    EXPECT_EQ(d::pastExit(rev, p), d::pastExit(ref, p));
  }
}

TEST(Geometry, ForwardAnswersAreTheOldOnes)
{
  // The forward answers are what the FSM computed before reverse support: entry = front,
  // exit = back, straight line to the entry while before it.
  const auto line = travelLine();
  const double len = d::polylineLength(line);
  EXPECT_NEAR(d::distanceToEntry(line, alongTravel(line, -2.0, 0.0)), 2.0, 1e-9);
  EXPECT_NEAR(d::distanceToEntry(line, alongTravel(line, 3.0, 0.0)), 3.0, 1e-9);
  EXPECT_TRUE(d::upstreamOfEntry(line, alongTravel(line, -1.0, 0.0), 0.5));
  EXPECT_FALSE(d::upstreamOfEntry(line, alongTravel(line, -0.2, 0.0), 0.5));
  EXPECT_FALSE(d::upstreamOfEntry(line, alongTravel(line, 1.0, 0.0), 0.5));
  EXPECT_FALSE(d::pastExit(line, alongTravel(line, len - 0.5, 0.0)));
  EXPECT_TRUE(d::pastExit(line, alongTravel(line, len + 0.5, 0.0)));
  EXPECT_EQ(d::distanceToEntry({{0.0, 0.0}}, {1.0, 1.0}), -1.0);
  EXPECT_FALSE(d::pastExit({}, {1.0, 1.0}));
}

TEST(LeadingEdge, BumperAheadInTravelDirectionInBothGears)
{
  const auto line = travelLine();
  for (double t = -3.0; t <= 14.0; t += 0.5) {
    const auto base = alongTravel(line, t, 0.0);
    const auto tangent = d::tangentYawAt(line, base);
    ASSERT_TRUE(tangent);
    // forward: body faces the travel direction; reverse: body faces the other way
    const auto f = d::leadingEdgePoint(base, *tangent, kFront, kRear, true);
    const auto r = d::leadingEdgePoint(base, *tangent + M_PI, kFront, kRear, false);
    const double along_f = (f.x - base.x) * std::cos(*tangent) + (f.y - base.y) * std::sin(*tangent);
    const double along_r = (r.x - base.x) * std::cos(*tangent) + (r.y - base.y) * std::sin(*tangent);
    EXPECT_NEAR(along_f, kFront, 1e-9);
    EXPECT_NEAR(along_r, kRear, 1e-9);
  }
  EXPECT_DOUBLE_EQ(d::leadingEdgeOffset(kFront, kRear, true), kFront);
  EXPECT_DOUBLE_EQ(d::leadingEdgeOffset(kFront, kRear, false), kRear);
}

TEST(RouteOrder, ReverseFollowAcceptedRetraceRejected)
{
  for (double tangent = -M_PI; tangent < M_PI; tangent += M_PI / 7.0) {
    // forward along the route
    EXPECT_TRUE(d::travelsInRouteOrder(tangent, true, tangent));
    // reverse gear on a reversed route: body faces away from the travel direction
    EXPECT_TRUE(d::travelsInRouteOrder(tangent + M_PI, false, tangent));
    // retrace: backing up along the route (body along the route, reverse gear)
    EXPECT_FALSE(d::travelsInRouteOrder(tangent, false, tangent));
    // driving forward against the route
    EXPECT_FALSE(d::travelsInRouteOrder(tangent + M_PI, true, tangent));
    // heading noise of a real robot
    EXPECT_TRUE(d::travelsInRouteOrder(tangent + 0.5, true, tangent));
    EXPECT_TRUE(d::travelsInRouteOrder(tangent + M_PI - 0.5, false, tangent));
  }
}

TEST(Gear, FromPlannedVelocityNotYaw)
{
  EXPECT_EQ(d::gearFromVelocities({0.4, 0.6}), std::optional<bool>(true));
  EXPECT_EQ(d::gearFromVelocities({-0.4, -0.6}), std::optional<bool>(false));
  EXPECT_EQ(d::gearFromVelocities({0.0, 0.0, -0.8}), std::optional<bool>(false));
  EXPECT_EQ(d::gearFromVelocities({0.0, 0.0}), std::nullopt);  // held at the line: keep
  EXPECT_EQ(d::gearFromVelocities({}), std::nullopt);
  EXPECT_EQ(d::gearFromVelocities({0.009, -0.009, 0.5}), std::optional<bool>(true));
}

TEST(Rollback, SameDecisionForTheSameSpeedInBothGears)
{
  for (double gap = -0.5; gap <= 3.0; gap += 0.05) {
    for (double v = 0.0; v <= 1.6; v += 0.05) {
      EXPECT_EQ(d::canStopBefore(gap, v, 1.0, 0.3), d::canStopBefore(gap, -v, 1.0, 0.3))
        << "gap=" << gap << " v=" << v;
    }
  }
  // 2026-11-02 lamp 895: bumper 0.3 m from the crossing at 1.19 m/s -> cannot stop, keep going.
  EXPECT_FALSE(d::canStopBefore(0.3, 1.19, 1.0, 0.3));
  EXPECT_FALSE(d::canStopBefore(0.3, -1.19, 1.0, 0.3));
  // standing a couple of metres before the line -> stop is still possible
  EXPECT_TRUE(d::canStopBefore(2.0, 0.5, 1.0, 0.3));
  EXPECT_TRUE(d::canStopBefore(2.0, -0.5, 1.0, 0.3));
  EXPECT_NEAR(d::stoppingDistance(-1.0, 1.0, 0.3), 0.8, 1e-9);
}

// ------------------------------------------------------------------ paired light

namespace
{
// Crossing 181 of the higashida map (lanelet2_map(31).osm), in its drawing direction.
d::Polyline crossing181Drawn()
{
  return {{0.0, 0.0}, {26.6, 0.0}};
}
const d::TaggedLight kLight892{892, d::Point2{26.1, 3.4}};   // forward tag, 0.98 of the drawing
const d::TaggedLight kLight1331{1331, d::Point2{2.1, -2.6}};  // reverse tag, 0.08 of the drawing
}  // namespace

TEST(Lights, TagFollowsTheSideCrossedFrom)
{
  EXPECT_STREQ(d::lightTagKey(false), "forward_pedestrian_tf");
  EXPECT_STREQ(d::lightTagKey(true), "reverse_pedestrian_tf");
}

TEST(Lights, BothDirectionsOfCrossing181PickTheFarLight)
{
  const auto drawn = crossing181Drawn();
  // along the drawing: forward tag, light 892 across the road
  const auto f = d::selectLights(drawn, {kLight892}, {kLight1331});
  EXPECT_EQ(f.choice, d::LightChoice::FROM_TAG);
  EXPECT_EQ(f.ids, std::vector<int64_t>{892});
  // against the drawing: reverse tag, light 1331 across the road
  const auto r = d::selectLights(reversed(drawn), {kLight1331}, {kLight892});
  EXPECT_EQ(r.choice, d::LightChoice::FROM_TAG);
  EXPECT_EQ(r.ids, std::vector<int64_t>{1331});
  // the far light ends up at the same fraction of the travel in both directions
  EXPECT_NEAR(*d::lightFraction(drawn, kLight892), *d::lightFraction(reversed(drawn), kLight1331), 0.15);
}

TEST(Lights, SwappedTagsAreCorrectedOnlyWhenClearCut)
{
  const auto drawn = crossing181Drawn();
  // tags swapped in the map: the forward tag names the light on ego's side
  const auto s = d::selectLights(drawn, {kLight1331}, {kLight892});
  EXPECT_EQ(s.choice, d::LightChoice::SWAPPED_TAGS);
  EXPECT_EQ(s.ids, std::vector<int64_t>{892});
  // kantor_luar crossing 335: the only light stands 24 m to the side, mid crossing --
  // ambiguous, the tag is followed (no swap, no change for forward driving there)
  const d::Polyline c335{{0.0, 0.0}, {3.7, 0.0}};
  const d::TaggedLight l342{342, d::Point2{1.8, 23.8}};
  const auto k = d::selectLights(c335, {l342}, {});
  EXPECT_EQ(k.choice, d::LightChoice::FROM_TAG);
  EXPECT_EQ(k.ids, std::vector<int64_t>{342});
  // light way unknown in the map (typo): follow the tag, never drop the light
  const d::TaggedLight unknown{999, std::nullopt};
  const auto u = d::selectLights(drawn, {unknown}, {kLight892});
  EXPECT_EQ(u.choice, d::LightChoice::FROM_TAG);
  EXPECT_EQ(u.ids, std::vector<int64_t>{999});
}

TEST(Lights, AMissingTagNeverMakesASignalizedCrossingUnsignalized)
{
  const auto drawn = crossing181Drawn();
  // reverse direction, only the forward tag in the map
  const auto m = d::selectLights(reversed(drawn), {}, {kLight892});
  EXPECT_EQ(m.choice, d::LightChoice::FROM_OTHER_TAG);
  EXPECT_FALSE(m.ids.empty());
  // no tag at all: unsignalized, as before
  const auto n = d::selectLights(drawn, {}, {});
  EXPECT_EQ(n.choice, d::LightChoice::NONE);
  EXPECT_TRUE(n.ids.empty());
}
