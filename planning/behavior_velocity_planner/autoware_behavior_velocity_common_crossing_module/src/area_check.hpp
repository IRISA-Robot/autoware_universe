// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef AREA_CHECK_HPP_
#define AREA_CHECK_HPP_

// Plain logic of the common_crossing module (tested in test/test_area_check.cpp):
//
//   * which object labels block the crossing,
//   * whether an object footprint touches one of the check areas,
//   * "clear for sustained_clear_sec" timing,
//   * when a released robot that has not entered the crossing yet is held again.

#include <autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental::common_crossing
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;

// Labels that block the crossing.  Default: every vehicle, not bicycles, pedestrians or UNKNOWN.
struct BlockFilter
{
  bool car{true};
  bool truck{true};
  bool bus{true};
  bool trailer{true};
  bool motorcycle{true};
  bool bicycle{false};
  bool pedestrian{false};
  bool unknown{false};
};

inline bool labelBlocks(const uint8_t label, const BlockFilter & f)
{
  switch (label) {
    case ObjectClassification::CAR:
      return f.car;
    case ObjectClassification::TRUCK:
      return f.truck;
    case ObjectClassification::BUS:
      return f.bus;
    case ObjectClassification::TRAILER:
      return f.trailer;
    case ObjectClassification::MOTORCYCLE:
      return f.motorcycle;
    case ObjectClassification::BICYCLE:
      return f.bicycle;
    case ObjectClassification::PEDESTRIAN:
      return f.pedestrian;
    case ObjectClassification::UNKNOWN:
    default:
      return f.unknown;
  }
}

// Label with the highest probability; UNKNOWN when there is none.
inline uint8_t topLabel(const std::vector<ObjectClassification> & classes)
{
  uint8_t label = ObjectClassification::UNKNOWN;
  float best = -1.0F;
  for (const auto & c : classes) {
    if (c.probability > best) {
      best = c.probability;
      label = c.label;
    }
  }
  return label;
}

// The footprint touches (overlaps or crosses the border of) one of the areas.
inline bool footprintInAreas(const Polygon2d & footprint, const std::vector<Polygon2d> & areas)
{
  for (const auto & area : areas) {
    if (boost::geometry::intersects(footprint, area)) return true;
  }
  return false;
}

// Time the check area has been clear without interruption.  One blocked cycle restarts it.
class ClearTimer
{
public:
  void reset() { clear_since_.reset(); }

  // Seconds clear so far (0 while blocked).
  double update(const bool blocked, const double now_sec)
  {
    if (blocked) {
      clear_since_.reset();
      return 0.0;
    }
    if (!clear_since_) clear_since_ = now_sec;
    return now_sec - *clear_since_;
  }

private:
  std::optional<double> clear_since_;
};

// A robot released to cross (CROSSING) but not on the crossing yet -- never inside its polygon,
// front still before the entry -- is held again when the area is blocked.  Same rule as the
// road_crossing module with its FSM gate going back to HOLD; once the robot has entered it goes
// on whatever happens.  DONE with the crossing ahead again (relocalized, looped route) is armed
// again the same way.
inline bool holdAgainBeforeEntry(
  const bool released, const bool entered, const bool entry_ahead_of_front, const bool blocked)
{
  return released && !entered && entry_ahead_of_front && blocked;
}

}  // namespace autoware::behavior_velocity_planner::experimental::common_crossing

#endif  // AREA_CHECK_HPP_
