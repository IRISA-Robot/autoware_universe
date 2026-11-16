// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "scene.hpp"

#include "direction.hpp"
#include "stop_zone.hpp"

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/distance.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BoundingBox.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace cc = common_crossing;

namespace
{
const char * stateName(const CommonCrossingModule::State s)
{
  switch (s) {
    case CommonCrossingModule::State::APPROACHING:
      return "APPROACHING";
    case CommonCrossingModule::State::STOPPING:
      return "STOPPING";
    case CommonCrossingModule::State::CHECKING:
      return "CHECKING";
    case CommonCrossingModule::State::CROSSING:
      return "CROSSING";
    case CommonCrossingModule::State::DONE:
      return "DONE";
  }
  return "UNKNOWN";
}

const char * labelName(const uint8_t label)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  switch (label) {
    case ObjectClassification::CAR:
      return "CAR";
    case ObjectClassification::TRUCK:
      return "TRUCK";
    case ObjectClassification::BUS:
      return "BUS";
    case ObjectClassification::TRAILER:
      return "TRAILER";
    case ObjectClassification::MOTORCYCLE:
      return "MOTORCYCLE";
    case ObjectClassification::BICYCLE:
      return "BICYCLE";
    case ObjectClassification::PEDESTRIAN:
      return "PEDESTRIAN";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

CommonCrossingModule::CommonCrossingModule(
  const lanelet::Id module_id, const PlannerParam & planner_param,
  std::vector<cc::Polygon2d> areas, std::string area_source, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface,
  rclcpp::Node & node)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  planner_param_(planner_param),
  areas_(std::move(areas)),
  area_source_(std::move(area_source))
{
  // A ROS name token must not start with a digit: "lanelet_<id>".
  const std::string topic = std::string(node.get_fully_qualified_name()) +
                            "/common_crossing/lanelet_" +
                            std::to_string(static_cast<int64_t>(module_id)) + "/state";
  pub_state_ = node.create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    topic, rclcpp::QoS(1));

  RCLCPP_INFO(
    logger_, "CommonCrossing module for lanelet %ld: %zu check area(s) from %s",
    static_cast<int64_t>(module_id), areas_.size(), area_source_.c_str());
}

void CommonCrossingModule::resetTimers()
{
  blocked_since_.reset();
  clear_since_.reset();
}

void CommonCrossingModule::setState(const State next, const char * why)
{
  if (next == state_) return;
  RCLCPP_INFO(
    logger_, "CommonCrossing %ld: %s -> %s (%s)", getModuleId(), stateName(state_),
    stateName(next), why);
  state_ = next;
}

std::optional<lanelet::ConstLanelet> CommonCrossingModule::getCrossingLanelet(
  const PlannerData & planner_data) const
{
  if (!planner_data.route_handler_) return std::nullopt;
  const auto map = planner_data.route_handler_->getLaneletMapPtr();
  if (!map) return std::nullopt;
  try {
    return map->laneletLayer.get(static_cast<lanelet::Id>(getModuleId()));
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return std::nullopt;
  }
}

void CommonCrossingModule::updateGear(const Trajectory & path, const PlannerData & planner_data)
{
  const auto points = path.restore();
  if (points.empty() || !planner_data.current_odometry) return;
  const auto & ego = planner_data.current_odometry->pose.position;
  size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i].point.pose.position;
    const double d2 = (p.x - ego.x) * (p.x - ego.x) + (p.y - ego.y) * (p.y - ego.y);
    if (d2 < best) {
      best = d2;
      nearest = i;
    }
  }
  std::vector<double> velocities;
  velocities.reserve(points.size() - nearest);
  for (size_t i = nearest; i < points.size(); ++i) {
    velocities.push_back(points[i].point.longitudinal_velocity_mps);
  }
  if (const auto gear = cc::gearFromVelocities(velocities)) {
    gear_forward_ = *gear;
    gear_known_ = true;
  } else if (!gear_known_) {
    gear_forward_ = planner_data.is_driving_forward;
  }
}

double CommonCrossingModule::leadingEdge(const PlannerData & planner_data) const
{
  return cc::leadingEdgeOffset(
    planner_data.vehicle_info_.max_longitudinal_offset_m,
    planner_data.vehicle_info_.min_longitudinal_offset_m, gear_forward_);
}

void CommonCrossingModule::updateTravelDirection(
  const Trajectory & path, const lanelet::ConstLanelet & crossing)
{
  const auto centerline = crossing.centerline2d();
  if (centerline.size() < 2) return;
  const auto points = path.restore();
  if (points.size() < 2) return;
  const auto & f = centerline.front();
  const auto & bk = centerline.back();
  const double mx = 0.5 * (f.x() + bk.x());
  const double my = 0.5 * (f.y() + bk.y());
  size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i].point.pose.position;
    const double d2 = (p.x - mx) * (p.x - mx) + (p.y - my) * (p.y - my);
    if (d2 < best) {
      best = d2;
      nearest = i;
    }
  }
  const size_t i0 = nearest + 1 < points.size() ? nearest : nearest - 1;
  const auto & a = points[i0].point.pose.position;
  const auto & b = points[i0 + 1].point.pose.position;
  if (std::hypot(b.x - a.x, b.y - a.y) < 1e-6) return;
  travel_against_drawing_ =
    cc::travelAgainstDrawing(f.x(), f.y(), bk.x(), bk.y(), a.x, a.y, b.x, b.y);
}

std::optional<std::pair<lanelet::BasicPoint2d, lanelet::BasicPoint2d>>
CommonCrossingModule::crossingEntryExit(const lanelet::ConstLanelet & crossing) const
{
  const auto centerline = crossing.centerline2d();
  if (centerline.size() < 2) return std::nullopt;
  const lanelet::BasicPoint2d first{centerline.front().x(), centerline.front().y()};
  const lanelet::BasicPoint2d last{centerline.back().x(), centerline.back().y()};
  return travel_against_drawing_ ? std::make_pair(last, first) : std::make_pair(first, last);
}

bool CommonCrossingModule::isAreaBlocked(const PlannerData & planner_data)
{
  debug_data_.blocking.clear();
  if (!planner_data.predicted_objects || areas_.empty()) return false;
  for (const auto & obj : planner_data.predicted_objects->objects) {
    const uint8_t label = cc::topLabel(obj.classification);
    if (!cc::labelBlocks(label, planner_param_.block)) continue;
    const auto footprint = autoware_utils::to_polygon2d(obj);
    if (!cc::footprintInAreas(footprint, areas_)) continue;
    char id[9];
    std::snprintf(
      id, sizeof(id), "%02x%02x%02x%02x", obj.object_id.uuid[0], obj.object_id.uuid[1],
      obj.object_id.uuid[2], obj.object_id.uuid[3]);
    debug_data_.blocking.push_back({id, label, footprint});
  }
  return !debug_data_.blocking.empty();
}

std::string CommonCrossingModule::blockingSummary() const
{
  std::string s;
  for (const auto & b : debug_data_.blocking) {
    if (!s.empty()) s += ", ";
    s += std::string(labelName(b.label)) + " " + b.id;
  }
  return s.empty() ? "none" : s;
}

std::pair<double, std::optional<double>> CommonCrossingModule::findEgoAndStopPoint(
  const Trajectory & path, const lanelet::ConstLanelet & crossing,
  const PlannerData & planner_data, std::optional<double> * entry_s_out) const
{
  entry_s_out->reset();
  const double ego_s =
    autoware::experimental::trajectory::closest(path, planner_data.current_odometry->pose);

  // Entry = first point AHEAD of ego inside the crossing polygon (geometry only, refined on the
  // continuous path because knots can be metres apart).
  const auto polygon = crossing.polygon2d().basicPolygon();
  const auto path_pts = path.restore();
  std::optional<double> raw_entry_s;
  double prev_knot_s = ego_s;
  for (const auto & pt : path_pts) {
    const lanelet::BasicPoint2d p{pt.point.pose.position.x, pt.point.pose.position.y};
    const double s = autoware::experimental::trajectory::closest(path, pt.point.pose);
    if (boost::geometry::covered_by(p, polygon) && s > ego_s) {
      constexpr double kStep = 0.05;  // [m]
      double entry_s = s;
      for (double q = std::max(prev_knot_s, ego_s); q < s; q += kStep) {
        const auto & qp = path.compute(q).point.pose.position;
        if (boost::geometry::covered_by(lanelet::BasicPoint2d{qp.x, qp.y}, polygon)) {
          entry_s = q;
          break;
        }
      }
      raw_entry_s = entry_s;
      break;
    }
    prev_knot_s = s;
  }

  *entry_s_out = raw_entry_s;
  if (!raw_entry_s) return {ego_s, std::nullopt};

  const double stop_s = *raw_entry_s - (leadingEdge(planner_data) + planner_param_.stop_margin);
  if (stop_s < 0.0) return {ego_s, std::nullopt};
  return {ego_s, stop_s};
}

// Arc length where the path enters the pre lanelet: the lanelet the path runs on right before
// the crossing entry.  One tagged pre_common_crossing wins over an untagged one; with several of
// the same kind, the one the path runs on longest wins (as in the road_crossing module).
std::optional<double> CommonCrossingModule::findPreEntryS(
  const Trajectory & path, const double entry_s, const lanelet::ConstLanelet & crossing,
  const PlannerData & planner_data, std::string * why)
{
  if (!planner_data.route_handler_ || !planner_data.route_handler_->getLaneletMapPtr()) {
    *why = "no map";
    return std::nullopt;
  }
  const auto map = planner_data.route_handler_->getLaneletMapPtr();

  constexpr double kStep = 0.05;  // [m]
  const double probe_s = std::max(0.0, entry_s - kStep);
  const auto point_at = [&path](const double s) {
    const auto & p = path.compute(s).point.pose.position;
    return lanelet::BasicPoint2d{p.x, p.y};
  };
  const auto qp = point_at(probe_s);

  std::optional<double> best_tagged;
  std::optional<double> best_any;
  for (const auto & llt : map->laneletLayer.search(lanelet::BoundingBox2d(qp, qp))) {
    if (llt.id() == crossing.id()) continue;
    const auto polygon = llt.polygon2d().basicPolygon();
    if (!boost::geometry::covered_by(qp, polygon)) continue;
    double s = probe_s;
    while (s - kStep >= 0.0 && boost::geometry::covered_by(point_at(s - kStep), polygon)) {
      s -= kStep;
    }
    const bool tagged =
      std::string(llt.attributeOr(planner_param_.pre_tag.c_str(), "false")) == "true";
    auto & best = tagged ? best_tagged : best_any;
    if (!best || s < *best) best = s;
  }
  if (best_tagged) return best_tagged;
  if (best_any) {
    if (!warned_untagged_pre_) {
      warned_untagged_pre_ = true;
      RCLCPP_WARN(
        logger_,
        "CommonCrossing %ld: the lanelet right before the crossing is not tagged %s=true "
        "(used anyway)",
        getModuleId(), planner_param_.pre_tag.c_str());
    }
    return best_any;
  }
  *why = "no lanelet under the path right before the crossing entry";
  return std::nullopt;
}

void CommonCrossingModule::updateState(
  const double distance_to_stop_point, const bool is_vehicle_stopped, const bool blocked,
  const rclcpp::Time & now, const PlannerData & planner_data,
  const lanelet::ConstLanelet & crossing)
{
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const lanelet::BasicPoint2d ego2d{ego_pos.x, ego_pos.y};
  const auto polygon = crossing.polygon2d().basicPolygon();
  const bool ego_inside = boost::geometry::covered_by(ego2d, polygon);

  switch (state_) {
    case State::APPROACHING: {
      if (distance_to_stop_point < 0.0) {
        // No stop point: ego inside the crossing, already past it, or not reached on the path.
        if (ego_inside) {
          setState(State::CROSSING, "ego inside the crossing");
        } else if (const auto ends = crossingEntryExit(crossing)) {
          const auto & [entry_pt, exit_pt] = *ends;
          if (cc::pastExit(
                entry_pt.x(), entry_pt.y(), exit_pt.x(), exit_pt.y(), ego_pos.x, ego_pos.y)) {
            setState(State::DONE, "already past");
          }
        }
      } else if (distance_to_stop_point < planner_param_.stop_distance_threshold) {
        resetTimers();
        setState(State::STOPPING, "near the stop point");
      }
      break;
    }

    case State::STOPPING:
      if (
        distance_to_stop_point < planner_param_.stop_arrival_threshold && is_vehicle_stopped) {
        resetTimers();
        setState(State::CHECKING, "arrived, ego stopped");
      }
      break;

    case State::CHECKING: {
      if (blocked) {
        clear_since_.reset();
        if (!blocked_since_) blocked_since_ = now;
        const double waited = (now - *blocked_since_).seconds();
        debug_data_.waited = waited;
        if (waited >= planner_param_.wait_warn_sec) {
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 10000,
            "CommonCrossing %ld: still waiting after %.0f s (>= %.0f s) -- blocked by: %s",
            getModuleId(), waited, planner_param_.wait_warn_sec, blockingSummary().c_str());
        } else {
          RCLCPP_INFO_THROTTLE(
            logger_, *clock_, 2000, "CommonCrossing %ld: waiting %.1f s -- blocked by: %s",
            getModuleId(), waited, blockingSummary().c_str());
        }
      } else {
        blocked_since_.reset();
        if (!clear_since_) clear_since_ = now;
        const double clear_elapsed = (now - *clear_since_).seconds();
        debug_data_.clear_elapsed = clear_elapsed;
        if (clear_elapsed >= planner_param_.sustained_clear_sec) {
          setState(State::CROSSING, "area clear");
        }
      }
      break;
    }

    case State::CROSSING: {
      // Exit: was inside and base_link left, or projected past the exit.
      if (ego_inside) was_inside_polygon_ = true;
      bool past_exit = false;
      if (const auto ends = crossingEntryExit(crossing)) {
        const auto & [entry, exit] = *ends;
        past_exit = cc::pastExit(entry.x(), entry.y(), exit.x(), exit.y(), ego_pos.x, ego_pos.y);
      }
      if ((was_inside_polygon_ && !ego_inside) || past_exit) {
        setState(State::DONE, "ego left the crossing");
      }
      break;
    }

    case State::DONE: {
      // The crossing is ahead again (initial pose reset, looped route): start over.
      if (distance_to_stop_point > planner_param_.stop_arrival_threshold && !ego_inside) {
        resetTimers();
        was_inside_polygon_ = false;
        setState(State::APPROACHING, "crossing ahead again");
      }
      break;
    }
  }
}

bool CommonCrossingModule::modifyPathVelocity(
  Trajectory & path, [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & right_bound,
  const PlannerData & planner_data)
{
  updateGear(path, planner_data);
  debug_data_.base_link2front = leadingEdge(planner_data);
  debug_data_.ego_z = planner_data.current_odometry->pose.position.z;
  debug_data_.stop_pose.reset();
  debug_data_.clear_elapsed = 0.0;
  debug_data_.waited = 0.0;

  const auto crossing_opt = getCrossingLanelet(planner_data);
  if (!crossing_opt) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "CommonCrossing %ld: lanelet not in map; module is a no-op",
      getModuleId());
    return true;
  }
  const auto & crossing = *crossing_opt;
  updateTravelDirection(path, crossing);

  std::optional<double> entry_s;
  auto [ego_s, stop_s] = findEgoAndStopPoint(path, crossing, planner_data, &entry_s);

  // Stop ZONE instead of a stop LINE (stop_zone.hpp), as in the road_crossing module.
  std::string stop_detail = "common_crossing";
  if (planner_param_.stop_zone_enable && entry_s) {
    cc::StopZoneParam zp;
    zp.leading_edge_offset = debug_data_.base_link2front;
    zp.zone_length = planner_param_.stop_zone_length;
    zp.stop_margin = planner_param_.stop_margin;
    zp.hold_tolerance = planner_param_.stop_zone_hold_tolerance;
    zp.decel = planner_param_.stop_zone_decel;
    zp.pre_margin = planner_param_.stop_zone_pre_margin;
    zp.min_departure_dist = planner_param_.stop_zone_min_departure_dist;
    const double ego_speed = planner_data.current_velocity
      ? std::abs(planner_data.current_velocity->twist.linear.x) : 0.0;
    std::string pre_why;
    const auto pre_entry_s = findPreEntryS(path, *entry_s, crossing, planner_data, &pre_why);
    const bool slow = ego_speed < planner_param_.stop_state_speed;
    const auto zone = cc::computeStopZone(ego_s, *entry_s, pre_entry_s, ego_speed, slow, zp);
    stop_s = zone.stop_s;
    stop_detail = zone.mode == cc::StopZoneMode::HOLD_IN_ZONE ? "COMMON_CROSSING: hold_in_zone"
                                                              : "COMMON_CROSSING: barrier";
    if (state_ == State::STOPPING || state_ == State::CHECKING) {
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 5000,
        "CommonCrossing %ld stop zone: %s ego_s=%.2f entry_s=%.2f pre_entry_s=%s stop_s=%.2f "
        "(%.2f m ahead)%s%s",
        getModuleId(), stop_detail.c_str(), ego_s, *entry_s,
        pre_entry_s ? std::to_string(*pre_entry_s).c_str() : "none", zone.stop_s,
        zone.stop_s - ego_s, pre_why.empty() ? "" : " -- pre: ", pre_why.c_str());
    }
  }

  // The area is checked every cycle: CHECKING decides on it, and a released robot that has not
  // reached the crossing yet is held again when it gets blocked.
  const bool blocked = isAreaBlocked(planner_data);
  debug_data_.blocked = blocked;

  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const bool ego_inside = boost::geometry::covered_by(
    lanelet::BasicPoint2d{ego_pos.x, ego_pos.y}, crossing.polygon2d().basicPolygon());
  const bool entry_ahead_of_front =
    entry_s && *entry_s > ego_s + debug_data_.base_link2front;

  // Released (CROSSING) but not on the crossing yet, or DONE with the crossing ahead again, and
  // the area is blocked: hold again (road_crossing: gate back to HOLD before ego entered).
  const bool released = (state_ == State::CROSSING && !was_inside_polygon_) ||
                        state_ == State::DONE;
  if (cc::holdAgainBeforeEntry(released, ego_inside, entry_ahead_of_front, blocked)) {
    RCLCPP_WARN(
      logger_,
      "CommonCrossing %ld: area blocked (%s) and the crossing entry is %.2f m ahead of the "
      "front, state=%s -> APPROACHING (hold again)",
      getModuleId(), blockingSummary().c_str(), *entry_s - ego_s - debug_data_.base_link2front,
      stateName(state_));
    resetTimers();
    was_inside_polygon_ = false;
    state_ = State::APPROACHING;
  }

  // always_stop_at_pre=false: an area that stays clear while approaching lets the robot go on.
  if (
    !planner_param_.always_stop_at_pre &&
    (state_ == State::APPROACHING || state_ == State::STOPPING) && stop_s) {
    if (blocked) {
      clear_since_.reset();
    } else {
      if (!clear_since_) clear_since_ = clock_->now();
      if ((clock_->now() - *clear_since_).seconds() >= planner_param_.sustained_clear_sec) {
        setState(State::CROSSING, "area clear while approaching (always_stop_at_pre=false)");
      }
    }
  }

  const double dist_to_stop = stop_s ? (*stop_s - ego_s) : -1.0;
  updateState(
    dist_to_stop, planner_data.isVehicleStopped(), blocked, clock_->now(), planner_data,
    crossing);

  // Blocked while ego stands with its front already over the stop line but base_link not on the
  // crossing yet: no line ahead left to stop at, so hold where ego stands.
  bool hold_in_place = false;
  if (
    state_ == State::APPROACHING && (!stop_s || dist_to_stop < 0.0) &&
    planner_data.isVehicleStopped() && blocked) {
    const lanelet::BasicPoint2d ego2d{ego_pos.x, ego_pos.y};
    const auto polygon = crossing.polygon2d().basicPolygon();
    const bool past_line = !boost::geometry::covered_by(ego2d, polygon) &&
                           boost::geometry::distance(ego2d, polygon) <=
                             debug_data_.base_link2front + planner_param_.stop_margin + 0.1;
    hold_in_place = past_line;
    if (hold_in_place) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "CommonCrossing %ld: blocked, ego stopped past the stop line before the crossing -> "
        "holding in place",
        getModuleId());
    }
  }
  if (hold_in_place && ego_s < path.length()) {
    path.longitudinal_velocity_mps().range(ego_s, path.length()).set(0.0);
    planning_factor_interface_->add(
      path.restore(), planner_data.current_odometry->pose, path.compute(ego_s).point.pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, gear_forward_, 0.0, 0.0,
      "common_crossing");
    debug_data_.stop_pose = path.compute(ego_s).point.pose;
  }

  const bool should_stop = (state_ == State::STOPPING || state_ == State::CHECKING);
  if (should_stop && stop_s && *stop_s >= 0.0 && *stop_s < path.length()) {
    path.longitudinal_velocity_mps().range(*stop_s, path.length()).set(0.0);
    planning_factor_interface_->add(
      path.restore(), planner_data.current_odometry->pose, path.compute(*stop_s).point.pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, gear_forward_, 0.0, 0.0,
      stop_detail);
    debug_data_.stop_pose = path.compute(*stop_s).point.pose;
  }

  debug_data_.current_state = state_;
  if (pub_state_) {
    autoware_internal_debug_msgs::msg::Int32Stamped m;
    m.stamp = clock_->now();
    m.data = static_cast<int32_t>(state_);
    pub_state_->publish(m);
  }
  return true;
}

visualization_msgs::msg::MarkerArray CommonCrossingModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray msg;
  const auto now = clock_->now();
  const int32_t base_id = static_cast<int32_t>(getModuleId());
  const double z = debug_data_.ego_z + 0.2;
  const auto area_color = debug_data_.blocked
    ? autoware_utils::create_marker_color(1.0, 0.1, 0.1, 0.9)
    : autoware_utils::create_marker_color(0.1, 1.0, 0.1, 0.9);

  // Check areas: red while a vehicle touches one, green otherwise.
  int32_t i = 0;
  for (const auto & area : areas_) {
    auto m = autoware_utils::create_default_marker(
      "map", now, "common_crossing_area_" + std::to_string(base_id), i++,
      visualization_msgs::msg::Marker::LINE_STRIP, autoware_utils::create_marker_scale(0.15, 0.0, 0.0),
      area_color);
    for (const auto & p : area.outer()) {
      m.points.push_back(autoware_utils::create_point(p.x(), p.y(), z));
    }
    if (!area.outer().empty()) {
      const auto & p = area.outer().front();
      m.points.push_back(autoware_utils::create_point(p.x(), p.y(), z));
    }
    msg.markers.push_back(m);
  }

  // Vehicles that block: orange outline + label.
  i = 0;
  for (const auto & b : debug_data_.blocking) {
    auto m = autoware_utils::create_default_marker(
      "map", now, "common_crossing_blocking_" + std::to_string(base_id), i,
      visualization_msgs::msg::Marker::LINE_STRIP, autoware_utils::create_marker_scale(0.1, 0.0, 0.0),
      autoware_utils::create_marker_color(1.0, 0.6, 0.0, 0.999));
    for (const auto & p : b.footprint.outer()) {
      m.points.push_back(autoware_utils::create_point(p.x(), p.y(), z + 0.3));
    }
    msg.markers.push_back(m);
    cc::Point2d c;
    boost::geometry::centroid(b.footprint, c);
    auto t = autoware_utils::create_default_marker(
      "map", now, "common_crossing_blocking_text_" + std::to_string(base_id), i++,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.0, 0.0, 0.6),
      autoware_utils::create_marker_color(1.0, 0.6, 0.0, 0.999));
    t.pose.position = autoware_utils::create_point(c.x(), c.y(), z + 1.5);
    t.text = std::string(labelName(b.label)) + " " + b.id;
    msg.markers.push_back(t);
  }

  // State text above the first area (or the stop pose).
  geometry_msgs::msg::Point anchor;
  if (debug_data_.stop_pose) {
    anchor = debug_data_.stop_pose->position;
  } else if (!areas_.empty()) {
    cc::Point2d c;
    boost::geometry::centroid(areas_.front(), c);
    anchor = autoware_utils::create_point(c.x(), c.y(), z);
  } else {
    return msg;
  }
  auto t = autoware_utils::create_default_marker(
    "map", now, "common_crossing_state", base_id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    autoware_utils::create_marker_scale(0.0, 0.0, 0.8), area_color);
  t.pose.position = anchor;
  t.pose.position.z = z + 2.5;
  char buf[160];
  if (debug_data_.current_state == State::CHECKING && debug_data_.blocked) {
    std::snprintf(
      buf, sizeof(buf), "common_crossing[%ld]: CHECKING blocked %.1fs", getModuleId(),
      debug_data_.waited);
  } else if (debug_data_.current_state == State::CHECKING) {
    std::snprintf(
      buf, sizeof(buf), "common_crossing[%ld]: CHECKING clear %.1f/%.1fs", getModuleId(),
      debug_data_.clear_elapsed, planner_param_.sustained_clear_sec);
  } else {
    std::snprintf(
      buf, sizeof(buf), "common_crossing[%ld]: %s%s", getModuleId(),
      stateName(debug_data_.current_state), debug_data_.blocked ? " (area blocked)" : "");
  }
  t.text = buf;
  msg.markers.push_back(t);
  return msg;
}

autoware::motion_utils::VirtualWalls CommonCrossingModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls walls;
  if (debug_data_.stop_pose) {
    autoware::motion_utils::VirtualWall wall;
    wall.text = "common_crossing";
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.ns = std::to_string(getModuleId()) + "_";
    wall.pose = autoware_utils::calc_offset_pose(
      *debug_data_.stop_pose, debug_data_.base_link2front, 0.0, 0.0);
    walls.push_back(wall);
  }
  return walls;
}

}  // namespace autoware::behavior_velocity_planner::experimental
