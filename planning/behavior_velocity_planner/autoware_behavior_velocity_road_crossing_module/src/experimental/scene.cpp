// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_road_crossing_msgs/msg/road_crossing_gate.hpp>

#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

using autoware_perception_msgs::msg::ObjectClassification;

// Helper: both GO and GO_GREEN count as "go" for normal FSM gate progression.
// GO_GREEN additionally enables the green-block bounded-timeout (robot waits at most
// green_block_timeout_sec before crossing anyway while lampu HIJAU).
static inline bool isGoCommand(uint8_t c)
{
  return c == autoware_road_crossing_msgs::msg::RoadCrossingGate::GO ||
         c == autoware_road_crossing_msgs::msg::RoadCrossingGate::GO_GREEN;
}

RoadCrossingModule::RoadCrossingModule(
  const lanelet::Id module_id, CrossingKind kind, const PlannerParam & planner_param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface,
  rclcpp::Node & node)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  kind_(kind),
  planner_param_(planner_param)
{
  // Per-lanelet state topic so each crossing has its own line in PlotJuggler.
  // Topic: /<node-fully-qualified>/road_crossing/lanelet_<id>/state — Int8 enum.
  //
  // ROS 2 rule: each token between '/' must NOT start with a digit. The
  // lanelet ID alone (e.g. "1980") violates that, so we prefix with "lanelet_"
  // to make the token "lanelet_1980" — token now starts with 'l', valid.
  const std::string topic = std::string(node.get_fully_qualified_name()) +
                            "/road_crossing/lanelet_" +
                            std::to_string(static_cast<int64_t>(module_id)) + "/state";
  // Volatile QoS — transient_local replays last cached message at subscriber
  // connect time, which made PlotJuggler render the curve at stale sim time.
  pub_state_ = node.create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    topic, rclcpp::QoS(1));

  // NOTE: the FSM gate subscription is intentionally NOT created here.
  // Scene modules are created/destroyed dynamically as the route changes.
  // Creating a subscription inside a scene module corrupts the executor wait_set
  // (SIGABRT: "guard condition implementation is invalid").
  // gate_command_ is pushed each planning cycle via setGateCommand() by the
  // manager, which owns ONE persistent subscription for the lifetime of the node.

  RCLCPP_INFO(
    logger_, "RoadCrossing module initialized for lanelet %ld kind=%s (state topic: %s)",
    static_cast<int64_t>(module_id),
    kind_ == CrossingKind::PRE ? "PRE" : "ROAD",
    topic.c_str());
}

bool RoadCrossingModule::shouldBlockOnLabel(uint8_t label) const
{
  switch (label) {
    case ObjectClassification::CAR:
      return planner_param_.block_on_car;
    case ObjectClassification::TRUCK:
      return planner_param_.block_on_truck;
    case ObjectClassification::BUS:
      return planner_param_.block_on_bus;
    case ObjectClassification::TRAILER:
      return planner_param_.block_on_trailer;
    case ObjectClassification::MOTORCYCLE:
      return planner_param_.block_on_motorcycle;
    case ObjectClassification::BICYCLE:
      return planner_param_.block_on_bicycle;
    case ObjectClassification::PEDESTRIAN:
      return planner_param_.block_on_pedestrian;
    case ObjectClassification::UNKNOWN:
    default:
      return planner_param_.block_on_unknown;
  }
}

std::optional<lanelet::ConstLanelet> RoadCrossingModule::getCrossingLanelet(
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

double RoadCrossingModule::computeHorizon(const lanelet::ConstLanelet & crossing) const
{
  double horizon = planner_param_.predict_horizon_sec;
  if (planner_param_.enable_dynamic_horizon) {
    const double L = boost::geometry::length(crossing.centerline2d().basicLineString());
    const double v = std::max(planner_param_.crossing_speed_for_horizon, 1e-3);
    horizon = std::max(horizon, L / v);
  }
  return horizon;
}

bool RoadCrossingModule::isPredictedBlocked(
  const lanelet::ConstLanelet & crossing, const PlannerData & planner_data) const
{
  if (!planner_data.predicted_objects) return false;

  const auto polygon = crossing.polygon2d().basicPolygon();
  const double horizon = computeHorizon(crossing);

  for (const auto & obj : planner_data.predicted_objects->objects) {
    // Class filter — pick highest-probability classification
    bool block = false;
    for (const auto & cls : obj.classification) {
      if (shouldBlockOnLabel(cls.label)) {
        block = true;
        break;
      }
    }
    if (!block) continue;

    // Predicted path scan
    for (const auto & pred_path : obj.kinematics.predicted_paths) {
      if (pred_path.confidence < planner_param_.min_confidence) continue;

      const double dt = rclcpp::Duration(pred_path.time_step).seconds();
      for (size_t i = 0; i < pred_path.path.size(); ++i) {
        const double t = static_cast<double>(i) * dt;
        if (t > horizon) break;

        const lanelet::BasicPoint2d p{
          pred_path.path[i].position.x, pred_path.path[i].position.y};
        if (boost::geometry::within(p, polygon)) {
          return true;
        }
      }
    }
  }
  return false;
}

std::pair<double, std::optional<double>> RoadCrossingModule::findEgoAndStopPoint(
  const Trajectory & path,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & right_bound,
  const lanelet::ConstLanelet & crossing, const PlannerData & planner_data) const
{
  const double ego_s = autoware::experimental::trajectory::closest(
    path, planner_data.current_odometry->pose);

  // ENTRY DETECTION via point-in-polygon (robust against any boundary geometry):
  // Walk path points and find the FIRST one whose pose is inside the crossing
  // lanelet polygon. The s-coordinate of that point is the entry on the path.
  //
  // Earlier we tried "line crossing" with the boundary's front/back edges, but
  // that depends on the boundary being roughly perpendicular to ego direction
  // AND on path points around the intersection carrying the crossing's lane_id.
  // Point-in-polygon is geometry-only, no constraint matching required.
  const auto polygon = crossing.polygon2d().basicPolygon();
  const auto path_pts = path.restore();

  std::optional<double> raw_entry_s;
  for (const auto & pt : path_pts) {
    const lanelet::BasicPoint2d p{pt.point.pose.position.x, pt.point.pose.position.y};
    if (boost::geometry::covered_by(p, polygon)) {
      // Found a point inside the polygon. Get its s on the trajectory.
      const double s = autoware::experimental::trajectory::closest(path, pt.point.pose);
      if (s > ego_s) {  // must be ahead of ego
        raw_entry_s = s;
        break;
      }
    }
  }

  if (!raw_entry_s) {
    // No future entry into the crossing polygon on this path. Either ego is
    // already inside it, or already past, or path doesn't reach yet (still
    // upstream and behavior_path_planner hasn't extended trajectory through
    // the crossing yet). updateState() disambiguates these via ego-in-polygon
    // check.
    return {ego_s, std::nullopt};
  }

  const double base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;

  // For PRE instances, place the stop point so that the robot's front bumper
  // is pre_stop_dist_m BEFORE the polygon entry — keeping the robot in the
  // parent (fork) lanelet so an alternate-branch reroute is still reachable.
  // For ROAD instances, use the existing stop_margin gap (unchanged behaviour).
  const double clearance = (kind_ == CrossingKind::PRE)
    ? planner_param_.pre_stop_dist_m
    : planner_param_.stop_margin;

  const double stop_s = *raw_entry_s - (base_link2front + clearance);
  if (stop_s < 0.0) {
    // Stop point is behind ego (ego already very close to / past entry); skip
    return {ego_s, std::nullopt};
  }

  return {ego_s, stop_s};
}

void RoadCrossingModule::updateState(
  double distance_to_stop_point, bool is_vehicle_stopped, bool blocked,
  const rclcpp::Time & now, const PlannerData & planner_data,
  const lanelet::ConstLanelet & crossing)
{
  switch (state_) {
    case State::APPROACHING: {
      // Three sub-cases when stop_s is unavailable (dist == -1 sentinel):
      //   (a) ego inside polygon       → CROSSING
      //   (b) lanelet is upstream (entry not in path yet) → STAY APPROACHING
      //   (c) ego past polygon         → DONE
      const auto & ego_pos = planner_data.current_odometry->pose.position;
      const lanelet::BasicPoint2d ego2d{ego_pos.x, ego_pos.y};
      const auto polygon = crossing.polygon2d().basicPolygon();
      const bool ego_inside = boost::geometry::covered_by(ego2d, polygon);

      if (distance_to_stop_point < 0.0) {
        if (ego_inside) {
          state_ = State::CROSSING;
          RCLCPP_INFO(
            logger_, "RoadCrossing %ld: APPROACHING → CROSSING (ego inside polygon)",
            getModuleId());
        } else {
          // Check if any path point is inside polygon and AHEAD of ego.
          // If not, ego is past → DONE. Otherwise path hasn't reached yet → wait.
          // We approximate this by checking the centroid: project ego onto
          // lanelet centerline; if arc length > centerline length, past.
          const auto centerline = crossing.centerline2d().basicLineString();
          if (centerline.size() >= 2) {
            const auto & exit_pt = centerline.back();
            const double dx = ego_pos.x - exit_pt.x();
            const double dy = ego_pos.y - exit_pt.y();
            const auto & entry_pt = centerline.front();
            const double cdx = exit_pt.x() - entry_pt.x();
            const double cdy = exit_pt.y() - entry_pt.y();
            // Project (ego - exit) onto centerline direction. Positive = past exit.
            const double proj = dx * cdx + dy * cdy;
            if (proj > 0.0) {
              state_ = State::DONE;
              RCLCPP_INFO(
                logger_, "RoadCrossing %ld: APPROACHING → DONE (already past)",
                getModuleId());
            }
            // else: stay APPROACHING — path will reach lanelet eventually
          }
        }
      } else if (distance_to_stop_point < planner_param_.stop_distance_threshold) {
        state_ = State::STOPPING;
        green_blocked_since_.reset();
        RCLCPP_INFO(
          logger_, "RoadCrossing %ld: APPROACHING → STOPPING (dist %.2f m)",
          getModuleId(), distance_to_stop_point);
      }
      break;
    }

    case State::STOPPING:
      if (
        distance_to_stop_point < planner_param_.stop_arrival_threshold && is_vehicle_stopped) {
        state_ = State::CHECKING;
        stopped_at_ = now;
        clear_since_.reset();
        green_blocked_since_.reset();
        RCLCPP_INFO(
          logger_, "RoadCrossing %ld: STOPPING → CHECKING (arrived, ego stopped)",
          getModuleId());
      }
      break;

    case State::CHECKING: {
      // Read latest gate command (thread-safe).
      uint8_t current_gate = autoware_road_crossing_msgs::msg::RoadCrossingGate::GO;
      if (planner_param_.require_fsm_gate) {
        std::lock_guard<std::mutex> lk(gate_mutex_);
        current_gate = gate_command_;
      }

      // Gate ABORT: treat as indefinite hold (don't cross; don't timeout-warn either).
      if (planner_param_.require_fsm_gate &&
          current_gate == autoware_road_crossing_msgs::msg::RoadCrossingGate::ABORT)
      {
        clear_since_.reset();
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "RoadCrossing %ld: FSM gate=ABORT — holding at crossing", getModuleId());
        break;
      }

      // ---- PRE mode: no vehicle scan — just wait for FSM gate GO or GO_GREEN ----
      if (kind_ == CrossingKind::PRE) {
        if (planner_param_.require_fsm_gate && !isGoCommand(current_gate))
        {
          RCLCPP_INFO_THROTTLE(
            logger_, *clock_, 2000,
            "RoadCrossing %ld [PRE]: waiting for FSM crosswalk gate=GO",
            getModuleId());
        } else {
          // Gate is GO or GO_GREEN (or require_fsm_gate is somehow false — shouldn't happen for PRE).
          state_ = State::CROSSING;
          RCLCPP_INFO(
            logger_, "RoadCrossing %ld [PRE]: CHECKING → CROSSING (crosswalk gate=GO/GO_GREEN)",
            getModuleId());
        }
        break;
      }

      // ---- ROAD mode: vehicle-scan logic with GO_GREEN bounded timeout ----

      // GO_GREEN path: keep predicted-path safety block, but bounded by green_block_timeout_sec.
      // Fires ONLY when require_fsm_gate=true and the FSM confirmed the pedestrian light is GREEN.
      // Plain GO (disable_tf_light_check bypass, light-not-detected timeout, or unsignalized)
      // does NOT enter this branch — those fall through to the sustained-clear logic below.
      if (planner_param_.require_fsm_gate &&
          current_gate == autoware_road_crossing_msgs::msg::RoadCrossingGate::GO_GREEN) {
        if (!blocked) {
          // Path is clear — cross immediately. Reset stale timer.
          green_blocked_since_.reset();
          state_ = State::CROSSING;
          RCLCPP_INFO(
            logger_,
            "RoadCrossing %ld [ROAD]: GREEN + clear → CROSSING",
            getModuleId());
        } else {
          // Path still blocked while GREEN. Start (or continue) the timeout clock.
          if (!green_blocked_since_) green_blocked_since_ = now;
          const double waited = (now - *green_blocked_since_).seconds();
          if (waited >= planner_param_.green_block_timeout_sec) {
            // Timeout expired — cross anyway. Lampu HIJAU = hak jalan; jangan deadlock selamanya.
            // Collision safety for actually-present obstacles still handled by obstacle_stop /
            // dynamic_obstacle_stop (same as normal driving).
            state_ = State::CROSSING;
            RCLCPP_WARN(
              logger_,
              "RoadCrossing %ld [ROAD]: GREEN blocked %.1fs >= timeout %.1fs → CROSSING anyway",
              getModuleId(), waited, planner_param_.green_block_timeout_sec);
          } else {
            RCLCPP_INFO_THROTTLE(
              logger_, *clock_, 1000,
              "RoadCrossing %ld [ROAD]: GREEN but vehicle predicted-block — waiting %.1f/%.1fs",
              getModuleId(), waited, planner_param_.green_block_timeout_sec);
          }
        }
        break;
      }

      // Plain GO / no-gate path: stale green timer has no meaning here — reset it.
      green_blocked_since_.reset();

      if (blocked) {
        clear_since_.reset();
        if (stopped_at_ && (now - *stopped_at_).seconds() > planner_param_.timeout_sec) {
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 5000,
            "RoadCrossing %ld: BLOCKED for >%.1f s — vehicle predictions still cross",
            getModuleId(), planner_param_.timeout_sec);
        }
      } else {
        if (!clear_since_) clear_since_ = now;
        const double clear_elapsed = (now - *clear_since_).seconds();
        if (clear_elapsed >= planner_param_.sustained_clear_sec) {
          // When require_fsm_gate=true, additionally require gate GO or GO_GREEN.
          if (planner_param_.require_fsm_gate && !isGoCommand(current_gate))
          {
            RCLCPP_INFO_THROTTLE(
              logger_, *clock_, 2000,
              "RoadCrossing %ld: vehicle clear (%.2f s) but FSM gate=HOLD — waiting for GO",
              getModuleId(), clear_elapsed);
          } else {
            state_ = State::CROSSING;
            RCLCPP_INFO(
              logger_, "RoadCrossing %ld: CHECKING → CROSSING (clear %.2f s%s)",
              getModuleId(), clear_elapsed,
              planner_param_.require_fsm_gate ? ", gate=GO/GO_GREEN" : "");
          }
        }
      }
      break;
    }

    case State::CROSSING: {
      // Exit detection — robust against the case where base_link never enters
      // the polygon (narrow crossing relative to vehicle wheelbase).
      // Two ways to fire DONE:
      //  (1) was_inside_polygon_ true (ever entered) AND base_link now outside.
      //  (2) Fallback: ego projected past the lanelet's exit on its centerline.
      const auto & ego_pos = planner_data.current_odometry->pose.position;
      const lanelet::BasicPoint2d ego2d{ego_pos.x, ego_pos.y};
      const auto polygon = crossing.polygon2d().basicPolygon();
      const bool inside_now = boost::geometry::covered_by(ego2d, polygon);

      if (inside_now) {
        was_inside_polygon_ = true;
      }

      bool past_exit = false;
      const auto centerline = crossing.centerline2d().basicLineString();
      if (centerline.size() >= 2) {
        const auto & entry = centerline.front();
        const auto & exit = centerline.back();
        const double cdx = exit.x() - entry.x();
        const double cdy = exit.y() - entry.y();
        // Vector from exit point to ego, dotted with (entry→exit) direction.
        // Positive = ego past exit (in lanelet direction).
        const double proj = (ego_pos.x - exit.x()) * cdx + (ego_pos.y - exit.y()) * cdy;
        past_exit = proj > 0.0;
      }

      if ((was_inside_polygon_ && !inside_now) || past_exit) {
        state_ = State::DONE;
        RCLCPP_INFO(
          logger_,
          "RoadCrossing %ld: CROSSING → DONE (ego exited, was_inside=%d, past_exit=%d)",
          getModuleId(), was_inside_polygon_, past_exit);
      }
      break;
    }

    case State::DONE:
      break;
  }
}

bool RoadCrossingModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  debug_data_.base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  debug_data_.current_state = state_;
  debug_data_.stop_pose.reset();

  const auto crossing_opt = getCrossingLanelet(planner_data);
  if (!crossing_opt) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "RoadCrossing %ld: lanelet not in map; module is a no-op", getModuleId());
    return true;
  }
  const auto & crossing = *crossing_opt;

  // Geometry: ego s + stop point s on path.
  auto [ego_s, stop_s] = findEgoAndStopPoint(path, left_bound, right_bound, crossing, planner_data);

  // Obstacle scan (only relevant in CHECKING state but we compute for transition logic).
  // PRE mode never scans vehicles — its CHECKING only waits for the FSM crosswalk gate.
  bool blocked = false;
  if (state_ == State::CHECKING && kind_ == CrossingKind::ROAD) {
    blocked = isPredictedBlocked(crossing, planner_data);
    debug_data_.last_check_blocked = blocked;
  } else {
    debug_data_.last_check_blocked = false;
  }

  // Update state machine
  const double dist_to_stop = stop_s ? (*stop_s - ego_s) : -1.0;
  updateState(
    dist_to_stop, planner_data.isVehicleStopped(), blocked, clock_->now(), planner_data, crossing);

  // Insert stop velocity if in STOPPING or CHECKING state.
  const bool should_stop = (state_ == State::STOPPING || state_ == State::CHECKING);
  if (should_stop && stop_s && *stop_s > 0.0 && *stop_s < path.length()) {
    path.longitudinal_velocity_mps().range(*stop_s, path.length()).set(0.0);

    planning_factor_interface_->add(
      path.restore(), planner_data.current_odometry->pose,
      path.compute(*stop_s).point.pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/,
      0.0, 0.0 /*shift distance*/, "road_crossing");

    debug_data_.stop_pose = path.compute(*stop_s).point.pose;
  }

  // Publish current state (Int32Stamped) every tick so PlotJuggler shows a
  // continuous timeline rather than discrete state-change pings.
  // 0=APPROACHING, 1=STOPPING, 2=CHECKING, 3=CROSSING, 4=DONE
  if (pub_state_) {
    autoware_internal_debug_msgs::msg::Int32Stamped m;
    m.stamp = clock_->now();
    m.data = static_cast<int32_t>(state_);
    pub_state_->publish(m);
  }

  return true;
}

visualization_msgs::msg::MarkerArray RoadCrossingModule::createDebugMarkerArray()
{
  return visualization_msgs::msg::MarkerArray{};
}

autoware::motion_utils::VirtualWalls RoadCrossingModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls walls;
  if (debug_data_.stop_pose &&
      (state_ == State::STOPPING || state_ == State::CHECKING)) {
    autoware::motion_utils::VirtualWall wall;
    wall.text = "road_crossing";
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.ns = std::to_string(getModuleId()) + "_";
    wall.pose = autoware_utils::calc_offset_pose(
      *debug_data_.stop_pose, debug_data_.base_link2front, 0.0, 0.0);
    walls.push_back(wall);
  }
  return walls;
}

}  // namespace autoware::behavior_velocity_planner::experimental
