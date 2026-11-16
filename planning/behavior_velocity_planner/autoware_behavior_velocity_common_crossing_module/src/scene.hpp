// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef SCENE_HPP_
#define SCENE_HPP_

#define EIGEN_MPL2_ONLY

#include "area_check.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

// One instance per lanelet tagged common_crossing:true on the path (a sidewalk segment that
// crosses a car lane, e.g. a parking lot exit).  The robot stops on the lanelet before it (the
// pre, tagged pre_common_crossing:true) and goes only once no vehicle has touched the check
// area(s) for sustained_clear_sec.  It waits for as long as it takes.  Once it has entered the
// crossing it does not stop for this module again.
//
// Structure and stop handling copied from the road_crossing module (experimental), with the FSM
// gate replaced by the area check: "gate HOLD" there is "area blocked" here.
class CommonCrossingModule : public SceneModuleInterface
{
public:
  enum class State { APPROACHING, STOPPING, CHECKING, CROSSING, DONE };

  struct PlannerParam
  {
    double stop_distance_threshold{5.0};
    double stop_arrival_threshold{0.2};
    double stop_state_speed{0.05};
    double sustained_clear_sec{1.5};
    // Waiting this long only logs a WARN (repeated); the robot never goes on its own.
    double wait_warn_sec{120.0};
    // true: always stop at the pre and check.  false: an area clear for sustained_clear_sec
    // while approaching lets the robot cross without stopping.
    bool always_stop_at_pre{true};

    common_crossing::BlockFilter block;

    double stop_margin{0.5};
    bool stop_zone_enable{true};
    double stop_zone_length{2.0};
    double stop_zone_hold_tolerance{1.5};
    double stop_zone_decel{1.0};
    double stop_zone_pre_margin{0.5};
    double stop_zone_min_departure_dist{1.5};

    // Lanelet tags.
    std::string pre_tag{"pre_common_crossing"};
  };

  struct BlockingObject
  {
    std::string id;  // first 4 bytes of the uuid, hex
    uint8_t label{0};
    common_crossing::Polygon2d footprint;
  };

  struct DebugData
  {
    double base_link2front{0.0};
    double ego_z{0.0};
    std::optional<geometry_msgs::msg::Pose> stop_pose;
    State current_state{State::APPROACHING};
    bool blocked{false};
    double clear_elapsed{0.0};
    double waited{0.0};
    std::vector<BlockingObject> blocking;
  };

  CommonCrossingModule(
    const lanelet::Id module_id, const PlannerParam & planner_param,
    std::vector<common_crossing::Polygon2d> areas, std::string area_source,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface,
    rclcpp::Node & node);

  bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  PlannerParam planner_param_;
  std::vector<common_crossing::Polygon2d> areas_;
  std::string area_source_;
  State state_{State::APPROACHING};
  std::optional<rclcpp::Time> blocked_since_;
  std::optional<rclcpp::Time> clear_since_;
  bool was_inside_polygon_{false};
  bool warned_untagged_pre_{false};
  DebugData debug_data_;

  // <node>/common_crossing/lanelet_<id>/state (Int32Stamped): APPROACHING=0 .. DONE=4.
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_state_;

  bool gear_forward_{true};
  bool gear_known_{false};
  bool travel_against_drawing_{false};

  std::optional<lanelet::ConstLanelet> getCrossingLanelet(const PlannerData & planner_data) const;
  void updateGear(const Trajectory & path, const PlannerData & planner_data);
  double leadingEdge(const PlannerData & planner_data) const;
  void updateTravelDirection(const Trajectory & path, const lanelet::ConstLanelet & crossing);
  std::optional<std::pair<lanelet::BasicPoint2d, lanelet::BasicPoint2d>> crossingEntryExit(
    const lanelet::ConstLanelet & crossing) const;

  // Vehicles touching the check area(s) right now (fills debug_data_.blocking).
  bool isAreaBlocked(const PlannerData & planner_data);

  std::pair<double, std::optional<double>> findEgoAndStopPoint(
    const Trajectory & path, const lanelet::ConstLanelet & crossing,
    const PlannerData & planner_data, std::optional<double> * entry_s_out) const;

  std::optional<double> findPreEntryS(
    const Trajectory & path, double entry_s, const lanelet::ConstLanelet & crossing,
    const PlannerData & planner_data, std::string * why);

  void updateState(
    double distance_to_stop_point, bool is_vehicle_stopped, bool blocked,
    const rclcpp::Time & now, const PlannerData & planner_data,
    const lanelet::ConstLanelet & crossing);

  void resetTimers();
  void setState(State next, const char * why);
  std::string blockingSummary() const;
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // SCENE_HPP_
