// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef EXPERIMENTAL__SCENE_HPP_
#define EXPERIMENTAL__SCENE_HPP_

#define EIGEN_MPL2_ONLY

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_road_crossing_msgs/msg/road_crossing_gate.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

class RoadCrossingModule : public SceneModuleInterface
{
public:
  // CrossingKind is determined from the lanelet tag at construction time:
  //   PRE  — lanelet tagged pre_road_crossing:true
  //          State machine: APPROACHING → STOPPING → CHECKING (no vehicle scan,
  //          wait only for FSM gate==GO) → CROSSING/DONE
  //   ROAD — lanelet tagged road_crossing:true (existing behaviour)
  //          State machine: existing (vehicle scan + gate if require_fsm_gate)
  enum class CrossingKind { ROAD, PRE };

  enum class State { APPROACHING, STOPPING, CHECKING, CROSSING, DONE };

  struct PlannerParam
  {
    // State machine
    double stop_distance_threshold{5.0};
    double stop_arrival_threshold{0.2};
    double stop_state_speed{0.05};
    double sustained_clear_sec{1.5};
    // Plain-GO / bypass path bounded timeout (mirrors green_block_timeout_sec below, but
    // for the non-signalized / bypass GO path): after this many seconds stuck `blocked`
    // in CHECKING, force CHECKING → CROSSING anyway so a stale/false-positive block can't
    // deadlock the module forever. Also still used as the WARN_THROTTLE threshold.
    double timeout_sec{30.0};

    // Prediction check
    double predict_horizon_sec{5.0};
    double min_confidence{0.5};
    bool enable_dynamic_horizon{true};
    double crossing_speed_for_horizon{0.833};

    // Object class filter (which labels block the crossing)
    bool block_on_car{true};
    bool block_on_truck{true};
    bool block_on_bus{true};
    bool block_on_trailer{true};
    bool block_on_motorcycle{true};
    bool block_on_bicycle{false};
    bool block_on_pedestrian{false};
    bool block_on_unknown{false};

    // Stop margin
    double stop_margin{0.5};  // m, gap between ego front and stop line (used by ROAD instances)

    // PRE instance stop offset: stop this far BEFORE the pre_road_crossing lanelet entry.
    // base_link2front is also subtracted so the robot's physical front stays pre_stop_dist_m
    // ahead of the polygon entry.  Default 1.0 m keeps the robot inside the parent lanelet
    // (fork point) so a reroute to the alternate branch is still possible.
    double pre_stop_dist_m{1.0};

    // FSM gate integration
    // When true: CHECKING → CROSSING only if FSM gate == GO for this module_id.
    // When false (default): original behaviour, backward compatible.
    bool require_fsm_gate{false};
    std::string gate_topic{"/planning/road_crossing/gate"};

    // GREEN-cross bounded timeout (ROAD mode, signalized crossing only).
    // Saat gate==GO_GREEN dan predicted-path kendaraan nge-block crossing, robot menunggu
    // maksimal green_block_timeout_sec sebelum nyebrang anyway (lampu HIJAU = hak jalan,
    // jangan deadlock selamanya). 0.0 = langsung nyebrang saat hijau tanpa nunggu (old behavior).
    // Plain GO (bypass/timeout/unsignalized) TIDAK menggunakan timeout ini — tetap pakai
    // sustained-clear normal.
    double green_block_timeout_sec{5.0};

    // Gate-wait watchdog (plain-GO / sustained-clear path, require_fsm_gate=true only).
    // Saat vehicle-path sudah clear (sustained_clear_sec terpenuhi) tapi FSM gate masih HOLD,
    // sebelumnya modul ini menunggu selamanya tanpa jalan keluar — kalau FSM macet, robot ikut
    // macet permanen. Setelah gate_wait_timeout_sec menunggu dalam kondisi clear-tapi-HOLD ini,
    // paksa CHECKING → CROSSING (ERROR log, bukan WARN, karena ini indikasi FSM itu sendiri
    // yang stuck, bukan kondisi normal). Tidak berlaku untuk gate ABORT (tetap indefinite hold)
    // maupun GO_GREEN (punya timeout sendiri: green_block_timeout_sec).
    double gate_wait_timeout_sec{45.0};
  };

  struct DebugData
  {
    double base_link2front{0.0};
    std::optional<geometry_msgs::msg::Pose> stop_pose;
    State current_state{State::APPROACHING};
    bool last_check_blocked{false};
  };

  RoadCrossingModule(
    const lanelet::Id module_id, CrossingKind kind, const PlannerParam & planner_param,
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

  /**
   * Called by RoadCrossingModuleManager each planning cycle (before modifyPathVelocity)
   * to push the latest gate command from the manager-owned subscription into this module.
   * Thread-safe: acquires gate_mutex_ internally.
   */
  void setGateCommand(uint8_t cmd)
  {
    std::lock_guard<std::mutex> lk(gate_mutex_);
    gate_command_ = cmd;
  }

private:
  CrossingKind kind_{CrossingKind::ROAD};
  PlannerParam planner_param_;
  State state_{State::APPROACHING};
  std::optional<rclcpp::Time> stopped_at_;
  std::optional<rclcpp::Time> clear_since_;
  // GO_GREEN bounded-timeout: timestamp when robot first got blocked while gate==GO_GREEN.
  // Reset when gate is no longer GO_GREEN, when clear, or when entering CROSSING/STOPPING.
  std::optional<rclcpp::Time> green_blocked_since_;
  // Plain-GO bounded-timeout (mirrors green_blocked_since_ above): timestamp when robot
  // first got blocked on the plain-GO / bypass path (require_fsm_gate=false, GO, or
  // disable_tf_light_check bypass/timeout). Reset when gate enters GO_GREEN, when clear,
  // or when entering CROSSING/STOPPING. Prevents an indefinite deadlock if `blocked` gets
  // stuck true (e.g. stale/false-positive predicted-object block) with no real obstacle.
  std::optional<rclcpp::Time> go_blocked_since_;
  // Gate-wait watchdog: timestamp when the vehicle-path first became clear (sustained_clear_sec
  // satisfied) while FSM gate is still HOLD (require_fsm_gate=true). Reset whenever the gate
  // becomes GO/GO_GREEN, whenever the clear condition is lost, or when entering CROSSING/STOPPING.
  // See PlannerParam::gate_wait_timeout_sec for why this exists.
  std::optional<rclcpp::Time> gate_wait_since_;
  bool was_inside_polygon_{false};  // CROSSING exit detection: ever entered polygon?
  DebugData debug_data_;

  // Per-scene state publisher for PlotJuggler. Topic:
  //   <node>/road_crossing/lanelet_<id>/state   (autoware_internal_debug_msgs/Int32Stamped)
  // Has a `stamp` field so PlotJuggler aligns this to sim time like every other
  // autoware topic. Value = enum State {APPROACHING=0, STOPPING=1, CHECKING=2,
  // CROSSING=3, DONE=4}
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_state_;

  // FSM gate integration.
  // gate_command_ is written by RoadCrossingModuleManager::modifyPathVelocity()
  // (via setGateCommand()) each planning cycle, and read here in CHECKING state.
  // The subscription that actually receives gate messages lives in the MANAGER,
  // not here — modules must not own subscriptions (wait_set corruption on dynamic
  // create/destroy in a multi-threaded executor).
  mutable std::mutex gate_mutex_;
  // 0=HOLD (default when no gate received), 1=GO, 2=ABORT
  uint8_t gate_command_{autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD};

  /** Get the road_crossing lanelet from the map by module_id_. Returns empty optional if missing. */
  std::optional<lanelet::ConstLanelet> getCrossingLanelet(const PlannerData & planner_data) const;

  /** Compute the dynamic prediction horizon based on crossing length. */
  double computeHorizon(const lanelet::ConstLanelet & crossing) const;

  /** Return true if any tracked vehicle's predicted path enters the crossing polygon within horizon. */
  bool isPredictedBlocked(
    const lanelet::ConstLanelet & crossing, const PlannerData & planner_data) const;

  /** Class filter: should an object with this label block? */
  bool shouldBlockOnLabel(uint8_t label) const;

  /** Find ego s-coordinate and stop point s-coordinate on the trajectory. */
  std::pair<double, std::optional<double>> findEgoAndStopPoint(
    const Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const lanelet::ConstLanelet & crossing, const PlannerData & planner_data) const;

  /** Update state machine based on current geometry + obstacle scan. */
  void updateState(
    double distance_to_stop_point, bool is_vehicle_stopped, bool blocked, const rclcpp::Time & now,
    const PlannerData & planner_data, const lanelet::ConstLanelet & crossing);
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__SCENE_HPP_
