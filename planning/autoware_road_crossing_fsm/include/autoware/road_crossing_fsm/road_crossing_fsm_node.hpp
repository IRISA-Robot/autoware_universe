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

#ifndef AUTOWARE__ROAD_CROSSING_FSM__ROAD_CROSSING_FSM_NODE_HPP_
#define AUTOWARE__ROAD_CROSSING_FSM__ROAD_CROSSING_FSM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

// AdAPI
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>

#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_active_route_window_msgs/msg/active_route_window.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_road_crossing_msgs/msg/pedestrian_light_state.hpp>
#include <autoware_road_crossing_msgs/msg/pedestrian_light_target.hpp>
#include <autoware_road_crossing_msgs/msg/road_crossing_gate.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace autoware::road_crossing_fsm
{

// ---------------------------------------------------------------------------
// FSM state enum (published as Int32Stamped for PlotJuggler)
//
// Values are stable — do NOT renumber existing states.
//
//   IDLE             = 0
//   APPROACH_PRE     = 1   robot approaching crossing; holding until ego is at the pre
//   AT_PRE           = 2   ego at the pre; one-tick intermediate
//   REROUTING_TO_ALT = 3   Phase A: send robot to alternate crossing
//   ARMED_LIGHT      = 4   at road_crossing; waiting for pedestrian light
//   WAIT_GREEN       = 5   pedestrian light seen; waiting for GREEN
//   CROSSING         = 6   robot traversing the road_crossing lanelet
//   DONE             = 7   episode complete; will reset to IDLE next tick
//   REROUTING_TO_GOAL= 8   Phase B: restore original goal after alt reached
// ---------------------------------------------------------------------------
enum class FsmState : int32_t
{
  IDLE             = 0,
  APPROACH_PRE     = 1,
  AT_PRE           = 2,
  REROUTING_TO_ALT = 3,
  ARMED_LIGHT      = 4,
  WAIT_GREEN       = 5,
  CROSSING         = 6,
  DONE             = 7,
  REROUTING_TO_GOAL = 8,
};

// ---------------------------------------------------------------------------
// Main node
// ---------------------------------------------------------------------------
class RoadCrossingFsmNode : public rclcpp::Node
{
public:
  explicit RoadCrossingFsmNode(const rclcpp::NodeOptions & options);

private:
  // ------------------------------------------------------------------
  // Parameters (loaded once in ctor)
  // ------------------------------------------------------------------
  struct Param
  {
    double update_rate_hz{10.0};
    // Interlock lampu hijau di WAIT_GREEN (interlock.plan.md).  Satuan: tick FSM
    // (1 / update_rate_hz).  Lihat road_crossing_fsm.param.yaml untuk aturan tuning.
    int threshold_merah_valid{3};
    int threshold_hijau_valid{3};
    int threshold_merah_off{10};
    int threshold_hijau_off{3};
    int threshold_interlock_hijau_solid{20};
    bool reroute_enabled{true};
    double reroute_cooldown_sec{60.0};
    // Interlock params for REROUTING_TO_ALT step3 → REROUTING_TO_GOAL transition.
    // Both conditions must be satisfied simultaneously before rerouting back to goal.
    double reroute_min_time_sec{5.0};  // [s] min time in the "moving to alternate" sub-phase
    double reroute_min_dist_m{5.0};    // [m] min distance ego must have moved away from pre entry
    // Engage retry interval: how often to re-call change_to_autonomous while waiting
    // for AUTONOMOUS mode (step 2 of both reroute phases).
    double engage_retry_sec{2.0};      // [s] minimum interval between retry engage calls

    // topic / service names
    std::string topic_pedestrian_light;
    std::string topic_gate;
    std::string topic_fsm_state;
    // Interlock status_now (Int32Stamped: 0 NONE, 1 MERAH, 2 HIJAU), for rviz / PlotJuggler.
    std::string topic_light_status;
    std::string topic_route;
    std::string topic_map;
    std::string topic_ego_pose;
    std::string srv_ped_light_detector;
    std::string srv_rotary_aimer;

    // --- Rotary aimer spam-enable ---
    // Topik output rotary aimer; dipakai untuk deteksi apakah aimer sudah running.
    std::string topic_rotary_angle_cmd;
    // Cmd dianggap "fresh" (aimer running) jika umurnya < aim_cmd_fresh_sec detik.
    double aim_cmd_fresh_sec{0.5};
    // Interval minimum (detik) antar kiriman enable ke rotary_aimer selama spam-loop.
    double aim_enable_retry_sec{0.3};

    // Jarak (m) dari pre entry di bawahnya aimer diaktifkan dan membidik lampu.
    double aim_approach_dist_m{10.0};
    // false = the body only turns while ego stands still (rotary_aimer, same switch) and, on
    // green, the FSM keeps the road gate HOLD until the aimer has brought the body back to the
    // front -- so it never turns while driving.  true = old behaviour (re-zero while crossing).
    bool rotate_body_while_driving{true};
    double rezero_before_go_tol_deg{1.0};      // aim cmd within this of 0 = body at the front
    double rezero_before_go_settle_sec{0.5};   // cmd at 0 this long before GO (body catches up)
    double rezero_before_go_timeout_sec{60.0};  // GO anyway (WARN) after this long
    // Green latch: once the interlock allowed crossing, keep allowing for this long while the
    // light is NOT seen (status NONE) -- the camera loses it when the body turns back to the
    // front.  Cleared by a valid red, or by green blinking (green seen again after a gap
    // green_latch_blink_count times).  0 = no latch.
    double green_latch_sec{30.0};
    int green_latch_blink_count{2};
    // true: only turn the body to the light when it is NOT detected from where the body
    // faces now (any camera) for aim_unseen_grace_sec in WAIT_GREEN; a light already seen
    // (red or green) needs no turn -- saves rotary motor work.  Once turned, the aimer stays
    // on until ego goes.  false: aim whenever waiting (old behaviour).
    bool aim_only_if_light_unseen{true};
    double aim_unseen_grace_sec{2.0};
    // A light seen near the image edge counts as unseen too (detection there is poor): the
    // classifier ROI centre, (cx - w/2) / (w/2) from the detector, beyond this |offset| for
    // aim_unseen_grace_sec turns the body.  <= 0 = off.  An offset older than
    // aim_roi_offset_timeout_sec is ignored (seen = no turn, as before).
    double aim_max_roi_offset{0.4};
    double aim_roi_offset_timeout_sec{0.5};
    std::string topic_light_roi_offset{"/perception/road_crossing/pedestrian_light_roi_offset"};
    // PedestrianLightTarget: the light(s) paired with the active crossing.
    std::string topic_target_light{"/planning/road_crossing/target_light"};

    // --- Position gates (mencegah FSM maju sebelum robot sampai di lokasi) ---
    // Jarak (m) dari pre entry di bawahnya ego dianggap "sudah di pre".
    // Harus >= pre_stop_dist_m agar gate aktif saat robot sudah berhenti di pre-stop.
    double pre_arrive_dist_m{3.0};
    // Jarak (m) dari crossing entry di bawahnya ego dianggap "sudah di crossing".
    double crossing_arrive_dist_m{1.5};
    // [m] ego moving more than this in one FSM tick = relocalized → reset the episode.
    double relocalize_jump_m{2.0};
    // --- Safety-timeout watchdogs (defensive, additive — do not gate normal-path exits) ---
    // CROSSING (state 6) safety timeout: its only normal exit is egoPastCrossingExit()
    // (a centerline-projection check), which can permanently fail to register "past exit"
    // if is_driving_forward_ flips near/inside the crossing lanelet (bidirectional-driving
    // reverse scenario). If ego is stuck in CROSSING longer than this, force the SAME
    // cleanup as the normal exit path (DONE + GO gates + disable detectors), logged loudly
    // as a safety-timeout-forced exit, not a normal geometric exit.
    double crossing_timeout_sec{60.0};
    // Engage-wait absolute timeout for REROUTING_TO_ALT/REROUTING_TO_GOAL step2: total time
    // allowed waiting for autonomous_available while retrying change_to_autonomous at
    // engage_retry_sec cadence. If exceeded without ever becoming autonomous, abandon the
    // reroute attempt (resetEpisode + IDLE) instead of retrying forever.
    double engage_timeout_sec{30.0};
  } param_;

  // ------------------------------------------------------------------
  // Subscribers
  // ------------------------------------------------------------------
  rclcpp::Subscription<autoware_road_crossing_msgs::msg::PedestrianLightState>::SharedPtr
    sub_ped_light_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_;
  // Where ego is on the route, from autoware_active_route_window: on an out-and-back route over
  // overlapping lanelets the closest route lanelet can belong to the other pass.
  rclcpp::Subscription<autoware_active_route_window_msgs::msg::ActiveRouteWindow>::SharedPtr
    sub_active_route_window_;
  autoware_active_route_window_msgs::msg::ActiveRouteWindow::ConstSharedPtr active_route_window_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  // Subscriber output rotary aimer — untuk deteksi aimer sudah running (spam-enable).
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_aim_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_light_roi_offset_;

  // AdAPI state subscribers
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_op_mode_;

  // ------------------------------------------------------------------
  // Publishers
  // ------------------------------------------------------------------
  rclcpp::Publisher<autoware_road_crossing_msgs::msg::RoadCrossingGate>::SharedPtr pub_gate_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_fsm_state_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_light_status_;
  // AdAPI goal publisher (transient_local, reliable, depth 1 — matching auto_cycle_goals.py)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  // The light(s) paired with the active crossing — detector, aimer and simulator follow it.
  rclcpp::Publisher<autoware_road_crossing_msgs::msg::PedestrianLightTarget>::SharedPtr
    pub_target_light_;

  // ------------------------------------------------------------------
  // Service clients
  // ------------------------------------------------------------------
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_ped_light_detector_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_rotary_aimer_;
  // AdAPI reroute clients
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr cli_clear_route_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr cli_engage_;

  // ------------------------------------------------------------------
  // Timer
  // ------------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr timer_;

  // ------------------------------------------------------------------
  // State
  // ------------------------------------------------------------------
  FsmState state_{FsmState::IDLE};

  // Latest sensor / map messages (guarded against nullptr)
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr last_trajectory_;
  // Ego position at the previous tick, to detect relocalization jumps.
  std::optional<geometry_msgs::msg::Point> prev_tick_position_;
  autoware_map_msgs::msg::LaneletMapBin::SharedPtr last_map_msg_;

  // Route handler wrapping map + route
  autoware::route_handler::RouteHandler route_handler_;
  bool map_initialized_{false};
  bool route_initialized_{false};

  // ------------------------------------------------------------------
  // Bidirectional-driving support (Phase 3c).
  //
  // This node has no access to behavior_velocity_planner_common::PlannerData
  // (that's a different process/package), so direction is derived locally
  // from the subscribed Odometry's longitudinal velocity sign — mirroring
  // what autoware::motion_utils::isDrivingForwardWithTwist() does for a
  // single twist sample. Updated once per tick in updateDrivingDirection()
  // (called from onTimer before any direction-dependent logic runs).
  // Defaults to true (forward) and only flips when |vx| exceeds a small
  // deadband, to avoid chattering while stopped/near-zero speed.
  // ------------------------------------------------------------------
  bool is_driving_forward_{true};
  static constexpr double kDrivingDirectionDeadbandMps{0.05};

  // Debounce for the raw-twist deadband check above: require this many
  // CONSECUTIVE ticks past the deadband, in the same direction, before
  // actually flipping is_driving_forward_. Guards against a single noisy
  // odom sample latching the wrong direction (no recovery path once latched,
  // since checkArmCondition() refuses to arm a new episode while
  // !is_driving_forward_).
  static constexpr int kDrivingDirectionConsecutiveSamples{3};
  int driving_direction_candidate_count_{0};
  // Sign of the candidate direction currently being confirmed (true=forward
  // candidate, false=backward candidate). Only meaningful while
  // driving_direction_candidate_count_ > 0.
  bool driving_direction_candidate_forward_{true};

  // Fallback watchdog: how long (consecutive ticks worth of wall-clock time)
  // the robot has been within the twist deadband — i.e. "near-stationary" —
  // with no fresh direction confirmation from updateDrivingDirection()'s
  // normal twist path. Once this exceeds kNearStationaryFallbackSec, direction
  // is instead derived geometrically from the route (see
  // deriveDrivingDirectionFromRoute()) rather than leaving is_driving_forward_
  // latched at a possibly-stale value forever.
  std::optional<rclcpp::Time> near_stationary_since_;
  static constexpr double kNearStationaryFallbackSec{2.0};

  // Recompute is_driving_forward_ from last_odom_->twist.twist.linear.x.
  // No-op (keeps previous value) if last_odom_ is null. Requires
  // kDrivingDirectionConsecutiveSamples consecutive samples past the deadband
  // (in the same direction) before flipping is_driving_forward_, to reduce
  // false latches from a single noisy sample. If the robot has been within
  // the deadband (near-stationary) for more than kNearStationaryFallbackSec,
  // falls back to deriveDrivingDirectionFromRoute() instead of leaving
  // is_driving_forward_ stuck at whatever it last was.
  void updateDrivingDirection();

  // Geometric fallback for direction-of-travel when the twist signal is
  // uninformative (near-stationary for a while). Reuses the same geometric
  // idea as autoware::motion_utils::isDrivingForward() / the underlying
  // autoware_utils_geometry::is_driving_forward() helper (compare ego's own
  // orientation against the azimuth toward the next point ahead on the
  // path) — except sourced from the route centerline via route_handler_
  // (this node has no Trajectory/Path subscription, only the route). Returns
  // nullopt if it cannot be determined (e.g. ego not resolvable on route).
  std::optional<bool> deriveDrivingDirectionFromRoute() const;
  // Direction of the planned trajectory at ego (forward = its orientation agrees with the
  // way it runs).  Right for lanelets the route traverses against their drawing direction.
  std::optional<bool> deriveDrivingDirectionFromTrajectory() const;

  // The route lanelet ego is on: the active route window's, when a fresh valid one exists for
  // this route, else the closest route lanelet (the previous behaviour).
  bool egoRouteLanelet(lanelet::ConstLanelet * lanelet) const;

  // Ordered sequence of lanelet IDs extracted from the mission-planner route.
  // Populated in onRoute() from msg->segments[i].preferred_primitive.id (in route order).
  // Refreshed on every onRoute call so reroute acks update the sequence immediately.
  // Used by checkArmCondition() and findAlternateInNextBranches() to resolve
  // crossing/pre/alternate lanelets from the route topology rather than BFS from ego.
  std::vector<lanelet::Id> route_lanelets_;

  // Goal capture: only store on first route that is NOT triggered by our own re-route
  std::optional<geometry_msgs::msg::Pose> original_goal_;

  // Reroute phase tracking:
  //   our_reroute_pending_phase_ == 0  → no pending reroute
  //   our_reroute_pending_phase_ == 1  → Phase A (REROUTING_TO_ALT) goal published, not yet acked
  //   our_reroute_pending_phase_ == 2  → Phase B (REROUTING_TO_GOAL) goal published, not yet acked
  int our_reroute_pending_phase_{0};

  // Active crossing lanelets for this crossing episode.
  // active_pre_id_     = the pre_road_crossing lanelet (nullopt if none found or
  //                      ego approaches road_crossing directly without a pre)
  // active_crossing_id_ = the road_crossing lanelet (always set when episode is active)
  std::optional<lanelet::Id> active_pre_id_;
  std::optional<lanelet::Id> active_crossing_id_;

  // Only written by the (unreachable) reroute path.  Arming no longer needs suppression:
  // it only happens on the pre of a crossing, and a finished crossing leaves ego past it.
  std::optional<lanelet::Id> suppressed_crossing_id_;

  // Set on the first ARMED_LIGHT tick (entry-action guard); cleared when the episode crosses.
  std::optional<rclcpp::Time> light_timeout_start_;

  // CROSSING safety-timeout timer (defensive watchdog #1).
  // Set once at each WAIT_GREEN -> CROSSING transition (green / bypass / timeout-fallback).
  // If ego is still in CROSSING after crossing_timeout_sec, the CROSSING handler forces
  // the same cleanup as a normal exit, logging a distinct safety-timeout WARN.
  // Reset in resetEpisode().
  std::optional<rclcpp::Time> crossing_timeout_start_;

  // True when the CROSSING episode was initiated by a GREEN pedestrian light.
  // Set at the WAIT_GREEN → CROSSING (green) transition.
  // Cleared at bypass → CROSSING and timeout → CROSSING transitions.
  // Reset to false in resetEpisode().
  // Used by the CROSSING-state handler to persist GO_GREEN while the robot
  // traverses the crossing (so the behavior module sees GO_GREEN reliably).
  bool road_cross_is_green_{false};

  // Pedestrian light(s) paired with active_crossing_id_, resolved once at arming from the
  // crossing's direction tag (forward_pedestrian_tf / reverse_pedestrian_tf) so the target
  // cannot flip mid-episode.  Empty = the crossing has no light: WAIT_GREEN goes straight
  // to GO.  Cleared in resetEpisode().
  std::vector<int64_t> active_light_ids_;
  uint8_t active_light_direction_{
    autoware_road_crossing_msgs::msg::PedestrianLightTarget::FORWARD};

  // Interlock lampu hijau (interlock.plan.md): state yang di-reset tiap penyeberangan
  // (masuk ARMED_LIGHT dan resetEpisode()).
  // Values are published on topic_light_status — keep them stable.
  enum class LightStatus : uint8_t { NONE = 0, MERAH = 1, HIJAU = 2 };
  struct LightInterlock
  {
    // Diset onPedLight() dari lampu pasangan sejak tick terakhir; dibaca & dikosongkan
    // tiap tick.  Pesan NONE/BLANK (atau dari lampu lain) tidak menghapus deteksi.
    bool merah_seen{false};
    bool hijau_seen{false};
    uint16_t prev_counter_merah_detected{0};
    uint16_t prev_counter_hijau_detected{0};
    uint16_t counter_merah_detected{0};
    uint16_t counter_hijau_detected{0};
    uint16_t counter_merah_not_detected{0};
    uint16_t counter_hijau_not_detected{0};
    bool status_merah_valid{false};
    bool status_hijau_valid{false};
    LightStatus status_now{LightStatus::NONE};
    LightStatus prev_status{LightStatus::NONE};
    bool red_to_green{false};     // Rule 1, latched while green stays continuous
    bool allows_crossing{false};  // Rule 1 || Rule 2
  } interlock_;

  // Green latch state (see green_latch_sec).
  std::optional<rclcpp::Time> green_latch_since_;
  LightStatus green_latch_prev_status_{LightStatus::NONE};
  int green_latch_reappear_{0};

  // aim_only_if_light_unseen: since when the light has been unseen (status NONE) in
  // WAIT_GREEN, and whether the aimer was needed (sticky until ego goes / the episode ends).
  std::optional<rclcpp::Time> light_unseen_since_;
  bool aim_needed_{false};
  // Latest ROI offset from the detector (aim_max_roi_offset).
  std::optional<rclcpp::Time> last_roi_offset_time_;
  double last_roi_offset_{0.0};

  // Re-route state
  std::optional<rclcpp::Time> last_reroute_time_;
  std::set<lanelet::Id> reroute_blocklist_;  // alternate IDs already attempted

  // Timestamp set once when REROUTING_TO_ALT enters sub-step 3 (engage sent, gates released).
  // Used together with reroute_min_dist_m to guard the transition to REROUTING_TO_GOAL.
  // Reset in resetRerouteSubstep() and resetEpisode().
  std::optional<rclcpp::Time> reroute_alt_move_start_;

  // The alternate lanelet selected for the current reroute episode.
  std::optional<lanelet::Id> active_alt_id_;

  // Detectors enabled state (track to avoid redundant service calls)
  bool ped_light_detector_enabled_{false};

  // --- Rotary aimer spam-enable state ---
  // Waktu terakhir menerima pesan /control/rotary_angle_cmd (inisialisasi 0/lama).
  // Dipakai untuk cek apakah aimer sudah running (cmd "fresh").
  rclcpp::Time last_aim_cmd_time_{0, 0, RCL_ROS_TIME};
  // Waktu terakhir mengirim enable ke rotary_aimer (inisialisasi 0/lama).
  // Dipakai untuk throttle pengiriman enable (setiap aim_enable_retry_sec detik).
  rclcpp::Time last_aim_enable_send_{0, 0, RCL_ROS_TIME};
  // Last /control/rotary_angle_cmd value; unset until the aimer has published.
  std::optional<double> last_aim_cmd_value_;
  // rotate_body_while_driving=false: green seen, waiting for the body to face the front.
  std::optional<rclcpp::Time> rezero_before_go_since_;
  std::optional<rclcpp::Time> rezero_at_zero_since_;

  // ------------------------------------------------------------------
  // AdAPI reroute sub-step state machine
  // ------------------------------------------------------------------
  // Sub-step index for REROUTING_TO_ALT and REROUTING_TO_GOAL phases.
  // Each phase runs a 4-step async sequence (see onTimer REROUTING_TO_ALT /
  // REROUTING_TO_GOAL cases):
  //   step 0: call clear_route once (guard: reroute_clear_called_)
  //   step 1: after clear ack (or 1 tick), publish goal pose once (guard: reroute_goal_published_)
  //   step 2: wait route_state==SET AND is_autonomous_mode_available → call change_to_autonomous once
  //           (guard: reroute_engage_called_)
  //   step 3: after engage sent, wait for departure / completion condition
  int reroute_substep_{0};

  // One-shot guards per reroute sequence (reset when starting a new phase).
  bool reroute_clear_called_{false};    // clear_route sent for current phase
  bool reroute_goal_published_{false};  // goal published for current phase
  bool reroute_engage_called_{false};   // change_to_autonomous sent for current phase

  // AdAPI state cache (updated by subscribers, read in timer — no locking needed
  // since ROS2 callbacks and timer run in the same executor thread).
  uint16_t adapi_route_state_{0};           // RouteState::UNKNOWN
  bool adapi_autonomous_available_{false};  // OperationModeState::is_autonomous_mode_available
  // OperationModeState::mode cache.
  // Constants from OperationModeState: UNKNOWN=0, STOP=1, AUTONOMOUS=2, LOCAL=3, REMOTE=4.
  uint8_t adapi_op_mode_{0};                // OperationModeState::mode (UNKNOWN on startup)

  // Throttle timestamp for step-2 engage retries (reset each time a new reroute phase starts).
  std::optional<rclcpp::Time> last_engage_attempt_;

  // Engage-wait absolute-timeout timer (defensive watchdog #2).
  // Set ONCE when first entering the step-2 engage-retry substep of REROUTING_TO_ALT/
  // REROUTING_TO_GOAL (not reset on every retry attempt — tracks TOTAL elapsed time in
  // this substep, not time-since-last-retry). If autonomous_available never becomes true
  // within engage_timeout_sec, the reroute attempt is abandoned (resetEpisode + IDLE).
  // Reset in resetRerouteSubstep() (start of each new phase) and resetEpisode().
  std::optional<rclcpp::Time> engage_wait_start_;

  // Gate flag: true once onRoute() confirms the route published by the CURRENT reroute
  // phase has been acked by the mission planner.  Prevents step-2 from reading a stale
  // adapi_op_mode_ value (which may still be AUTONOMOUS from the OLD route) and short-
  // circuiting the engage call before the new route is actually SET.
  // Reset to false at the start of each new reroute phase (in resetRerouteSubstep()).
  // Set to true in onRoute() when our_reroute_pending_phase_ != 0 (i.e. ack for our goal).
  bool reroute_route_acked_{false};

  // ------------------------------------------------------------------
  // Internal helpers
  // ------------------------------------------------------------------

  // FSM tick (called by timer)
  void onTimer();

  // Callbacks
  void onPedLight(
    const autoware_road_crossing_msgs::msg::PedestrianLightState::SharedPtr msg);
  void onRoute(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg);
  void onMap(const autoware_map_msgs::msg::LaneletMapBin::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  // Callback output rotary aimer — catat waktu pesan terakhir untuk deteksi running.
  void onAimCmd(const std_msgs::msg::Float32::SharedPtr msg);
  void onLightRoiOffset(const std_msgs::msg::Float32::SharedPtr msg);
  void onAdApiRouteState(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
  void onAdApiOpMode(const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg);

  // FSM transition helpers
  // Returns true if ego is within arm_hops_max lanelets of a tagged lanelet.
  // Fills active_crossing_id_ with the found road_crossing lanelet.
  bool checkArmCondition();

  // Distance from ego to the entry of active_crossing_id_ centerline front [m].
  double distanceToCrossingEntry() const;

  // Distance from ego to the entry of active_pre_id_ centerline front [m].
  // Returns -1.0 if active_pre_id_ is nullopt or map is not ready.
  double distanceToPreEntry() const;

  // Shared implementation for distanceToCrossingEntry()/distanceToPreEntry():
  // arc-length-along-centerline distance from ego to the lanelet's entry point,
  // consistent with how autoware_behavior_velocity_road_crossing_module's
  // scene.cpp measures the stop point (arc-length ahead of ego on the planned
  // path), rather than plain Euclidean distance to a fixed vertex.
  // Returns -1.0 on failure (no id, no odom, map not ready, lookup error).
  double arcLengthDistanceToLaneletEntry(const std::optional<lanelet::Id> & id) const;

  // True if ego is past the exit of active_crossing_id_.
  bool egoPastCrossingExit() const;

  // True if ego's closest lanelet is no longer the active pre lanelet.
  // Used by REROUTING_TO_ALT to detect when ego has departed the pre stop.
  bool egoLeftPreLanelet() const;

  // Find alternate_road_crossing lanelet by inspecting the DIRECT next-branches
  // (1 level only — no BFS/recursion) from ego's current lanelet within the route.
  //
  // Correct topology:
  //   PARENT (current ego lanelet)
  //     ├─ pre_road_crossing   (next branch 0)
  //     └─ alternate_road_crossing  (next branch 1)  ← what we look for
  //
  // Algorithm:
  //   1. getClosestLaneletWithinRoute → ego lanelet (= PARENT while robot is held
  //      ~1 m before the pre stop, i.e. still on the parent lanelet).
  //   2. getNextLanelets(ego) → ITERATE 1 LEVEL ONLY (direct branches; no BFS).
  //   3. For each branch: if tagged "alternate_road_crossing"=="true"
  //      AND NOT in reroute_blocklist_ → return it.
  //   4. If no such branch → nullopt (caller holds at pre).
  std::optional<lanelet::ConstLanelet> findAlternateInNextBranches() const;

  // Compute pose at centerline midpoint of a lanelet (heading from centerline direction).
  geometry_msgs::msg::Pose poseAtCenterlineMid(const lanelet::ConstLanelet & llt) const;

  // Call SetBool service non-blocking (fire-and-forget).
  void setDetectorEnabled(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client, bool enable,
    const std::string & name);

  // AdAPI reroute helpers (non-blocking, called from timer sub-steps).
  // Returns true if the async call was dispatched, false if service not ready (caller retries).
  bool sendAdApiClearRoute();
  bool sendAdApiPublishGoal(const geometry_msgs::msg::Pose & goal_pose);
  bool sendAdApiEngage();

  // Reset reroute sub-step guards (call when entering a new reroute phase).
  void resetRerouteSubstep();

  // Re-zero the rotary aimer at crossing/reroute transitions — direction-aware
  // (Phase 3c fix site #4). When driving forward, behaves exactly as before:
  // disables the aimer, which re-zeroes to the vehicle's physical nose
  // (rotary angle 0 == base_link +X), correct because the nose leads.
  // When reversing, the nose is trailing, so re-zeroing to it would point the
  // sensor turret away from the direction of travel. The rotary_aimer node
  // itself (a separate package, out of scope for this fix) has no
  // direction-aware zero target — only a hardcoded 0.0. Rather than touching
  // that node, we leave the aimer ENABLED in this case: its own tf_pedestrian
  // light search is already TF/pose-based (not heading-hardcoded, see
  // APPROACH-phase comments in onTimer), so it keeps correctly tracking the
  // nearest light regardless of travel direction instead of snapping to a
  // wrong fixed angle.
  void reZeroAimerDirectionAware();

  // rotate_body_while_driving=false, on green: disable the aimer (its ramp brings the body to the
  // front while ego still stands) and return true once the aim command has been at 0 for
  // rezero_before_go_settle_sec (or after rezero_before_go_timeout_sec).  False = keep HOLD.
  bool bodyAtFrontBeforeGo(const rclcpp::Time & now);
  void cancelRezeroBeforeGo();

  // Green latch (see green_latch_sec).  armGreenLatch() when the interlock allows crossing;
  // greenLatched() once per tick, after updateLightInterlock(): true while it holds, and it
  // clears itself on a valid red, on green blinking, or when green_latch_sec has run out.
  void armGreenLatch(const rclcpp::Time & now);
  bool greenLatched(const rclcpp::Time & now);
  void clearGreenLatch();

  // aim_only_if_light_unseen: update aim_needed_ from the interlock (once per WAIT_GREEN tick,
  // after updateLightInterlock()); resetAimNeed() starts over (new wait at the pre).
  void updateAimNeed(const rclcpp::Time & now);
  void resetAimNeed();

  // Publish gate command for active_crossing_id_ (ROAD lanelet).
  void publishGate(uint8_t command);

  // Publish gate command for active_pre_id_ (PRE lanelet). No-op if active_pre_id_ is nullopt.
  void publishGatePre(uint8_t command);

  // Publish FSM state debug.
  void publishFsmState();

  // Publish active_light_ids_ for the active crossing (empty when idle).
  void publishTargetLight();
  // active_light_ids_ as "1471,1490" for logs.
  std::string lightIdsText() const;

  // The tf_pedestrian way ids paired with `crossing_id` for the given direction of travel,
  // read from the crossing's forward_pedestrian_tf / reverse_pedestrian_tf tag.
  std::vector<int64_t> pairedLights(lanelet::Id crossing_id, bool forward) const;

  // Transition helper: change state and log.
  void transitionTo(FsmState new_state);

  // One FSM tick of the green-light interlock (interlock.plan.md); updates interlock_.
  // `where` (state name) only prefixes the log.
  void updateLightInterlock(const char * where);
  // True when ego's base_link is inside lanelet `id`.
  bool egoOnLanelet(lanelet::Id id) const;
  // On the pre: base_link OR the front bumper on it.  A robot standing at the seam into the pre
  // (front in, base_link not yet) is at the pre -- the road_crossing stop zone holds it there
  // rather than creeping it the last decimetres.  The crossing itself stays base_link only.
  bool egoOnPre(lanelet::Id id) const;
  double base_link_to_front_m_{0.0};
  // True when ego is before lanelet `id` (on the travel-direction side) by more than margin.
  bool egoUpstreamOf(lanelet::Id id, double margin) const;
  // The route lanelet right after the active crossing, if it is a pre_road_crossing.
  std::optional<lanelet::Id> nextPreAfterCrossing() const;
  static const char * lightStatusName(LightStatus status);

  // Walk N hops from cur_lanelet via getNextLanelets, check attribute.
  bool hasTaggedLaneletWithinHops(
    const lanelet::ConstLanelet & cur, const std::string & attribute_key, int hops,
    lanelet::Id * found_id = nullptr) const;

  /** True jika lanelet crossing aktif (atau pre) punya tag bernilai "true". */
  bool crossingHasTag(const std::string & key) const;

  // Reset episode state for next crossing.
  void resetEpisode();
};

}  // namespace autoware::road_crossing_fsm

#endif  // AUTOWARE__ROAD_CROSSING_FSM__ROAD_CROSSING_FSM_NODE_HPP_
