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
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_road_crossing_msgs/msg/crosswalk_detection.hpp>
#include <autoware_road_crossing_msgs/msg/pedestrian_light_state.hpp>
#include <autoware_road_crossing_msgs/msg/road_crossing_gate.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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
//   ARMED_CROSSWALK  = 1   robot approaching crossing; waiting for crosswalk detection
//   CROSSWALK_OK     = 2   crosswalk detected; one-tick intermediate
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
  ARMED_CROSSWALK  = 1,
  CROSSWALK_OK     = 2,
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
    int arm_hops_max{3};
    double crosswalk_min_confidence{0.5};
    double fail_distance_m{2.0};
    // reroute_after_sec: how long (s) ego must be stopped at the pre lanelet without
    // a crosswalk detection before initiating reroute.  Replaces the old
    // fail_consecutive_sec which was only a 2-second hold; the new default is 15 s
    // to give the crosswalk detector a fair chance before committing to reroute.
    double reroute_after_sec{15.0};
    std::vector<int64_t> green_states{2, 3};
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
    std::string topic_crosswalk_detection;
    std::string topic_pedestrian_light;
    std::string topic_gate;
    std::string topic_fsm_state;
    std::string topic_route;
    std::string topic_map;
    std::string topic_ego_pose;
    std::string srv_crosswalk_detector;
    std::string srv_ped_light_detector;
    std::string srv_rotary_aimer;

    // --- Rotary aimer spam-enable ---
    // Topik output rotary aimer; dipakai untuk deteksi apakah aimer sudah running.
    std::string topic_rotary_angle_cmd;
    // Cmd dianggap "fresh" (aimer running) jika umurnya < aim_cmd_fresh_sec detik.
    double aim_cmd_fresh_sec{0.5};
    // Interval minimum (detik) antar kiriman enable ke rotary_aimer selama spam-loop.
    double aim_enable_retry_sec{0.3};

    // --- Rotary aimer APPROACH phase ---
    // Jarak (m) dari pre entry di bawahnya FSM masuk fase APPROACH:
    //   aimer diaktifkan + centroid road_crossing dikirim ke /rotary_aimer/aim_target
    //   agar kamera nadap crosswalk SEBELUM robot tiba di pre.
    // Begitu robot tiba di pre (at_pre), kirim target berhenti → aimer fallback ke lampu.
    double aim_approach_dist_m{10.0};
    // Topik PointStamped (map frame) untuk mengirim target eksternal ke rotary_aimer.
    std::string topic_aim_target{"/rotary_aimer/aim_target"};

    // --- Crosswalk detector lazy-enable by distance ---
    // Jarak (m) ke pre entry untuk mulai enable crosswalk_detector — ~2× aim_approach_dist_m,
    // biar detektor sudah jalan sebelum kamera aim ke crosswalk.
    double crosswalk_enable_dist_m{20.0};

    // --- Position gates (mencegah FSM maju sebelum robot sampai di lokasi) ---
    // Jarak (m) dari pre entry di bawahnya ego dianggap "sudah di pre".
    // Harus >= pre_stop_dist_m agar gate aktif saat robot sudah berhenti di pre-stop.
    double pre_arrive_dist_m{3.0};
    // Jarak (m) dari crossing entry di bawahnya ego dianggap "sudah di crossing".
    double crossing_arrive_dist_m{1.5};
    // Durasi terus-menerus TIDAK mendeteksi lampu (NONE atau detektor diam) sebelum
    // FSM menyerah dan GO (fallback).  Deteksi RED/GREEN/BLANK mereset counter ini
    // sehingga selama lampu merah robot menunggu GREEN selamanya (benar).
    double light_detect_timeout_sec{10.0};

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
    // Blanket episode-duration watchdog (catch-all): if the FSM is not IDLE for longer
    // than this, force resetEpisode()+IDLE regardless of which state it is stuck in.
    // Defense-in-depth for any stuck-state bug not covered by the two timeouts above.
    double max_episode_duration_sec{120.0};
  } param_;

  // ------------------------------------------------------------------
  // Subscribers
  // ------------------------------------------------------------------
  rclcpp::Subscription<autoware_road_crossing_msgs::msg::CrosswalkDetection>::SharedPtr
    sub_crosswalk_;
  rclcpp::Subscription<autoware_road_crossing_msgs::msg::PedestrianLightState>::SharedPtr
    sub_ped_light_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  // Subscriber output rotary aimer — untuk deteksi aimer sudah running (spam-enable).
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_aim_cmd_;

  // AdAPI state subscribers
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_op_mode_;

  // ------------------------------------------------------------------
  // Publishers
  // ------------------------------------------------------------------
  rclcpp::Publisher<autoware_road_crossing_msgs::msg::RoadCrossingGate>::SharedPtr pub_gate_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_fsm_state_;
  // AdAPI goal publisher (transient_local, reliable, depth 1 — matching auto_cycle_goals.py)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  // APPROACH phase: publish road_crossing centroid (map PointStamped) ke rotary_aimer
  // sebagai target eksternal agar kamera menghadap crosswalk sebelum robot tiba di pre.
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_aim_target_;

  // ------------------------------------------------------------------
  // Service clients
  // ------------------------------------------------------------------
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_crosswalk_detector_;
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
  autoware_road_crossing_msgs::msg::CrosswalkDetection::SharedPtr last_crosswalk_det_;
  autoware_road_crossing_msgs::msg::PedestrianLightState::SharedPtr last_ped_light_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
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

  // Recompute is_driving_forward_ from last_odom_->twist.twist.linear.x.
  // No-op (keeps previous value) if last_odom_ is null or speed is within
  // the deadband (near-zero — direction is ambiguous while nearly stopped).
  void updateDrivingDirection();

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

  // Blanket episode-duration watchdog (defensive watchdog #3, catch-all).
  // Set once at the IDLE -> ARMED_CROSSWALK transition (i.e. right when
  // checkArmCondition() first succeeds and active_pre_id_/active_crossing_id_ get
  // assigned). Checked at the very top of onTimer(), before the state switch: if
  // state_ != IDLE and the elapsed time exceeds max_episode_duration_sec, force
  // resetEpisode()+IDLE regardless of which state the FSM is stuck in. Defense-in-depth
  // for any stuck-state bug the two more targeted timeouts above don't cover.
  // Reset in resetEpisode().
  std::optional<rclcpp::Time> episode_start_time_;

  // Suppression: the road_crossing lanelet ID of the episode that just completed (DONE).
  // checkArmCondition() will not re-arm for this ID until ego leaves the vicinity
  // (i.e. no crossing at all is found within arm_hops_max, or a DIFFERENT crossing
  // is found — meaning ego has moved past this one).
  // resetEpisode() intentionally does NOT clear this field.
  std::optional<lanelet::Id> suppressed_crossing_id_;

  // ARMED_CROSSWALK pre-stop reroute timer.
  // Starts once ego is close enough to the crossing entry (within fail_distance_m)
  // but crosswalk is still NOT detected.  If crosswalk is detected before the timer
  // expires the timer is cancelled.  If it expires → REROUTING_TO_ALT.
  std::optional<rclcpp::Time> pre_wait_timer_start_;

  // WAIT_GREEN timeout timer
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

  // Counter waktu "tidak mendeteksi lampu" di WAIT_GREEN (skema baru).
  // Di-set saat pertama kali masuk kondisi tidak-deteksi (NONE atau pesan null).
  // Di-reset ke nullopt saat ada deteksi VALID (GREEN/RED/BLANK) — bukan NONE.
  // Jika (now - *no_detect_since_) > light_detect_timeout_sec → fallback GO.
  // Di-reset di resetEpisode() dan saat masuk/keluar WAIT_GREEN.
  std::optional<rclcpp::Time> no_detect_since_;

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
  bool crosswalk_detector_enabled_{false};
  bool ped_light_detector_enabled_{false};
  // Flag: crosswalk_detector sudah di-enable via distance-gate di episode ini.
  // Di-reset di resetEpisode() agar setiap episode baru memanggil enable sekali.
  bool crosswalk_detector_lazy_enabled_{false};

  // --- Rotary aimer spam-enable state ---
  // Waktu terakhir menerima pesan /control/rotary_angle_cmd (inisialisasi 0/lama).
  // Dipakai untuk cek apakah aimer sudah running (cmd "fresh").
  rclcpp::Time last_aim_cmd_time_{0, 0, RCL_ROS_TIME};
  // Waktu terakhir mengirim enable ke rotary_aimer (inisialisasi 0/lama).
  // Dipakai untuk throttle pengiriman enable (setiap aim_enable_retry_sec detik).
  rclcpp::Time last_aim_enable_send_{0, 0, RCL_ROS_TIME};

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
  void onCrosswalkDetection(
    const autoware_road_crossing_msgs::msg::CrosswalkDetection::SharedPtr msg);
  void onPedLight(
    const autoware_road_crossing_msgs::msg::PedestrianLightState::SharedPtr msg);
  void onRoute(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg);
  void onMap(const autoware_map_msgs::msg::LaneletMapBin::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  // Callback output rotary aimer — catat waktu pesan terakhir untuk deteksi running.
  void onAimCmd(const std_msgs::msg::Float32::SharedPtr msg);
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
  //   4. If no such branch → nullopt (caller holds at pre, waits for crosswalk).
  std::optional<lanelet::ConstLanelet> findAlternateInNextBranches() const;

  // Compute pose at centerline midpoint of a lanelet (heading from centerline direction).
  geometry_msgs::msg::Pose poseAtCenterlineMid(const lanelet::ConstLanelet & llt) const;

  // Compute centroid of the road_crossing lanelet that follows the current active_pre_id_
  // in route_lanelets_.  Centroid = average of all leftBound + rightBound 3-D points.
  // Returns nullopt when: map not ready, no active_pre_id_, road_crossing not found in route,
  // or the lanelet has no boundary points.
  std::optional<geometry_msgs::msg::Point> getRoadCrossingCentroid() const;

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

  // Publish gate command for active_crossing_id_ (ROAD lanelet).
  void publishGate(uint8_t command);

  // Publish gate command for active_pre_id_ (PRE lanelet). No-op if active_pre_id_ is nullopt.
  void publishGatePre(uint8_t command);

  // Publish FSM state debug.
  void publishFsmState();

  // Transition helper: change state and log.
  void transitionTo(FsmState new_state);

  // Check if PedestrianLightState.state is in green_states param.
  bool isGreenState(uint8_t state) const;

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
