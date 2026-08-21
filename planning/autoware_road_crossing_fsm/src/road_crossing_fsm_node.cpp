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

#include "autoware/road_crossing_fsm/road_crossing_fsm_node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::road_crossing_fsm
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
RoadCrossingFsmNode::RoadCrossingFsmNode(const rclcpp::NodeOptions & options)
: Node("road_crossing_fsm", options)
{
  // ------------------------------------------------------------------
  // Load parameters
  // ------------------------------------------------------------------
  param_.update_rate_hz = declare_parameter<double>("update_rate_hz", 10.0);
  param_.arm_hops_max = declare_parameter<int>("arm_hops_max", 3);
  param_.crosswalk_min_confidence = declare_parameter<double>("crosswalk_min_confidence", 0.5);
  param_.fail_distance_m = declare_parameter<double>("fail_distance_m", 2.0);
  // reroute_after_sec replaces the old fail_consecutive_sec.
  // Old name: fail_consecutive_sec (2.0 s short hold before reroute — too aggressive).
  // New name: reroute_after_sec (15.0 s wait at pre stop before committing to reroute).
  param_.reroute_after_sec = declare_parameter<double>("reroute_after_sec", 15.0);
  {
    auto gs = declare_parameter<std::vector<int64_t>>("green_states", std::vector<int64_t>{2, 3});
    param_.green_states = gs;
  }
  param_.reroute_enabled = declare_parameter<bool>("reroute_enabled", true);
  param_.reroute_cooldown_sec = declare_parameter<double>("reroute_cooldown_sec", 60.0);
  param_.reroute_min_time_sec = declare_parameter<double>("reroute_min_time_sec", 5.0);
  param_.reroute_min_dist_m = declare_parameter<double>("reroute_min_dist_m", 5.0);
  param_.engage_retry_sec = declare_parameter<double>("engage_retry_sec", 2.0);

  param_.topic_crosswalk_detection = declare_parameter<std::string>(
    "topic_crosswalk_detection", "/perception/road_crossing/crosswalk_detection");
  param_.topic_pedestrian_light = declare_parameter<std::string>(
    "topic_pedestrian_light", "/perception/road_crossing/pedestrian_light");
  param_.topic_gate = declare_parameter<std::string>(
    "topic_gate", "/planning/road_crossing/gate");
  param_.topic_fsm_state = declare_parameter<std::string>(
    "topic_fsm_state", "/planning/road_crossing/fsm_state");
  param_.topic_route = declare_parameter<std::string>(
    "topic_route", "/planning/mission_planning/route");
  param_.topic_map = declare_parameter<std::string>(
    "topic_map", "/map/vector_map");
  param_.topic_ego_pose = declare_parameter<std::string>(
    "topic_ego_pose", "/localization/kinematic_state");
  param_.srv_crosswalk_detector = declare_parameter<std::string>(
    "srv_crosswalk_detector", "/perception/road_crossing/crosswalk_detector/enable");
  param_.srv_ped_light_detector = declare_parameter<std::string>(
    "srv_ped_light_detector", "/perception/road_crossing/pedestrian_light_detector/enable");
  param_.srv_rotary_aimer = declare_parameter<std::string>(
    "srv_rotary_aimer", "/rotary_aimer/enable");
  // NOTE: srv_set_waypoint_route is intentionally removed — replaced by AdAPI flow.

  // --- Parameter spam-enable rotary aimer ---
  // Topik output rotary aimer untuk mendeteksi apakah sudah running.
  param_.topic_rotary_angle_cmd = declare_parameter<std::string>(
    "topic_rotary_angle_cmd", "/control/rotary_angle_cmd");
  // Pesan aim_cmd dianggap fresh jika lebih muda dari nilai ini (detik).
  param_.aim_cmd_fresh_sec = declare_parameter<double>("aim_cmd_fresh_sec", 0.5);
  // Interval minimum antar pengiriman enable ke rotary_aimer (detik).
  param_.aim_enable_retry_sec = declare_parameter<double>("aim_enable_retry_sec", 0.3);

  // --- Parameter APPROACH phase aimer ---
  // Jarak (m) dari pre entry — robot dalam mode APPROACH jika jarak <= nilai ini.
  param_.aim_approach_dist_m = declare_parameter<double>("aim_approach_dist_m", 10.0);
  // Topik PointStamped target eksternal untuk rotary_aimer (map frame).
  param_.topic_aim_target = declare_parameter<std::string>(
    "topic_aim_target", "/rotary_aimer/aim_target");

  // --- Parameter crosswalk detector lazy-enable by distance ---
  // Jarak (m) ke pre entry untuk mulai enable crosswalk_detector — ~2× aim_approach_dist_m,
  // biar detektor sudah jalan sebelum kamera aim ke crosswalk.
  param_.crosswalk_enable_dist_m = declare_parameter<double>("crosswalk_enable_dist_m", 20.0);

  // --- Parameter position gate (mencegah FSM maju sebelum robot tiba) ---
  // Ego dianggap "sudah di pre" jika distanceToPreEntry() ∈ [0, pre_arrive_dist_m].
  // Harus >= pre_stop_dist_m agar memicu saat robot sudah berhenti di pre-stop.
  param_.pre_arrive_dist_m = declare_parameter<double>("pre_arrive_dist_m", 3.0);
  // Ego dianggap "sudah di crossing" jika distanceToCrossingEntry() ∈ [0, crossing_arrive_dist_m].
  param_.crossing_arrive_dist_m = declare_parameter<double>("crossing_arrive_dist_m", 1.5);
  // Durasi terus-menerus tidak mendeteksi lampu (NONE/diam) sebelum fallback GO.
  // Deteksi valid (RED/GREEN/BLANK) mereset counter — robot menunggu GREEN selamanya selama lampu RED.
  param_.light_detect_timeout_sec = declare_parameter<double>("light_detect_timeout_sec", 10.0);

  // --- Safety-timeout watchdogs (defensive, additive) ---
  // CROSSING safety timeout: forces DONE if egoPastCrossingExit() never fires
  // (e.g. is_driving_forward_ flips near/inside the crossing lanelet).
  param_.crossing_timeout_sec = declare_parameter<double>("crossing_timeout_sec", 60.0);
  // Engage-wait absolute timeout: abandon reroute attempt if autonomous_available
  // never becomes true within this many seconds of entering the engage-retry substep.
  param_.engage_timeout_sec = declare_parameter<double>("engage_timeout_sec", 30.0);
  // Blanket episode-duration watchdog: force resetEpisode()+IDLE if stuck non-IDLE
  // for longer than this, regardless of state.
  param_.max_episode_duration_sec =
    declare_parameter<double>("max_episode_duration_sec", 120.0);

  // ------------------------------------------------------------------
  // Subscribers
  // ------------------------------------------------------------------
  sub_crosswalk_ = create_subscription<autoware_road_crossing_msgs::msg::CrosswalkDetection>(
    param_.topic_crosswalk_detection, rclcpp::QoS(1),
    std::bind(&RoadCrossingFsmNode::onCrosswalkDetection, this, std::placeholders::_1));

  sub_ped_light_ = create_subscription<autoware_road_crossing_msgs::msg::PedestrianLightState>(
    param_.topic_pedestrian_light, rclcpp::QoS(1),
    std::bind(&RoadCrossingFsmNode::onPedLight, this, std::placeholders::_1));

  // Route: use transient_local so we receive the last route even after subscription
  sub_route_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    param_.topic_route,
    rclcpp::QoS(1).transient_local(),
    std::bind(&RoadCrossingFsmNode::onRoute, this, std::placeholders::_1));

  // Map: transient_local (published once at startup)
  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    param_.topic_map,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&RoadCrossingFsmNode::onMap, this, std::placeholders::_1));

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    param_.topic_ego_pose, rclcpp::QoS(1),
    std::bind(&RoadCrossingFsmNode::onOdom, this, std::placeholders::_1));

  // Subscriber output rotary aimer — catat waktu pesan untuk deteksi running (spam-enable).
  sub_aim_cmd_ = create_subscription<std_msgs::msg::Float32>(
    param_.topic_rotary_angle_cmd, rclcpp::QoS(1),
    std::bind(&RoadCrossingFsmNode::onAimCmd, this, std::placeholders::_1));

  // AdAPI state subscribers (transient_local to get latest cached state immediately)
  {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    sub_route_state_ = create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
      "/api/routing/state", qos,
      std::bind(&RoadCrossingFsmNode::onAdApiRouteState, this, std::placeholders::_1));
    sub_op_mode_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/api/operation_mode/state", qos,
      std::bind(&RoadCrossingFsmNode::onAdApiOpMode, this, std::placeholders::_1));
  }

  // ------------------------------------------------------------------
  // Publishers
  // ------------------------------------------------------------------
  pub_gate_ = create_publisher<autoware_road_crossing_msgs::msg::RoadCrossingGate>(
    param_.topic_gate, rclcpp::QoS(1));
  pub_fsm_state_ = create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    param_.topic_fsm_state, rclcpp::QoS(1));

  // AdAPI goal publisher — transient_local reliable depth 1 (same as auto_cycle_goals.py)
  {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", qos);
  }

  // APPROACH phase: target eksternal rotary_aimer (best-effort depth 1, no latching).
  // Rotary_aimer subscriptions ke topik ini dan mengarah ke point tersebut selama fresh.
  pub_aim_target_ = create_publisher<geometry_msgs::msg::PointStamped>(
    param_.topic_aim_target, rclcpp::QoS(1));

  // ------------------------------------------------------------------
  // Service clients
  // ------------------------------------------------------------------
  cli_crosswalk_detector_ = create_client<std_srvs::srv::SetBool>(
    param_.srv_crosswalk_detector);
  cli_ped_light_detector_ = create_client<std_srvs::srv::SetBool>(
    param_.srv_ped_light_detector);
  cli_rotary_aimer_ = create_client<std_srvs::srv::SetBool>(
    param_.srv_rotary_aimer);

  // AdAPI reroute clients
  cli_clear_route_ = create_client<autoware_adapi_v1_msgs::srv::ClearRoute>(
    "/api/routing/clear_route");
  cli_engage_ = create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous");

  // ------------------------------------------------------------------
  // Timer
  // ------------------------------------------------------------------
  const auto period = std::chrono::duration<double>(1.0 / param_.update_rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RoadCrossingFsmNode::onTimer, this));

  RCLCPP_INFO(get_logger(), "RoadCrossingFsmNode initialized (rate=%.1f Hz)", param_.update_rate_hz);
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void RoadCrossingFsmNode::onCrosswalkDetection(
  const autoware_road_crossing_msgs::msg::CrosswalkDetection::SharedPtr msg)
{
  last_crosswalk_det_ = msg;
}

void RoadCrossingFsmNode::onPedLight(
  const autoware_road_crossing_msgs::msg::PedestrianLightState::SharedPtr msg)
{
  last_ped_light_ = msg;
}

void RoadCrossingFsmNode::onRoute(
  const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg)
{
  if (!map_initialized_) {
    // Cannot set route without map. Capture goal for later.
    if (our_reroute_pending_phase_ == 0 && !original_goal_) {
      original_goal_ = msg->goal_pose;
      RCLCPP_INFO(get_logger(), "FSM captured original goal (no map yet)");
    }
    return;
  }

  route_handler_.setRoute(*msg);
  route_initialized_ = true;

  // Populate route lanelet sequence from the preferred primitives.
  // This covers both the initial external route and every reroute ack, so
  // checkArmCondition() always scans the up-to-date sequence.
  route_lanelets_.clear();
  for (const auto & seg : msg->segments) {
    route_lanelets_.push_back(static_cast<lanelet::Id>(seg.preferred_primitive.id));
  }
  RCLCPP_INFO(
    get_logger(), "FSM route_lanelets_ populated: %zu lanelets", route_lanelets_.size());

  if (our_reroute_pending_phase_ != 0) {
    // This route callback is the ack for one of our own reroute goal-publishes.
    const int completed_phase = our_reroute_pending_phase_;
    our_reroute_pending_phase_ = 0;
    // Signal to step-2 that the new route is confirmed SET — safe to check mode.
    reroute_route_acked_ = true;
    RCLCPP_INFO(
      get_logger(),
      "FSM route updated (ack for reroute phase %d) — reroute_route_acked_ set; "
      "original_goal preserved",
      completed_phase);
    // The sub-step state machine in onTimer drives what happens next;
    // adapi_route_state_ will also update via sub_route_state_.
  } else {
    // External route (new goal from user / mission planner not triggered by us).
    original_goal_ = msg->goal_pose;
    RCLCPP_INFO(
      get_logger(), "FSM captured new original goal (%.2f, %.2f)",
      msg->goal_pose.position.x, msg->goal_pose.position.y);

    // If we were mid-episode, reset for fresh start.
    if (state_ != FsmState::IDLE) {
      RCLCPP_INFO(get_logger(), "External route change — resetting FSM to IDLE");
      resetEpisode();
      transitionTo(FsmState::IDLE);
    }
  }
}

void RoadCrossingFsmNode::onMap(
  const autoware_map_msgs::msg::LaneletMapBin::SharedPtr msg)
{
  last_map_msg_ = msg;
  route_handler_.setMap(*msg);
  map_initialized_ = true;
  RCLCPP_INFO(get_logger(), "FSM received map");
}

void RoadCrossingFsmNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_ = msg;
}

void RoadCrossingFsmNode::onAimCmd(const std_msgs::msg::Float32::SharedPtr /*msg*/)
{
  // Catat waktu pesan terakhir dari rotary aimer.
  // Dipakai di onTimer untuk menentukan apakah aimer sudah running (cmd "fresh").
  last_aim_cmd_time_ = this->now();
}

void RoadCrossingFsmNode::onAdApiRouteState(
  const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
  adapi_route_state_ = msg->state;
}

void RoadCrossingFsmNode::onAdApiOpMode(
  const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg)
{
  adapi_autonomous_available_ = msg->is_autonomous_mode_available;
  adapi_op_mode_ = msg->mode;
}

// ---------------------------------------------------------------------------
// FSM tick
// ---------------------------------------------------------------------------

void RoadCrossingFsmNode::updateDrivingDirection()
{
  if (!last_odom_) return;
  const rclcpp::Time now = this->now();
  const double vx = last_odom_->twist.twist.linear.x;

  if (std::abs(vx) <= kDrivingDirectionDeadbandMps) {
    // Near-zero speed on this sample — twist alone is ambiguous. Drop any
    // in-progress consecutive-sample candidate (a stale candidate must not
    // survive across a gap) and start/continue the near-stationary timer.
    driving_direction_candidate_count_ = 0;
    if (!near_stationary_since_) near_stationary_since_ = now;

    const double near_stationary_elapsed = (now - *near_stationary_since_).seconds();
    if (near_stationary_elapsed >= kNearStationaryFallbackSec) {
      // Genuinely stopped for a while — twist will never recover on its own.
      // Fall back to the route's own geometric heading at ego's position
      // instead of leaving is_driving_forward_ latched at a stale value
      // (which would otherwise deadlock checkArmCondition() forever).
      const auto route_forward = deriveDrivingDirectionFromRoute();
      if (route_forward) {
        is_driving_forward_ = *route_forward;
      }
      // else: cannot resolve ego on the route yet — keep previous value.
    }
    return;
  }

  // Speed is past the deadband — no longer near-stationary.
  near_stationary_since_.reset();

  // Require kDrivingDirectionConsecutiveSamples consecutive samples on the
  // SAME side of the deadband before actually flipping is_driving_forward_,
  // so a single noisy sample can't latch the wrong direction.
  const bool candidate_forward = vx > 0.0;
  if (driving_direction_candidate_count_ > 0 &&
      driving_direction_candidate_forward_ == candidate_forward) {
    ++driving_direction_candidate_count_;
  } else {
    driving_direction_candidate_forward_ = candidate_forward;
    driving_direction_candidate_count_ = 1;
  }

  if (driving_direction_candidate_count_ >= kDrivingDirectionConsecutiveSamples) {
    is_driving_forward_ = driving_direction_candidate_forward_;
  }
}

std::optional<bool> RoadCrossingFsmNode::deriveDrivingDirectionFromRoute() const
{
  if (!last_odom_ || !route_initialized_) return std::nullopt;

  lanelet::ConstLanelet cur;
  if (!route_handler_.getClosestLaneletWithinRoute(last_odom_->pose.pose, &cur)) {
    return std::nullopt;
  }

  const auto centerline = cur.centerline2d().basicLineString();
  if (centerline.size() < 2) return std::nullopt;

  // Find the centerline vertex closest to ego, then take the next vertex
  // along the lanelet's own drawing direction as the "ahead" point — same
  // role as points.at(1) in autoware::motion_utils::isDrivingForward().
  const auto & ego_pos = last_odom_->pose.pose.position;
  const lanelet::BasicPoint2d ego2d{ego_pos.x, ego_pos.y};
  size_t nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < centerline.size(); ++i) {
    const double d = boost::geometry::distance(ego2d, centerline[i]);
    if (d < nearest_dist) {
      nearest_dist = d;
      nearest_idx = i;
    }
  }
  const size_t ahead_idx =
    (nearest_idx + 1 < centerline.size()) ? nearest_idx + 1 : nearest_idx - 1;
  const bool ahead_is_after = ahead_idx > nearest_idx;
  const auto & ahead_pt = centerline[ahead_idx];

  geometry_msgs::msg::Point dst_point;
  dst_point.x = ahead_pt.x();
  dst_point.y = ahead_pt.y();
  dst_point.z = 0.0;

  // If the "ahead" vertex is actually behind nearest_idx in the lanelet's own
  // vertex order (only happens at the very last vertex), the forward/backward
  // sense relative to the lanelet's drawing direction is flipped.
  const bool forward_along_centerline =
    autoware_utils_geometry::is_driving_forward(last_odom_->pose.pose, dst_point);
  return ahead_is_after ? forward_along_centerline : !forward_along_centerline;
}

void RoadCrossingFsmNode::onTimer()
{
  publishFsmState();

  // Guard: need map, route, and pose before FSM can do anything useful.
  if (!map_initialized_ || !route_initialized_ || !last_odom_) {
    return;
  }

  // Refresh direction-of-travel signal before any direction-dependent logic
  // (entry/exit resolution, route-order scans, aimer re-zero) runs this tick.
  updateDrivingDirection();

  const rclcpp::Time now = this->now();

  // ---------------------------------------------------------------------------
  // Blanket episode-duration watchdog (defensive watchdog #3, catch-all).
  //
  // Runs BEFORE the state switch below, on every tick, regardless of which state
  // the FSM is in. This is intentionally coarse and state-agnostic: it exists to
  // catch any stuck-state bug not covered by the more targeted CROSSING-timeout
  // (watchdog #1) or engage-wait-timeout (watchdog #2) fixes below. Placed here
  // (after the map/route/pose guard above, before the switch) so it can force a
  // reset+IDLE and short-circuit the rest of this tick without letting the switch
  // run stale/inconsistent state afterward.
  // ---------------------------------------------------------------------------
  if (state_ != FsmState::IDLE && episode_start_time_) {
    const double episode_elapsed = (now - *episode_start_time_).seconds();
    if (episode_elapsed > param_.max_episode_duration_sec) {
      RCLCPP_WARN(
        get_logger(),
        "onTimer WATCHDOG: episode stuck in state=%d for %.1f s (max=%.1f s) — "
        "forcing resetEpisode()+IDLE (blanket safety watchdog, catch-all)",
        static_cast<int>(state_), episode_elapsed, param_.max_episode_duration_sec);
      // Release any gate(s) we may still be holding at HOLD before tearing down the
      // episode — mirrors every other forced-exit path in this file (no-alternate
      // reroute fallback, REROUTING_TO_ALT failures, normal DONE). Without this the
      // scene module(s) we were gating would be left stuck at HOLD forever even
      // though the FSM itself moved on to IDLE.
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      // Suppress re-arming for the crossing we were managing BEFORE resetEpisode()
      // clears active_crossing_id_ — same re-arm-cooldown mechanism used for normal
      // DONE, so the same crossing doesn't immediately re-arm and re-wedge on the
      // very next tick (checkArmCondition() already honors suppressed_crossing_id_
      // generically, regardless of who set it).
      suppressed_crossing_id_ = active_crossing_id_;
      resetEpisode();
      transitionTo(FsmState::IDLE);
      return;
    }
  }

  // ---------------------------------------------------------------------------
  // Rotary aimer control block — dua fase:
  //
  // APPROACH (baru):
  //   Kondisi: FSM dalam waiting state, BELUM at_pre, distanceToPreEntry() valid dan
  //            <= aim_approach_dist_m, DAN route punya road_crossing ke depan.
  //   Aksi: aktifkan aimer + publish centroid road_crossing sebagai PointStamped ke
  //         /rotary_aimer/aim_target agar kamera menghadap crosswalk.
  //
  // AT PRE (bestehend, tidak berubah):
  //   Kondisi: ego's current lanelet IS pre_road_crossing DAN next route lanelet IS
  //            road_crossing (topologi-exact, bukan heuristik).
  //   Aksi: spam-enable aimer jika belum running — TIDAK publish target sehingga
  //         target approach yang sudah dikirim menjadi stale dan aimer fallback ke
  //         tf_pedestrian_light (current behavior).
  //
  // Disable (re-zero) tetap dilakukan di setiap situs transisi keluar (CROSSING entry,
  // REROUTING_TO_ALT, resetEpisode) — tidak diubah di sini.
  // ---------------------------------------------------------------------------
  {
    const bool waiting =
      (state_ == FsmState::ARMED_CROSSWALK || state_ == FsmState::CROSSWALK_OK ||
       state_ == FsmState::ARMED_LIGHT    || state_ == FsmState::WAIT_GREEN);

    // at_pre: ego's current lanelet IS a pre_road_crossing AND the NEXT lanelet in the
    // route sequence IS a road_crossing.  Uses route_lanelets_ (ordered mission-planner
    // sequence) so the check is topology-exact, not a velocity/distance heuristic.
    bool at_pre = false;
    lanelet::ConstLanelet cur;
    if (last_odom_ && route_handler_.getClosestLaneletWithinRoute(last_odom_->pose.pose, &cur)) {
      bool cur_is_pre = (cur.attributeOr("pre_road_crossing", std::string("false")) == "true");
      if (cur_is_pre) {
        for (size_t i = 0; i + 1 < route_lanelets_.size(); ++i) {
          if (route_lanelets_[i] == cur.id()) {
            try {
              const auto nxt =
                route_handler_.getLaneletMapPtr()->laneletLayer.get(route_lanelets_[i + 1]);
              if (nxt.attributeOr("road_crossing", std::string("false")) == "true") {
                at_pre = true;
              }
            } catch (const lanelet::NoSuchPrimitiveError &) {}
            break;
          }
        }
      }
    }
    // Guard: only activate when in a waiting state.
    at_pre = at_pre && waiting;

    // APPROACH: robot mendekati pre tapi belum masuk (waiting state, bukan at_pre,
    // jarak ke pre entry <= aim_approach_dist_m, dan ada road_crossing di route).
    const double pre_dist = distanceToPreEntry();
    const bool approaching =
      waiting &&
      !at_pre &&
      (pre_dist >= 0.0) &&
      (pre_dist <= param_.aim_approach_dist_m) &&
      active_crossing_id_.has_value();  // ada episode aktif dengan road_crossing

    // Aimer dianggap running jika ada pesan aim_cmd yang fresh.
    const double aim_age = (now - last_aim_cmd_time_).seconds();
    const bool aim_running = (aim_age < param_.aim_cmd_fresh_sec);

    if (approaching) {
      // Aktifkan aimer jika belum running.
      if (!aim_running) {
        const double since_last_send = (now - last_aim_enable_send_).seconds();
        if (since_last_send > param_.aim_enable_retry_sec) {
          setDetectorEnabled(cli_rotary_aimer_, true, "rotary_aimer");
          last_aim_enable_send_ = now;
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "onTimer APPROACH: spam-enable rotary_aimer (state=%d, pre_dist=%.2f m) — "
            "menunggu aimer running",
            static_cast<int>(state_), pre_dist);
        }
      }
      // Publish centroid road_crossing sebagai target eksternal setiap tick.
      // Rotary_aimer menganggap target "fresh" selama pesan terus datang;
      // begitu kita berhenti publish (masuk at_pre), target menjadi stale → fallback ke lampu.
      const auto centroid_opt = getRoadCrossingCentroid();
      if (centroid_opt) {
        geometry_msgs::msg::PointStamped target_msg;
        target_msg.header.stamp = now;
        target_msg.header.frame_id = "map";
        target_msg.point = *centroid_opt;
        pub_aim_target_->publish(target_msg);
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "onTimer APPROACH: publish aim_target centroid (%.2f, %.2f) pre_dist=%.2f m",
          centroid_opt->x, centroid_opt->y, pre_dist);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "onTimer APPROACH: getRoadCrossingCentroid() returned nullopt — "
          "aimer enabled but no target published (pre_dist=%.2f m)",
          pre_dist);
      }
    } else if (at_pre && !aim_running) {
      // AT PRE: spam-enable aimer; TIDAK publish target → target approach menjadi stale
      // → aimer fallback ke tf_pedestrian_light (current behavior tidak berubah).
      const double since_last_send = (now - last_aim_enable_send_).seconds();
      if (since_last_send > param_.aim_enable_retry_sec) {
        setDetectorEnabled(cli_rotary_aimer_, true, "rotary_aimer");
        last_aim_enable_send_ = now;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "onTimer AT PRE: spam-enable rotary_aimer (state=%d, cur_llt=%ld, aim_age=%.2f s) — "
          "menunggu aimer running",
          static_cast<int>(state_),
          last_odom_ ? cur.id() : -1L,
          aim_age);
      }
    }

    // -----------------------------------------------------------------------
    // CROSSWALK DETECTOR: lazy-enable by distance (primary trigger).
    //
    // Enable crosswalk_detector ONCE per episode when ego is within
    // crosswalk_enable_dist_m of the pre entry and a crossing episode is active
    // (active_pre_id_ OR active_crossing_id_ set).
    //
    // Fires during approach — BEFORE the aimer aims at the crosswalk (aim at
    // aim_approach_dist_m=10 m) — so the detector is already running and warmed
    // up when the camera first frames the crosswalk at 10 m.
    //
    // Guard: crosswalk_detector_lazy_enabled_ prevents repeat calls every tick.
    // Skip: if crossing tagged disable_crosswalk_check — same policy as today's
    //        arming-time enable.
    // Works regardless of FSM state as long as the crossing episode is active
    // (active_crossing_id_ set), which happens as soon as checkArmCondition()
    // arms the episode (sets active_pre_id_ + active_crossing_id_).
    // -----------------------------------------------------------------------
    if (!crosswalk_detector_lazy_enabled_ && active_crossing_id_.has_value()) {
      const double cw_dist = distanceToPreEntry();  // -1.0 if no pre / map not ready
      const bool in_range =
        (cw_dist >= 0.0 && cw_dist <= param_.crosswalk_enable_dist_m) ||
        // Fallback: no pre lanelet → use distance to crossing entry directly.
        (!active_pre_id_.has_value() &&
         distanceToCrossingEntry() >= 0.0 &&
         distanceToCrossingEntry() <= param_.crosswalk_enable_dist_m);
      if (in_range) {
        if (!crossingHasTag("disable_crosswalk_check")) {
          setDetectorEnabled(cli_crosswalk_detector_, true, "crosswalk_detector");
          crosswalk_detector_lazy_enabled_ = true;
          RCLCPP_INFO(
            get_logger(),
            "onTimer DISTANCE-GATE: crosswalk_detector enabled "
            "(crossing=%ld, pre_dist=%.2f m, threshold=%.1f m)",
            static_cast<long>(*active_crossing_id_),
            cw_dist, param_.crosswalk_enable_dist_m);
        } else {
          // Tagged disable_crosswalk_check: mark as handled so this block doesn't
          // re-run every tick, but do NOT send enable.
          crosswalk_detector_lazy_enabled_ = true;
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "onTimer DISTANCE-GATE: crossing %ld tagged disable_crosswalk_check — "
            "skipping crosswalk_detector enable (pre_dist=%.2f m)",
            static_cast<long>(*active_crossing_id_), cw_dist);
        }
      }
    }
  }

  switch (state_) {
    // -------------------------------------------------------------------
    case FsmState::IDLE: {
      if (checkArmCondition()) {
        transitionTo(FsmState::ARMED_CROSSWALK);
        // Arm the blanket episode-duration watchdog (defensive watchdog #3) for this
        // new episode — checkArmCondition() just (re)assigned active_pre_id_/
        // active_crossing_id_, so this is the natural "episode start" point.
        episode_start_time_ = now;
        // NOTE: crosswalk_detector is now enabled lazily by DISTANCE (crosswalk_enable_dist_m)
        // in the distance-gate block above, not here at arming time.
        // By the time checkArmCondition() succeeds (episode active), the distance-gate
        // has already fired (or will fire on the next tick once within range).
        // This keeps the enable earlier (20 m out) rather than at arming (~arm_hops_max
        // lanelets, which can be very close to the pre stop).
        // The disable_crosswalk_check skip is enforced in the distance-gate block.
        // Rotary aimer is NOT enabled here — enabling at approach distance causes the
        // platform to rotate from far away (unstable / weird).  Instead the aimer is
        // enabled lazily in the waiting states below once ego is AT the pre entry
        // (dist <= fail_distance_m).  See the at-pre aimer block in onTimer.
      }
      break;
    }

    // --------------------------------------------------------------------
    // ARMED_CROSSWALK
    //
    // Robot is approaching the pre_road_crossing stop.
    // Gate(pre) = HOLD (robot stops and waits).
    //
    // Two exit paths:
    //   (A) Crosswalk DETECTED (confidence >= threshold) before reroute_after_sec:
    //       → Release pre gate, transition CROSSWALK_OK → ARMED_LIGHT.
    //   (B) Crosswalk NOT detected and ego close to entry for >= reroute_after_sec:
    //       → REROUTING_TO_ALT (or ARMED_LIGHT fallback if reroute disabled).
    // -------------------------------------------------------------------
    case FsmState::ARMED_CROSSWALK: {
      // PRE gate = HOLD (robot waits at pre stop).
      // ROAD gate = HOLD (don't release road crossing yet).
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);

      // -----------------------------------------------------------------------
      // Position gate: robot harus sudah tiba di pre (atau crossing jika tanpa pre)
      // sebelum FSM boleh maju ke CROSSWALK_OK/ARMED_LIGHT.
      //   at_pre = true  jika distanceToPreEntry() ∈ [0, pre_arrive_dist_m]
      //   at_pre = true  (fallback tanpa pre) jika dist-crossing ∈ [0, crossing_arrive_dist_m]
      // Selama belum at_pre: tahan di ARMED_CROSSWALK, gate tetap HOLD.
      // -----------------------------------------------------------------------
      const bool at_pre = [&]() -> bool {
        if (active_pre_id_) {
          const double d = distanceToPreEntry();
          return d >= 0.0 && d <= param_.pre_arrive_dist_m;
        } else {
          // Tidak ada pre lanelet: pakai jarak ke crossing entry.
          const double d = distanceToCrossingEntry();
          return d >= 0.0 && d <= param_.crossing_arrive_dist_m;
        }
      }();

      if (!at_pre) {
        // Robot belum tiba — tetap tahan, log sekali per 2 detik.
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ARMED_CROSSWALK: robot belum tiba (pre_dist=%.2f m, gate=pre_arrive_dist_m=%.1f m) "
          "— tetap HOLD, menunggu robot mendekati pre/crossing",
          active_pre_id_ ? distanceToPreEntry() : distanceToCrossingEntry(),
          active_pre_id_ ? param_.pre_arrive_dist_m : param_.crossing_arrive_dist_m);
        break;
      }

      // Tag-based bypass: skip crosswalk detection for lanelets tagged disable_crosswalk_check.
      const bool skip_crosswalk = crossingHasTag("disable_crosswalk_check");

      // Path A: crosswalk detected — OR bypass via tag.
      if (skip_crosswalk ||
          (last_crosswalk_det_ && last_crosswalk_det_->detected &&
           last_crosswalk_det_->confidence >= param_.crosswalk_min_confidence))
      {
        if (skip_crosswalk) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "ARMED_CROSSWALK: crossing %ld tagged disable_crosswalk_check — "
            "bypassing crosswalk wait, proceeding directly to ARMED_LIGHT",
            active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L);
        }
        // Cancel any pending reroute timer — crosswalk found in time (or bypassed).
        pre_wait_timer_start_.reset();
        // Release PRE stop: crosswalk verified. Robot can now advance to road_crossing.
        publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
        // ONE transition per tick: go to CROSSWALK_OK. On the NEXT tick, CROSSWALK_OK
        // handler will transition to ARMED_LIGHT. Entry actions for ARMED_LIGHT
        // (ped_light enable + light_timeout_start_) run in the ARMED_LIGHT handler.
        transitionTo(FsmState::CROSSWALK_OK);
        break;
      }

      // Path B: crosswalk NOT detected — run the reroute_after_sec timer while
      // ego is close enough to the pre entry (or crossing entry if no pre).
      // When active_pre_id_ is set, ego stops at the PRE lanelet, which is
      // upstream of the road_crossing — so we must measure distance to the PRE
      // entry, not the crossing entry, otherwise dist > fail_distance_m always
      // and the timer never starts.
      const double dist = active_pre_id_ ? distanceToPreEntry() : distanceToCrossingEntry();
      if (dist >= 0.0 && dist <= param_.fail_distance_m) {
        if (!pre_wait_timer_start_) {
          pre_wait_timer_start_ = now;
          RCLCPP_INFO(
            get_logger(),
            "ARMED_CROSSWALK: ego within %.1f m of %s entry (dist=%.2f m) — "
            "starting reroute_after_sec timer (%.1f s)",
            param_.fail_distance_m,
            active_pre_id_ ? "pre" : "crossing",
            dist, param_.reroute_after_sec);
        } else {
          const double elapsed = (now - *pre_wait_timer_start_).seconds();
          if (elapsed >= param_.reroute_after_sec) {
            // Timer expired and still no crosswalk.
            // Check BEFORE committing to any reroute whether an alternate exists.
            if (param_.reroute_enabled) {
              const auto alt_opt = findAlternateInNextBranches();
              if (alt_opt) {
                // Alternate found — commit to reroute path.
                pre_wait_timer_start_.reset();
                RCLCPP_WARN(
                  get_logger(),
                  "ARMED_CROSSWALK: crosswalk not detected after %.1f s at pre — "
                  "alternate lanelet %ld found, initiating REROUTING_TO_ALT",
                  param_.reroute_after_sec,
                  static_cast<int64_t>(alt_opt->id()));
                // Keep PRE gate = HOLD while we search for alternate.
                transitionTo(FsmState::REROUTING_TO_ALT);
                // Robot mulai bergerak ke alternate — re-zero aimer (direction-aware; see
                // reZeroAimerDirectionAware()).
                reZeroAimerDirectionAware();
                resetRerouteSubstep();
              } else {
                // No alternate — stay in ARMED_CROSSWALK, keep gate HOLD,
                // wait indefinitely for crosswalk detection.
                // Reset the timer so this branch fires again after reroute_after_sec
                // if crosswalk is still not detected (avoids re-logging every tick).
                pre_wait_timer_start_ = now;
                RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 5000,
                  "ARMED_CROSSWALK: crosswalk not detected (%.1f s elapsed) and "
                  "no alternate_road_crossing available — holding at pre, "
                  "waiting for crosswalk detection indefinitely",
                  param_.reroute_after_sec);
              }
            } else {
              // Reroute disabled: fall through to ARMED_LIGHT without crosswalk.
              pre_wait_timer_start_.reset();
              RCLCPP_WARN(
                get_logger(),
                "ARMED_CROSSWALK: crosswalk not detected (%.1f s) and reroute disabled — "
                "falling through to ARMED_LIGHT",
                param_.reroute_after_sec);
              publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
              // ONE transition per tick. Entry actions for ARMED_LIGHT (ped_light enable
              // + light_timeout_start_) run in the ARMED_LIGHT handler on the next tick.
              transitionTo(FsmState::ARMED_LIGHT);
            }
          }
          // else: still within timeout, keep waiting.
        }
      } else {
        // Ego not yet close enough (or no active crossing) — reset timer.
        if (pre_wait_timer_start_) {
          pre_wait_timer_start_.reset();
        }
      }
      break;
    }

    // -------------------------------------------------------------------
    // CROSSWALK_OK
    //
    // Crosswalk has been verified (or bypassed via disable_crosswalk_check).
    // PRE gate is already GO (set before transitionTo(CROSSWALK_OK) upstream).
    // This state runs for exactly one tick, then transitions to ARMED_LIGHT.
    // Entry actions for ARMED_LIGHT (ped_light enable + light_timeout_start_)
    // are deferred to the ARMED_LIGHT handler — strictly one transition per tick.
    // -------------------------------------------------------------------
    case FsmState::CROSSWALK_OK: {
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
      // ONE transition per tick: CROSSWALK_OK → ARMED_LIGHT.
      transitionTo(FsmState::ARMED_LIGHT);
      break;
    }

    // -------------------------------------------------------------------
    // REROUTING_TO_ALT  (Phase A)
    //
    // Crosswalk was not detected within reroute_after_sec.
    // Strategy: find nearest alternate_road_crossing lanelet, set a new goal
    // via AdAPI (clear_route → publish goal → wait SET + autonomous_available
    // → change_to_autonomous), then release the pre gate so the robot can move.
    // Wait until ego leaves the pre lanelet before transitioning to Phase B.
    //
    // Sub-step machine (reroute_substep_):
    //   0 → call clear_route once
    //   1 → publish goal = alt pose once
    //   2 → wait route_state==SET AND autonomous_available → call change_to_autonomous once
    //   3 → wait egoLeftPreLanelet() → transition to REROUTING_TO_GOAL
    // -------------------------------------------------------------------
    case FsmState::REROUTING_TO_ALT: {
      // Keep gates HOLD until step 3 (ego actually moving).
      if (reroute_substep_ < 3) {
        publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
        publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
      } else {
        publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
        publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      }

      // --- Pre-flight checks (cooldown, original_goal, alternate) only before substep 0 ---
      if (reroute_substep_ == 0 && !reroute_clear_called_) {
        // Cooldown check.
        if (last_reroute_time_) {
          const double since = (now - *last_reroute_time_).seconds();
          if (since < param_.reroute_cooldown_sec) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "REROUTING_TO_ALT: cooldown active (%.1f / %.1f s) — waiting",
              since, param_.reroute_cooldown_sec);
            break;
          }
        }

        if (!original_goal_) {
          RCLCPP_ERROR(
            get_logger(),
            "REROUTING_TO_ALT: original_goal not captured — cannot reroute; "
            "fallback to ARMED_LIGHT");
          publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
          // ONE transition per tick. Entry actions for ARMED_LIGHT run next tick.
          transitionTo(FsmState::ARMED_LIGHT);
          break;
        }

        // Verify alternate still exists (SAFETY NET: alternate was checked before
        // entering REROUTING_TO_ALT; this guards against map update / blocklist
        // exhaustion that can happen between the check and arriving here).
        const auto alt_opt = findAlternateInNextBranches();
        if (!alt_opt) {
          // Alternate lost mid-process — do NOT release gate or cross.
          // Return to ARMED_CROSSWALK and hold at pre, waiting for crosswalk detection.
          RCLCPP_WARN(
            get_logger(),
            "REROUTING_TO_ALT step0: no alternate_road_crossing found (lost mid-process) — "
            "returning to ARMED_CROSSWALK, gate pre=HOLD, waiting for crosswalk detection");
          publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
          publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
          transitionTo(FsmState::ARMED_CROSSWALK);
          // Reset the pre_wait timer so ARMED_CROSSWALK starts fresh without immediately
          // re-triggering the reroute_after_sec check on the very next tick.
          pre_wait_timer_start_.reset();
          break;
        }

        // Cache alt pose and id for use in step 1.
        active_alt_id_ = alt_opt->id();
        reroute_blocklist_.insert(alt_opt->id());
        last_reroute_time_ = now;
        our_reroute_pending_phase_ = 1;

        RCLCPP_INFO(
          get_logger(),
          "REROUTING_TO_ALT step0: clear_route, then goal to alt lanelet %ld",
          static_cast<int64_t>(alt_opt->id()));
      }

      // --- Sub-step dispatch ---
      switch (reroute_substep_) {
        case 0: {
          // Send clear_route (best-effort) then immediately advance to step 1.
          // clear_route may fail with "route cannot be cleared while in use" when the
          // robot is moving — that is normal for mid-drive reroute.  Publishing a new
          // goal is sufficient to replan; do NOT block progress on clear success.
          if (!reroute_clear_called_) {
            sendAdApiClearRoute();  // fire-and-forget; result logged in async callback
            reroute_clear_called_ = true;
            reroute_substep_ = 1;
            RCLCPP_INFO(
              get_logger(),
              "REROUTING_TO_ALT step0: clear_route dispatched (best-effort) — advancing to step 1");
          }
          break;
        }
        case 1: {
          // Publish goal = alt pose (once).
          if (!reroute_goal_published_) {
            if (!active_alt_id_) {
              RCLCPP_ERROR(get_logger(), "REROUTING_TO_ALT step1: active_alt_id_ lost — fallback");
              publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
              // ONE transition per tick. Entry actions for ARMED_LIGHT run next tick.
              transitionTo(FsmState::ARMED_LIGHT);
              break;
            }
            try {
              const auto alt_llt =
                route_handler_.getLaneletMapPtr()->laneletLayer.get(*active_alt_id_);
              const geometry_msgs::msg::Pose alt_pose = poseAtCenterlineMid(alt_llt);
              if (sendAdApiPublishGoal(alt_pose)) {
                reroute_goal_published_ = true;
                reroute_substep_ = 2;
                RCLCPP_INFO(
                  get_logger(),
                  "REROUTING_TO_ALT step1: goal published to alt lanelet %ld (%.2f,%.2f)",
                  static_cast<int64_t>(*active_alt_id_),
                  alt_pose.position.x, alt_pose.position.y);
              }
              // else: retry next tick (publisher always succeeds; sendAdApiPublishGoal
              // returns false only if we want to gate — here it always returns true)
            } catch (const lanelet::NoSuchPrimitiveError &) {
              RCLCPP_ERROR(
                get_logger(),
                "REROUTING_TO_ALT step1: alt lanelet %ld not in map — fallback",
                static_cast<int64_t>(*active_alt_id_));
              publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
              // ONE transition per tick. Entry actions for ARMED_LIGHT run next tick.
              transitionTo(FsmState::ARMED_LIGHT);
            }
          }
          break;
        }
        case 2: {
          // Gate: wait for the new route ack BEFORE checking adapi_op_mode_.
          // adapi_op_mode_ may still be AUTONOMOUS from the OLD route — if we advance
          // without waiting for the ack we skip the engage call entirely.
          if (!reroute_route_acked_) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "REROUTING_TO_ALT step2: waiting for new route ack (reroute_route_acked_=false) "
              "— NOT advancing (mode=%u stale)",
              adapi_op_mode_);
            break;
          }

          // Engage-wait absolute timeout (defensive watchdog #2). Set ONCE on the first
          // tick this substep is actually processed (route acked) — tracks TOTAL elapsed
          // time waiting for AUTONOMOUS here, not time-since-last-retry (that's
          // last_engage_attempt_ / engage_retry_sec, a retry CADENCE, unaffected by this).
          if (!engage_wait_start_) {
            engage_wait_start_ = now;
          }
          if ((now - *engage_wait_start_).seconds() > param_.engage_timeout_sec) {
            RCLCPP_WARN(
              get_logger(),
              "REROUTING_TO_ALT step2 WATCHDOG: autonomous_available never became true "
              "after %.1f s (timeout=%.1f s, mode=%u, autonomous_available=%d) — "
              "abandoning reroute attempt, resetting IDLE",
              (now - *engage_wait_start_).seconds(), param_.engage_timeout_sec,
              adapi_op_mode_, adapi_autonomous_available_);
            resetEpisode();
            transitionTo(FsmState::IDLE);
            break;
          }

          // Route ack confirmed — now inspect mode on the NEW route.
          // OperationModeState constants: UNKNOWN=0, STOP=1, AUTONOMOUS=2, LOCAL=3, REMOTE=4.
          constexpr uint8_t kAutonomousMode = 2u;
          if (adapi_op_mode_ == kAutonomousMode) {
            // Mode is already AUTONOMOUS on the new route — advance.
            reroute_substep_ = 3;
            RCLCPP_INFO(
              get_logger(),
              "REROUTING_TO_ALT step2: route acked + mode AUTONOMOUS — advancing to step 3");
          } else if (adapi_autonomous_available_) {
            // Route acked, autonomous available but mode not yet AUTONOMOUS — retry engage.
            const bool should_retry =
              !last_engage_attempt_ ||
              (now - *last_engage_attempt_).seconds() >= param_.engage_retry_sec;
            if (should_retry) {
              if (sendAdApiEngage()) {
                last_engage_attempt_ = now;
                RCLCPP_INFO(
                  get_logger(),
                  "REROUTING_TO_ALT step2: engage attempt "
                  "(mode=%u, route_state=%u, autonomous_available=%d) — retrying until AUTONOMOUS",
                  adapi_op_mode_, adapi_route_state_, adapi_autonomous_available_);
              }
              // else: service not ready — no timestamp update, retry next tick
            }
          } else {
            // Route acked but autonomous not yet available — wait.
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "REROUTING_TO_ALT step2: route acked but autonomous_available=false "
              "(mode=%u, route_state=%u) — waiting",
              adapi_op_mode_, adapi_route_state_);
          }
          break;
        }
        case 3: {
          // Engage sent — gates already set GO above.
          // Set the move-start timestamp once (on first tick of this sub-step).
          if (!reroute_alt_move_start_) {
            reroute_alt_move_start_ = now;
            RCLCPP_INFO(
              get_logger(),
              "REROUTING_TO_ALT step3: move-start timestamp set — waiting "
              "min_time=%.1f s AND min_dist=%.1f m before rerouting to goal",
              param_.reroute_min_time_sec, param_.reroute_min_dist_m);
          }

          // Dual interlock: min elapsed time AND min distance from pre entry.
          const double elapsed = (now - *reroute_alt_move_start_).seconds();
          const double dist_from_pre = distanceToPreEntry();

          // Guard: distanceToPreEntry() returns -1 when active_pre_id_ is null.
          // In that case there is no pre lanelet to measure from, so we cannot
          // use the distance gate — fall back to time-only interlock.
          const bool dist_ok =
            (dist_from_pre < 0.0) ? true : (dist_from_pre >= param_.reroute_min_dist_m);
          const bool time_ok = (elapsed >= param_.reroute_min_time_sec);

          if (time_ok && dist_ok) {
            RCLCPP_INFO(
              get_logger(),
              "REROUTING_TO_ALT step3: interlock satisfied "
              "(elapsed=%.2f s >= %.1f s, dist_from_pre=%.2f m >= %.1f m%s) — "
              "transitioning to REROUTING_TO_GOAL",
              elapsed, param_.reroute_min_time_sec,
              dist_from_pre, param_.reroute_min_dist_m,
              (dist_from_pre < 0.0) ? " [no pre — time-only fallback]" : "");
            transitionTo(FsmState::REROUTING_TO_GOAL);
            resetRerouteSubstep();
            our_reroute_pending_phase_ = 2;
          } else {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "REROUTING_TO_ALT step3: waiting interlock "
              "(elapsed=%.2f/%.1f s, dist_from_pre=%.2f/%.1f m%s)",
              elapsed, param_.reroute_min_time_sec,
              dist_from_pre, param_.reroute_min_dist_m,
              (dist_from_pre < 0.0) ? " [no pre]" : "");
          }
          break;
        }
        default:
          break;
      }
      break;
    }

    // -------------------------------------------------------------------
    // REROUTING_TO_GOAL  (Phase B)
    //
    // Ego has left the pre lanelet and is heading toward (or at) the
    // alternate crossing.  Restore the original goal via AdAPI so the
    // mission planner routes ego from its current position back to
    // the original destination.
    //
    // Sub-step machine (reroute_substep_):
    //   0 → call clear_route once
    //   1 → publish goal = original_goal_ once
    //   2 → wait route_state==SET AND autonomous_available → call change_to_autonomous once
    //   3 → suppress original crossing + resetEpisode → IDLE
    // -------------------------------------------------------------------
    case FsmState::REROUTING_TO_GOAL: {
      switch (reroute_substep_) {
        case 0: {
          if (!original_goal_) {
            RCLCPP_ERROR(
              get_logger(),
              "REROUTING_TO_GOAL step0: original_goal lost — cannot restore route; "
              "resetting IDLE");
            // Reroute aborted — clear blocklist so the alternate is not permanently skipped
            // on future laps.
            reroute_blocklist_.clear();
            resetEpisode();
            transitionTo(FsmState::IDLE);
            break;
          }
          if (!reroute_clear_called_) {
            // Best-effort clear: robot may be driving (code 50000 "route in use") — that is
            // normal for mid-drive reroute.  Publishing the original goal is sufficient to
            // replan; do NOT block progress on clear success.
            sendAdApiClearRoute();  // fire-and-forget; result logged in async callback
            reroute_clear_called_ = true;
            reroute_substep_ = 1;
            RCLCPP_INFO(
              get_logger(),
              "REROUTING_TO_GOAL step0: clear_route dispatched (best-effort) — advancing to step 1");
          }
          break;
        }
        case 1: {
          if (!reroute_goal_published_) {
            if (!original_goal_) {
              RCLCPP_ERROR(
                get_logger(),
                "REROUTING_TO_GOAL step1: original_goal lost — resetting IDLE");
              // Reroute aborted — clear blocklist so the alternate is not permanently skipped
              // on future laps.
              reroute_blocklist_.clear();
              resetEpisode();
              transitionTo(FsmState::IDLE);
              break;
            }
            if (sendAdApiPublishGoal(*original_goal_)) {
              reroute_goal_published_ = true;
              reroute_substep_ = 2;
              RCLCPP_INFO(
                get_logger(),
                "REROUTING_TO_GOAL step1: goal published to original (%.2f,%.2f)",
                original_goal_->position.x, original_goal_->position.y);
            }
          }
          break;
        }
        case 2: {
          // Gate: wait for the new (original) route ack BEFORE checking adapi_op_mode_.
          // adapi_op_mode_ may still be AUTONOMOUS from the ALTERNATE route — if we
          // advance without the ack we skip the engage call and the robot stays IDLE.
          if (!reroute_route_acked_) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "REROUTING_TO_GOAL step2: waiting for original route ack "
              "(reroute_route_acked_=false) — NOT advancing (mode=%u stale)",
              adapi_op_mode_);
            break;
          }

          // Engage-wait absolute timeout (defensive watchdog #2). Set ONCE on the first
          // tick this substep is actually processed (route acked) — tracks TOTAL elapsed
          // time waiting for AUTONOMOUS here, not time-since-last-retry (that's
          // last_engage_attempt_ / engage_retry_sec, a retry CADENCE, unaffected by this).
          if (!engage_wait_start_) {
            engage_wait_start_ = now;
          }
          if ((now - *engage_wait_start_).seconds() > param_.engage_timeout_sec) {
            RCLCPP_WARN(
              get_logger(),
              "REROUTING_TO_GOAL step2 WATCHDOG: autonomous_available never became true "
              "after %.1f s (timeout=%.1f s, mode=%u, autonomous_available=%d) — "
              "abandoning reroute attempt, resetting IDLE",
              (now - *engage_wait_start_).seconds(), param_.engage_timeout_sec,
              adapi_op_mode_, adapi_autonomous_available_);
            resetEpisode();
            transitionTo(FsmState::IDLE);
            break;
          }

          // Original route ack confirmed — now inspect mode on the restored route.
          // OperationModeState constants: UNKNOWN=0, STOP=1, AUTONOMOUS=2, LOCAL=3, REMOTE=4.
          constexpr uint8_t kAutonomousMode = 2u;
          if (adapi_op_mode_ == kAutonomousMode) {
            // Mode is already AUTONOMOUS on the restored route — advance.
            reroute_substep_ = 3;
            RCLCPP_INFO(
              get_logger(),
              "REROUTING_TO_GOAL step2: route acked + mode AUTONOMOUS — advancing to step 3");
          } else if (adapi_autonomous_available_) {
            // Route acked, autonomous available but mode not yet AUTONOMOUS — retry engage.
            const bool should_retry =
              !last_engage_attempt_ ||
              (now - *last_engage_attempt_).seconds() >= param_.engage_retry_sec;
            if (should_retry) {
              if (sendAdApiEngage()) {
                last_engage_attempt_ = now;
                RCLCPP_INFO(
                  get_logger(),
                  "REROUTING_TO_GOAL step2: engage attempt "
                  "(mode=%u, route_state=%u, autonomous_available=%d) — retrying until AUTONOMOUS",
                  adapi_op_mode_, adapi_route_state_, adapi_autonomous_available_);
              }
              // else: service not ready — no timestamp update, retry next tick
            }
          } else {
            // Route acked but autonomous not yet available — wait.
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "REROUTING_TO_GOAL step2: route acked but autonomous_available=false "
              "(mode=%u, route_state=%u) — waiting",
              adapi_op_mode_, adapi_route_state_);
          }
          break;
        }
        case 3: {
          // Engage dispatched — suppress the original crossing and reset to IDLE.
          RCLCPP_INFO(
            get_logger(),
            "REROUTING_TO_GOAL step3: engage sent — suppressing crossing, resetting IDLE");
          suppressed_crossing_id_ = active_crossing_id_;
          // Reroute sequence fully complete — clear the alternate blocklist so a FUTURE lap
          // can reroute through the same alternate again. (Loop-prevention within a single
          // reroute is already handled by suppressed_crossing_id_.)
          reroute_blocklist_.clear();
          resetEpisode();
          transitionTo(FsmState::IDLE);
          break;
        }
        default:
          break;
      }
      break;
    }

    // -------------------------------------------------------------------
    // ARMED_LIGHT
    //
    // Crosswalk verified; PRE gate stays GO.  Road gate = HOLD until green.
    //
    // Entry actions (first tick only, guarded by !light_timeout_start_):
    //   • Enable ped_light_detector — unless disable_tf_light_check tagged.
    //   • Record light_timeout_start_ so WAIT_GREEN can apply the detector
    //     silence timeout.
    //
    // Then ONE transition per tick: ARMED_LIGHT → WAIT_GREEN.
    // -------------------------------------------------------------------
    case FsmState::ARMED_LIGHT: {
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);

      // Entry actions — run once on the first tick in this state.
      // light_timeout_start_ is always unset when entering ARMED_LIGHT
      // (resetEpisode clears it; no upstream code sets it before this state).
      if (!light_timeout_start_) {
        if (!crossingHasTag("disable_tf_light_check")) {
          setDetectorEnabled(cli_ped_light_detector_, true, "ped_light_detector");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "ARMED_LIGHT entry: crossing %ld tagged disable_tf_light_check — "
            "skipping ped_light_detector enable",
            active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L);
        }
        light_timeout_start_ = now;
      }

      // ONE transition per tick: proceed to WAIT_GREEN.
      transitionTo(FsmState::WAIT_GREEN);
      break;
    }

    // -------------------------------------------------------------------
    case FsmState::WAIT_GREEN: {
      // PRE gate stays GO (crosswalk verified, pre stop already released).
      // ROAD gate stays HOLD until green or timeout.
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);

      // -----------------------------------------------------------------------
      // Position gate: robot harus sudah tiba di crossing sebelum FSM boleh GO.
      // at_crossing = true jika distanceToCrossingEntry() ∈ [0, crossing_arrive_dist_m].
      // Selama belum at_crossing: tahan di WAIT_GREEN, gate HOLD, jangan GO.
      // -----------------------------------------------------------------------
      const double dist_to_crossing = distanceToCrossingEntry();
      const bool at_crossing =
        (dist_to_crossing >= 0.0 && dist_to_crossing <= param_.crossing_arrive_dist_m);

      if (!at_crossing) {
        // Belum tiba di crossing — reset no-detect counter dan tunggu.
        no_detect_since_.reset();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "WAIT_GREEN: robot belum tiba di crossing (dist=%.2f m, threshold=%.1f m) "
          "— tetap HOLD, menunggu robot mendekati crossing",
          dist_to_crossing, param_.crossing_arrive_dist_m);
        break;
      }

      // Tag-based bypass: skip pedestrian-light wait for lanelets tagged disable_tf_light_check.
      // This is NOT a green-light cross — use plain GO so the module's trust-light shortcut
      // does NOT fire; vehicle-scan (isPredictedBlocked) governs the crossing.
      if (crossingHasTag("disable_tf_light_check")) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "WAIT_GREEN: crossing %ld tagged disable_tf_light_check — skip lampu, GO (vehicle-scan governs)",
          active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L);
        light_timeout_start_.reset();
        no_detect_since_.reset();
        road_cross_is_green_ = false;   // bypass: trust-light shortcut must NOT fire
        crossing_timeout_start_ = now;  // arm CROSSING safety-timeout watchdog
        transitionTo(FsmState::CROSSING);
        // Robot mulai menyeberang — re-zero aimer (direction-aware; see
        // reZeroAimerDirectionAware()).
        reZeroAimerDirectionAware();
        publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
        break;
      }

      // -----------------------------------------------------------------------
      // Counter-based light evaluation (skema baru — hanya berlaku setelah at_crossing
      // dan hanya ketika disable_tf_light_check TIDAK di-set).
      //
      // Klasifikasi:
      //   GREEN (state ∈ green_states, yaitu 2/3) → GO + reset counter.
      //   RED (1) or COUNTDOWN_BLANK (4)           → HOLD + reset counter
      //                                               (deteksi valid, bukan NONE).
      //   NONE (0) atau no message (detektor diam) → akumulasi counter.
      //     • Jika counter terus-menerus > light_detect_timeout_sec → fallback GO.
      //     • Selama masih dalam batas → HOLD.
      //
      // Counter di-reset ke nullopt saat ada deteksi valid (GREEN/RED/BLANK ≠ NONE).
      // Dengan demikian: selama lampu RED, counter terus di-reset → robot menunggu
      // GREEN selamanya (benar). Hanya persistent NONE/diam memicu fallback GO.
      // -----------------------------------------------------------------------

      // Tentukan apakah ada deteksi valid dan apa state-nya.
      const bool has_message = (last_ped_light_ != nullptr);
      const uint8_t light_state = has_message ? last_ped_light_->state : 0u;
      const bool is_none_or_silent =
        !has_message ||
        (light_state == autoware_road_crossing_msgs::msg::PedestrianLightState::NONE);

      if (!is_none_or_silent) {
        // ---------------------------------------------------------------
        // Deteksi VALID (GREEN, RED, atau BLANK) — reset no-detect counter.
        // ---------------------------------------------------------------
        no_detect_since_.reset();

        if (isGreenState(light_state)) {
          // GREEN(2) atau COUNTDOWN_GREEN(3) → GO_GREEN.
          // GO_GREEN is a distinct "go" that ALSO encodes "because pedestrian light is GREEN".
          // The behavior module uses this to distinguish green-initiated crosses (trust-the-light
          // shortcut fires) from bypass/timeout crosses (vehicle-scan governs).
          RCLCPP_INFO(
            get_logger(),
            "WAIT_GREEN: lampu HIJAU (state=%u) — GO_GREEN", light_state);
          light_timeout_start_.reset();
          road_cross_is_green_ = true;   // persist GO_GREEN through CROSSING state
          crossing_timeout_start_ = now;  // arm CROSSING safety-timeout watchdog
          transitionTo(FsmState::CROSSING);
          // Robot mulai menyeberang (lampu HIJAU) — re-zero aimer (direction-aware).
          reZeroAimerDirectionAware();
          publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO_GREEN);
        } else {
          // RED(1) atau COUNTDOWN_BLANK(4) → HOLD, tunggu GREEN.
          // Counter sudah di-reset di atas — robot akan menunggu selama lampu merah.
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "WAIT_GREEN: lampu MERAH/BLANK (state=%u) — HOLD, menunggu GREEN", light_state);
        }
      } else {
        // ---------------------------------------------------------------
        // Tidak mendeteksi (NONE atau detektor diam) — akumulasi counter.
        // ---------------------------------------------------------------
        if (!no_detect_since_) {
          no_detect_since_ = now;
          RCLCPP_INFO(
            get_logger(),
            "WAIT_GREEN: tidak mendeteksi lampu (has_msg=%d, state=%u) — "
            "mulai no-detect counter (timeout=%.1f s)",
            has_message, light_state, param_.light_detect_timeout_sec);
        }

        const double no_detect_elapsed = (now - *no_detect_since_).seconds();

        if (no_detect_elapsed >= param_.light_detect_timeout_sec) {
          // Timeout: detektor tidak melaporkan apapun selama lebih dari batas — fallback GO.
          // This is NOT a green-light cross — use plain GO so the module's trust-light
          // shortcut does NOT fire; vehicle-scan (isPredictedBlocked) governs.
          RCLCPP_INFO(
            get_logger(),
            "WAIT_GREEN: tidak mendeteksi lampu selama %.1f s (timeout=%.1f s) "
            "— fallback GO (tidak ada lampu pejalan kaki di crossing ini)",
            no_detect_elapsed, param_.light_detect_timeout_sec);
          light_timeout_start_.reset();
          no_detect_since_.reset();
          road_cross_is_green_ = false;   // timeout fallback: trust-light shortcut must NOT fire
          crossing_timeout_start_ = now;  // arm CROSSING safety-timeout watchdog
          transitionTo(FsmState::CROSSING);
          // Robot mulai menyeberang — re-zero aimer (direction-aware).
          reZeroAimerDirectionAware();
          publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
        } else {
          // Masih dalam batas — tahan, log setiap 1 detik.
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "WAIT_GREEN: tidak mendeteksi lampu (%.1f / %.1f s) — HOLD",
            no_detect_elapsed, param_.light_detect_timeout_sec);
        }
      }
      break;
    }

    // -------------------------------------------------------------------
    case FsmState::CROSSING: {
      // Both gates GO — robot crosses the road.
      // Persist GO_GREEN throughout the crossing if initiated by a green light,
      // so the behavior module reliably sees GO_GREEN in its CHECKING state
      // (no single-tick race between FSM publishing and module reading).
      // Plain GO for bypass/timeout crosses — vehicle-scan governs those.
      const uint8_t road_gate_cmd = road_cross_is_green_
        ? autoware_road_crossing_msgs::msg::RoadCrossingGate::GO_GREEN
        : autoware_road_crossing_msgs::msg::RoadCrossingGate::GO;
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      publishGate(road_gate_cmd);

      // Detect ego has exited the crossing.
      //
      // Safety-timeout watchdog #1: egoPastCrossingExit() is a centerline-projection
      // check that can permanently fail to register "past exit" if is_driving_forward_
      // flips near/inside the crossing lanelet (bidirectional-driving reverse scenario).
      // checkArmCondition() — the only place active_pre_id_/active_crossing_id_ get
      // (re)assigned — only runs from IDLE, so a wedged CROSSING never self-corrects.
      // If crossing_timeout_start_ (set at the WAIT_GREEN -> CROSSING transition) has
      // been running longer than crossing_timeout_sec, force the SAME cleanup as the
      // normal exit path below, but flag it loudly as a safety-timeout-forced exit.
      const bool timed_out =
        crossing_timeout_start_ &&
        (now - *crossing_timeout_start_).seconds() > param_.crossing_timeout_sec;

      if (egoPastCrossingExit() || timed_out) {
        if (timed_out) {
          RCLCPP_WARN(
            get_logger(),
            "CROSSING WATCHDOG: safety-timeout-forced exit (NOT a normal geometric exit) — "
            "egoPastCrossingExit() never fired after %.1f s (timeout=%.1f s), "
            "pre=%ld crossing=%ld — forcing DONE",
            (now - *crossing_timeout_start_).seconds(), param_.crossing_timeout_sec,
            active_pre_id_ ? static_cast<int64_t>(*active_pre_id_) : -1L,
            active_crossing_id_ ? static_cast<int64_t>(*active_crossing_id_) : -1L);
        }
        transitionTo(FsmState::DONE);
        publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
        publishGate(road_gate_cmd);
        // Disable detectors — save compute.
        setDetectorEnabled(cli_crosswalk_detector_, false, "crosswalk_detector");
        setDetectorEnabled(cli_ped_light_detector_, false, "ped_light_detector");
        // Aimer is already disabled at CROSSING entry (WAIT_GREEN → CROSSING transitions).
        // resetEpisode() in DONE will send a redundant/safety disable as well.
      }
      break;
    }

    // -------------------------------------------------------------------
    case FsmState::DONE: {
      // Episode complete. Stay DONE for one cycle (for PlotJuggler to register),
      // then reset to IDLE for next crossing.
      // Suppress re-arming for the crossing we just finished BEFORE resetEpisode()
      // clears active_crossing_id_.
      suppressed_crossing_id_ = active_crossing_id_;
      resetEpisode();
      transitionTo(FsmState::IDLE);
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool RoadCrossingFsmNode::checkArmCondition()
{
  if (!map_initialized_ || !route_initialized_ || !last_odom_) return false;
  if (route_lanelets_.empty()) return false;

  // Bidirectional-driving guard (Phase 3c): the forward scan below assumes
  // route_lanelets_ index order == direction of travel (i.e. ego's route
  // index only increases). That holds while driving forward, but not while
  // ego is backing away/retreating along the route (index would need to
  // decrease). Rather than making the scan direction-aware (route-order
  // semantics for a reversing ego are still under design), we take the
  // simpler and safer option flagged in the audit: do not newly arm a
  // crossing episode while ego is actively reversing. Any already-armed
  // episode (active_crossing_id_ set) is unaffected — this only gates NEW
  // arming decisions.
  if (!is_driving_forward_) {
    return false;
  }

  // -----------------------------------------------------------------------
  // Step 1: resolve ego's current lanelet ID from the route.
  // We keep getClosestLaneletWithinRoute solely to get a reliable lanelet id
  // anchored on the route topology.  The BFS / look-ahead is replaced by a
  // forward scan of the stored route_lanelets_ sequence.
  // -----------------------------------------------------------------------
  lanelet::ConstLanelet cur;
  const auto & ego_pose = last_odom_->pose.pose;
  if (!route_handler_.getClosestLaneletWithinRoute(ego_pose, &cur)) {
    return false;
  }
  const lanelet::Id ego_id = cur.id();

  // -----------------------------------------------------------------------
  // Step 2: find ego's index in route_lanelets_.
  // If ego's lanelet is not in the route list (e.g. very first tick after
  // route set before the localization has fully converged), return false.
  // -----------------------------------------------------------------------
  int ego_idx = -1;
  for (int i = 0; i < static_cast<int>(route_lanelets_.size()); ++i) {
    if (route_lanelets_[i] == ego_id) {
      ego_idx = i;
      break;
    }
  }
  if (ego_idx < 0) {
    // Ego lanelet not found in route — cannot determine position in route sequence.
    return false;
  }

  // -----------------------------------------------------------------------
  // Step 3: clear stale suppression via route-index comparison.
  //
  // Find the route index of the suppressed crossing. If ego's index is strictly
  // PAST that crossing's index, the suppression is stale and must be released so
  // future laps or forward crossings can arm normally.
  // Also clear if the suppressed crossing ID is not in route_lanelets_ at all
  // (route has been replaced and the crossing is no longer on the path).
  // -----------------------------------------------------------------------
  if (suppressed_crossing_id_) {
    int supp_idx = -1;
    for (int i = 0; i < static_cast<int>(route_lanelets_.size()); ++i) {
      if (route_lanelets_[i] == *suppressed_crossing_id_) {
        supp_idx = i;
        break;
      }
    }
    // Also find the suppressed crossing's pre index (the lanelet before it that
    // is tagged pre_road_crossing) to avoid clearing suppression while ego is
    // still on the pre approach to the same crossing.
    bool ego_on_suppressed = (ego_id == *suppressed_crossing_id_);
    bool ego_on_pre_of_suppressed = false;
    if (supp_idx > 0) {
      const lanelet::Id candidate_pre_id = route_lanelets_[supp_idx - 1];
      try {
        const auto pre_cand =
          route_handler_.getLaneletMapPtr()->laneletLayer.get(candidate_pre_id);
        ego_on_pre_of_suppressed =
          (ego_id == candidate_pre_id) &&
          (pre_cand.attributeOr("pre_road_crossing", std::string("false")) == "true");
      } catch (const lanelet::NoSuchPrimitiveError &) {}
    }

    const bool should_clear =
      (!ego_on_suppressed && !ego_on_pre_of_suppressed) &&
      (supp_idx < 0 || ego_idx > supp_idx);

    if (should_clear) {
      RCLCPP_INFO(
        get_logger(),
        "checkArmCondition: ego route-idx %d past suppressed crossing %ld "
        "(supp route-idx %d) — clearing suppression",
        ego_idx,
        static_cast<int64_t>(*suppressed_crossing_id_),
        supp_idx);
      suppressed_crossing_id_.reset();
    }
  }

  // -----------------------------------------------------------------------
  // Step 4: forward scan of route_lanelets_ from ego's index.
  // Scan up to arm_hops_max entries ahead. For each entry:
  //   - if tagged pre_road_crossing → this is the PRE; then find the FIRST
  //     road_crossing AFTER it in the route → CROSSING. Arm.
  //   - elif tagged road_crossing (no pre seen in window) → PRE-less crossing.
  //     active_pre_id_ = nullopt, active_crossing_id_ = id. Arm.
  // Respect suppression: if resolved crossing == suppressed_crossing_id_, skip.
  // -----------------------------------------------------------------------
  const int scan_end =
    std::min(static_cast<int>(route_lanelets_.size()), ego_idx + param_.arm_hops_max + 1);

  for (int i = ego_idx; i < scan_end; ++i) {
    const lanelet::Id scan_id = route_lanelets_[i];
    lanelet::ConstLanelet scan_llt;
    try {
      scan_llt = route_handler_.getLaneletMapPtr()->laneletLayer.get(scan_id);
    } catch (const lanelet::NoSuchPrimitiveError &) {
      continue;
    }

    // --- PRE found in forward window ---
    if (scan_llt.attributeOr("pre_road_crossing", std::string("false")) == "true") {
      // Find the FIRST road_crossing lanelet AFTER this pre in the route sequence.
      std::optional<lanelet::Id> resolved_crossing;
      for (int j = i + 1; j < static_cast<int>(route_lanelets_.size()); ++j) {
        const lanelet::Id cand_id = route_lanelets_[j];
        try {
          const auto cand_llt =
            route_handler_.getLaneletMapPtr()->laneletLayer.get(cand_id);
          if (cand_llt.attributeOr("road_crossing", std::string("false")) == "true") {
            resolved_crossing = cand_id;
            break;
          }
        } catch (const lanelet::NoSuchPrimitiveError &) {
          continue;
        }
      }

      // Suppression: if the resolved crossing is the one we just finished, skip.
      if (suppressed_crossing_id_ && resolved_crossing == suppressed_crossing_id_) {
        RCLCPP_DEBUG(
          get_logger(),
          "checkArmCondition: pre %ld leads to suppressed crossing %ld — skipping",
          static_cast<int64_t>(scan_id),
          static_cast<int64_t>(*suppressed_crossing_id_));
        continue;
      }
      // If pre leads to a DIFFERENT crossing than suppressed, clear suppression.
      if (suppressed_crossing_id_ && resolved_crossing != suppressed_crossing_id_) {
        RCLCPP_INFO(
          get_logger(),
          "checkArmCondition: pre %ld leads to crossing %ld (≠ suppressed %ld) — "
          "clearing suppression, arming",
          static_cast<int64_t>(scan_id),
          resolved_crossing ? static_cast<int64_t>(*resolved_crossing) : -1L,
          static_cast<int64_t>(*suppressed_crossing_id_));
        suppressed_crossing_id_.reset();
      }

      active_pre_id_ = scan_id;
      active_crossing_id_ = resolved_crossing;
      return true;
    }

    // --- ROAD_CROSSING found directly (no pre encountered in window yet) ---
    if (scan_llt.attributeOr("road_crossing", std::string("false")) == "true") {
      if (suppressed_crossing_id_ && scan_id == *suppressed_crossing_id_) {
        RCLCPP_DEBUG(
          get_logger(),
          "checkArmCondition: road_crossing %ld suppressed — skipping",
          static_cast<int64_t>(scan_id));
        continue;
      }
      // Different crossing in range — clear stale suppression.
      if (suppressed_crossing_id_) {
        RCLCPP_INFO(
          get_logger(),
          "checkArmCondition: road_crossing %ld (≠ suppressed %ld) — "
          "clearing suppression, arming (PRE-less)",
          static_cast<int64_t>(scan_id),
          static_cast<int64_t>(*suppressed_crossing_id_));
        suppressed_crossing_id_.reset();
      }
      active_pre_id_ = std::nullopt;
      active_crossing_id_ = scan_id;
      return true;
    }
  }

  // No crossing in the forward window — clear suppression (ego has moved past it).
  if (suppressed_crossing_id_) {
    RCLCPP_INFO(
      get_logger(),
      "checkArmCondition: no crossing in forward window (ego-idx=%d, window=%d) — "
      "clearing suppression %ld",
      ego_idx, param_.arm_hops_max,
      static_cast<int64_t>(*suppressed_crossing_id_));
    suppressed_crossing_id_.reset();
  }
  return false;
}

bool RoadCrossingFsmNode::hasTaggedLaneletWithinHops(
  const lanelet::ConstLanelet & start, const std::string & attribute_key, int hops,
  lanelet::Id * found_id) const
{
  // BFS up to `hops` levels via getNextLanelets.
  std::vector<lanelet::ConstLanelet> frontier{start};
  std::set<lanelet::Id> visited{start.id()};

  for (int h = 0; h < hops && !frontier.empty(); ++h) {
    std::vector<lanelet::ConstLanelet> next_frontier;
    for (const auto & llt : frontier) {
      for (const auto & next : route_handler_.getNextLanelets(llt)) {
        if (!visited.insert(next.id()).second) continue;
        if (next.attributeOr(attribute_key, std::string("false")) == "true") {
          if (found_id) *found_id = next.id();
          return true;
        }
        next_frontier.push_back(next);
      }
    }
    frontier = std::move(next_frontier);
  }
  return false;
}

bool RoadCrossingFsmNode::crossingHasTag(const std::string & key) const
{
  auto check = [this, &key](const std::optional<lanelet::Id> & id) -> bool {
    if (!id) return false;
    try {
      const auto llt = route_handler_.getLaneletMapPtr()->laneletLayer.get(*id);
      return llt.attributeOr(key, std::string("false")) == "true";
    } catch (const lanelet::NoSuchPrimitiveError &) { return false; }
  };
  return check(active_crossing_id_) || check(active_pre_id_);
}

double RoadCrossingFsmNode::distanceToCrossingEntry() const
{
  if (!active_crossing_id_ || !last_odom_) return -1.0;
  if (!map_initialized_) return -1.0;

  try {
    const auto llt =
      route_handler_.getLaneletMapPtr()->laneletLayer.get(*active_crossing_id_);
    const auto & cl = llt.centerline2d().basicLineString();
    if (cl.empty()) return -1.0;

    const auto & ego_pos = last_odom_->pose.pose.position;
    const lanelet::BasicPoint2d ego2d{ego_pos.x, ego_pos.y};
    // Entry is the endpoint ego actually arrives at first: front() when driving
    // forward (lanelet centerline direction matches travel direction), back()
    // when reversing through this lanelet.
    const auto & entry_pt = is_driving_forward_ ? cl.front() : cl.back();
    const lanelet::BasicPoint2d entry{entry_pt.x(), entry_pt.y()};

    return boost::geometry::distance(ego2d, entry);
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return -1.0;
  }
}

double RoadCrossingFsmNode::distanceToPreEntry() const
{
  if (!active_pre_id_ || !last_odom_) return -1.0;
  if (!map_initialized_) return -1.0;

  try {
    const auto llt =
      route_handler_.getLaneletMapPtr()->laneletLayer.get(*active_pre_id_);
    const auto & cl = llt.centerline2d().basicLineString();
    if (cl.empty()) return -1.0;

    const auto & ego_pos = last_odom_->pose.pose.position;
    const lanelet::BasicPoint2d ego2d{ego_pos.x, ego_pos.y};
    // See distanceToCrossingEntry(): entry endpoint swaps with travel direction.
    const auto & entry_pt = is_driving_forward_ ? cl.front() : cl.back();
    const lanelet::BasicPoint2d entry{entry_pt.x(), entry_pt.y()};

    return boost::geometry::distance(ego2d, entry);
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return -1.0;
  }
}

bool RoadCrossingFsmNode::egoPastCrossingExit() const
{
  if (!active_crossing_id_ || !last_odom_) return false;
  if (!map_initialized_) return false;

  try {
    const auto llt =
      route_handler_.getLaneletMapPtr()->laneletLayer.get(*active_crossing_id_);
    const auto cl = llt.centerline2d().basicLineString();
    if (cl.size() < 2) return false;

    const auto & ego_pos = last_odom_->pose.pose.position;
    // Entry/exit swap with travel direction — see distanceToCrossingEntry().
    const auto & entry = is_driving_forward_ ? cl.front() : cl.back();
    const auto & exit = is_driving_forward_ ? cl.back() : cl.front();

    // Project (ego - exit) onto (entry → exit).
    const double cdx = exit.x() - entry.x();
    const double cdy = exit.y() - entry.y();
    const double proj =
      (ego_pos.x - exit.x()) * cdx + (ego_pos.y - exit.y()) * cdy;
    return proj > 0.0;
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return false;
  }
}

bool RoadCrossingFsmNode::egoLeftPreLanelet() const
{
  // Returns true when ego's closest-within-route lanelet is no longer the
  // active pre lanelet.  Used by REROUTING_TO_ALT to detect that the robot
  // has actually started moving away from the pre stop.
  if (!active_pre_id_ || !last_odom_) {
    // No active pre lanelet to compare against — consider departed.
    return true;
  }
  if (!map_initialized_ || !route_initialized_) return false;

  lanelet::ConstLanelet cur;
  if (!route_handler_.getClosestLaneletWithinRoute(last_odom_->pose.pose, &cur)) {
    // Cannot resolve current lanelet — conservatively say not yet departed.
    return false;
  }
  // Departed when closest lanelet is anything other than the pre lanelet.
  return cur.id() != *active_pre_id_;
}

std::optional<lanelet::ConstLanelet> RoadCrossingFsmNode::findAlternateInNextBranches() const
{
  // -----------------------------------------------------------------------
  // 1-level next-branch alternate lookup.
  //
  // Topology:
  //
  //   FORK_PARENT ─┬─ pre_road_crossing  → road_crossing
  //                └─ alternate_road_crossing   ← target
  //
  // The fork parent is resolved from route_lanelets_ as the lanelet immediately
  // BEFORE active_pre_id_ in the route sequence.  This is more reliable than
  // getPreviousLanelets(pre) because it is anchored on the mission-planner
  // route and is immune to ego stop distance: even if ego parks 2 m before the
  // pre, the parent index is always (index_of(pre) - 1), not ego's current
  // lanelet.
  //
  // Algorithm:
  //   1. If active_pre_id_ is set AND it appears in route_lanelets_ at index i > 0,
  //      the fork parent = route_lanelets_[i-1].
  //   2. Fallback A (pre in route but at index 0, or map lookup fails):
  //      use getPreviousLanelets(pre) — same as before.
  //   3. Fallback B (no active pre — crossing-only map):
  //      use ego's current lanelet via getClosestLaneletWithinRoute.
  //   4. For each parent candidate, scan getNextLanelets(parent) 1 level only.
  //      Return the first next lanelet tagged "alternate_road_crossing"=="true"
  //      AND NOT in reroute_blocklist_.
  //   5. Nothing found → nullopt.
  // -----------------------------------------------------------------------

  if (!map_initialized_) return std::nullopt;

  // Bidirectional-driving guard (Phase 3c): same reasoning as checkArmCondition() —
  // the fork-parent resolution below assumes route_lanelets_ index order == travel
  // order (parent = pre_idx - 1). While ego is reversing that assumption doesn't
  // hold, and picking the wrong fork parent could route the ARM_CROSSWALK reroute
  // decision onto a wrong branch. Per the audit's simpler/safer option, reroute
  // branch-finding is disabled entirely while ego is reversing — the FSM will hold
  // at the pre stop and keep waiting for crosswalk detection instead.
  if (!is_driving_forward_) return std::nullopt;

  lanelet::ConstLanelets parents;

  if (active_pre_id_) {
    // Step 1: find pre index in route_lanelets_.
    int pre_idx = -1;
    for (int i = 0; i < static_cast<int>(route_lanelets_.size()); ++i) {
      if (route_lanelets_[i] == *active_pre_id_) {
        pre_idx = i;
        break;
      }
    }

    if (pre_idx > 0) {
      // Route predecessor = the lanelet immediately before pre in route order.
      const lanelet::Id parent_id = route_lanelets_[pre_idx - 1];
      try {
        const auto parent_llt =
          route_handler_.getLaneletMapPtr()->laneletLayer.get(parent_id);
        parents.push_back(parent_llt);
        RCLCPP_DEBUG(
          get_logger(),
          "findAlternateInNextBranches: fork-parent %ld derived from route "
          "(pre_idx=%d, parent_idx=%d)",
          static_cast<int64_t>(parent_id), pre_idx, pre_idx - 1);
      } catch (const lanelet::NoSuchPrimitiveError &) {
        // Map lookup failed — fall through to getPreviousLanelets.
      }
    }

    // Fallback A: getPreviousLanelets(pre) when route-based parent is unavailable.
    if (parents.empty()) {
      try {
        const auto pre_llt =
          route_handler_.getLaneletMapPtr()->laneletLayer.get(*active_pre_id_);
        parents = route_handler_.getPreviousLanelets(pre_llt);
        RCLCPP_DEBUG(
          get_logger(),
          "findAlternateInNextBranches: using getPreviousLanelets(pre=%ld) fallback "
          "(%zu parents)",
          static_cast<int64_t>(*active_pre_id_), parents.size());
      } catch (const lanelet::NoSuchPrimitiveError &) {}
    }
  }

  // Fallback B: no active pre (crossing-only map) → use ego's current lanelet.
  if (parents.empty() && last_odom_) {
    lanelet::ConstLanelet ego_llt;
    if (route_handler_.getClosestLaneletWithinRoute(last_odom_->pose.pose, &ego_llt)) {
      parents.push_back(ego_llt);
      RCLCPP_DEBUG(
        get_logger(),
        "findAlternateInNextBranches: no active pre — using ego lanelet %ld as parent",
        static_cast<int64_t>(ego_llt.id()));
    }
  }

  if (parents.empty()) {
    RCLCPP_DEBUG(
      get_logger(),
      "findAlternateInNextBranches: no parent/ego lanelet resolved — returning nullopt");
    return std::nullopt;
  }

  // Step 4: search each parent's direct next-branches, 1 level only.
  for (const auto & parent : parents) {
    for (const auto & next : route_handler_.getNextLanelets(parent)) {
      if (next.attributeOr("alternate_road_crossing", std::string("false")) != "true") {
        continue;
      }
      if (reroute_blocklist_.count(next.id()) > 0) {
        RCLCPP_DEBUG(
          get_logger(),
          "findAlternateInNextBranches: alternate %ld blocklisted — skipping",
          static_cast<int64_t>(next.id()));
        continue;
      }
      RCLCPP_INFO(
        get_logger(),
        "findAlternateInNextBranches: found alternate_road_crossing %ld "
        "(next-branch of fork-parent %ld; pre=%ld)",
        static_cast<int64_t>(next.id()),
        static_cast<int64_t>(parent.id()),
        active_pre_id_ ? static_cast<long>(*active_pre_id_) : -1L);
      return next;
    }
  }

  // Step 5: no tagged branch found (or all blocklisted).
  RCLCPP_DEBUG(
    get_logger(),
    "findAlternateInNextBranches: no alternate_road_crossing in next-branches of "
    "%zu parent(s) — returning nullopt (hold-forever)",
    parents.size());
  return std::nullopt;
}

geometry_msgs::msg::Pose RoadCrossingFsmNode::poseAtCenterlineMid(
  const lanelet::ConstLanelet & llt) const
{
  geometry_msgs::msg::Pose pose;
  const auto cl = llt.centerline2d().basicLineString();

  if (cl.empty()) {
    return pose;
  }

  if (cl.size() == 1) {
    pose.position.x = cl.front().x();
    pose.position.y = cl.front().y();
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
  }

  // Mid-point index.
  const size_t mid = cl.size() / 2;
  pose.position.x = cl[mid].x();
  pose.position.y = cl[mid].y();
  pose.position.z = 0.0;

  // Heading from mid-1 → mid.
  const double dx = cl[mid].x() - cl[mid - 1].x();
  const double dy = cl[mid].y() - cl[mid - 1].y();
  const double yaw = std::atan2(dy, dx);
  // Convert yaw to quaternion (z-axis rotation).
  pose.orientation.w = std::cos(yaw * 0.5);
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw * 0.5);

  return pose;
}

std::optional<geometry_msgs::msg::Point> RoadCrossingFsmNode::getRoadCrossingCentroid() const
{
  // -----------------------------------------------------------------------
  // Cari road_crossing lanelet yang mengikuti active_pre_id_ dalam route_lanelets_.
  // Mirrors the at_pre route-walk logic in the onTimer aimer block:
  //   find active_pre_id_ in route_lanelets_, then scan forward for the first
  //   lanelet tagged road_crossing=="true".
  // Jika active_crossing_id_ sudah diset (checkArmCondition() telah jalan), pakai
  // langsung tanpa scan ulang — lebih cepat dan konsisten.
  //
  // Centroid = rata-rata semua titik leftBound + rightBound (x, y, z).
  // -----------------------------------------------------------------------
  if (!map_initialized_) return std::nullopt;

  // Prefer active_crossing_id_ if already set by checkArmCondition().
  lanelet::Id crossing_id;
  if (active_crossing_id_) {
    crossing_id = *active_crossing_id_;
  } else {
    // Fallback: scan route_lanelets_ from active_pre_id_ forward.
    if (!active_pre_id_) return std::nullopt;
    int pre_idx = -1;
    for (int i = 0; i < static_cast<int>(route_lanelets_.size()); ++i) {
      if (route_lanelets_[i] == *active_pre_id_) {
        pre_idx = i;
        break;
      }
    }
    if (pre_idx < 0) return std::nullopt;

    bool found = false;
    for (int j = pre_idx + 1; j < static_cast<int>(route_lanelets_.size()); ++j) {
      try {
        const auto cand =
          route_handler_.getLaneletMapPtr()->laneletLayer.get(route_lanelets_[j]);
        if (cand.attributeOr("road_crossing", std::string("false")) == "true") {
          crossing_id = route_lanelets_[j];
          found = true;
          break;
        }
      } catch (const lanelet::NoSuchPrimitiveError &) {
        continue;
      }
    }
    if (!found) return std::nullopt;
  }

  try {
    const auto llt = route_handler_.getLaneletMapPtr()->laneletLayer.get(crossing_id);

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int count = 0;

    for (const auto & pt : llt.leftBound()) {
      sum_x += pt.x();
      sum_y += pt.y();
      sum_z += pt.z();
      ++count;
    }
    for (const auto & pt : llt.rightBound()) {
      sum_x += pt.x();
      sum_y += pt.y();
      sum_z += pt.z();
      ++count;
    }

    if (count == 0) return std::nullopt;

    geometry_msgs::msg::Point centroid;
    centroid.x = sum_x / count;
    centroid.y = sum_y / count;
    centroid.z = sum_z / count;
    return centroid;
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return std::nullopt;
  }
}

void RoadCrossingFsmNode::setDetectorEnabled(
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client,
  bool enable, const std::string & name)
{
  if (!client->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service %s not ready; skipping enable=%d", name.c_str(), enable);
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = enable;
  client->async_send_request(
    req,
    [this, name, enable](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto & res = future.get();
      if (res->success) {
        RCLCPP_INFO(get_logger(), "%s enable=%d: OK (%s)", name.c_str(), enable, res->message.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "%s enable=%d: FAILED (%s)", name.c_str(), enable, res->message.c_str());
      }
    });
}

void RoadCrossingFsmNode::reZeroAimerDirectionAware()
{
  if (is_driving_forward_) {
    // Forward: unchanged behavior — disable triggers the aimer's own
    // zero_on_disable ramp back to rotary angle 0 (nose), which is correct
    // since the nose leads while driving forward.
    setDetectorEnabled(cli_rotary_aimer_, false, "rotary_aimer");
    return;
  }

  // Reversing: do NOT disable. See header comment on this function — the
  // aimer's hardcoded "re-zero to nose" would point the turret away from the
  // direction of travel while reversing. Leaving it enabled keeps it running
  // its own TF/pose-based tf_pedestrian light search, which is direction-
  // correct by construction (unlike the fixed-zero fallback).
  RCLCPP_INFO(
    get_logger(),
    "reZeroAimerDirectionAware: ego reversing — skipping aimer disable/nose-zero, "
    "leaving aimer enabled to keep tracking via TF/pose-based light search");
}

// ---------------------------------------------------------------------------
// AdAPI reroute helpers
// ---------------------------------------------------------------------------

bool RoadCrossingFsmNode::sendAdApiClearRoute()
{
  if (!cli_clear_route_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "AdAPI clear_route service not ready — will retry next tick");
    return false;
  }
  auto req = std::make_shared<autoware_adapi_v1_msgs::srv::ClearRoute::Request>();
  cli_clear_route_->async_send_request(
    req,
    [this](rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedFuture future) {
      const auto & res = future.get();
      if (res->status.success) {
        RCLCPP_INFO(get_logger(), "AdAPI clear_route: OK");
      } else {
        RCLCPP_WARN(
          get_logger(), "AdAPI clear_route: FAILED (code=%u msg='%s')",
          res->status.code, res->status.message.c_str());
      }
    });
  return true;
}

bool RoadCrossingFsmNode::sendAdApiPublishGoal(const geometry_msgs::msg::Pose & goal_pose)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.stamp = this->now();
  stamped.header.frame_id = "map";
  stamped.pose = goal_pose;
  pub_goal_->publish(stamped);
  RCLCPP_INFO(
    get_logger(), "AdAPI goal published (%.2f, %.2f)",
    goal_pose.position.x, goal_pose.position.y);
  return true;
}

bool RoadCrossingFsmNode::sendAdApiEngage()
{
  if (!cli_engage_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "AdAPI change_to_autonomous service not ready — will retry next tick");
    return false;
  }
  auto req = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
  cli_engage_->async_send_request(
    req,
    [this](rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future) {
      const auto & res = future.get();
      if (res->status.success) {
        RCLCPP_INFO(get_logger(), "AdAPI change_to_autonomous: OK");
      } else {
        RCLCPP_WARN(
          get_logger(), "AdAPI change_to_autonomous: FAILED (code=%u msg='%s')",
          res->status.code, res->status.message.c_str());
      }
    });
  return true;
}

void RoadCrossingFsmNode::resetRerouteSubstep()
{
  reroute_substep_ = 0;
  reroute_clear_called_ = false;
  reroute_goal_published_ = false;
  reroute_engage_called_ = false;
  reroute_alt_move_start_.reset();
  last_engage_attempt_.reset();  // allow immediate first attempt in next phase
  // Engage-wait absolute-timeout timer (defensive watchdog #2) — reset per phase so
  // each new REROUTING_TO_ALT/REROUTING_TO_GOAL phase gets a fresh engage_timeout_sec
  // budget, tracked from that phase's own first step-2 tick.
  engage_wait_start_.reset();
  // Must be reset to false so step-2 waits for the NEW route's ack before
  // checking adapi_op_mode_.  A phase-A ack must NOT carry into phase B.
  reroute_route_acked_ = false;
}

void RoadCrossingFsmNode::publishGate(uint8_t command)
{
  autoware_road_crossing_msgs::msg::RoadCrossingGate gate;
  gate.header.stamp = this->now();
  gate.header.frame_id = "map";
  gate.lanelet_id = active_crossing_id_ ? static_cast<int64_t>(*active_crossing_id_) : -1;
  gate.command = command;
  gate.fsm_state = static_cast<uint8_t>(static_cast<int32_t>(state_));
  pub_gate_->publish(gate);
}

void RoadCrossingFsmNode::publishGatePre(uint8_t command)
{
  // No-op when there is no paired pre_road_crossing lanelet for this episode.
  if (!active_pre_id_) return;

  autoware_road_crossing_msgs::msg::RoadCrossingGate gate;
  gate.header.stamp = this->now();
  gate.header.frame_id = "map";
  gate.lanelet_id = static_cast<int64_t>(*active_pre_id_);
  gate.command = command;
  gate.fsm_state = static_cast<uint8_t>(static_cast<int32_t>(state_));
  pub_gate_->publish(gate);
}

void RoadCrossingFsmNode::publishFsmState()
{
  autoware_internal_debug_msgs::msg::Int32Stamped msg;
  msg.stamp = this->now();
  msg.data = static_cast<int32_t>(state_);
  pub_fsm_state_->publish(msg);
}

void RoadCrossingFsmNode::transitionTo(FsmState new_state)
{
  static const auto state_name = [](FsmState s) -> const char * {
    switch (s) {
      case FsmState::IDLE:              return "IDLE";
      case FsmState::ARMED_CROSSWALK:   return "ARMED_CROSSWALK";
      case FsmState::CROSSWALK_OK:      return "CROSSWALK_OK";
      case FsmState::REROUTING_TO_ALT:  return "REROUTING_TO_ALT";
      case FsmState::ARMED_LIGHT:       return "ARMED_LIGHT";
      case FsmState::WAIT_GREEN:        return "WAIT_GREEN";
      case FsmState::CROSSING:          return "CROSSING";
      case FsmState::DONE:              return "DONE";
      case FsmState::REROUTING_TO_GOAL: return "REROUTING_TO_GOAL";
      default:                          return "UNKNOWN";
    }
  };

  RCLCPP_INFO(
    get_logger(), "FSM: %s → %s",
    state_name(state_), state_name(new_state));
  state_ = new_state;
}

bool RoadCrossingFsmNode::isGreenState(uint8_t state_val) const
{
  for (const auto & gs : param_.green_states) {
    if (gs == static_cast<int64_t>(state_val)) return true;
  }
  return false;
}

void RoadCrossingFsmNode::resetEpisode()
{
  active_pre_id_.reset();
  active_crossing_id_.reset();
  pre_wait_timer_start_.reset();
  light_timeout_start_.reset();
  crossing_timeout_start_.reset();  // reset CROSSING safety-timeout watchdog (#1)
  episode_start_time_.reset();      // reset blanket episode-duration watchdog (#3)
  no_detect_since_.reset();  // reset no-detect counter saat episode baru/abort
  last_crosswalk_det_.reset();
  last_ped_light_.reset();
  road_cross_is_green_ = false;  // reset green-cross flag for next episode
  // Reset reroute sub-step state so the next episode starts fresh.
  resetRerouteSubstep();
  active_alt_id_.reset();
  // Note: our_reroute_pending_phase_ is intentionally NOT reset here.
  // If we resetEpisode while a route call is in-flight, the pending flag must
  // remain so onRoute does not misinterpret the ack as an external route and
  // overwrite original_goal_.  It is cleared by onRoute when the ack arrives.

  // Reset lazy-enable flag so the next episode triggers crosswalk_detector enable
  // via the distance-gate block again (once within crosswalk_enable_dist_m of new pre).
  crosswalk_detector_lazy_enabled_ = false;

  // Disable rotary aimer setiap episode reset/abort-ke-IDLE agar platform re-zero
  // saat penyeberangan dibatalkan atau selesai lewat jalur lain — direction-aware
  // (see reZeroAimerDirectionAware()). Idempotent — SetBool false aman dikirim
  // meski sudah disabled.
  reZeroAimerDirectionAware();
}

}  // namespace autoware::road_crossing_fsm

// ---------------------------------------------------------------------------
// Component registration
// ---------------------------------------------------------------------------
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::road_crossing_fsm::RoadCrossingFsmNode)
