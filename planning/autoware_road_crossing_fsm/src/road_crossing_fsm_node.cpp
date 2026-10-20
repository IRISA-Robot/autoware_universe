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
#include <sstream>
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
  // Interlock lampu hijau (interlock.plan.md) — satuan tick FSM (1/update_rate_hz).
  param_.threshold_merah_valid = declare_parameter<int>("threshold_merah_valid", 3);
  param_.threshold_hijau_valid = declare_parameter<int>("threshold_hijau_valid", 3);
  param_.threshold_merah_off = declare_parameter<int>("threshold_merah_off", 10);
  param_.threshold_hijau_off = declare_parameter<int>("threshold_hijau_off", 3);
  param_.threshold_interlock_hijau_solid =
    declare_parameter<int>("threshold_interlock_hijau_solid", 20);
  param_.reroute_enabled = declare_parameter<bool>("reroute_enabled", true);
  param_.reroute_cooldown_sec = declare_parameter<double>("reroute_cooldown_sec", 60.0);
  param_.reroute_min_time_sec = declare_parameter<double>("reroute_min_time_sec", 5.0);
  param_.reroute_min_dist_m = declare_parameter<double>("reroute_min_dist_m", 5.0);
  param_.engage_retry_sec = declare_parameter<double>("engage_retry_sec", 2.0);

  param_.topic_pedestrian_light = declare_parameter<std::string>(
    "topic_pedestrian_light", "/perception/road_crossing/pedestrian_light");
  param_.topic_gate = declare_parameter<std::string>(
    "topic_gate", "/planning/road_crossing/gate");
  param_.topic_fsm_state = declare_parameter<std::string>(
    "topic_fsm_state", "/planning/road_crossing/fsm_state");
  param_.topic_light_status = declare_parameter<std::string>(
    "topic_light_status", "/planning/road_crossing/light_status");
  param_.topic_route = declare_parameter<std::string>(
    "topic_route", "/planning/mission_planning/route");
  param_.topic_map = declare_parameter<std::string>(
    "topic_map", "/map/vector_map");
  param_.topic_ego_pose = declare_parameter<std::string>(
    "topic_ego_pose", "/localization/kinematic_state");
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

  // --- Parameter aimer ---
  // Jarak (m) dari pre entry di bawahnya aimer diaktifkan dan membidik lampu.
  param_.aim_approach_dist_m = declare_parameter<double>("aim_approach_dist_m", 10.0);
  param_.topic_target_light = declare_parameter<std::string>(
    "topic_target_light", "/planning/road_crossing/target_light");

  // --- Parameter position gate (mencegah FSM maju sebelum robot tiba) ---
  // Ego dianggap "sudah di pre" jika distanceToPreEntry() ∈ [0, pre_arrive_dist_m].
  // Harus >= pre_stop_dist_m agar memicu saat robot sudah berhenti di pre-stop.
  param_.pre_arrive_dist_m = declare_parameter<double>("pre_arrive_dist_m", 3.0);
  // Ego dianggap "sudah di crossing" jika distanceToCrossingEntry() ∈ [0, crossing_arrive_dist_m].
  param_.crossing_arrive_dist_m = declare_parameter<double>("crossing_arrive_dist_m", 1.5);
  param_.relocalize_jump_m = declare_parameter<double>("relocalize_jump_m", 2.0);

  // --- Safety-timeout watchdogs (defensive, additive) ---
  // CROSSING safety timeout: forces DONE if egoPastCrossingExit() never fires
  // (e.g. is_driving_forward_ flips near/inside the crossing lanelet).
  param_.crossing_timeout_sec = declare_parameter<double>("crossing_timeout_sec", 60.0);
  // Engage-wait absolute timeout: abandon reroute attempt if autonomous_available
  // never becomes true within this many seconds of entering the engage-retry substep.
  param_.engage_timeout_sec = declare_parameter<double>("engage_timeout_sec", 30.0);

  // ------------------------------------------------------------------
  // Subscribers
  // ------------------------------------------------------------------
  sub_ped_light_ = create_subscription<autoware_road_crossing_msgs::msg::PedestrianLightState>(
    param_.topic_pedestrian_light, rclcpp::QoS(1),
    std::bind(&RoadCrossingFsmNode::onPedLight, this, std::placeholders::_1));

  // Route: use transient_local so we receive the last route even after subscription
  sub_route_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    param_.topic_route,
    rclcpp::QoS(1).transient_local(),
    std::bind(&RoadCrossingFsmNode::onRoute, this, std::placeholders::_1));

  sub_active_route_window_ =
    create_subscription<autoware_active_route_window_msgs::msg::ActiveRouteWindow>(
      declare_parameter<std::string>(
        "topic_active_route_window", "/planning/mission_planning/active_route_window"),
      rclcpp::QoS(1),
      [this](const autoware_active_route_window_msgs::msg::ActiveRouteWindow::ConstSharedPtr msg) {
        active_route_window_ = msg;
      });

  // Map: transient_local (published once at startup)
  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    param_.topic_map,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&RoadCrossingFsmNode::onMap, this, std::placeholders::_1));

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    param_.topic_ego_pose, rclcpp::QoS(1),
    std::bind(&RoadCrossingFsmNode::onOdom, this, std::placeholders::_1));

  // base_link -> front bumper, from the vehicle parameters every node is launched with.
  base_link_to_front_m_ = declare_parameter<double>("wheel_base", 0.0) +
                          declare_parameter<double>("front_overhang", 0.0);

  // Planned trajectory: driving direction while standing still (see updateDrivingDirection()).
  sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    declare_parameter<std::string>("topic_trajectory", "/planning/scenario_planning/trajectory"),
    rclcpp::QoS(1),
    [this](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
      last_trajectory_ = msg;
    });

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
  // Depth 10: PRE and ROAD gates go out back to back every tick (see the planner side).
  pub_gate_ = create_publisher<autoware_road_crossing_msgs::msg::RoadCrossingGate>(
    param_.topic_gate, rclcpp::QoS(10));
  pub_fsm_state_ = create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    param_.topic_fsm_state, rclcpp::QoS(1));
  pub_light_status_ = create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    param_.topic_light_status, rclcpp::QoS(1));

  // AdAPI goal publisher — transient_local reliable depth 1 (same as auto_cycle_goals.py)
  {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", qos);
  }

  pub_target_light_ = create_publisher<autoware_road_crossing_msgs::msg::PedestrianLightTarget>(
    param_.topic_target_light, rclcpp::QoS(1));

  // ------------------------------------------------------------------
  // Service clients
  // ------------------------------------------------------------------
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

void RoadCrossingFsmNode::onPedLight(
  const autoware_road_crossing_msgs::msg::PedestrianLightState::SharedPtr msg)
{
  // A light state only counts when it comes from a light paired with the active crossing;
  // anything else is some other crossing's light that happens to be in view.  NONE carries
  // no light and is always taken (it drives the no-detect counter).
  if (msg->state != autoware_road_crossing_msgs::msg::PedestrianLightState::NONE &&
      std::find(active_light_ids_.begin(), active_light_ids_.end(), msg->light_id) ==
        active_light_ids_.end())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "onPedLight: ignoring state=%u from light %ld — not paired with crossing %ld",
      msg->state, static_cast<long>(msg->light_id),
      active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L);
    return;
  }
  // Interlock input: what the paired light showed since the last FSM tick.
  using PLS = autoware_road_crossing_msgs::msg::PedestrianLightState;
  if (msg->state == PLS::RED) {
    interlock_.merah_seen = true;
  } else if (msg->state == PLS::GREEN || msg->state == PLS::COUNTDOWN_GREEN) {
    interlock_.hijau_seen = true;
  }
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
      // Take the direction from the planned trajectory (oriented the way the route really runs,
      // also on a lanelet traversed against its drawing direction), else from the route's own
      // geometric heading at ego's position, instead of leaving is_driving_forward_ latched at
      // a stale value (which would otherwise deadlock checkArmCondition() forever).
      auto route_forward = deriveDrivingDirectionFromTrajectory();
      if (!route_forward) {
        route_forward = deriveDrivingDirectionFromRoute();
      }
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

bool RoadCrossingFsmNode::egoRouteLanelet(lanelet::ConstLanelet * lanelet) const
{
  if (!last_odom_ || !route_initialized_) return false;
  constexpr double max_window_age_s = 1.0;
  const auto & w = active_route_window_;
  if (
    w && w->valid && w->route_uuid == route_handler_.getRouteUuid() &&
    (now() - rclcpp::Time(w->header.stamp)).seconds() < max_window_age_s &&
    route_handler_.getLaneletMapPtr() &&
    route_handler_.getLaneletMapPtr()->laneletLayer.exists(w->ego_lanelet_id)) {
    // Oriented the way the route runs it (getLaneletsFromId re-applies the route's direction).
    *lanelet = route_handler_.getLaneletsFromId(w->ego_lanelet_id);
    return true;
  }
  return route_handler_.getClosestLaneletWithinRoute(last_odom_->pose.pose, lanelet);
}

std::optional<bool> RoadCrossingFsmNode::deriveDrivingDirectionFromRoute() const
{
  if (!last_odom_ || !route_initialized_) return std::nullopt;

  lanelet::ConstLanelet cur;
  if (!egoRouteLanelet(&cur)) {
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

std::optional<bool> RoadCrossingFsmNode::deriveDrivingDirectionFromTrajectory() const
{
  constexpr double max_trajectory_age_s = 1.0;
  if (!last_odom_ || !last_trajectory_ || last_trajectory_->points.size() < 2) {
    return std::nullopt;
  }
  if ((now() - rclcpp::Time(last_trajectory_->header.stamp)).seconds() > max_trajectory_age_s) {
    return std::nullopt;
  }
  const auto & pts = last_trajectory_->points;
  const auto & ego = last_odom_->pose.pose.position;
  size_t nearest = 0;
  double nearest_d2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < pts.size(); ++i) {
    const double dx = pts[i].pose.position.x - ego.x;
    const double dy = pts[i].pose.position.y - ego.y;
    if (dx * dx + dy * dy < nearest_d2) {
      nearest_d2 = dx * dx + dy * dy;
      nearest = i;
    }
  }
  // Same test as motion_utils::isDrivingForward(): is the next point ahead of the pose?
  for (size_t j = nearest + 1; j < pts.size(); ++j) {
    const double dx = pts[j].pose.position.x - pts[nearest].pose.position.x;
    const double dy = pts[j].pose.position.y - pts[nearest].pose.position.y;
    if (dx * dx + dy * dy > 1e-6) {
      return autoware_utils_geometry::is_driving_forward(pts[nearest].pose, pts[j].pose.position);
    }
  }
  return std::nullopt;
}

void RoadCrossingFsmNode::onTimer()
{
  publishFsmState();
  publishTargetLight();

  // Guard: need map, route, and pose before FSM can do anything useful.
  if (!map_initialized_ || !route_initialized_ || !last_odom_) {
    return;
  }

  // Refresh direction-of-travel signal before any direction-dependent logic
  // (entry/exit resolution, route-order scans, aimer re-zero) runs this tick.
  updateDrivingDirection();

  const rclcpp::Time now = this->now();

  // Relocalized (2D Pose Estimate / localization jump): ego is somewhere else entirely, so
  // the episode (and the re-arm suppression) refer to a crossing that may no longer be the
  // next one ahead.  Start over from the new position; gates go stale (= HOLD) meanwhile.
  {
    const auto & pos = last_odom_->pose.pose.position;
    if (prev_tick_position_) {
      const double jump = std::hypot(pos.x - prev_tick_position_->x, pos.y - prev_tick_position_->y);
      if (jump > param_.relocalize_jump_m) {
        RCLCPP_WARN(
          get_logger(),
          "ego jumped %.1f m in one tick (> relocalize_jump_m %.1f) — relocalized: resetting "
          "episode (state=%d, crossing=%ld)",
          jump, param_.relocalize_jump_m, static_cast<int>(state_),
          active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L);
        prev_tick_position_ = pos;
        if (state_ != FsmState::IDLE) {
          resetEpisode();
          transitionTo(FsmState::IDLE);
        }
        return;
      }
    }
    prev_tick_position_ = pos;
  }

  // ---------------------------------------------------------------------------
  // Rotary aimer: aktif selama FSM menunggu (APPROACH_PRE .. WAIT_GREEN) untuk crossing yang
  // punya lampu, begitu ego dalam aim_approach_dist_m dari pre entry atau sudah di pre
  // (base_link ATAU bumper depan, sama seperti arming).  Aimer selalu membidik lampu target
  // (target_light); tidak ada target eksternal lagi.  Crossing tanpa lampu: aimer tidak
  // diaktifkan -- dia tidak punya target dan tidak akan publish, jadi enable akan di-spam.
  //
  // Disable (re-zero) tetap dilakukan di setiap situs transisi keluar (CROSSING entry,
  // REROUTING_TO_ALT, resetEpisode).
  // ---------------------------------------------------------------------------
  {
    const bool waiting =
      (state_ == FsmState::APPROACH_PRE || state_ == FsmState::AT_PRE ||
       state_ == FsmState::ARMED_LIGHT  || state_ == FsmState::WAIT_GREEN);

    const double pre_dist = distanceToPreEntry();
    const bool on_pre = active_pre_id_ && egoOnPre(*active_pre_id_);
    const bool want_aim =
      waiting && !active_light_ids_.empty() &&
      (on_pre || (pre_dist >= 0.0 && pre_dist <= param_.aim_approach_dist_m));

    // Aimer dianggap running jika ada pesan aim_cmd yang fresh.
    const double aim_age = (now - last_aim_cmd_time_).seconds();
    const bool aim_running = (aim_age < param_.aim_cmd_fresh_sec);

    if (want_aim && !aim_running) {
      const double since_last_send = (now - last_aim_enable_send_).seconds();
      if (since_last_send > param_.aim_enable_retry_sec) {
        setDetectorEnabled(cli_rotary_aimer_, true, "rotary_aimer");
        last_aim_enable_send_ = now;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "aimer: spam-enable rotary_aimer (state=%d, on_pre=%d, pre_dist=%.2f m, "
          "aim_age=%.2f s) — menunggu aimer running",
          static_cast<int>(state_), on_pre ? 1 : 0, pre_dist, aim_age);
      }
    }
  }

  // Membership: an active episode only makes sense while ego is on its pre or its crossing.
  // Anywhere else (relocalized, rerouted, drove off) the episode is stale: drop it — gates go
  // stale (= HOLD) — and arm again from wherever ego is.  Leaving while CROSSING is simply the
  // end of the crossing.
  const bool episode_active =
    state_ == FsmState::APPROACH_PRE || state_ == FsmState::AT_PRE ||
    state_ == FsmState::ARMED_LIGHT || state_ == FsmState::WAIT_GREEN ||
    state_ == FsmState::CROSSING;
  if (episode_active && active_crossing_id_) {
    const bool on_pre = active_pre_id_ && egoOnPre(*active_pre_id_);
    const bool on_crossing = egoOnLanelet(*active_crossing_id_);
    if (!on_pre && !on_crossing) {
      if (state_ == FsmState::CROSSING) {
        RCLCPP_INFO(
          get_logger(), "ego left pre %ld / crossing %ld while CROSSING — DONE",
          active_pre_id_ ? static_cast<long>(*active_pre_id_) : -1L,
          static_cast<long>(*active_crossing_id_));
        setDetectorEnabled(cli_ped_light_detector_, false, "ped_light_detector");
        transitionTo(FsmState::DONE);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "ego is on neither pre %ld nor crossing %ld (state=%d) — resetting the episode",
          active_pre_id_ ? static_cast<long>(*active_pre_id_) : -1L,
          static_cast<long>(*active_crossing_id_), static_cast<int>(state_));
        resetEpisode();
        transitionTo(FsmState::IDLE);
      }
      return;
    }
  }

  switch (state_) {
    // -------------------------------------------------------------------
    case FsmState::IDLE: {
      if (checkArmCondition()) {
        transitionTo(FsmState::APPROACH_PRE);
        // Lock the paired light(s) for this episode.  Arming only happens while driving
        // forward (see checkArmCondition()), so today this is always the forward tag.
        active_light_direction_ =
          is_driving_forward_ ? autoware_road_crossing_msgs::msg::PedestrianLightTarget::FORWARD
                              : autoware_road_crossing_msgs::msg::PedestrianLightTarget::REVERSE;
        active_light_ids_ = active_crossing_id_
                              ? pairedLights(*active_crossing_id_, is_driving_forward_)
                              : std::vector<int64_t>{};
        RCLCPP_INFO(
          get_logger(), "IDLE: crossing %ld %s light(s): %s",
          active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L,
          is_driving_forward_ ? "forward" : "reverse",
          active_light_ids_.empty() ? "none (unsignalized, no light wait)"
                                    : lightIdsText().c_str());
        // Rotary aimer is NOT enabled here — enabling at approach distance causes the
        // platform to rotate from far away (unstable / weird).  Instead the aimer is
        // enabled lazily in the waiting states.  See the aimer block in onTimer.
      }
      break;
    }

    // --------------------------------------------------------------------
    // APPROACH_PRE
    //
    // Robot is approaching the pre_road_crossing stop.
    // Gate(pre) = HOLD until ego is at the pre, then release pre gate and
    // transition AT_PRE → ARMED_LIGHT.  The crossing is trusted from the map tags alone
    // (no crosswalk detection).
    // -------------------------------------------------------------------
    case FsmState::APPROACH_PRE: {
      // PRE gate = HOLD (robot waits at pre stop).
      // ROAD gate = HOLD (don't release road crossing yet).
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);

      // -----------------------------------------------------------------------
      // Position gate: robot harus sudah tiba di pre (atau crossing jika tanpa pre)
      // sebelum FSM boleh maju ke AT_PRE/ARMED_LIGHT.
      //   at_pre = true  jika distanceToPreEntry() ∈ [0, pre_arrive_dist_m]
      //   at_pre = true  (fallback tanpa pre) jika dist-crossing ∈ [0, crossing_arrive_dist_m]
      // Selama belum at_pre: tahan di APPROACH_PRE, gate tetap HOLD.
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
          "APPROACH_PRE: robot belum tiba (pre_dist=%.2f m, gate=pre_arrive_dist_m=%.1f m) "
          "— tetap HOLD, menunggu robot mendekati pre/crossing",
          active_pre_id_ ? distanceToPreEntry() : distanceToCrossingEntry(),
          active_pre_id_ ? param_.pre_arrive_dist_m : param_.crossing_arrive_dist_m);
        break;
      }

      // Robot is at the pre stop: release it to the road_crossing and go wait for the light.
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      // ONE transition per tick: go to AT_PRE. On the NEXT tick, AT_PRE
      // handler will transition to ARMED_LIGHT. Entry actions for ARMED_LIGHT
      // (ped_light enable + light_timeout_start_) run in the ARMED_LIGHT handler.
      transitionTo(FsmState::AT_PRE);
      break;
    }

    // -------------------------------------------------------------------
    // AT_PRE
    //
    // Ego has arrived at the pre stop.
    // PRE gate is already GO (set before transitionTo(AT_PRE) upstream).
    // This state runs for exactly one tick, then transitions to ARMED_LIGHT.
    // Entry actions for ARMED_LIGHT (ped_light enable + light_timeout_start_)
    // are deferred to the ARMED_LIGHT handler — strictly one transition per tick.
    // -------------------------------------------------------------------
    case FsmState::AT_PRE: {
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
      // ONE transition per tick: AT_PRE → ARMED_LIGHT.
      transitionTo(FsmState::ARMED_LIGHT);
      break;
    }

    // -------------------------------------------------------------------
    // REROUTING_TO_ALT  (Phase A)
    //
    // NOTE: unreachable since the crosswalk detection interlock was removed — this
    // was entered only when no crosswalk was detected at the pre stop.
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
          // Return to APPROACH_PRE and hold at pre.
          RCLCPP_WARN(
            get_logger(),
            "REROUTING_TO_ALT step0: no alternate_road_crossing found (lost mid-process) — "
            "returning to APPROACH_PRE, gate pre=HOLD");
          publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
          publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
          transitionTo(FsmState::APPROACH_PRE);
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
    // Ego at the pre; PRE gate stays GO.  Road gate = HOLD until green.
    //
    // Entry actions (first tick only, guarded by !light_timeout_start_):
    //   • Enable ped_light_detector — unless the crossing has no light or is tagged
    //     disable_tf_light_check.
    //   • Reset the green-light interlock: its counters start fresh in WAIT_GREEN.
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
        if (active_light_ids_.empty()) {
          RCLCPP_INFO(
            get_logger(),
            "ARMED_LIGHT entry: crossing %ld has no paired pedestrian light — "
            "skipping ped_light_detector enable",
            active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L);
        } else if (!crossingHasTag("disable_tf_light_check")) {
          setDetectorEnabled(cli_ped_light_detector_, true, "ped_light_detector");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "ARMED_LIGHT entry: crossing %ld tagged disable_tf_light_check — "
            "skipping ped_light_detector enable",
            active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L);
        }
        light_timeout_start_ = now;
        interlock_ = LightInterlock{};
      }

      // ONE transition per tick: proceed to WAIT_GREEN.
      transitionTo(FsmState::WAIT_GREEN);
      break;
    }

    // -------------------------------------------------------------------
    case FsmState::WAIT_GREEN: {
      // PRE gate stays GO (pre stop already released).
      // ROAD gate stays HOLD until the green-light interlock allows crossing.
      publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);

      // Interlock lampu hijau (interlock.plan.md) jalan tiap tick sejak masuk WAIT_GREEN,
      // termasuk saat robot masih mendekat — supaya transisi merah → hijau tidak terlewat.
      updateLightInterlock("WAIT_GREEN");

      // No position gate: ego is on this crossing's pre (the membership check guarantees it),
      // so GO as soon as the light allows — the planner module then crosses without stopping
      // at the line instead of braking to a halt first.  Once released, ego keeps going.

      // base_link already on the crossing (the planner let ego through past the point where it
      // could still stop at the line after a fall back from CROSSING): ego is committed.
      if (active_crossing_id_ && egoOnLanelet(*active_crossing_id_)) {
        RCLCPP_WARN(
          get_logger(),
          "WAIT_GREEN: ego sudah di crossing %ld — tidak bisa kembali, lanjut CROSSING",
          static_cast<long>(*active_crossing_id_));
        light_timeout_start_.reset();
        road_cross_is_green_ = false;
        crossing_timeout_start_ = now;
        transitionTo(FsmState::CROSSING);
        reZeroAimerDirectionAware();
        publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
        break;
      }

      // No light to wait for: the crossing has no paired pedestrian light (unsignalized), or
      // is tagged disable_tf_light_check.
      // This is NOT a green-light cross — use plain GO so the module's trust-light shortcut
      // does NOT fire; vehicle-scan (isPredictedBlocked) governs the crossing.
      if (active_light_ids_.empty() || crossingHasTag("disable_tf_light_check")) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "WAIT_GREEN: crossing %ld %s — skip lampu, GO (vehicle-scan governs)",
          active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L,
          active_light_ids_.empty() ? "has no paired pedestrian light"
                                    : "tagged disable_tf_light_check");
        light_timeout_start_.reset();
        road_cross_is_green_ = false;   // bypass: trust-light shortcut must NOT fire
        crossing_timeout_start_ = now;  // arm CROSSING safety-timeout watchdog
        transitionTo(FsmState::CROSSING);
        // Robot mulai menyeberang — re-zero aimer (direction-aware; see
        // reZeroAimerDirectionAware()).
        reZeroAimerDirectionAware();
        publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
        break;
      }

      // Tidak ada timeout di sini: tanpa izin interlock robot HOLD, berapapun lamanya
      // (merah bisa ~60 s; lampu yang tidak terlihat juga HOLD).
      if (!interlock_.allows_crossing) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "WAIT_GREEN: lampu %s status=%s — HOLD, menunggu hijau solid",
          lightIdsText().c_str(), lightStatusName(interlock_.status_now));
        break;
      }

      RCLCPP_INFO(
        get_logger(), "WAIT_GREEN: lampu %s hijau (%s) — GO_GREEN", lightIdsText().c_str(),
        interlock_.red_to_green ? "merah -> hijau" : "hijau solid >= threshold");
      light_timeout_start_.reset();
      road_cross_is_green_ = true;   // persist GO_GREEN through CROSSING state
      crossing_timeout_start_ = now;  // arm CROSSING safety-timeout watchdog
      transitionTo(FsmState::CROSSING);
      // Robot mulai menyeberang (lampu HIJAU) — re-zero aimer (direction-aware).
      reZeroAimerDirectionAware();
      publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO_GREEN);
      break;
    }

    // -------------------------------------------------------------------
    case FsmState::CROSSING: {
      // Keep status_now following the light while crossing.
      updateLightInterlock("CROSSING");

      // Released by the light but base_link is still on the pre: the release only stands while
      // the light still allows it.  Green flashing, red or nothing seen (status_now NONE) →
      // back to WAIT_GREEN and wait at the line again, e.g. when ego was stopped on the pre
      // after the GO and the light has changed since.  Once base_link is on the crossing there
      // is no way back: ego keeps going whatever the light does.
      if (
        road_cross_is_green_ && !interlock_.allows_crossing && active_pre_id_ &&
        active_crossing_id_ && egoOnPre(*active_pre_id_) &&
        !egoOnLanelet(*active_crossing_id_)) {
        RCLCPP_WARN(
          get_logger(),
          "CROSSING: ego masih di pre %ld dan lampu %s tidak lagi mengizinkan (status=%s) — "
          "kembali ke WAIT_GREEN",
          static_cast<long>(*active_pre_id_), lightIdsText().c_str(),
          lightStatusName(interlock_.status_now));
        road_cross_is_green_ = false;
        crossing_timeout_start_.reset();
        light_timeout_start_ = now;
        transitionTo(FsmState::WAIT_GREEN);
        publishGatePre(autoware_road_crossing_msgs::msg::RoadCrossingGate::GO);
        publishGate(autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
        break;
      }

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

      // A pre_road_crossing that directly follows this crossing puts its pre stop INSIDE
      // this crossing, so ego can never get past this crossing's exit before the next
      // episode releases that stop.  Reaching the next pre ends this episode instead.
      const auto next_pre = nextPreAfterCrossing();
      bool at_next_pre = false;
      // Only once ego is actually in the crossing — a short crossing may put the next pre
      // within reach while ego is still waiting at this crossing's stop line.
      if (next_pre && !egoUpstreamOf(*active_crossing_id_, 0.0)) {
        const double d = arcLengthDistanceToLaneletEntry(next_pre);
        at_next_pre = d >= 0.0 && d <= param_.pre_arrive_dist_m;
      }

      if (egoPastCrossingExit() || at_next_pre || timed_out) {
        if (at_next_pre && !egoPastCrossingExit()) {
          RCLCPP_INFO(
            get_logger(),
            "CROSSING: ego reached pre %ld that directly follows crossing %ld — DONE",
            static_cast<long>(*next_pre),
            active_crossing_id_ ? static_cast<long>(*active_crossing_id_) : -1L);
        } else if (timed_out) {
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
  // An episode starts only once ego is ON the lanelet right before a road_crossing in the
  // route (its pre_road_crossing), and lasts only while ego stays on that lanelet or on the
  // crossing itself (see the membership check in onTimer).  No look-ahead arming: an episode
  // for a crossing ego is not at would hold its gates for nothing and go stale after a
  // relocalization.
  if (!map_initialized_ || !route_initialized_ || !last_odom_) return false;
  if (route_lanelets_.size() < 2) return false;

  const auto is_crossing = [this](const lanelet::Id id) {
    try {
      return route_handler_.getLaneletMapPtr()->laneletLayer.get(id).attributeOr(
               "road_crossing", std::string("false")) == "true";
    } catch (const lanelet::NoSuchPrimitiveError &) {
      return false;
    }
  };
  // Why IDLE does not arm, logged only while the route has a crossing at all (a robot held
  // still at the pre otherwise waits for a green it can never get, with nothing in the log).
  const auto not_armed = [&](const char * reason, const lanelet::Id cur_id) {
    if (std::any_of(route_lanelets_.begin(), route_lanelets_.end(), is_crossing)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "IDLE: not arming -- %s (route lanelet %ld)", reason,
        static_cast<long>(cur_id));
    }
    return false;
  };

  // Bidirectional-driving guard: route order == travel order only while driving forward.
  if (!is_driving_forward_) {
    return not_armed("not driving forward", -1);
  }

  lanelet::ConstLanelet cur;
  if (!egoRouteLanelet(&cur)) {
    return not_armed("ego not resolved on the route", -1);
  }
  for (size_t i = 0; i + 1 < route_lanelets_.size(); ++i) {
    if (route_lanelets_[i] != cur.id()) continue;
    if (is_crossing(route_lanelets_[i + 1])) {
      if (!egoOnPre(cur.id())) {
        return not_armed("base_link not on the pre lanelet", cur.id());
      }
      active_pre_id_ = cur.id();
      active_crossing_id_ = route_lanelets_[i + 1];
      return true;
    }
    // The route window still has ego on the lanelet before the pre while base_link is already
    // on the pre: a robot started (or held by the road_crossing stop zone) just past their
    // seam never advances the window by itself, so arm on the pre it physically stands on.
    if (
      i + 2 < route_lanelets_.size() && is_crossing(route_lanelets_[i + 2]) &&
      egoOnPre(route_lanelets_[i + 1])) {
      active_pre_id_ = route_lanelets_[i + 1];
      active_crossing_id_ = route_lanelets_[i + 2];
      return true;
    }
    return not_armed("next route lanelet is not a crossing", cur.id());
  }
  return not_armed("ego lanelet not in the route", cur.id());
}

bool RoadCrossingFsmNode::egoOnPre(const lanelet::Id id) const
{
  if (egoOnLanelet(id)) return true;
  if (!last_odom_ || !map_initialized_ || base_link_to_front_m_ <= 0.0) return false;
  try {
    const auto & pose = last_odom_->pose.pose;
    const auto & q = pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const lanelet::BasicPoint2d front{
      pose.position.x + base_link_to_front_m_ * std::cos(yaw),
      pose.position.y + base_link_to_front_m_ * std::sin(yaw)};
    const auto polygon =
      route_handler_.getLaneletMapPtr()->laneletLayer.get(id).polygon2d().basicPolygon();
    return boost::geometry::covered_by(front, polygon);
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return false;
  }
}

bool RoadCrossingFsmNode::egoOnLanelet(const lanelet::Id id) const
{
  if (!last_odom_ || !map_initialized_) return false;
  try {
    const auto polygon =
      route_handler_.getLaneletMapPtr()->laneletLayer.get(id).polygon2d().basicPolygon();
    const auto & p = last_odom_->pose.pose.position;
    return boost::geometry::covered_by(lanelet::BasicPoint2d{p.x, p.y}, polygon);
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return false;
  }
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

// ---------------------------------------------------------------------------
// Arrival-gate distance: durable fix for the geometry mismatch that wedged
// fsm_state=1 (APPROACH_PRE) indefinitely (see param-bump commits
// 97ffe21b6 / f8cb84d for the stopgap this supersedes).
//
// Root cause: this function used to return the plain 2D EUCLIDEAN distance
// from ego's odom position to a fixed centerline vertex (cl.front()/back()).
// autoware_behavior_velocity_road_crossing_module's scene.cpp — the node that
// actually places the stop pose — instead works purely in ARC-LENGTH ALONG
// THE PLANNED PATH: stop_s = raw_entry_s - (leading_edge_offset + margin),
// with raw_entry_s the arc-length of the first path point covered by the
// crossing lanelet's polygon (see scene.cpp findEgoAndStopPoint()). That
// distance has NO lateral component. Euclidean-to-vertex does, so any lateral
// offset between ego's actual (stopped) pose and the raw centerline vertex —
// lane curvature, resampling, EKF noise, replanned/shifted trajectory — was
// added straight into the "have I arrived" measurement, on top of whatever
// margin scene.cpp already stopped short by. That is what left
// distanceToPreEntry() reporting >3.0 m while the robot sat stopped at its
// actual stop line.
//
// Fix: measure progress the same way scene.cpp does — along-track only.
//
// Design choice (duplicate the arc-length calc vs. subscribe to scene.cpp's
// internal quantity): scene.cpp's raw_entry_s/stop_s are computed against the
// resampled PathWithLaneId/Trajectory owned by behavior_velocity_planner,
// which this node has no access to (and should not — it would add a topic
// coupling between two independently-composable planning nodes, and the
// scene module is dynamically created/destroyed per-episode, see
// scene-modules-no-subscriptions policy). Instead we duplicate the
// arc-length idea directly against the lanelet's own centerline via
// lanelet::geometry::toArcCoordinates(), which is what the path is resampled
// from in the first place — so the two should stay within a few tens of cm
// of each other outside of sharp corner-cutting. This keeps the FSM
// self-contained (map + route + odom only, its existing dependency set) and
// avoids reinventing a full path-following arc-length machinery for a
// coarse arrival gate that only needs meter-scale accuracy.
//
// Caveat: because we project onto the RAW lanelet centerline instead of the
// smoothed/optimized path, this can still diverge from scene.cpp's exact
// stop_s at sharp turns or where the optimizer shifts the path noticeably
// off the centerline (e.g. avoidance shift, obstacle-triggered lateral
// offset). In those edge cases this distance is still an along-track
// approximation, not an exact duplicate — but it is no longer contaminated
// by ego's lateral offset in the way Euclidean-to-vertex was, which was the
// actual failure mode observed.
double RoadCrossingFsmNode::arcLengthDistanceToLaneletEntry(
  const std::optional<lanelet::Id> & id) const
{
  if (!id || !last_odom_) return -1.0;
  if (!map_initialized_) return -1.0;

  try {
    const auto llt = route_handler_.getLaneletMapPtr()->laneletLayer.get(*id);
    const auto centerline_2d = llt.centerline2d();
    if (centerline_2d.size() < 2) return -1.0;

    const double total_length =
      static_cast<double>(boost::geometry::length(centerline_2d.basicLineString()));

    const auto & ego_pos = last_odom_->pose.pose.position;
    const lanelet::BasicPoint2d ego2d{ego_pos.x, ego_pos.y};

    // Ego's arc-length position along the centerline, measured in the
    // lanelet's own stored orientation (front()=0 .. back()=total_length).
    // toArcCoordinates() projects onto the nearest segment — no lateral-offset
    // contamination, unlike Euclidean-to-vertex — but clamps to [0, L] when ego
    // is outside the lanelet (see the upstream case below).
    const double ego_arc_s =
      lanelet::geometry::toArcCoordinates(centerline_2d, ego2d).length;

    // Entry is the endpoint ego actually arrives at first: arc-length 0
    // (front()) when driving forward, total_length (back()) when reversing
    // through this lanelet — same direction convention as before.
    const double entry_arc_s = is_driving_forward_ ? 0.0 : total_length;

    // Return an along-track (never lateral-inflated) gap. abs() keeps the
    // same "always >= 0, sign-free proximity" contract the Euclidean version
    // had (callers gate on d in [0, threshold]), so a robot that is already
    // slightly past the entry still reads as "arrived" instead of silently
    // failing the d >= 0.0 checks used throughout this file.
    const double along = std::abs(entry_arc_s - ego_arc_s);

    // toArcCoordinates() does NOT extrapolate: a point before the start of the
    // centerline gets length 0 (and one beyond the end gets total_length). So an
    // ego still upstream of the entry — however far — projects exactly onto the
    // entry and would read 0 m, i.e. "arrived", letting the FSM run its whole
    // episode (light wait, GO) while the robot is still driving there.  When ego
    // projects onto the entry point itself, the along-track gap carries no
    // information; use the straight-line distance to the entry instead.
    if (along < 1e-3) {
      const auto & entry_pt =
        is_driving_forward_ ? centerline_2d.front() : centerline_2d.back();
      return std::hypot(ego2d.x() - entry_pt.x(), ego2d.y() - entry_pt.y());
    }
    return along;
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return -1.0;
  }
}

double RoadCrossingFsmNode::distanceToCrossingEntry() const
{
  return arcLengthDistanceToLaneletEntry(active_crossing_id_);
}

double RoadCrossingFsmNode::distanceToPreEntry() const
{
  return arcLengthDistanceToLaneletEntry(active_pre_id_);
}

bool RoadCrossingFsmNode::egoUpstreamOf(const lanelet::Id id, const double margin) const
{
  if (!last_odom_ || !map_initialized_) return false;
  try {
    const auto centerline = route_handler_.getLaneletMapPtr()->laneletLayer.get(id).centerline2d();
    if (centerline.size() < 2) return false;
    const double total_length =
      static_cast<double>(boost::geometry::length(centerline.basicLineString()));
    const lanelet::BasicPoint2d ego2d{
      last_odom_->pose.pose.position.x, last_odom_->pose.pose.position.y};
    const double ego_arc_s = lanelet::geometry::toArcCoordinates(centerline, ego2d).length;
    // Upstream = ego projects onto the entry end itself (toArcCoordinates clamps points
    // before the lanelet there) and is more than `margin` away from it.
    const bool at_entry_end =
      is_driving_forward_ ? ego_arc_s < 1e-3 : ego_arc_s > total_length - 1e-3;
    if (!at_entry_end) return false;
    const auto & entry_pt = is_driving_forward_ ? centerline.front() : centerline.back();
    return std::hypot(ego2d.x() - entry_pt.x(), ego2d.y() - entry_pt.y()) > margin;
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return false;
  }
}

std::optional<lanelet::Id> RoadCrossingFsmNode::nextPreAfterCrossing() const
{
  if (!active_crossing_id_) return std::nullopt;
  for (size_t i = 0; i + 1 < route_lanelets_.size(); ++i) {
    if (route_lanelets_[i] != *active_crossing_id_) continue;
    try {
      const auto next = route_handler_.getLaneletMapPtr()->laneletLayer.get(route_lanelets_[i + 1]);
      if (next.attributeOr("pre_road_crossing", std::string("false")) == "true") {
        return next.id();
      }
    } catch (const lanelet::NoSuchPrimitiveError &) {
    }
    return std::nullopt;
  }
  return std::nullopt;
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
  if (!egoRouteLanelet(&cur)) {
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
  //      use ego's current lanelet via egoRouteLanelet().
  //   4. For each parent candidate, scan getNextLanelets(parent) 1 level only.
  //      Return the first next lanelet tagged "alternate_road_crossing"=="true"
  //      AND NOT in reroute_blocklist_.
  //   5. Nothing found → nullopt.
  // -----------------------------------------------------------------------

  if (!map_initialized_) return std::nullopt;

  // Bidirectional-driving guard (Phase 3c): same reasoning as checkArmCondition() —
  // the fork-parent resolution below assumes route_lanelets_ index order == travel
  // order (parent = pre_idx - 1). While ego is reversing that assumption doesn't
  // hold, and picking the wrong fork parent could route the APPROACH_PRE reroute
  // decision onto a wrong branch. Per the audit's simpler/safer option, reroute
  // branch-finding is disabled entirely while ego is reversing — the FSM will hold
  // at the pre stop instead.
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
    if (egoRouteLanelet(&ego_llt)) {
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

std::string RoadCrossingFsmNode::lightIdsText() const
{
  std::string ids;
  for (const auto id : active_light_ids_) {
    ids += (ids.empty() ? "" : ",") + std::to_string(id);
  }
  return ids;
}

void RoadCrossingFsmNode::publishTargetLight()
{
  autoware_road_crossing_msgs::msg::PedestrianLightTarget msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.crossing_id = active_crossing_id_ ? static_cast<int64_t>(*active_crossing_id_) : -1;
  msg.direction = active_light_direction_;
  msg.light_ids = active_light_ids_;
  pub_target_light_->publish(msg);
}

std::vector<int64_t> RoadCrossingFsmNode::pairedLights(
  const lanelet::Id crossing_id, const bool forward) const
{
  const std::string key = forward ? "forward_pedestrian_tf" : "reverse_pedestrian_tf";
  std::vector<int64_t> ids;
  try {
    const auto map = route_handler_.getLaneletMapPtr();
    const std::string value =
      map->laneletLayer.get(crossing_id).attributeOr(key.c_str(), std::string(""));
    std::stringstream ss(value);
    std::string token;
    while (std::getline(ss, token, ',')) {
      token.erase(0, token.find_first_not_of(" \t"));
      token.erase(token.find_last_not_of(" \t") + 1);
      if (token.empty()) continue;
      int64_t id = 0;
      try {
        id = std::stoll(token);
      } catch (const std::exception &) {
        RCLCPP_WARN(
          get_logger(), "crossing %ld: %s has a non-numeric entry '%s' — ignored",
          static_cast<long>(crossing_id), key.c_str(), token.c_str());
        continue;
      }
      // Catch map typos early: the id must be a tf_pedestrian way.
      const bool is_light =
        map->lineStringLayer.exists(id) &&
        map->lineStringLayer.get(id).attributeOr("tf_pedestrian", std::string("")) == "true";
      if (!is_light) {
        RCLCPP_WARN(
          get_logger(), "crossing %ld: %s=%ld is not a tf_pedestrian way in the map",
          static_cast<long>(crossing_id), key.c_str(), static_cast<long>(id));
      }
      ids.push_back(id);
    }
  } catch (const lanelet::NoSuchPrimitiveError &) {
  }
  return ids;
}

void RoadCrossingFsmNode::publishFsmState()
{
  autoware_internal_debug_msgs::msg::Int32Stamped msg;
  msg.stamp = this->now();
  msg.data = static_cast<int32_t>(state_);
  pub_fsm_state_->publish(msg);

  // Interlock status_now (0 NONE, 1 MERAH, 2 HIJAU) — NONE outside WAIT_GREEN, since the
  // interlock is reset for every crossing.
  msg.data = static_cast<int32_t>(interlock_.status_now);
  pub_light_status_->publish(msg);
}

void RoadCrossingFsmNode::transitionTo(FsmState new_state)
{
  static const auto state_name = [](FsmState s) -> const char * {
    switch (s) {
      case FsmState::IDLE:              return "IDLE";
      case FsmState::APPROACH_PRE:   return "APPROACH_PRE";
      case FsmState::AT_PRE:         return "AT_PRE";
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

namespace
{
uint16_t saturatingIncrement(uint16_t v)
{
  return v == std::numeric_limits<uint16_t>::max() ? v : static_cast<uint16_t>(v + 1);
}
}  // namespace

const char * RoadCrossingFsmNode::lightStatusName(LightStatus status)
{
  switch (status) {
    case LightStatus::MERAH:
      return "MERAH";
    case LightStatus::HIJAU:
      return "HIJAU";
    default:
      return "NONE";
  }
}

void RoadCrossingFsmNode::updateLightInterlock(const char * where)
{
  // One FSM tick of interlock.plan.md.  Units are FSM ticks (1 / update_rate_hz).
  auto & il = interlock_;
  const auto & p = param_;

  if (il.merah_seen) {
    il.counter_merah_not_detected = 0;
    il.counter_merah_detected = saturatingIncrement(il.counter_merah_detected);
  }
  if (il.hijau_seen) {
    il.counter_hijau_not_detected = 0;
    il.counter_hijau_detected = saturatingIncrement(il.counter_hijau_detected);
  }
  il.merah_seen = false;
  il.hijau_seen = false;

  if (il.counter_merah_detected == il.prev_counter_merah_detected) {
    il.counter_merah_not_detected = saturatingIncrement(il.counter_merah_not_detected);
  }
  if (il.counter_hijau_detected == il.prev_counter_hijau_detected) {
    il.counter_hijau_not_detected = saturatingIncrement(il.counter_hijau_not_detected);
  }

  if (il.counter_merah_not_detected >= p.threshold_merah_off) {
    il.counter_merah_detected = 0;
  }
  if (il.counter_hijau_not_detected >= p.threshold_hijau_off) {
    il.counter_hijau_detected = 0;
  }

  il.status_merah_valid = il.counter_merah_detected >= p.threshold_merah_valid;
  il.status_hijau_valid = il.counter_hijau_detected >= p.threshold_hijau_valid;

  if (il.status_hijau_valid) {
    il.status_now = LightStatus::HIJAU;
  } else if (il.status_merah_valid) {
    il.status_now = LightStatus::MERAH;
  } else {
    il.status_now = LightStatus::NONE;
  }

  // Interlock
  // Rule 1: merah lalu hijau → awal fase hijau, pasti bukan hijau kedip.  Di-latch selama
  // hijau tetap terlihat tanpa putus, karena robot mungkin belum tiba di crossing saat
  // transisinya terjadi.
  if (il.prev_status == LightStatus::MERAH && il.status_now == LightStatus::HIJAU) {
    il.red_to_green = true;
  }
  if (il.status_now != LightStatus::HIJAU) {
    il.red_to_green = false;
  }
  // Rule 2: hijau tanpa merah sebelumnya — harus terlihat terus >= threshold solid, lebih
  // lama dari satu nyala hijau kedip (kedip memutus counter lewat threshold_hijau_off).
  const bool solid =
    il.status_now == LightStatus::HIJAU &&
    il.counter_hijau_detected >= p.threshold_interlock_hijau_solid;
  const bool allows = il.red_to_green || solid;

  const bool changed = il.status_now != il.prev_status || allows != il.allows_crossing;
  il.allows_crossing = allows;
  const auto log = [&]() {
    return std::string("interlock: status=") + lightStatusName(il.status_now) +
           " prev=" + lightStatusName(il.prev_status) +
           " merah[det=" + std::to_string(il.counter_merah_detected) +
           " off=" + std::to_string(il.counter_merah_not_detected) +
           " valid=" + std::to_string(il.status_merah_valid) +
           "] hijau[det=" + std::to_string(il.counter_hijau_detected) +
           " off=" + std::to_string(il.counter_hijau_not_detected) +
           " valid=" + std::to_string(il.status_hijau_valid) +
           "] merah->hijau=" + std::to_string(il.red_to_green) +
           " solid=" + std::to_string(solid) + " boleh_nyebrang=" + std::to_string(allows);
  };
  if (changed) {
    RCLCPP_INFO(get_logger(), "%s %s", where, log().c_str());
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s %s", where, log().c_str());
  }

  // Update prev value
  il.prev_counter_merah_detected = il.counter_merah_detected;
  il.prev_counter_hijau_detected = il.counter_hijau_detected;
  il.prev_status = il.status_now;
}

void RoadCrossingFsmNode::resetEpisode()
{
  active_pre_id_.reset();
  active_crossing_id_.reset();
  light_timeout_start_.reset();
  crossing_timeout_start_.reset();  // reset CROSSING safety-timeout watchdog (#1)
  interlock_ = LightInterlock{};  // penyeberangan baru: semua counter & status dari nol
  active_light_ids_.clear();
  road_cross_is_green_ = false;  // reset green-cross flag for next episode
  // Reset reroute sub-step state so the next episode starts fresh.
  resetRerouteSubstep();
  active_alt_id_.reset();
  // Note: our_reroute_pending_phase_ is intentionally NOT reset here.
  // If we resetEpisode while a route call is in-flight, the pending flag must
  // remain so onRoute does not misinterpret the ack as an external route and
  // overwrite original_goal_.  It is cleared by onRoute when the ack arrives.

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
