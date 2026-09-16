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

#include "autoware/stuck_recovery_supervisor/stuck_recovery_supervisor_node.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <tf2/utils.h>
// See corridor_checker.cpp: tf2::getYaw needs the fromMsg definition from here, and
// omitting it links fine but dies at the first call.
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::stuck_recovery_supervisor
{

namespace
{

using autoware_stuck_recovery_msgs::msg::StuckDiagnosis;

double distance2d(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

std::string state_name(State state)
{
  switch (state) {
    case State::NOMINAL:
      return "NOMINAL";
    case State::SUSPECT:
      return "SUSPECT";
    case State::PROBE:
      return "PROBE";
    case State::RECOVERY:
      return "RECOVERY";
    case State::BLOCKED:
      return "BLOCKED";
    case State::COOLDOWN:
      return "COOLDOWN";
    case State::ABORT:
      return "ABORT";
    case State::REPLAN:
      return "REPLAN";
    case State::TRANSIT:
      return "TRANSIT";
  }
  return "UNKNOWN";
}

const char * side_name(Side side)
{
  switch (side) {
    case Side::LEFT:
      return "left";
    case Side::RIGHT:
      return "right";
    case Side::NONE:
      return "none";
  }
  return "none";
}

Mode parse_mode(const std::string & text)
{
  if (text == "recovery") {
    return Mode::RECOVERY;
  }
  if (text == "probe") {
    return Mode::PROBE_ONLY;
  }
  return Mode::DETECT_ONLY;
}

}  // namespace

StuckRecoverySupervisorNode::StuckRecoverySupervisorNode(const rclcpp::NodeOptions & options)
: Node("stuck_recovery_supervisor", options)
{
  load_parameters();
  setup_interfaces();

  const auto now = this->now();
  state_since_ = now;
  episode_since_ = now;
  stopped_since_ = now;
  commanded_but_still_since_ = now;
  clear_since_ = now;
  cooldown_until_ = now;

  RCLCPP_INFO(
    get_logger(), "stuck_recovery_supervisor started in mode '%s' (vehicle_width=%.2f m)",
    param_.mode.c_str(), vehicle_width_m_);
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
void StuckRecoverySupervisorNode::load_parameters()
{
  auto & p = param_;

  p.update_rate_hz = declare_parameter<double>("update_rate_hz", 10.0);
  p.mode = declare_parameter<std::string>("mode", "detect_only");

  p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps", 0.05);
  p.unstuck_progress_m = declare_parameter<double>("unstuck_progress_m", 0.5);
  p.suspect_timeout_sec = declare_parameter<double>("suspect_timeout_sec", 12.0);
  p.goal_reached_dist_m = declare_parameter<double>("goal_reached_dist_m", 1.0);
  p.map_stop_patience_sec = declare_parameter<double>("map_stop_patience_sec", 90.0);
  p.th_stop_factor_distance_m = declare_parameter<double>("th_stop_factor_distance_m", 3.0);
  p.th_commanded_velocity_mps = declare_parameter<double>("th_commanded_velocity_mps", 0.15);
  p.mechanical_confirm_sec = declare_parameter<double>("mechanical_confirm_sec", 2.0);

  p.probe_warmup_sec = declare_parameter<double>("probe_warmup_sec", 1.0);
  p.probe_step_m = declare_parameter<double>("probe_step_m", 0.25);
  p.lateral_margin_m = declare_parameter<double>("lateral_margin_m", 0.15);
  p.max_scan_half_width_m = declare_parameter<double>("max_scan_half_width_m", 4.0);
  p.occupancy_threshold = static_cast<int>(declare_parameter<int64_t>("occupancy_threshold", 50));

  p.corridor_scan_distance_m = declare_parameter<double>("corridor_scan_distance_m", 8.0);
  p.min_long_distance_to_check = declare_parameter<double>("min_long_distance_to_check", 3.0);
  p.goal_offset_beyond_obstacle_m =
    declare_parameter<double>("goal_offset_beyond_obstacle_m", 2.0);
  p.goal_check_interval_m = declare_parameter<double>("goal_check_interval_m", 1.0);
  p.goal_min_clear_length_m =
    declare_parameter<double>("goal_min_clear_length_m", 2.0);
  p.max_goal_search_distance_m = declare_parameter<double>("max_goal_search_distance_m", 30.0);
  p.goal_shape_margin_m = declare_parameter<double>("goal_shape_margin_m", 0.2);
  p.recovery_timeout_sec = declare_parameter<double>("recovery_timeout_sec", 60.0);
  p.clear_hold_sec = declare_parameter<double>("clear_hold_sec", 1.0);
  p.transit_yaw_tolerance_rad =
    declare_parameter<double>("transit_yaw_tolerance_rad", 0.26);
  p.transit_hold_sec = declare_parameter<double>("transit_hold_sec", 1.0);
  p.transit_timeout_sec = declare_parameter<double>("transit_timeout_sec", 30.0);
  p.transit_lookahead_m = declare_parameter<double>("transit_lookahead_m", 5.0);
  p.transit_join_min_m = declare_parameter<double>("transit_join_min_m", 1.5);
  p.transit_join_gain = declare_parameter<double>("transit_join_gain", 2.5);
  p.rejoin_lateral_tolerance_m =
    declare_parameter<double>("rejoin_lateral_tolerance_m", 0.5);
  p.aeb_lookahead_m = declare_parameter<double>("aeb_lookahead_m", 1.5);
  p.aeb_margin_m = declare_parameter<double>("aeb_margin_m", 0.05);
  p.aeb_skip_ahead_m = declare_parameter<double>("aeb_skip_ahead_m", 0.3);
  p.aeb_max_deviation_m = declare_parameter<double>("aeb_max_deviation_m", 0.6);
  p.aeb_occupancy_threshold =
    static_cast<int>(declare_parameter<int64_t>("aeb_occupancy_threshold", 100));
  p.aeb_hold_replan_sec = declare_parameter<double>("aeb_hold_replan_sec", 3.0);
  p.containment_grace_m = declare_parameter<double>("containment_grace_m", 2.0);
  p.publish_footprint_markers = declare_parameter<bool>("publish_footprint_markers", true);
  p.reference_path_source =
    declare_parameter<std::string>("reference_path_source", p.reference_path_source);
  p.topic_map = declare_parameter<std::string>("topic_map", p.topic_map);
  p.max_lateral_excursion_m = declare_parameter<double>("max_lateral_excursion_m", 2.0);
  p.min_occupied_fraction = declare_parameter<double>("min_occupied_fraction", 0.05);

  p.recovery_speed_limit_mps = declare_parameter<double>("recovery_speed_limit_mps", 0.278);
  p.min_engageable_stop_distance_m =
    declare_parameter<double>("min_engageable_stop_distance_m", 1.5);
  p.goal_retry_sec = declare_parameter<double>("goal_retry_sec", 4.0);
  p.replan_wait_sec = declare_parameter<double>("replan_wait_sec", 5.0);
  p.spent_hold_sec = declare_parameter<double>("spent_hold_sec", 2.0);
  p.freespace_plan_timeout_sec =
    declare_parameter<double>("freespace_plan_timeout_sec", 1.0);
  p.costmap_timeout_sec = declare_parameter<double>("costmap_timeout_sec", 1.0);
  p.max_replan_attempts =
    static_cast<int>(declare_parameter<int64_t>("max_replan_attempts", 3));
  p.max_attempts = static_cast<int>(declare_parameter<int64_t>("max_attempts", 2));
  p.cooldown_sec = declare_parameter<double>("cooldown_sec", 60.0);
  p.max_episode_duration_sec = declare_parameter<double>("max_episode_duration_sec", 180.0);
  p.abort_release_sec = declare_parameter<double>("abort_release_sec", 5.0);
  p.unknown_stop_clear_hold_sec =
    declare_parameter<double>("unknown_stop_clear_hold_sec", 3.0);
  p.abort_shadow_replan_sec = declare_parameter<double>("abort_shadow_replan_sec", 3.0);
  p.abort_retry_sec = declare_parameter<double>("abort_retry_sec", 15.0);
  p.max_abort_retries = static_cast<int>(declare_parameter<int64_t>("max_abort_retries", 3));

  p.pathological_factor_topics = declare_parameter<std::vector<std::string>>(
    "pathological_factor_topics", std::vector<std::string>{});
  p.legit_factor_topics =
    declare_parameter<std::vector<std::string>>("legit_factor_topics", std::vector<std::string>{});

  p.topic_odometry =
    declare_parameter<std::string>("topic_odometry", "/localization/kinematic_state");
  p.topic_velocity_status =
    declare_parameter<std::string>("topic_velocity_status", "/vehicle/status/velocity_status");
  p.topic_control_cmd =
    declare_parameter<std::string>("topic_control_cmd", "/control/command/control_cmd");
  p.topic_trajectory =
    declare_parameter<std::string>("topic_trajectory", "/planning/scenario_planning/trajectory");
  p.topic_route =
    declare_parameter<std::string>("topic_route", "/planning/mission_planning/route");
  p.topic_route_state = declare_parameter<std::string>("topic_route_state", "/api/routing/state");
  p.topic_operation_mode =
    declare_parameter<std::string>("topic_operation_mode", "/api/operation_mode/state");
  p.topic_control_mode =
    declare_parameter<std::string>("topic_control_mode", "/vehicle/status/control_mode");
  p.topic_gear = declare_parameter<std::string>("topic_gear", "/vehicle/status/gear_status");
  p.topic_scenario =
    declare_parameter<std::string>("topic_scenario", "/planning/scenario_planning/scenario");
  p.topic_occupancy_grid = declare_parameter<std::string>(
    "topic_occupancy_grid", "/planning/recovery/costmap_generator/occupancy_grid");
  p.topic_recovery_trajectory =
    declare_parameter<std::string>("topic_recovery_trajectory", "/planning/recovery/trajectory");
  p.topic_freespace_trajectory = declare_parameter<std::string>(
    "topic_freespace_trajectory", "/planning/recovery/freespace_trajectory");
  p.topic_recovery_is_completed = declare_parameter<std::string>(
    "topic_recovery_is_completed", "/planning/recovery/is_completed");
  p.topic_road_crossing_state = declare_parameter<std::string>(
    "topic_road_crossing_state", "/planning/road_crossing/fsm_state");
  p.topic_emergency_code =
    declare_parameter<std::string>("topic_emergency_code", "/gama/mw/emergency_code");
  p.topic_robot_fsm = declare_parameter<std::string>("topic_robot_fsm", "/robot_fsm");

  p.topic_out_scenario =
    declare_parameter<std::string>("topic_out_scenario", "/planning/recovery/scenario");
  p.topic_out_route = declare_parameter<std::string>("topic_out_route", "/planning/recovery/route");
  p.topic_out_force_recovery = declare_parameter<std::string>(
    "topic_out_force_recovery", "/planning/recovery/force_recovery");
  p.topic_out_state =
    declare_parameter<std::string>("topic_out_state", "/planning/stuck_recovery/state");
  p.topic_out_state_int =
    declare_parameter<std::string>("topic_out_state_int", "/planning/stuck_recovery/state_int");
  p.topic_out_diagnosis =
    declare_parameter<std::string>("topic_out_diagnosis", "/planning/stuck_recovery/diagnosis");
  p.topic_out_markers =
    declare_parameter<std::string>("topic_out_markers", "/planning/stuck_recovery/debug/markers");
  p.topic_out_escape_goal = declare_parameter<std::string>(
    "topic_out_escape_goal", "/planning/stuck_recovery/debug/escape_goal");
  p.topic_out_recovery_trajectory = declare_parameter<std::string>(
    "topic_out_recovery_trajectory", "/planning/recovery/trajectory");
  p.service_force_recovery = declare_parameter<std::string>(
    "service_force_recovery", "/planning/stuck_recovery/force_recovery");

  p.road_crossing_hold_states = declare_parameter<std::vector<int64_t>>(
    "road_crossing_hold_states", std::vector<int64_t>{1, 3, 4, 5, 8});

  mode_ = parse_mode(p.mode);

  // Hard ceilings.  A typo in a yaml must not be able to make the robot reverse
  // ten metres at walking pace, so clamp instead of trusting the file.
  const auto clamp_with_warning = [this](double value, double lo, double hi, const char * name) {
    const double clamped = std::clamp(value, lo, hi);
    if (std::abs(clamped - value) > 1e-9) {
      RCLCPP_WARN(
        get_logger(), "parameter '%s' = %.3f clamped to %.3f", name, value, clamped);
    }
    return clamped;
  };
  p.max_lateral_excursion_m =
    clamp_with_warning(p.max_lateral_excursion_m, 0.0, 3.0, "max_lateral_excursion_m");
  p.corridor_scan_distance_m =
    clamp_with_warning(p.corridor_scan_distance_m, 1.0, 20.0, "corridor_scan_distance_m");
  p.goal_check_interval_m =
    clamp_with_warning(p.goal_check_interval_m, 0.2, 5.0, "goal_check_interval_m");
  p.recovery_timeout_sec =
    clamp_with_warning(p.recovery_timeout_sec, 5.0, 120.0, "recovery_timeout_sec");
  p.recovery_speed_limit_mps =
    clamp_with_warning(p.recovery_speed_limit_mps, 0.05, 0.5, "recovery_speed_limit_mps");


  try {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    vehicle_width_m_ = vehicle_info.vehicle_width_m;
    base_to_front_m_ = vehicle_info.max_longitudinal_offset_m;
    base_to_rear_m_ = -vehicle_info.min_longitudinal_offset_m;
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      get_logger(), "could not read vehicle_info (%s); falling back to %.2f m", e.what(),
      vehicle_width_m_);
  }
}

// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------
void StuckRecoverySupervisorNode::setup_interfaces()
{
  const auto qos = rclcpp::QoS{1};
  const auto latched = rclcpp::QoS{1}.transient_local();

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    param_.topic_odometry, qos,
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; });
  sub_velocity_ = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    param_.topic_velocity_status, qos,
    [this](const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
      velocity_ = msg;
    });
  sub_control_cmd_ = create_subscription<autoware_control_msgs::msg::Control>(
    param_.topic_control_cmd, qos,
    [this](const autoware_control_msgs::msg::Control::ConstSharedPtr msg) { control_cmd_ = msg; });
  sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    param_.topic_trajectory, qos,
    [this](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
      trajectory_ = msg;
    });
  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    param_.topic_map, rclcpp::QoS{1}.transient_local(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
      route_handler_.setMap(*msg);
      map_ready_ = true;
      // The route may well have arrived first; it could not be fed then, so feed it now.
      if (route_) {
        route_handler_.setRoute(*route_);
        route_handler_ready_ = true;
      }
    });
  sub_route_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    param_.topic_route, latched,
    [this](const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) {
      // [FRESH-MISSION 2026-09-15] A new mission must not inherit anything from the old
      // one.  Observed: the robot reached its goal while the state machine was still in
      // RECOVERY/REPLAN -- nothing forces those states back to NOMINAL on arrival -- and
      // the moment a new goal was set it carried straight on recovering, so the robot sat
      // waiting at the start of a fresh mission with the obstacle still far away.
      //
      // Only the flag is set here.  The reset itself runs on the timer, because this is a
      // subscription callback and transitioning the state machine from another thread
      // would race every step_*() function.
      if (
        !have_route_uuid_ ||
        !std::equal(
          prev_route_uuid_.begin(), prev_route_uuid_.end(), msg->uuid.uuid.begin())) {
        route_changed_ = have_route_uuid_;  // not on the very first route of the session
        std::copy(msg->uuid.uuid.begin(), msg->uuid.uuid.end(), prev_route_uuid_.begin());
        have_route_uuid_ = true;
        arrival_handled_ = false;
      }
      route_ = msg;
      // The handler needs both halves; feeding the route before the map throws.
      if (map_ready_) {
        route_handler_.setRoute(*msg);
        route_handler_ready_ = true;
      }
    });
  sub_route_state_ = create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    param_.topic_route_state, latched,
    [this](const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg) {
      route_state_ = msg;
    });
  sub_op_mode_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    param_.topic_operation_mode, latched,
    [this](const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg) {
      operation_mode_ = msg;
    });
  sub_control_mode_ = create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
    param_.topic_control_mode, qos,
    [this](const autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg) {
      control_mode_ = msg;
    });
  sub_gear_ = create_subscription<autoware_vehicle_msgs::msg::GearReport>(
    param_.topic_gear, qos,
    [this](const autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr msg) { gear_ = msg; });
  sub_scenario_ = create_subscription<autoware_internal_planning_msgs::msg::Scenario>(
    param_.topic_scenario, qos,
    [this](const autoware_internal_planning_msgs::msg::Scenario::ConstSharedPtr msg) {
      scenario_ = msg;
    });
  sub_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    param_.topic_occupancy_grid, qos,
    [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      grid_ = msg;
      grid_at_ = this->now();
      // A recovery grid with no occupied cell anywhere is not an empty road, it is a
      // broken one.  costmap_generator fills the primitives layer by painting the whole
      // grid occupied and carving the road lanelets out of it -- but if the map has no
      // subtype=road lanelets it returns early and the layer stays free everywhere, with
      // nothing logged.  Every "is this pose on drivable ground" test would then answer
      // yes, anywhere, which is exactly the guarantee that must not fail quietly.
      const auto occupied = std::count_if(
        msg->data.begin(), msg->data.end(),
        [this](int8_t v) { return v < 0 || v >= param_.aeb_occupancy_threshold; });
      grid_occupied_fraction_ =
        msg->data.empty() ? 0.0
                          : static_cast<double>(occupied) / static_cast<double>(msg->data.size());
    });
  sub_recovery_traj_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    param_.topic_recovery_trajectory, qos,
    [this](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
      recovery_trajectory_ = msg;
    });
  sub_freespace_traj_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    param_.topic_freespace_trajectory, qos,
    [this](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
      freespace_trajectory_ = msg;
      freespace_traj_at_ = this->now();
    });
  sub_recovery_completed_ = create_subscription<std_msgs::msg::Bool>(
    param_.topic_recovery_is_completed, qos,
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) { recovery_completed_ = msg->data; });
  sub_road_crossing_ = create_subscription<autoware_internal_debug_msgs::msg::Int32Stamped>(
    param_.topic_road_crossing_state, qos,
    [this](const autoware_internal_debug_msgs::msg::Int32Stamped::ConstSharedPtr msg) {
      road_crossing_state_ = msg;
    });
  sub_emergency_ = create_subscription<std_msgs::msg::UInt8>(
    param_.topic_emergency_code, qos,
    [this](const std_msgs::msg::UInt8::ConstSharedPtr msg) { emergency_code_ = msg->data; });
  sub_robot_fsm_ = create_subscription<std_msgs::msg::Int16>(
    param_.topic_robot_fsm, latched, [this](const std_msgs::msg::Int16::ConstSharedPtr msg) {
      robot_fsm_ = msg->data;
      has_robot_fsm_ = true;
    });

  // Planning factors are the only way to learn *who* asked for the stop.  Topic
  // names follow planning_factor_interface.hpp: "/planning/planning_factors/" + name.
  const auto subscribe_factor = [this](const std::string & name) {
    const std::string topic = "/planning/planning_factors/" + name;
    sub_factors_.push_back(
      create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
        topic, rclcpp::QoS{1},
        [this, name](
          const autoware_internal_planning_msgs::msg::PlanningFactorArray::ConstSharedPtr msg) {
          planning_factors_[name] = msg;
        }));
  };
  for (const auto & name : param_.pathological_factor_topics) {
    subscribe_factor(name);
  }
  for (const auto & name : param_.legit_factor_topics) {
    subscribe_factor(name);
  }

  pub_scenario_ = create_publisher<autoware_internal_planning_msgs::msg::Scenario>(
    param_.topic_out_scenario, rclcpp::QoS{1});
  pub_route_ = create_publisher<autoware_planning_msgs::msg::LaneletRoute>(
    param_.topic_out_route, latched);
  pub_force_recovery_ =
    create_publisher<std_msgs::msg::Bool>(param_.topic_out_force_recovery, rclcpp::QoS{1});
  pub_state_ = create_publisher<autoware_stuck_recovery_msgs::msg::RecoveryState>(
    param_.topic_out_state, rclcpp::QoS{1});
  pub_state_int_ = create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    param_.topic_out_state_int, rclcpp::QoS{1});
  pub_diagnosis_ = create_publisher<autoware_stuck_recovery_msgs::msg::StuckDiagnosis>(
    param_.topic_out_diagnosis, rclcpp::QoS{1});
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    param_.topic_out_markers, rclcpp::QoS{1});
  // Volatile on purpose: a latched arrow from a previous episode lingering in RViz is
  // actively misleading while debugging.  It is republished every tick anyway.
  pub_escape_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    param_.topic_out_escape_goal, rclcpp::QoS{1});
  pub_recovery_traj_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    param_.topic_out_recovery_trajectory, rclcpp::QoS{1});

  srv_force_ = create_service<autoware_stuck_recovery_msgs::srv::ForceRecovery>(
    param_.service_force_recovery,
    std::bind(
      &StuckRecoverySupervisorNode::on_force_recovery, this, std::placeholders::_1,
      std::placeholders::_2));

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("stuck_recovery");
  diagnostics_->add(
    "stuck_recovery_status", this, &StuckRecoverySupervisorNode::on_diagnostics);

  const auto period = rclcpp::Rate(std::max(1.0, param_.update_rate_hz)).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period, std::bind(&StuckRecoverySupervisorNode::on_timer, this));
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void StuckRecoverySupervisorNode::on_timer()
{
  update_motion();
  classify_stop();

  // A new mission wipes the slate before anything else looks at the state.
  if (route_changed_) {
    route_changed_ = false;
    reset_for_new_mission("a new route was set");
  }

  // Arrival ends the episode too.  Without this, a robot that reached its goal while an
  // episode was still open stayed in RECOVERY/REPLAN/TRANSIT/BLOCKED indefinitely, and
  // the next mission started from the middle of the last one's manoeuvre.  Judged on the
  // geometry as well as the announcement, for the same reason classify_stop() does: the
  // routing state has been seen claiming ARRIVED 15 m short of the goal.
  if (
    !arrival_handled_ && state_ != State::NOMINAL && route_state_ && route_ && odom_ &&
    route_state_->state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED &&
    distance2d(odom_->pose.pose, route_->goal_pose) <= param_.goal_reached_dist_m) {
    arrival_handled_ = true;
    reset_for_new_mission("the robot arrived at its goal");
  }

  switch (state_) {
    case State::NOMINAL:
      step_nominal();
      break;
    case State::SUSPECT:
      step_suspect();
      break;
    case State::PROBE:
      step_probe();
      break;
    case State::RECOVERY:
      step_recovery();
      break;
    case State::BLOCKED:
      step_blocked();
      break;
    case State::COOLDOWN:
      step_cooldown();
      break;
    case State::ABORT:
      step_abort();
      break;
    case State::REPLAN:
      step_replan();
      break;
    case State::TRANSIT:
      step_transit();
      break;
  }

  // Arming is derived from the state rather than latched, so that any exit path --
  // including an exception-driven one -- also disarms.  BLOCKED keeps the costmap
  // alive because that is how we notice the blockage going away.
  // [ABORT-SHADOW 2026-09-15] ABORT is armed too, unless the stall was mechanical.  It
  // used to be a state with nothing running, which is what made it a dead end: neither of
  // its exits (the robot moving, the blockage clearing) can come true for a robot frozen
  // inside an obstacle_stop margin, and with the costmap and freespace both shut down
  // there was nothing watching for the situation to improve either.  Arming keeps them
  // planning in the background; note that `force` below deliberately does NOT include
  // ABORT, so none of this moves the robot -- it only looks.
  const bool arm_costmap =
    mode_ >= Mode::PROBE_ONLY &&
    (state_ == State::PROBE || state_ == State::RECOVERY || state_ == State::BLOCKED ||
     state_ == State::REPLAN || state_ == State::TRANSIT ||
     (state_ == State::ABORT && !abort_was_mechanical_) ||
     // [UNKNOWN-STOP 2026-09-16] SUSPECT needs a grid too, but only while it is holding a
     // stop nothing has claimed -- that is the one case where it has to measure for itself
     // instead of reading the answer off a planning factor.  Gated on the source so the
     // ordinary SUSPECT (which is most of them, and usually brief) costs nothing extra.
     (state_ == State::SUSPECT && stop_source_ == StuckDiagnosis::SOURCE_UNKNOWN));
  publish_recovery_arming(arm_costmap);

  std_msgs::msg::Bool force;
  // TRANSIT drives the robot too, so the scenario has to stay switched over for it.
  force.data =
    (mode_ == Mode::RECOVERY &&
     (state_ == State::RECOVERY || state_ == State::REPLAN || state_ == State::TRANSIT));
  pub_force_recovery_->publish(force);

  relay_recovery_trajectory();
  publish_outputs();
  publish_markers();
}

void StuckRecoverySupervisorNode::update_motion()
{
  const auto now = this->now();

  double measured = 0.0;
  if (velocity_) {
    measured = velocity_->longitudinal_velocity;
  } else if (odom_) {
    measured = odom_->twist.twist.linear.x;
  }

  const bool stopped_now = std::abs(measured) < param_.th_stopped_velocity_mps;
  if (!stopped_now) {
    stopped_since_ = now;
    // Movement is the only evidence that the situation actually changed, so it is what
    // clears the abort-retry budget.  Resetting on entering NOMINAL instead would make
    // the budget meaningless -- every retry passes through NOMINAL by construction.
    abort_retries_ = 0;
  }
  is_stopped_ = stopped_now;

  const double commanded = control_cmd_ ? control_cmd_->longitudinal.velocity : 0.0;
  const bool asked_to_move = std::abs(commanded) > param_.th_commanded_velocity_mps;
  if (!(asked_to_move && std::abs(measured) < 0.03)) {
    commanded_but_still_since_ = now;
    commanded_but_still_ = false;
  } else {
    commanded_but_still_ =
      (now - commanded_but_still_since_).seconds() > param_.mechanical_confirm_sec;
  }

  if (odom_) {
    const auto & pose = odom_->pose.pose;
    // There is no reverse budget any more.  It existed to bound "the obstacle keeps
    // closing in and the robot keeps retreating", and that case is now structurally
    // impossible: recovery refuses to drive any plan that travels against the mission
    // direction.  The budget was also measuring the wrong thing -- it projected
    // displacement onto the VEHICLE heading, so in reverse motion mode every metre of
    // ordinary recovery travel counted as reverse (4.03 m recorded in a run where the
    // retreat never ran at all).
    last_pose_ = pose;
    has_last_pose_ = true;

    // Displacement, not path length: a robot that shuffles back and forth by 10 cm
    // has not escaped anything, and must keep counting as stuck.
    if (state_ != State::NOMINAL) {
      progress_since_entry_m_ = distance2d(entry_pose_, pose);
    }
  }
}

void StuckRecoverySupervisorNode::classify_stop()
{
  legit_stop_ = false;
  legit_reason_.clear();
  stop_source_ = StuckDiagnosis::SOURCE_NONE;
  stop_source_module_.clear();

  const auto now = this->now();

  if (!is_operational()) {
    legit_stop_ = true;
    legit_reason_ = "not_autonomous";
    return;
  }

  if (emergency_code_ != 0) {
    legit_stop_ = true;
    legit_reason_ = "emergency_code";
    return;
  }

  if (route_state_ && route_state_->state != autoware_adapi_v1_msgs::msg::RouteState::SET) {
    // ARRIVED is only a legitimate reason to stand still if the robot has actually
    // arrived.  Seen on the robot: routing reported ARRIVED while the vehicle was still
    // 15 m from the goal of that very route, held there by an obstacle_stop 5 cm ahead.
    // The watchdog took the report at face value, never even classified the stop, and
    // sat in NOMINAL while the robot was stuck exactly the way this node exists to
    // catch.  Trust the geometry over the announcement: it is the same route's own goal
    // pose being measured against, so a large distance contradicts the claim outright.
    const bool arrived_claim =
      route_state_->state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
    const double to_goal = (odom_ && route_)
                             ? distance2d(odom_->pose.pose, route_->goal_pose)
                             : 0.0;
    if (!arrived_claim || !odom_ || !route_ || to_goal <= param_.goal_reached_dist_m) {
      legit_stop_ = true;
      legit_reason_ = "route_not_set";
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "routing reports ARRIVED but the goal of that route is still %.2f m away (> %.2f m): "
      "not treating this as a finished mission",
      to_goal, param_.goal_reached_dist_m);
  }

  if (odom_ && route_) {
    if (distance2d(odom_->pose.pose, route_->goal_pose) < param_.goal_reached_dist_m) {
      legit_stop_ = true;
      legit_reason_ = "goal_reached";
      return;
    }
  }

  // The road-crossing FSM owns its own watchdog and its own reroute logic.  While it
  // is holding, this supervisor must stay completely out of the way -- two
  // controllers fighting over the same robot is worse than either problem alone.
  if (road_crossing_state_) {
    const auto & hold = param_.road_crossing_hold_states;
    if (std::find(hold.begin(), hold.end(), road_crossing_state_->data) != hold.end()) {
      legit_stop_ = true;
      legit_reason_ = "road_crossing_hold";
      return;
    }
  }

  // A STOP factor only counts when its control point is actually near: an
  // obstacle_stop reported 20 m ahead while the robot happens to be standing still for
  // some other reason is not a blockage.
  // A STOP factor counts as a blockage only when its control point is AHEAD and near.
  // Sign matters: a negative distance means the robot has already driven past whatever
  // caused the stop, and there is nothing in front to manoeuvre around.
  bool stop_behind = false;
  const auto has_stop_factor = [this, &stop_behind](const std::string & name) {
    const auto it = planning_factors_.find(name);
    if (it == planning_factors_.end() || !it->second) {
      return false;
    }
    for (const auto & factor : it->second->factors) {
      if (factor.behavior != autoware_internal_planning_msgs::msg::PlanningFactor::STOP) {
        continue;
      }
      if (factor.control_points.empty()) {
        return true;  // no distance reported; take it at face value
      }
      for (const auto & cp : factor.control_points) {
        if (cp.distance >= 0.0 && cp.distance <= param_.th_stop_factor_distance_m) {
          return true;
        }
        if (cp.distance < 0.0) {
          stop_behind = true;
        }
      }
    }
    return false;
  };

  std::string map_stop_module;
  for (const auto & name : param_.legit_factor_topics) {
    if (has_stop_factor(name)) {
      map_stop_module = name;
      break;
    }
  }
  if (!map_stop_module.empty()) {
    if (!map_stop_active_) {
      map_stop_active_ = true;
      map_stop_since_ = now;
    }
    stop_source_ = StuckDiagnosis::SOURCE_MAP_STOP;
    stop_source_module_ = map_stop_module;
    // A map stop that never releases is itself pathological, so the patience is
    // long but finite.
    if ((now - map_stop_since_).seconds() < param_.map_stop_patience_sec) {
      legit_stop_ = true;
      legit_reason_ = "map_stop:" + map_stop_module;
      return;
    }
  } else {
    map_stop_active_ = false;
  }

  // Commanded to move but standing still: wheels slipping, something caught, or an
  // actuator interlock.  Reversing out of that is the wrong answer, so this is
  // deliberately classified *before* the obstacle case.
  if (commanded_but_still_) {
    stop_source_ = StuckDiagnosis::SOURCE_MECHANICAL;
    // Name the most likely cause when it applies.  On the real robot the low level
    // ignores the gear, so this only bites in the planning simulator when the geared
    // vehicle model is selected -- but there it is indistinguishable from a traction
    // failure, so say which one it is.
    if (gear_ && control_cmd_) {
      const bool in_reverse =
        gear_->report == autoware_vehicle_msgs::msg::GearReport::REVERSE;
      const bool wants_forward = control_cmd_->longitudinal.velocity > 0.0;
      if (in_reverse && wants_forward) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "commanded %+.2f m/s (forward) but the gear reads REVERSE and the robot is not "
          "moving. On the real robot the gear is ignored, so this points at the simulator "
          "using DELAY_STEER_ACC_GEARED -- switch simulator_model.param.yaml to "
          "DELAY_STEER_ACC, which takes the sign from the command as the real one does.",
          control_cmd_->longitudinal.velocity);
      }
    }
    return;
  }

  for (const auto & name : param_.pathological_factor_topics) {
    if (has_stop_factor(name)) {
      stop_source_ = StuckDiagnosis::SOURCE_OBSTACLE_STOP;
      stop_source_module_ = name;
      return;
    }
  }

  // A stop point BEHIND the robot does not mean the obstacle is behind it -- it means the
  // robot is INSIDE the stop margin of an obstacle that is still ahead, so the margin
  // puts the required stop line back where it has already been.  insert_stop() then
  // zeroes the trajectory from that index to the end, which freezes everything in front:
  // lane driving has no way out of this at all, which makes it the case recovery matters
  // most for.  Classify it as actionable and let PROBE decide using the costmap, which
  // is the thing that actually knows whether there is room to go around.
  if (stop_behind && is_stopped_) {
    stop_source_ = StuckDiagnosis::SOURCE_STOP_BEHIND;
    stop_source_module_ = "obstacle_stop (stop point behind: inside the stop margin)";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "inside the stop margin: the required stop line is behind the robot, so insert_stop() "
      "has zeroed the whole trajectory ahead. Lane driving cannot escape this; treating it "
      "as a blockage and letting the corridor sweep decide.");
    return;
  }

  if (trajectory_ && trajectory_->points.empty()) {
    stop_source_ = StuckDiagnosis::SOURCE_PLANNER_EMPTY;
    return;
  }

  if (is_stopped_) {
    stop_source_ = StuckDiagnosis::SOURCE_UNKNOWN;
  }
}

bool StuckRecoverySupervisorNode::is_operational() const
{
  if (!operation_mode_) {
    return false;
  }
  if (
    operation_mode_->mode != autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS ||
    !operation_mode_->is_autoware_control_enabled) {
    return false;
  }
  if (
    control_mode_ &&
    control_mode_->mode != autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS) {
    return false;
  }
  return true;
}

bool StuckRecoverySupervisorNode::is_stuck_candidate() const
{
  return is_stopped_ && !legit_stop_ && is_operational() && odom_ != nullptr;
}

// ---------------------------------------------------------------------------
// FSM steps
// ---------------------------------------------------------------------------
void StuckRecoverySupervisorNode::transition(State next, const std::string & reason)
{
  if (next == state_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "[episode %u] %s -> %s (%s)", episode_id_, state_name(state_).c_str(),
    state_name(next).c_str(), reason.c_str());
  state_ = next;
  state_since_ = this->now();
  aeb_holding_ = false;
  if (next == State::TRANSIT) {
    // The manoeuvre is over; only the line-up remains.  Drop the freespace plan so a stale
    // one can never be relayed here, and start the hold from now rather than from whatever
    // the last alignment was.
    freespace_trajectory_.reset();
    freespace_traj_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    transit_aligned_since_ = this->now();
  }
  if (next == State::COOLDOWN || next == State::BLOCKED) {
    // The scenario is no longer driving; drop the plan and goal so that re-arming later
    // cannot resurrect them.
    discard_recovery_state();
  }
  // [ABORT-SHADOW 2026-09-15] ABORT deliberately keeps its plan, where it used to be in
  // the list above.  The usual reason an episode aborts is that the plan it had was
  // obstructed -- so that plan is precisely the thing worth re-checking once the
  // obstruction goes away, and throwing it away meant starting from nothing.
  //
  // The staleness this discard was guarding against is already covered twice over by
  // freespace_plan_is_valid(): a plan older than freespace_plan_timeout_sec is rejected,
  // and so is any plan that predates the most recently published goal.  step_abort()
  // republishes that goal every abort_shadow_replan_sec, so only a plan freespace made
  // after the latest request can ever be driven.  On a mechanical abort the pipeline is
  // not armed at all, so the plan simply ages out.
  if (next == State::ABORT || next == State::BLOCKED) {
    abort_reason_ = reason;
  }
  if (next == State::SUSPECT) {
    // Same reason as BLOCKED below: a timestamp left over from an earlier episode would
    // make the hold appear already satisfied on the first tick.
    unknown_clear_since_ = this->now();
  }
  if (next == State::BLOCKED) {
    // Arm the exit hysteresis on ENTRY.  Left carrying a timestamp from some earlier
    // episode, clear_hold_sec had already elapsed by the time the first tick ran, so the
    // hold was never applied at all and BLOCKED could be left on its very first look.
    clear_since_ = this->now();
  }
  if (next == State::ABORT) {
    // A mechanical stall means the robot was told to move and did not; recovering from
    // that is not a planning decision, so it stays latched for a human.
    abort_was_mechanical_ = (stop_source_ == StuckDiagnosis::SOURCE_MECHANICAL);
    abort_clear_since_ = this->now();
  }
  if (next == State::NOMINAL) {
    // Leave nothing behind that a later episode could act on.
    discard_recovery_state();
    abort_reason_.clear();
    reference_path_.clear();
    start_blocked_ = false;
    abort_was_mechanical_ = false;
    attempts_ = 0;
    progress_since_entry_m_ = 0.0;
  }
}

void StuckRecoverySupervisorNode::step_nominal()
{
  if (!is_stuck_candidate()) {
    return;
  }
  if ((this->now() - stopped_since_).seconds() < 1.0) {
    return;  // debounce: a momentary zero is not a suspicion
  }

  ++episode_id_;
  episode_since_ = this->now();
  episode_stop_source_ = stop_source_;
  attempts_ = 0;
  progress_since_entry_m_ = 0.0;
  entry_pose_ = odom_->pose.pose;
  capture_reference_path();
  transition(State::SUSPECT, "ego stopped without a legitimate reason");
}

void StuckRecoverySupervisorNode::step_suspect()
{
  const auto now = this->now();

  if (legit_stop_ || !is_operational()) {
    transition(State::NOMINAL, "stop became legitimate: " + legit_reason_);
    return;
  }
  if (progress_since_entry_m_ > param_.unstuck_progress_m) {
    transition(State::NOMINAL, "ego made progress");
    return;
  }
  if (stop_source_ == StuckDiagnosis::SOURCE_MECHANICAL) {
    transition(State::ABORT, "mechanical stall - recovery would be unsafe");
    return;
  }
  if ((now - episode_since_).seconds() > param_.max_episode_duration_sec) {
    transition(State::ABORT, "episode watchdog expired");
    return;
  }

  const bool timer_elapsed = (now - state_since_).seconds() > param_.suspect_timeout_sec;
  bool triggerable = stop_source_ == StuckDiagnosis::SOURCE_OBSTACLE_STOP ||
                     stop_source_ == StuckDiagnosis::SOURCE_STOP_BEHIND ||
                     stop_source_ == StuckDiagnosis::SOURCE_MAP_STOP;

  // [UNKNOWN-STOP 2026-09-16] A stop that no planning factor claims used to be a dead end.
  // SOURCE_UNKNOWN is not in the triggerable list above and is not a legitimate stop either,
  // so SUSPECT could neither advance to PROBE nor fall back to NOMINAL -- it simply waited,
  // for ever.  Caught live: 83 s in SUSPECT with stop_source=UNKNOWN, stop_source_module
  // empty, a zero-velocity point 0.36 m ahead of the robot (inside the controller's 1.50 m
  // engage distance, so it commanded 0 m/s) and nothing anywhere saying who put it there.
  //
  // Rather than trust a classification that has already failed, measure.  The costmap knows
  // what is actually in front of the robot, and it is armed for this state now.  Two
  // outcomes, and the hold is on the CLEAR side only: releasing a robot that is genuinely
  // boxed in is the expensive mistake, so that direction has to be sustained, while a
  // blockage is acted on as soon as it is seen.
  if (!triggerable && timer_elapsed && stop_source_ == StuckDiagnosis::SOURCE_UNKNOWN) {
    if (!costmap_is_fresh() || reference_path_.empty()) {
      unknown_clear_since_ = now;  // no grid yet; the warm-up is not evidence of anything
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[episode %u] stopped for %.0f s and no planning factor claims it; waiting for the "
        "recovery costmap before judging for myself",
        episode_id_, (now - state_since_).seconds());
      return;
    }
    size_t from = 0;
    if (odom_) {
      if (const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose)) {
        from = *nearest;
      }
    }
    last_sweep_ = sweep(from);
    const bool clear_now = last_sweep_.valid && !last_sweep_.has_blockage;
    if (!clear_now) {
      unknown_clear_since_ = now;
    }
    corridor_clear_ = clear_now;

    if (last_sweep_.valid && last_sweep_.has_blockage) {
      // Something IS in the way, and Autoware never said so -- which is exactly the case
      // this module exists for.  Treat it as actionable and let PROBE decide with the
      // costmap, the same argument the STOP_BEHIND branch makes.
      RCLCPP_WARN(
        get_logger(),
        "[episode %u] stop was unexplained, but the corridor is blocked %.2f m ahead at "
        "(%.2f, %.2f) -- nothing in planning claimed it, so treating it as actionable",
        episode_id_, last_sweep_.first_blocked_arc_m, last_sweep_.narrowest_pose.position.x,
        last_sweep_.narrowest_pose.position.y);
      triggerable = true;
    } else if ((now - unknown_clear_since_).seconds() >= param_.unknown_stop_clear_hold_sec) {
      RCLCPP_INFO(
        get_logger(),
        "[episode %u] stop was unexplained and the corridor has read clear for %.1f s -- "
        "nothing to recover from; handing back to lane driving",
        episode_id_, param_.unknown_stop_clear_hold_sec);
      transition(State::NOMINAL, "unexplained stop, but the way ahead is clear");
      return;
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[episode %u] stop is unexplained; corridor has been clear for %.1f s of the %.1f s "
        "needed before handing back",
        episode_id_, (now - unknown_clear_since_).seconds(), param_.unknown_stop_clear_hold_sec);
      return;
    }
  }

  if (!(force_requested_ || (timer_elapsed && triggerable))) {
    return;
  }

  if (mode_ == Mode::DETECT_ONLY) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[episode %u] detect_only: would probe now (blocked by '%s' for %.1f s)", episode_id_,
      stop_source_module_.c_str(), (now - state_since_).seconds());
    return;
  }

  // One-shot: an operator forcing a recovery must not leave the flag armed for every
  // subsequent episode.
  const bool was_forced = force_requested_;
  force_requested_ = false;

  // Re-capture: the trajectory may have been refreshed since we entered SUSPECT and
  // the newest one is the better description of where we were trying to go.
  capture_reference_path();
  transition(State::PROBE, was_forced ? "forced by operator" : "stuck confirmed");
}

void StuckRecoverySupervisorNode::step_probe()
{
  const auto now = this->now();

  if (legit_stop_ || !is_operational()) {
    transition(State::NOMINAL, "stop became legitimate: " + legit_reason_);
    return;
  }
  if (progress_since_entry_m_ > param_.unstuck_progress_m) {
    transition(State::NOMINAL, "ego made progress");
    return;
  }

  // The recovery costmap only starts publishing once we arm it, so give it a moment
  // before believing an empty answer.
  if ((now - state_since_).seconds() < param_.probe_warmup_sec) {
    return;
  }
  if (!costmap_is_fresh()) {
    if ((now - state_since_).seconds() > param_.probe_warmup_sec + 5.0) {
      transition(
        State::ABORT, grid_
                        ? "recovery costmap went stale (is the recovery container alive?)"
                        : "recovery costmap never published (is the recovery container alive?)");
    }
    return;
  }

  if (grid_occupied_fraction_ < param_.min_occupied_fraction) {
    RCLCPP_ERROR(
      get_logger(),
      "[episode %u] only %.1f%% of the recovery costmap is occupied. Everything off the "
      "road should be, so the road layer was never filled -- the lanelet map has no "
      "subtype=road lanelets, and costmap_generator leaves the grid free everywhere "
      "without saying so. Recovery cannot keep the robot on the road with this grid, so "
      "it will not drive.",
      episode_id_, 100.0 * grid_occupied_fraction_);
    transition(State::ABORT, "recovery costmap has no road boundary; refusing to drive");
    return;
  }

  if (reference_path_.empty() && !capture_reference_path()) {
    transition(State::ABORT, "no reference path to probe");
    return;
  }

  // The reference path now starts behind ego, so the sweep must begin at ego rather
  // than at index 0.
  size_t ego_index = 0;
  if (odom_) {
    if (const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose)) {
      ego_index = *nearest;
    }
  }
  last_sweep_ = sweep(ego_index);
  if (!last_sweep_.valid) {
    transition(State::ABORT, "corridor sweep produced no samples");
    return;
  }

  // [TWO-LINE-PROBE 2026-09-15] The sweep above runs along the lanelet centerline, which
  // is the right reference for the EXIT question -- lane driving resumes on the
  // centerline, so that is where the robot's footprint has to fit before handing back.
  // It is the wrong reference for the ENTRY question.  What stops the robot is whatever
  // sits on the trajectory lane driving is actually asking it to follow, and once the
  // robot is displaced or the path curves, those are two different lines.
  //
  // Measured live 2026-09-15, projected into the robot's travel frame at the obstacle's
  // own station 2.50 m ahead:
  //
  //     obstacle                        lateral -0.05 m   (dead ahead of the robot)
  //     trajectory being followed       lateral -0.94 m
  //     centerline the sweep used       lateral -1.70 m
  //
  // The centerline clears the obstacle by 1.65 m, so the sweep reported free_width
  // 2.325 m and has_blockage = false -- "nothing to manoeuvre around" -- and PROBE went
  // to BLOCKED.  Meanwhile obstacle_stop had frozen the robot for that same obstacle,
  // because the rejoin curve lane driving planned passes within about 0.95 m of it.  Both
  // were right about their own line; the robot sat still between them.
  //
  // So ask about the followed trajectory as well.  A blockage on EITHER line means there
  // is something to manoeuvre around.  This loosens no safety gate: the fit test and the
  // exit test below still use the centerline sweep, which is the one that speaks for lane
  // driving.
  const auto followed = followed_path();
  if (!last_sweep_.has_blockage && followed.size() >= 2) {
    size_t followed_from = 0;
    if (const auto nearest = find_nearest_index(followed, odom_->pose.pose)) {
      followed_from = *nearest;
    }
    const auto followed_sweep = sweep_path(followed, followed_from);
    if (followed_sweep.valid && followed_sweep.has_blockage) {
      RCLCPP_INFO(
        get_logger(),
        "[episode %u] the centerline is clear, but the trajectory the robot is actually "
        "following is blocked %.2f m along it, at (%.2f, %.2f) -- treating that as the "
        "blockage to manoeuvre around",
        episode_id_, followed_sweep.first_blocked_arc_m,
        followed_sweep.narrowest_pose.position.x, followed_sweep.narrowest_pose.position.y);
      // Take only the blockage facts from it.  The widths, the chosen side and the offset
      // stay as the centerline measured them, because the escape goals and the exit test
      // are both expressed against the centerline.
      last_sweep_.has_blockage = true;
      last_sweep_.first_blocked_arc_m = followed_sweep.first_blocked_arc_m;
      last_sweep_.clear = false;
    }
  }

  // If the costmap sees no blockage at all, there is nothing to manoeuvre around and
  // handing over to freespace is meaningless -- it would plan a pointless detour, or
  // worse, straight through something the costmap cannot see.  Say so instead.
  if (!last_sweep_.has_blockage) {
    // Normally a clear corridor means there is nothing to manoeuvre around, so handing
    // over would buy a pointless detour -- or worse, a detour around something the
    // costmap cannot see.  STOP_BEHIND is the exception, and it is the whole reason that
    // stop source exists: the stop line sits BEHIND the robot, so insert_stop() has
    // zeroed the entire trajectory ahead and lane driving provably cannot escape, however
    // clear the road in front is.  A clear corridor is the EXPECTED reading there, not a
    // contradiction, and driving straight out of the stop margin is exactly the escape.
    if (stop_source_ != StuckDiagnosis::SOURCE_STOP_BEHIND) {
      RCLCPP_WARN(
        get_logger(),
        "[episode %u] stopped by '%s' but the recovery costmap sees a clear %.2f m corridor: "
        "the obstacle is not in the costmap (or is behind the robot). Not handing over.",
        episode_id_, stop_source_module_.c_str(), last_sweep_.free_width_m);
      transition(State::BLOCKED, "costmap shows no blockage to manoeuvre around");
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[episode %u] corridor is clear (%.2f m) and the stop line is behind the robot: "
      "nothing to drive around, just drive out of the stop margin",
      episode_id_, last_sweep_.free_width_m);
  }

  const bool fits = last_sweep_.free_width_m >= last_sweep_.required_width_m;
  RCLCPP_INFO(
    get_logger(),
    "[episode %u] probe over %zu stations: left=%.2f m right=%.2f m required=%.2f m -> %s (%s, "
    "offset %+.2f m)",
    episode_id_, last_sweep_.stations, last_sweep_.free_left_m, last_sweep_.free_right_m,
    last_sweep_.required_width_m, fits ? "FITS" : "TOO NARROW", side_name(last_sweep_.best_side),
    last_sweep_.best_offset_m);

  if (!fits) {
    transition(State::BLOCKED, "lateral corridor too narrow for the robot");
    return;
  }

  if (mode_ != Mode::RECOVERY) {
    RCLCPP_WARN(
      get_logger(), "[episode %u] probe mode: would recover now, but mode is '%s'", episode_id_,
      param_.mode.c_str());
    cooldown_until_ = now + rclcpp::Duration::from_seconds(param_.cooldown_sec);
    transition(State::COOLDOWN, "probe-only: recovery withheld");
    return;
  }

  if (attempts_ >= param_.max_attempts) {
    transition(State::ABORT, "attempt budget exhausted");
    return;
  }

  goal_candidates_ = collect_goal_candidates();
  goal_index_ = 0;
  if (goal_candidates_.empty()) {
    transition(State::BLOCKED, "no usable escape goal within the remaining path");
    return;
  }

  ++attempts_;
  recovery_completed_ = false;
  clear_since_ = now;
  last_valid_plan_ = now;
  spent_since_ = now;
  if (!publish_current_goal()) {
    transition(State::BLOCKED, "could not publish an escape goal");
    return;
  }
  transition(
    State::RECOVERY, "corridor fits; handing over to freespace (" +
                       std::to_string(goal_candidates_.size()) + " goal candidates)");
}

void StuckRecoverySupervisorNode::step_recovery()
{
  const auto now = this->now();

  // --- abort conditions, checked before anything else -----------------------
  if (!is_operational()) {
    transition(State::ABORT, "left autonomous mode during recovery");
    return;
  }
  if (emergency_code_ != 0) {
    transition(State::ABORT, "emergency code raised during recovery");
    return;
  }
  if ((now - state_since_).seconds() > param_.recovery_timeout_sec) {
    transition(State::ABORT, "recovery timed out");
    return;
  }
  if ((now - episode_since_).seconds() > param_.max_episode_duration_sec) {
    transition(State::ABORT, "episode watchdog expired");
    return;
  }
  // A brake that has been on for seconds is not protecting against something passing
  // through -- the plan it is braking against is not drivable.  Holding it until the
  // episode times out is how a whole 60 s recovery was lost with the robot never moving,
  // so ask for a different plan instead.
  if (aeb_holding_ && (now - aeb_hold_since_).seconds() > param_.aeb_hold_replan_sec) {
    aeb_holding_ = false;
    force_freespace_replan("the emergency brake has been holding against this plan");
    return;
  }

  // Freespace keeps publishing stop trajectories while it fails, so "has a plan" has
  // to mean "the trajectory actually moves the robot".
  start_blocked_ = odom_ && costmap_is_fresh() && !footprint_is_free(odom_->pose.pose);

  // The robot stopped past the live part of the plan: the nearest point to it already
  // has zero velocity, so the smoother reads target_vel = 0 and will never restart it,
  // while freespace sees no reason to replan.  Ask for a fresh plan from here.
  // Passing the escape goal means the manoeuvre did its job.  Move to the next one that
  // is still ahead rather than continuing to aim at a point now behind the robot.
  if (escape_goal_ && !goal_is_ahead(*escape_goal_)) {
    if (advance_past_passed_goals()) {
      RCLCPP_INFO(
        get_logger(), "[episode %u] escape goal passed; aiming at candidate %zu/%zu",
        episode_id_, goal_index_ + 1, goal_candidates_.size());
      publish_current_goal();
      spent_since_ = now;
      last_valid_plan_ = now;
      return;
    }
    if (!rejoined_reference_path()) {
      // Past the last goal but not back on the path: the manoeuvre is not finished, it
      // has just run out of goals.  Look for a new one from where the robot actually is
      // -- those are placed ON the reference path, so aiming at one pulls it back.
      RCLCPP_WARN(
        get_logger(),
        "[episode %u] escape goal passed but the robot is %.2f m off the path (limit %.2f m) "
        "-- looking for a goal to rejoin on rather than handing back",
        episode_id_, lateral_offset_from_path(), param_.rejoin_lateral_tolerance_m);
      goal_candidates_ = collect_goal_candidates();
      goal_index_ = 0;
      if (!goal_candidates_.empty() && publish_current_goal()) {
        spent_since_ = now;
        last_valid_plan_ = now;
        return;
      }
      // Nothing to aim at and off the path: stopping is the only safe answer.  Handing an
      // off-road robot back to lane driving is how this went wrong in the first place.
      transition(State::ABORT, "ran out of escape goals while still off the path");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "[episode %u] escape goal passed and none left ahead", episode_id_);
    cooldown_until_ = now + rclcpp::Duration::from_seconds(param_.cooldown_sec);
    transition(State::TRANSIT, "escape goal passed; lining up before handing back");
    return;
  }

  if (plan_is_spent_confirmed()) {
    force_freespace_replan("robot stopped beyond the usable part of the plan");
    return;
  }

  if (freespace_plan_is_valid()) {
    last_valid_plan_ = now;
  } else if ((now - last_valid_plan_).seconds() > param_.goal_retry_sec) {
    if (start_blocked_) {
      // Freespace rejects the START pose: the robot's own footprint is already inside the
      // inflated obstacle.  A retreat used to be the answer, and backing out of it is
      // exactly the behaviour that had to go.  No goal can help from this pose.
      transition(
        State::ABORT, "start pose is inside an obstacle; freespace cannot plan from here");
      return;
    }
    reject_plan_and_try_next_goal("freespace produced no usable plan for this goal");
    return;
  }
  if (odom_ && !reference_path_.empty()) {
    const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose);
    if (nearest) {
      const double excursion = distance2d(reference_path_[*nearest], odom_->pose.pose);
      if (excursion > param_.max_lateral_excursion_m) {
        transition(State::ABORT, "strayed too far from the reference path");
        return;
      }
    }
  }

  // --- exit condition -------------------------------------------------------
  //
  // We do NOT wait for freespace's is_completed, and we do not require reaching the
  // escape goal.  The question is only "is the way ahead clear again?", swept over
  // the same corridor the PROBE used.  The escape goal is an upper bound on the
  // manoeuvre, not a target that has to be met.
  size_t from = 0;
  if (odom_) {
    if (const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose)) {
      from = *nearest;
    }
  }
  last_sweep_ = sweep(from);
  const bool clear_now = last_sweep_.valid && last_sweep_.clear;
  if (!clear_now) {
    clear_since_ = now;
  }
  corridor_clear_ = clear_now;

  if (!clear_now || (now - clear_since_).seconds() < param_.clear_hold_sec) {
    // [RECOVERY-SILENCE 2026-09-15] This return used to be silent, and it is the one the
    // state sits on for the whole of a failing episode.  Measured in episode 19: RECOVERY
    // was entered, then the supervisor said NOTHING for 60 s and aborted on the timeout --
    // six log lines for a whole minute, none of which named a reason.  Every other exit
    // path out of this function logs; this one, the most common, did not, so a recovery
    // that quietly never finished was indistinguishable from one that was still working.
    //
    // The sweep already knows the answer, so say it: whether the corridor reads clear, and
    // if not, how far ahead the blockage is and where.  Note the cell semantics -- occupied
    // means obstacle OR off-road -- so "blocked" here does not necessarily mean an object;
    // it can equally be the edge of the drivable area.
    if (!last_sweep_.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[episode %u] waiting to hand back: the corridor sweep produced no samples (%.0f s of "
        "%.0f s used)",
        episode_id_, (now - state_since_).seconds(), param_.recovery_timeout_sec);
    } else if (!last_sweep_.clear) {
      if (last_sweep_.has_blockage) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[episode %u] waiting to hand back: corridor still blocked %.2f m ahead at "
          "(%.2f, %.2f) -- free left %.2f m right %.2f m, need %.2f m (%.0f s of %.0f s used)",
          episode_id_, last_sweep_.first_blocked_arc_m, last_sweep_.narrowest_pose.position.x,
          last_sweep_.narrowest_pose.position.y, last_sweep_.free_left_m,
          last_sweep_.free_right_m, last_sweep_.required_width_m,
          (now - state_since_).seconds(), param_.recovery_timeout_sec);
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[episode %u] waiting to hand back: no blockage found, but the robot footprint does "
          "not fit centred on the path -- free left %.2f m right %.2f m, need %.2f m "
          "(%.0f s of %.0f s used)",
          episode_id_, last_sweep_.free_left_m, last_sweep_.free_right_m,
          last_sweep_.required_width_m, (now - state_since_).seconds(),
          param_.recovery_timeout_sec);
      }
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[episode %u] corridor is clear, holding it for %.1f s of the %.1f s required",
        episode_id_, (now - clear_since_).seconds(), param_.clear_hold_sec);
    }
    return;
  }

  // The corridor being clear is not enough: lane driving resumes on the PATH, so the
  // robot has to be back on it.  Judging the exit on the sweep alone let recovery hand
  // back while the robot was still standing beside the obstacle -- its projection onto
  // the path had advanced past the blockage, so the sweep read clear, while lane driving
  // still saw the obstacle and stopped again.  Recovery then re-triggered, and the state
  // flipped between Recovery and LaneDriving indefinitely with the obstacle never passed.
  // A STOP_BEHIND episode needs one extra thing before handing back.  Its corridor is
  // clear from the very first tick -- that is the definition of the case: nothing is in
  // the way, the robot is simply frozen inside a stop margin whose stop line lies behind
  // it.  So the clear-corridor test is satisfied instantly, and handing back there
  // returns the robot to lane driving still inside that margin, still frozen, to be
  // detected all over again.  Require it to have actually driven out first.
  if (
    episode_stop_source_ == StuckDiagnosis::SOURCE_STOP_BEHIND &&
    progress_since_entry_m_ <= param_.unstuck_progress_m) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[episode %u] corridor clear, but this episode began inside a stop margin and the "
      "robot has only moved %.2f m of the %.2f m needed to be out of it",
      episode_id_, progress_since_entry_m_, param_.unstuck_progress_m);
    return;
  }

  const double offset = lateral_offset_from_path();
  if (offset > param_.rejoin_lateral_tolerance_m) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[episode %u] corridor clear but still %.2f m off the path (need <= %.2f m before "
      "lane driving can take it back)",
      episode_id_, offset, param_.rejoin_lateral_tolerance_m);
    return;
  }

  cooldown_until_ = now + rclcpp::Duration::from_seconds(param_.cooldown_sec);
  transition(State::TRANSIT, "corridor ahead is clear; lining up before handing back");
}

std::vector<geometry_msgs::msg::Pose> StuckRecoverySupervisorNode::collect_goal_candidates() const
{
  std::vector<geometry_msgs::msg::Pose> candidates;
  if (reference_path_.size() < 2) {
    return candidates;
  }
  const bool mirror = path_yaw_opposes_ego();

  size_t ego_index = 0;
  if (odom_) {
    if (const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose)) {
      ego_index = *nearest;
    }
  }

  // Start the search just PAST the blockage, not at a fixed distance from the robot: a
  // goal on the near side of the obstacle gives freespace no reason to go around it.
  // The sweep already measured how far ahead the blockage is.
  double search_from = param_.min_long_distance_to_check;
  if (last_sweep_.valid && last_sweep_.has_blockage) {
    search_from = std::max(
      search_from, last_sweep_.first_blocked_arc_m + param_.goal_offset_beyond_obstacle_m);
  }

  // First fit along the path, rather than a fixed offset from the nearest obstacle.
  //
  // Testing only the goal's own footprint accepted a pose that was free but wedged
  // against the obstacle it had just cleared -- the goal ended up a few centimetres from
  // something solid behind it, which is no place to ask the robot to come to rest.  So
  // walk the path continuously, track how long it has been free without interruption,
  // and take the FIRST point that already has goal_min_clear_length_m of clear path
  // behind it.  Later candidates are collected the same way as fallbacks.
  // Upper bound on the search.  The static parameter is only a ceiling: the real limit
  // is how far the mission goal itself is, measured ALONG the path rather than straight
  // line, so the escape goal can never be placed past the place the robot is trying to
  // reach.  On a short remaining route that shrinks on its own as the robot advances.
  double search_limit = param_.max_goal_search_distance_m;
  if (route_) {
    if (const auto goal_idx = find_nearest_index(reference_path_, route_->goal_pose)) {
      if (*goal_idx > ego_index) {
        double to_goal = 0.0;
        for (size_t k = ego_index + 1; k <= *goal_idx; ++k) {
          to_goal += distance2d(reference_path_[k - 1], reference_path_[k]);
        }
        search_limit = std::fmin(search_limit, to_goal);
      } else {
        search_limit = 0.0;  // the mission goal is already behind us
      }
    }
  }

  double arc = 0.0;
  double next_check = search_from;
  double clear_run_m = 0.0;   // continuous free path ending at the current station
  size_t skipped_offgrid = 0;
  size_t rejected_tight = 0;
  bool first_fit_logged = false;

  for (size_t k = ego_index + 1; k < reference_path_.size(); ++k) {
    const double step = distance2d(reference_path_[k - 1], reference_path_[k]);
    arc += step;
    if (arc > search_limit) {
      break;
    }

    auto candidate = reference_path_[k];
    if (mirror) {
      const double yaw = tf2::getYaw(candidate.orientation) + M_PI;
      candidate.orientation.x = 0.0;
      candidate.orientation.y = 0.0;
      candidate.orientation.z = std::sin(yaw * 0.5);
      candidate.orientation.w = std::cos(yaw * 0.5);
    }

    // The run is measured at every path point, not only at the ones we would consider
    // placing a goal on -- otherwise an obstacle between two checks goes unnoticed.
    if (footprint_is_free(candidate)) {
      clear_run_m += step;
    } else {
      clear_run_m = 0.0;
      if (!pose_is_on_grid(candidate)) {
        ++skipped_offgrid;
      }
      continue;
    }

    if (arc < next_check) {
      continue;
    }
    if (clear_run_m < param_.goal_min_clear_length_m) {
      ++rejected_tight;  // free here, but not yet far enough from what is behind it
      continue;
    }
    next_check = arc + param_.goal_check_interval_m;
    if (!first_fit_logged) {
      RCLCPP_INFO(
        get_logger(), "[episode %u] first fit at %.1f m along the path (%.1f m clear behind it)",
        episode_id_, arc, clear_run_m);
      first_fit_logged = true;
    }
    candidates.push_back(candidate);
  }

  RCLCPP_INFO(
    get_logger(),
    "[episode %u] %zu escape-goal candidates from %.1f m (blockage at %.1f m + %.1f m offset), "
    "each with >= %.1f m of clear path behind it, searched out to %.1f m; "
    "%zu rejected as too tight, %zu skipped as off-grid%s",
    episode_id_, candidates.size(), search_from,
    last_sweep_.has_blockage ? last_sweep_.first_blocked_arc_m : -1.0,
    param_.goal_offset_beyond_obstacle_m, param_.goal_min_clear_length_m, search_limit,
    rejected_tight, skipped_offgrid,
    mirror ? ", headings flipped for reversed route" : "");
  return candidates;
}

bool StuckRecoverySupervisorNode::publish_current_goal()
{
  if (goal_index_ >= goal_candidates_.size()) {
    return false;
  }
  const auto & candidate = goal_candidates_[goal_index_];

  // Last gate before the goal leaves for freespace: never hand it one the robot has
  // already driven past.  Aiming at a point behind made freespace turn the robot around
  // and force its way back to it, into a wall.  Checked along the PATH, not against the
  // vehicle heading -- on a reversed route the goal is legitimately behind the nose.
  if (!goal_is_ahead(candidate)) {
    RCLCPP_WARN(
      get_logger(), "[episode %u] refusing to publish goal candidate %zu: it is behind the robot",
      episode_id_, goal_index_ + 1);
    return false;
  }

  // Only publish when the goal actually changed.  freespace_planner calls reset() on
  // every route message (onRoute -> reset()), so re-sending the same goal throws away
  // the plan it had just computed -- which looked exactly like "freespace never
  // produces a plan".
  if (escape_goal_ && distance2d(*escape_goal_, candidate) < 0.05) {
    return true;
  }

  escape_goal_ = candidate;
  publish_recovery_route(*escape_goal_);
  return true;
}

bool StuckRecoverySupervisorNode::freespace_plan_is_valid()
{
  if (!freespace_trajectory_ || freespace_trajectory_->points.size() < 2) {
    return false;
  }

  // Must belong to THIS episode.  The route topic is latched, so freespace can answer a
  // previous episode's goal as soon as it is re-armed; relaying that drove the robot to
  // the wrong place entirely.
  if (freespace_traj_at_ <= route_published_at_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[episode %u] ignoring a freespace plan that predates this episode's goal", episode_id_);
    return false;
  }
  if ((this->now() - freespace_traj_at_).seconds() > param_.freespace_plan_timeout_sec) {
    return false;  // freespace has gone quiet; do not act on a stale plan
  }
  // create_stop_trajectory zeroes every velocity; a real plan moves the robot.
  for (const auto & point : freespace_trajectory_->points) {
    if (std::abs(point.longitudinal_velocity_mps) > 1e-3) {
      return true;
    }
  }
  return false;
}

void StuckRecoverySupervisorNode::relay_recovery_trajectory()
{
  // Single publisher on the topic scenario_selector reads.  Freespace publishes to its
  // own topic and we forward it verbatim -- verbatim matters, because freespace already
  // splits its plan at every direction cusp and waits for the robot to stop there,
  // which is the only reason the MPC can follow a Reeds-Shepp path at all.
  autoware_planning_msgs::msg::Trajectory traj;
  if (state_ == State::TRANSIT) {
    traj = build_transit_trajectory();
  } else if (state_ == State::RECOVERY && freespace_trajectory_) {
    traj = *freespace_trajectory_;
  } else {
    return;
  }
  if (traj.points.empty()) {
    return;
  }

  // Two refusals before anything is driven.  A plan that goes the wrong way is rejected
  // WHOLE -- there is no safe prefix of driving in the wrong direction.
  //
  // [TRANSIT-TRUNCATE 2026-09-15] Containment is different, and in TRANSIT it has to be.
  // Rejecting the whole plan publishes it stopped, and a stopped robot cannot change its
  // heading -- but converging the heading is the ONLY way out of TRANSIT.  So a
  // containment refusal anywhere along the plan deadlocked the state until its 30 s
  // timeout, every time.  Measured 2026-09-15, episode 8: 96 consecutive refusals of
  // "plan leaves the drivable area 4.29 m along it" while the exit test sat at "heading
  // 25.4 deg (need 14.9), 0.45 m off the path (need 0.50)" -- lateral already inside
  // tolerance, heading frozen because nothing was moving.
  //
  // Wholesale rejection is right for a freespace escape: that is a committed manoeuvre,
  // and half of it is not a manoeuvre.  TRANSIT is not committed to anything -- it is
  // creeping along the reference path at 1 km/h until it lines up.  Driving the part that
  // is verified free and stopping before the part that is not is exactly what should
  // happen, and every point kept has already passed the same footprint test the refusal
  // used.
  size_t first_bad = 0;
  const bool direction_ok = plan_follows_mission_direction(traj);
  const bool road_ok = direction_ok && plan_stays_inside_road(traj, &first_bad);
  if (direction_ok && !road_ok && state_ == State::TRANSIT && first_bad >= 2) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[episode %u] lining up on the first %zu of %zu plan points and stopping short: %s",
      episode_id_, first_bad, traj.points.size(), aeb_reason_.c_str());
    traj.points.resize(first_bad);
    traj.points.back().longitudinal_velocity_mps = 0.0f;
    record_footprint_debug(traj);
    clamp_recovery_speed(traj);
    pub_recovery_traj_->publish(traj);
    return;
  }
  if (!direction_ok || !road_ok) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "[episode %u] refusing the freespace plan: %s",
      episode_id_, aeb_reason_.c_str());
    // Publish it stopped rather than publishing nothing.  This scenario owns the
    // trajectory while it is armed, and going silent leaves the controller holding
    // whatever it had last -- a refusal has to read as "stop", not as "no opinion".
    for (auto & point : traj.points) {
      point.longitudinal_velocity_mps = 0.0f;
    }
    pub_recovery_traj_->publish(traj);
    // Cycling escape goals is meaningless in TRANSIT -- the manoeuvre is over and what is
    // being driven is the reference path, not a freespace plan.  Doing it anyway walked
    // TRANSIT out through "no usable escape goal left".  Hold instead, and let the TRANSIT
    // timeout be the thing that gives up.
    if (state_ != State::TRANSIT) {
      reject_plan_and_try_next_goal(aeb_reason_);
    }
    return;
  }

  record_footprint_debug(traj);

  clamp_recovery_speed(traj);

  // Emergency brake.  While this scenario drives, the motion_velocity_planner stop
  // modules are bypassed entirely, so nothing else in planning is watching.
  if (recovery_path_is_blocked(traj)) {
    // Zero the velocities but keep the geometry, so the controller still has a reference
    // to hold position against rather than losing its trajectory entirely.
    for (auto & point : traj.points) {
      point.longitudinal_velocity_mps = 0.0f;
    }
    if (!aeb_holding_) {
      aeb_holding_ = true;
      aeb_hold_since_ = this->now();
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[episode %u] recovery AEB in %s: %s -- holding for %.1f s",
      episode_id_, state_name(state_).c_str(), aeb_reason_.c_str(),
      (this->now() - aeb_hold_since_).seconds());
  } else {
    aeb_holding_ = false;
  }

  pub_recovery_traj_->publish(traj);
}

void StuckRecoverySupervisorNode::clamp_recovery_speed(
  autoware_planning_msgs::msg::Trajectory & traj) const
{
  // Magnitude only: the sign carries the direction of travel and must survive, or the
  // MPC would read a reverse segment as a forward one.
  const auto limit = static_cast<float>(param_.recovery_speed_limit_mps);
  for (auto & point : traj.points) {
    point.longitudinal_velocity_mps =
      std::copysign(std::min(std::abs(point.longitudinal_velocity_mps), limit),
                    point.longitudinal_velocity_mps);
  }
}

bool StuckRecoverySupervisorNode::plan_is_spent(double * stop_dist_out) const
{
  // The velocity smoother will not start the robot moving if the nearest stop point is
  // closer than its stop_dist_to_prohibit_engage.  When that happens the trajectory
  // looks perfectly healthy here and arrives at the controller as all zeros, which is
  // extremely confusing to debug -- so say it out loud.
  if (!odom_ || !is_stopped_ || !freespace_trajectory_) {
    return false;
  }
  const auto & traj = *freespace_trajectory_;
  if (traj.points.empty()) {
    return false;
  }
  const auto nearest_it = std::min_element(
    traj.points.begin(), traj.points.end(), [this](const auto & a, const auto & b) {
      return distance2d(a.pose, odom_->pose.pose) < distance2d(b.pose, odom_->pose.pose);
    });

  double stop_dist = 0.0;
  bool found_stop = false;
  for (auto it = nearest_it; it != traj.points.end(); ++it) {
    if (it != nearest_it) {
      stop_dist += distance2d(std::prev(it)->pose, it->pose);
    }
    if (std::abs(it->longitudinal_velocity_mps) < 1e-3) {
      found_stop = true;
      break;
    }
  }
  if (!found_stop) {
    return false;  // nothing but drivable velocity ahead; the plan is alive
  }

  // Less than this ahead of the robot and the controller will not engage, so the plan
  // cannot restart the robot no matter how long we wait.
  if (stop_dist_out != nullptr) {
    *stop_dist_out = stop_dist;
  }
  return stop_dist < param_.min_engageable_stop_distance_m;
}

bool StuckRecoverySupervisorNode::plan_is_spent_confirmed()
{
  const auto now = this->now();
  double stop_dist = std::numeric_limits<double>::quiet_NaN();
  const bool spent = plan_is_spent(&stop_dist);
  // Say the number either way, whenever the robot is standing still.  Whether the plan's
  // next stop point falls inside the controller's engage distance is the whole question,
  // and having to infer it from the ABSENCE of a replan warning is how a 60 s episode was
  // lost with no idea why.
  if (is_stopped_ && std::isfinite(stop_dist)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[episode %u] robot stopped with the plan's next stop point %.2f m ahead (engage "
      "needs %.2f m) -> %s",
      episode_id_, stop_dist, param_.min_engageable_stop_distance_m,
      spent ? "plan is spent, will replan" : "plan still drivable");
  }
  if (!spent) {
    spent_since_ = now;
    return false;
  }
  // Give the robot a fair chance to drive the segment before concluding it is stuck on
  // it, and require the symptom to persist -- a single short partial from freespace is
  // not evidence of anything.
  if ((now - state_since_).seconds() < param_.spent_hold_sec) {
    return false;
  }
  return (now - spent_since_).seconds() >= param_.spent_hold_sec;
}

void StuckRecoverySupervisorNode::force_freespace_replan(const std::string & why)
{
  if (!escape_goal_) {
    transition(State::BLOCKED, "no goal to replan towards: " + why);
    return;
  }

  // Never replan towards a goal the robot has already driven past.  Doing so made
  // freespace turn the robot around and force its way back into a wall.  Step forward to
  // the first candidate still ahead; if the whole list is behind us, the manoeuvre is
  // over by definition -- there is nothing left to escape towards.
  if (!goal_is_ahead(*escape_goal_)) {
    if (!advance_past_passed_goals()) {
      if (!rejoined_reference_path()) {
        transition(State::ABORT, "every escape goal is behind the robot and it is off the path");
        return;
      }
      RCLCPP_INFO(
        get_logger(), "[episode %u] every escape goal is behind the robot -- manoeuvre done (%s)",
        episode_id_, why.c_str());
      transition(State::TRANSIT, "drove past the last escape goal; lining up");
      return;
    }
    if (!publish_current_goal()) {
      if (rejoined_reference_path()) {
        transition(State::TRANSIT, "no escape goal left ahead; lining up");
      } else {
        transition(State::ABORT, "no escape goal left and the robot is off the path");
      }
      return;
    }
  }

  ++replan_attempts_;
  RCLCPP_WARN(
    get_logger(), "[episode %u] forcing freespace replan (attempt %d/%d) -- %s", episode_id_,
    replan_attempts_, param_.max_replan_attempts, why.c_str());
  // Re-publishing the route is what makes freespace reset() and plan again from the
  // robot's actual pose.  It will not do so on its own here: its own replan triggers
  // (empty trajectory / obstacle on path / course out) are all false.
  publish_recovery_route(*escape_goal_);
  transition(State::REPLAN, why);
}

void StuckRecoverySupervisorNode::step_replan()
{
  const auto now = this->now();

  if (!is_operational() || emergency_code_ != 0) {
    transition(State::ABORT, "left autonomous mode or emergency while replanning");
    return;
  }
  if ((now - episode_since_).seconds() > param_.max_episode_duration_sec) {
    transition(State::ABORT, "episode watchdog expired while replanning");
    return;
  }

  if (freespace_plan_is_valid() && !plan_is_spent()) {
    last_valid_plan_ = now;
    spent_since_ = now;
    replan_attempts_ = 0;
    RCLCPP_INFO(get_logger(), "[episode %u] replan produced a usable plan", episode_id_);
    transition(State::RECOVERY, "replan succeeded");
    return;
  }

  if ((now - state_since_).seconds() < param_.replan_wait_sec) {
    return;  // freespace needs a moment: it only plans once the vehicle has stopped
  }

  if (replan_attempts_ < param_.max_replan_attempts) {
    force_freespace_replan("previous replan did not produce a usable plan");
    return;
  }

  replan_attempts_ = 0;
  reject_plan_and_try_next_goal("replanning from this pose keeps failing");
}

std::optional<double> StuckRecoverySupervisorNode::heading_error_to_reference() const
{
  if (!odom_ || reference_path_.size() < 2) {
    return std::nullopt;
  }
  const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose);
  if (!nearest) {
    return std::nullopt;
  }
  // Against the DIRECTION OF TRAVEL, not the raw path yaw: in reverse motion the robot
  // faces away from the way it is going, and comparing against the path yaw there would
  // read a perfectly aligned robot as 180 degrees out.
  const double ref_yaw = tf2::getYaw(reference_path_[*nearest].orientation) +
                         (path_yaw_opposes_ego() ? M_PI : 0.0);
  return autoware_utils::normalize_radian(tf2::getYaw(odom_->pose.pose.orientation) - ref_yaw);
}

autoware_planning_msgs::msg::Trajectory StuckRecoverySupervisorNode::build_transit_trajectory()
  const
{
  autoware_planning_msgs::msg::Trajectory traj;
  traj.header.frame_id = "map";
  traj.header.stamp = this->now();
  if (!odom_ || reference_path_.size() < 2) {
    return traj;
  }
  const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose);
  if (!nearest) {
    return traj;
  }

  // The reference path itself, from the robot forward -- the path lane driving is about to
  // resume on.  Driving it here, at the recovery crawl, is what turns the handover from a
  // step change into something the robot can follow.
  const double sign = mission_travel_sign();
  const bool mirror = path_yaw_opposes_ego();

  const auto set_yaw = [](geometry_msgs::msg::Pose & pose, const double yaw) {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw * 0.5);
    pose.orientation.w = std::cos(yaw * 0.5);
  };

  // [TRANSIT-JOIN 2026-09-15] The plan used to start at reference_path_[nearest] -- a point
  // half a metre or more to the SIDE of the robot.  That made the robot permanently off its
  // own plan, which deadlocked the state against its own safety check:
  //
  //   recovery AEB: robot is 0.67 m off the plan it is following (limit 0.60 m)
  //   lining up before handing back: heading -5.1 deg (need 14.9), 0.67 m off the path
  //                                  (need 0.50)
  //   TRANSIT -> ABORT (could not line up with the path in time)
  //
  // The AEB brakes above 0.60 m; TRANSIT may only exit below 0.50 m; and the only way to
  // shrink the offset is to drive.  Braking therefore froze the very number the exit test
  // reads, so every TRANSIT entered above 0.60 m was guaranteed to run its 30 s timeout and
  // abort.  Observed 2026-09-15 when the obstacle was deleted mid-manoeuvre: the corridor
  // went clear three seconds into RECOVERY, so TRANSIT began while the robot was still out
  // beside the path.
  //
  // Starting the plan at the robot fixes the deadlock at its source rather than by loosening
  // a threshold, and it closes a real gap at the same time: the ground between the robot and
  // the path is the ground the robot is about to cross, and until now it was in no plan at
  // all, so neither plan_stays_inside_road() nor the blocked-footprint sweep ever looked at
  // it.
  const double offset = distance2d(reference_path_[*nearest], odom_->pose.pose);
  const double join_length =
    std::max(param_.transit_join_min_m, param_.transit_join_gain * offset);

  // Walk far enough along the reference path that the join has room to be gentle.  A join
  // onto the nearest point would be a sideways step; onto a point join_length ahead it is a
  // shallow approach the controller can actually track at the recovery crawl.
  size_t join_idx = *nearest;
  double join_arc = 0.0;
  while (join_idx + 1 < reference_path_.size() && join_arc < join_length) {
    join_arc += distance2d(reference_path_[join_idx], reference_path_[join_idx + 1]);
    ++join_idx;
  }

  const auto reference_pose_at = [&](const size_t i) {
    geometry_msgs::msg::Pose pose = reference_path_[i];
    if (mirror) {
      // Hand the controller the vehicle's own heading, as the escape goals are given.
      set_yaw(pose, tf2::getYaw(pose.orientation) + M_PI);
    }
    return pose;
  };

  const auto push = [&](const geometry_msgs::msg::Pose & pose) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = pose;
    point.longitudinal_velocity_mps =
      static_cast<float>(sign * param_.recovery_speed_limit_mps);
    traj.points.push_back(point);
  };

  // The join, sampled at the reference path's own spacing so the whole plan is uniform.
  // Skipped when the robot is already on the path: interpolating over a few centimetres
  // would only add points the controller has to chase.
  const auto join_pose = reference_pose_at(join_idx);
  const double join_span = distance2d(odom_->pose.pose, join_pose);
  const bool needs_join = offset > param_.probe_step_m && join_span > 1e-3;
  if (needs_join) {
    // Heading along the join is the direction the robot actually travels, which is the
    // bearing to the join point -- reversed when the mission is driving backwards, exactly
    // as the mirrored reference headings above are.
    const double bearing = std::atan2(
      join_pose.position.y - odom_->pose.pose.position.y,
      join_pose.position.x - odom_->pose.pose.position.x);
    const double join_yaw = (sign < 0.0) ? bearing + M_PI : bearing;
    const auto steps = static_cast<size_t>(std::ceil(join_span / param_.probe_step_m));
    for (size_t k = 0; k < steps; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(steps);
      geometry_msgs::msg::Pose pose;
      pose.position.x = odom_->pose.pose.position.x +
                        t * (join_pose.position.x - odom_->pose.pose.position.x);
      pose.position.y = odom_->pose.pose.position.y +
                        t * (join_pose.position.y - odom_->pose.pose.position.y);
      pose.position.z = odom_->pose.pose.position.z;
      set_yaw(pose, join_yaw);
      push(pose);
    }
  }

  // Without a join the plan must still begin AT the robot, not join_length ahead of it:
  // the AEB measures the distance to the nearest point of the plan being followed, so a
  // plan that starts 1.5 m ahead reads as 1.5 m of deviation and brakes for a robot that is
  // sitting exactly on its path.
  const size_t start_idx = needs_join ? join_idx : *nearest;

  double arc = 0.0;
  for (size_t i = start_idx; i < reference_path_.size(); ++i) {
    if (i > start_idx) {
      arc += distance2d(reference_path_[i - 1], reference_path_[i]);
    }
    if (arc > param_.transit_lookahead_m) {
      break;
    }
    push(reference_pose_at(i));
  }
  if (!traj.points.empty()) {
    traj.points.back().longitudinal_velocity_mps = 0.0f;
  }
  return traj;
}

void StuckRecoverySupervisorNode::step_transit()
{
  const auto now = this->now();

  if (!is_operational() || emergency_code_ != 0) {
    transition(State::ABORT, "left autonomous mode or emergency while lining up");
    return;
  }
  if ((now - state_since_).seconds() > param_.transit_timeout_sec) {
    transition(State::ABORT, "could not line up with the path in time");
    return;
  }
  if ((now - episode_since_).seconds() > param_.max_episode_duration_sec) {
    transition(State::ABORT, "episode watchdog expired while lining up");
    return;
  }

  const auto yaw_error = heading_error_to_reference();
  if (!yaw_error) {
    transition(State::ABORT, "no reference path to line up with");
    return;
  }

  const double offset = lateral_offset_from_path();
  const bool aligned = std::abs(*yaw_error) <= param_.transit_yaw_tolerance_rad &&
                       offset <= param_.rejoin_lateral_tolerance_m;
  if (!aligned) {
    transit_aligned_since_ = now;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[episode %u] lining up before handing back: heading %.1f deg (need %.1f), %.2f m off the "
      "path (need %.2f)",
      episode_id_, *yaw_error * 180.0 / M_PI, param_.transit_yaw_tolerance_rad * 180.0 / M_PI,
      offset, param_.rejoin_lateral_tolerance_m);
    return;
  }

  // Held, not instantaneous: a robot swinging through alignment would otherwise hand back
  // mid-swing, which is the failure this state exists to prevent.
  if ((now - transit_aligned_since_).seconds() < param_.transit_hold_sec) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "[episode %u] lined up: heading %.1f deg, %.2f m off the path -- handing back to lane driving",
    episode_id_, *yaw_error * 180.0 / M_PI, offset);
  cooldown_until_ = now + rclcpp::Duration::from_seconds(param_.cooldown_sec);
  transition(State::COOLDOWN, "lined up with the path");
}

void StuckRecoverySupervisorNode::step_blocked()
{
  if (!is_operational() || legit_stop_) {
    transition(State::NOMINAL, "no longer applicable: " + legit_reason_);
    return;
  }

  size_t from = 0;
  if (odom_ && !reference_path_.empty()) {
    if (const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose)) {
      from = *nearest;
    }
  }
  last_sweep_ = sweep(from);
  corridor_clear_ = last_sweep_.valid && last_sweep_.clear;

  // Leaving BLOCKED must mean the robot is no longer STUCK -- not merely that the
  // corridor reads clear.  Those are different questions, and conflating them span the
  // watchdog: PROBE enters BLOCKED precisely BECAUSE the corridor reads clear (whatever
  // holds the robot is not in the costmap), so exiting on "corridor clear" dropped it
  // straight back into SUSPECT, one lap every few seconds, forever.  Ask the stop
  // source and the odometer instead.
  const bool still_held = stop_source_ == StuckDiagnosis::SOURCE_OBSTACLE_STOP ||
                          stop_source_ == StuckDiagnosis::SOURCE_STOP_BEHIND ||
                          stop_source_ == StuckDiagnosis::SOURCE_MAP_STOP;
  if (!still_held || progress_since_entry_m_ > param_.unstuck_progress_m) {
    if ((this->now() - clear_since_).seconds() >= param_.clear_hold_sec) {
      transition(State::NOMINAL, "no longer held: the robot is free to drive again");
    }
    return;
  }
  clear_since_ = this->now();

  // BLOCKED is not a dead end.  Perception and the costmap keep updating, so a gap
  // that was not there a minute ago may be there now -- re-measure periodically
  // instead of waiting for the path to become completely clear.  The retry interval
  // (cooldown_sec) is long enough that this cannot oscillate with PROBE.
  if (
    attempts_ < param_.max_attempts &&
    (this->now() - state_since_).seconds() >= param_.cooldown_sec) {
    capture_reference_path();
    transition(State::PROBE, "re-measuring the corridor after being blocked");
  }
}

void StuckRecoverySupervisorNode::step_cooldown()
{
  if (this->now() >= cooldown_until_) {
    transition(State::NOMINAL, "cooldown finished");
  }
}

void StuckRecoverySupervisorNode::step_abort()
{
  // A mechanical stall stays latched on the strength of the evidence that caused it --
  // but observed motion is stronger evidence that it is over.  Releasing only on measured
  // movement keeps the "needs a human" property for a robot that genuinely cannot move,
  // without leaving the watchdog dead for the rest of the session once it demonstrably
  // can.  (Seen in practice: the gear was left in REVERSE after a manoeuvre, forward
  // commands did nothing, MECHANICAL latched, and the feature stayed dead.)
  if (abort_was_mechanical_) {
    if (is_stopped_) {
      abort_clear_since_ = this->now();
      return;
    }
    if ((this->now() - abort_clear_since_).seconds() < param_.abort_release_sec) {
      return;
    }
    cooldown_until_ = this->now() + rclcpp::Duration::from_seconds(param_.cooldown_sec);
    RCLCPP_INFO(
      get_logger(), "[episode %u] mechanical abort released: the robot is moving again",
      episode_id_);
    transition(State::COOLDOWN, "mechanical abort released by observed motion");
    return;
  }

  // [ABORT-SHADOW 2026-09-15] Before anything else: keep asking whether there is a way
  // out now.  The costmap and freespace are still running (see arm_costmap in on_timer),
  // so the question can be answered continuously rather than waited on.
  //
  // Two halves, both cheap:
  //   - the plan freespace last produced is re-checked every tick against the live
  //     costmap.  The usual reason an episode aborted is that this plan was obstructed;
  //     once whatever obstructed it is gone, the plan is good again as it stands.
  //   - every abort_shadow_replan_sec a fresh plan is requested from where the robot now
  //     is, in case the old one can never work.
  //
  // If a plan passes the same three gates the relay applies -- goes the way the mission
  // goes, stays on the road, is not blocked -- drive it.  Straight to RECOVERY, not back
  // through COOLDOWN and PROBE: the manoeuvre is already worked out, and starting the
  // state machine over would only re-derive it.  If nothing passes, the robot stays where
  // it is, which is the correct answer when there really is no feasible way out.
  if (mode_ >= Mode::PROBE_ONLY) {
    const auto now = this->now();
    if ((now - shadow_replan_at_).seconds() >= param_.abort_shadow_replan_sec) {
      shadow_replan_at_ = now;
      // Re-collect from the CURRENT pose.  The goals that were exhausted were placed
      // relative to where the robot was when the episode began, and it has moved since.
      goal_candidates_ = collect_goal_candidates();
      goal_index_ = 0;
      if (!goal_candidates_.empty()) {
        publish_current_goal();
      }
    }

    if (freespace_plan_is_valid() && freespace_trajectory_) {
      auto candidate = *freespace_trajectory_;
      clamp_recovery_speed(candidate);
      const bool usable = plan_follows_mission_direction(candidate) &&
                          plan_stays_inside_road(candidate) &&
                          !recovery_path_is_blocked(candidate);
      if (usable) {
        RCLCPP_INFO(
          get_logger(),
          "[episode %u] a clear freespace plan appeared while aborted -- driving it "
          "instead of waiting. Previous abort: %s",
          episode_id_, abort_reason_.c_str());
        abort_reason_.clear();
        transition(State::RECOVERY, "a clear freespace plan appeared while aborted");
        return;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[episode %u] aborted and holding: the freespace plan on offer is still not "
        "usable (%s)",
        episode_id_, aeb_reason_.empty() ? "wrong direction or off the road"
                                         : aeb_reason_.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[episode %u] aborted and holding: freespace has no usable plan from here yet",
        episode_id_);
    }
  }

  // Otherwise ABORT releases itself once the thing that caused it is gone.  Latching it
  // permanently meant that after one failed episode the watchdog was dead for the rest
  // of the session, even with the obstacle long since removed.
  const bool still_blocked = stop_source_ == StuckDiagnosis::SOURCE_OBSTACLE_STOP ||
                             stop_source_ == StuckDiagnosis::SOURCE_STOP_BEHIND;
  const bool moving = !is_stopped_;
  if (still_blocked && !moving) {
    abort_clear_since_ = this->now();

    // [ABORT-RETRY 2026-09-15] Both of this state's normal exits are unreachable here,
    // and that is not an edge case -- it is the main case.  A robot frozen by an
    // obstacle_stop whose stop line is already behind it cannot move (insert_stop zeroes
    // the trajectory from that point on), so "moving" never comes true; and the thing
    // freezing it is an obstacle, so "still_blocked" never goes false either.  Lane
    // driving has no way out of this on its own, which is exactly the case recovery
    // exists for -- yet ABORT sat on it indefinitely.  Observed 2026-09-15: 170 s in
    // ABORT with an obstacle 2.45 m ahead along the direction of travel and 1.88 m of
    // free corridor either side, i.e. a manoeuvre PROBE would have accepted.
    //
    // So re-arm the watchdog instead of waiting.  Going out through COOLDOWN and NOMINAL
    // starts a genuinely fresh episode: a new PROBE against the current costmap, new
    // escape goals, new plans.  Nothing from the failed episode is carried over, because
    // transition(NOMINAL) already clears it.
    const double held = (this->now() - state_since_).seconds();
    if (held < param_.abort_retry_sec) {
      return;
    }
    if (abort_retries_ >= param_.max_abort_retries) {
      // Budget spent without the robot moving once.  Latch, and say so: at this point
      // the situation really is not one more attempt away.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "[episode %u] held in ABORT for %.0f s after %d retries without the robot moving "
        "at all -- not retrying again (%s)",
        episode_id_, held, abort_retries_, abort_reason_.c_str());
      return;
    }
    ++abort_retries_;
    cooldown_until_ = this->now() + rclcpp::Duration::from_seconds(param_.cooldown_sec);
    RCLCPP_WARN(
      get_logger(),
      "[episode %u] still blocked and still stopped %.0f s after aborting; re-arming the "
      "watchdog to try recovery again (retry %d/%d). Previous abort: %s",
      episode_id_, held, abort_retries_, param_.max_abort_retries, abort_reason_.c_str());
    transition(State::COOLDOWN, "re-arming after a blocked abort");
    return;
  }
  if ((this->now() - abort_clear_since_).seconds() < param_.abort_release_sec) {
    return;
  }

  // Come back through COOLDOWN rather than straight to NOMINAL, so a blockage that is
  // only intermittently detected cannot immediately re-trigger a whole episode.
  cooldown_until_ = this->now() + rclcpp::Duration::from_seconds(param_.cooldown_sec);
  RCLCPP_INFO(
    get_logger(), "[episode %u] abort released (%s); watchdog live again", episode_id_,
    moving ? "robot is moving again" : "blockage is gone");
  transition(State::COOLDOWN, "abort released");
}

// ---------------------------------------------------------------------------
// Recovery helpers
// ---------------------------------------------------------------------------
bool StuckRecoverySupervisorNode::capture_reference_path()
{
  reference_path_.clear();

  // Prefer the map.  The trajectory lane driving last published already has the avoidance
  // shift baked into it -- the very shift the robot was driving when it got stuck -- so
  // measuring the corridor and placing escape goals against it means reasoning about a
  // path that is itself part of the problem.  The centerline is recomputed from the map
  // each time, and carries the prefer_lateral_ratio bias that defines where in the lane
  // this robot is supposed to be.
  if (param_.reference_path_source == "centerline" && capture_reference_path_from_centerline()) {
    return true;
  }
  if (param_.reference_path_source == "centerline") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no centerline available yet (map %s, route %s); falling back to the last lane-driving "
      "trajectory for the reference path",
      map_ready_ ? "ready" : "missing", route_handler_ready_ ? "ready" : "missing");
  }
  return capture_reference_path_from_trajectory();
}

bool StuckRecoverySupervisorNode::capture_reference_path_from_centerline()
{
  if (!odom_ || !map_ready_ || !route_handler_ready_) {
    return false;
  }

  lanelet::ConstLanelet current_lane;
  if (!route_handler_.getClosestLaneletWithinRoute(odom_->pose.pose, &current_lane)) {
    return false;
  }

  // Reach far enough ahead for the goal search, and a little behind so the arc coordinate
  // of the robot is not pinned to the very first point.
  const double forward = param_.max_goal_search_distance_m + 10.0;
  const auto lanes =
    route_handler_.getLaneletSequence(current_lane, odom_->pose.pose, 5.0, forward);
  if (lanes.empty()) {
    return false;
  }

  const auto arc = lanelet::utils::getArcCoordinates(lanes, odom_->pose.pose);
  const auto raw = route_handler_.getCenterLinePath(
    lanes, std::max(0.0, arc.length), arc.length + param_.max_goal_search_distance_m);
  if (raw.points.size() < 2) {
    return false;
  }

  // Resample before use.  The map's centerline comes back coarse -- measured at 6 points
  // over 30 m, i.e. 5 m apart -- and everything downstream walks this path point by point:
  // the corridor sweep samples a station per point, and the goal search steps along it
  // looking for the first fit.  At 5 m spacing both would be nearly blind compared to the
  // trajectory this replaced, which arrived already dense.
  const auto path = autoware::motion_utils::resamplePath(raw, param_.probe_step_m);
  if (path.points.size() < 2) {
    return false;
  }

  reference_path_.reserve(path.points.size());
  for (const auto & point : path.points) {
    reference_path_.push_back(point.point.pose);
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "[episode %u] reference path rebuilt from the lanelet centerline: %zu points over %.1f m "
    "(resampled from %zu)",
    episode_id_, reference_path_.size(), param_.max_goal_search_distance_m, raw.points.size());
  return reference_path_.size() >= 2;
}

bool StuckRecoverySupervisorNode::capture_reference_path_from_trajectory()
{
  reference_path_.clear();
  if (!trajectory_ || trajectory_->points.empty() || !odom_) {
    return false;
  }

  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(trajectory_->points.size());
  for (const auto & point : trajectory_->points) {
    poses.push_back(point.pose);
  }

  const auto nearest = find_nearest_index(poses, odom_->pose.pose);
  if (!nearest) {
    return false;
  }

  // The path starts at the robot.  It used to keep a stretch behind ego as well, which
  // was only ever the geometry the retreat reversed along; nothing reverses any more.
  const size_t begin = *nearest;

  // Keep everything ahead of us, out to max_goal_search_distance_m: goal candidates
  // are searched along here all the way towards the real mission goal.  The corridor
  // sweep uses only the first corridor_scan_distance_m of it.  The velocities are
  // already zeroed by obstacle_stop -- we only want the geometry.
  double travelled = 0.0;
  for (size_t i = begin; i < poses.size(); ++i) {
    if (i > *nearest) {
      travelled += distance2d(poses[i - 1], poses[i]);
    }
    reference_path_.push_back(poses[i]);
    if (travelled >= param_.max_goal_search_distance_m) {
      break;
    }
  }
  return reference_path_.size() >= 2;
}

bool StuckRecoverySupervisorNode::footprint_is_free(const geometry_msgs::msg::Pose & pose) const
{
  // Goal placement stays on the stricter planning threshold: where the robot is asked
  // to come to REST, a cell that is only probably occupied is reason enough to move on.
  return footprint_is_free_with_margin(
    pose, param_.goal_shape_margin_m, param_.occupancy_threshold);
}

bool StuckRecoverySupervisorNode::footprint_is_free_with_margin(
  const geometry_msgs::msg::Pose & pose, double margin, int threshold) const
{
  if (!costmap_is_fresh() || grid_->info.resolution <= 0.0 || grid_->data.empty()) {
    return false;
  }
  const double res = grid_->info.resolution;
  const double yaw = tf2::getYaw(pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double half_width = vehicle_width_m_ * 0.5 + margin;
  const double front = base_to_front_m_ + margin;
  const double rear = base_to_rear_m_ + margin;
  const double step = std::max(res * 0.5, 0.02);

  for (double lon = -rear; lon <= front; lon += step) {
    for (double lat = -half_width; lat <= half_width; lat += step) {
      const double x = pose.position.x + cos_yaw * lon - sin_yaw * lat;
      const double y = pose.position.y + sin_yaw * lon + cos_yaw * lat;
      const int col = static_cast<int>(std::floor((x - grid_->info.origin.position.x) / res));
      const int row = static_cast<int>(std::floor((y - grid_->info.origin.position.y) / res));
      if (
        col < 0 || row < 0 || col >= static_cast<int>(grid_->info.width) ||
        row >= static_cast<int>(grid_->info.height)) {
        return false;  // off-grid: also keeps the goal inside the costmap window
      }
      const auto value =
        grid_->data[static_cast<size_t>(row) * grid_->info.width + static_cast<size_t>(col)];
      if (value < 0 || value >= threshold) {
        return false;
      }
    }
  }
  return true;
}

double StuckRecoverySupervisorNode::lateral_offset_from_path() const
{
  if (!odom_ || reference_path_.empty()) {
    return 0.0;
  }
  const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose);
  if (!nearest) {
    return 0.0;
  }
  return distance2d(reference_path_[*nearest], odom_->pose.pose);
}

bool StuckRecoverySupervisorNode::rejoined_reference_path() const
{
  // Handing back while the robot is still out beside the obstacle is how it ended up off
  // the road.  Only ONE of the hand-back paths used to check this; a manoeuvre that
  // finished by driving past its escape goal skipped it entirely, so recovery could
  // return a robot sitting two metres off the path to lane driving, which then had no
  // way to fix it -- the next episode found the robot's own footprint inside an obstacle.
  return lateral_offset_from_path() <= param_.rejoin_lateral_tolerance_m;
}

bool StuckRecoverySupervisorNode::goal_is_ahead(const geometry_msgs::msg::Pose & goal) const
{
  // Compare positions ALONG THE PATH, not against the vehicle heading: on a reversed
  // route the robot faces away from its direction of travel, so a perfectly good goal
  // sits behind its nose.  Arc order is correct in both cases.
  if (!odom_ || reference_path_.size() < 2) {
    return true;
  }

  // [GOAL-TOO-CLOSE 2026-09-16] A goal the controller cannot drive to is not a goal.
  //
  // The longitudinal controller will not leave STOPPED unless the plan's next stop point
  // is further than drive_state_stop_dist + drive_state_offset_stop_dist ahead
  // (min_engageable_stop_distance_m mirrors that).  So once the robot is inside that
  // distance of its escape goal, the last stretch cannot be driven -- no plan to it will
  // ever move the robot, however many times it is replanned.
  //
  // Measured 2026-09-16, episode 5: the robot drove 4.49 m of its plan, arrived near the
  // goal, and the remaining run was 1.00 m and then 0.69 m -- both under the 1.50 m engage
  // distance.  plan_is_spent() correctly called the plan undrivable, force_freespace_replan
  // asked for another one to a goal just as close, and RECOVERY <-> REPLAN churned until
  // the candidate list ran out (5/5) and the episode gave up into lane driving.
  //
  // Treating such a goal as reached rather than ahead is what the rest of the state machine
  // already knows how to handle: the goal-passed branch advances to the next candidate that
  // IS drivable, looks for one that rejoins the path, or hands back through TRANSIT.  The
  // same predicate also runs in collect_goal_candidates(), so candidates are never placed
  // that close in the first place.
  if (
    distance2d(odom_->pose.pose, goal) <= param_.min_engageable_stop_distance_m) {
    return false;
  }
  const auto ego_idx = find_nearest_index(reference_path_, odom_->pose.pose);
  const auto goal_idx = find_nearest_index(reference_path_, goal);
  if (!ego_idx || !goal_idx) {
    return true;
  }
  return *goal_idx > *ego_idx;
}

bool StuckRecoverySupervisorNode::advance_past_passed_goals()
{
  while (goal_index_ < goal_candidates_.size() && !goal_is_ahead(goal_candidates_[goal_index_])) {
    RCLCPP_INFO(
      get_logger(), "[episode %u] goal candidate %zu already passed; taking the next one",
      episode_id_, goal_index_ + 1);
    ++goal_index_;
  }
  return goal_index_ < goal_candidates_.size();
}

double StuckRecoverySupervisorNode::mission_travel_sign() const
{
  // The mission runs in one motion mode from start to goal: forward motion drives the
  // robot nose-first, reverse motion drives it backwards along the route the whole way.
  // path_yaw_opposes_ego() already answers which one is in force, and it is the same
  // answer the corridor sweep and the escape goals are built from -- so both modes go
  // through identical code from here on.
  return path_yaw_opposes_ego() ? -1.0 : 1.0;
}

bool StuckRecoverySupervisorNode::plan_follows_mission_direction(
  const autoware_planning_msgs::msg::Trajectory & traj)
{
  // Recovery may only carry the robot the way its mission is already going.
  //
  // Driving against it is the failure this feature kept producing: while the obstacle
  // closed in, the robot gave ground and gave ground again.  Deleting the retreat state
  // removed one way in; this closes the other, because freespace is free to answer with a
  // plan that sets off against the mission, and relaying that is the same behaviour by a
  // different route.  A plan that changes direction part way is refused for the same
  // reason -- half of it travels the wrong way.
  const double want = mission_travel_sign();
  int seen = 0;
  for (const auto & point : traj.points) {
    const double v = point.longitudinal_velocity_mps;
    if (std::abs(v) < 1e-3) {
      continue;  // the trailing stop point, and any pause, belong to neither direction
    }
    const int sign = v > 0.0 ? 1 : -1;
    if (seen != 0 && sign != seen) {
      aeb_reason_ = "plan changes direction part way through";
      return false;
    }
    seen = sign;
  }
  if (seen == 0) {
    return true;  // nothing but stop points; nothing to drive the wrong way
  }
  if (static_cast<double>(seen) * want < 0.0) {
    aeb_reason_ = std::string("plan travels against the mission, which is in ") +
                  (want > 0.0 ? "forward" : "reverse") + " motion";
    return false;
  }
  return true;
}

bool StuckRecoverySupervisorNode::plan_stays_inside_road(
  const autoware_planning_msgs::msg::Trajectory & traj, size_t * first_bad_index_out)
{
  // Every pose of the plan, footprint and all, has to sit on drivable ground.
  //
  // The recovery costmap paints everything outside the road lanelets as occupied --
  // fill_polygon_areas() lays down grid_max_value and carves the road polygons out of it
  // -- so this one test covers both "drives into something" and "leaves the road".  Cells
  // outside the grid window count as blocked too, which keeps the plan inside the part of
  // the world we can actually see.
  //
  // The whole plan is judged, not just the braking distance: a plan that leaves the road
  // five metres out is not a plan to drive carefully, it is the wrong plan.
  //
  // But the START is not the plan's fault.  Recovery is often called precisely because the
  // robot has ended up half off the road, so its own footprint reads blocked and so does
  // the first stretch of any plan that digs it out.  Judging those points refuses every
  // possible plan: on the robot, ten goal candidates in a row were rejected at the very
  // same coordinate -- the robot's own position, 0.00 m along each plan -- while the
  // corridor beside it was 3 m wide and clear.  So an opening run of blocked points is
  // tolerated, up to containment_grace_m; once the plan has reached clear ground, any
  // later violation is real and refuses the plan.
  if (!costmap_is_fresh()) {
    return true;  // the brake already refuses to drive on a stale grid; do not double-report
  }
  double arc = 0.0;
  bool reached_clear_ground = false;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (i > 0) {
      arc += distance2d(traj.points[i - 1].pose, traj.points[i].pose);
    }
    // Past the edge of the costmap there is nothing to judge with.  Off-window reads as
    // blocked in the footprint test -- which is right for placing a goal, since a goal you
    // cannot see is no good, but wrong here: it refused every plan longer than the 10 m
    // half-window while escape goals are placed out to 30 m.  Measured on the robot, a plan
    // was refused 13.3 m along it at a point 12.06 m from the vehicle, well outside the
    // 20 x 20 m grid.  Stop looking instead; the window travels with the robot, so the rest
    // of the plan gets judged as it comes into view.
    if (!footprint_within_grid(traj.points[i].pose)) {
      break;
    }
    if (footprint_is_free_with_margin(
          traj.points[i].pose, param_.aeb_margin_m, param_.aeb_occupancy_threshold)) {
      reached_clear_ground = true;
      continue;
    }
    if (!reached_clear_ground && arc <= param_.containment_grace_m) {
      continue;  // still extracting the robot from where it already stands
    }
    std::ostringstream why;
    why << "plan leaves the drivable area " << std::fixed << std::setprecision(2) << arc
        << " m along it, at (" << traj.points[i].pose.position.x << ", "
        << traj.points[i].pose.position.y << ")"
        << (reached_clear_ground ? "" : " and never reaches clear ground");
    aeb_reason_ = why.str();
    if (first_bad_index_out != nullptr) {
      *first_bad_index_out = i;
    }
    return false;
  }
  return true;
}

void StuckRecoverySupervisorNode::reject_plan_and_try_next_goal(const std::string & why)
{
  // What replaced the retreat.  When a plan cannot be used, the robot does not give
  // ground to buy a better starting position -- it aims at the next escape goal further
  // along the path, and when there are none left it stops.
  RCLCPP_WARN(
    get_logger(), "[episode %u] rejecting goal %zu/%zu: %s", episode_id_, goal_index_ + 1,
    goal_candidates_.size(), why.c_str());

  // Drop the plan, NOT the candidate list.  discard_recovery_state() clears the
  // candidates too, and calling it here sent the first rejection straight to ABORT with
  // thirteen goals still untried.
  freespace_trajectory_.reset();
  freespace_traj_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  recovery_trajectory_.reset();
  ++goal_index_;
  if (!advance_past_passed_goals() || !publish_current_goal()) {
    transition(State::ABORT, "no usable escape goal left: " + why);
    return;
  }
  last_valid_plan_ = this->now();
  spent_since_ = this->now();
  RCLCPP_INFO(
    get_logger(), "[episode %u] trying escape goal %zu/%zu instead", episode_id_,
    goal_index_ + 1, goal_candidates_.size());
}

bool StuckRecoverySupervisorNode::recovery_path_is_blocked(
  const autoware_planning_msgs::msg::Trajectory & traj)
{
  // Recovery's own emergency brake.  The plan being relayed was computed against an
  // older costmap; if something has since appeared in it, driving the plan anyway is
  // exactly the thing this scenario must never do.
  if (!odom_ || traj.points.size() < 2) {
    return false;
  }

  // No fresh costmap means no eyes.  While this scenario drives, the
  // motion_velocity_planner stop modules are bypassed entirely, so continuing blind is
  // not a neutral choice -- fail towards stopping.
  if (!costmap_is_fresh()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[episode %u] recovery AEB: costmap is stale, holding rather than driving blind",
      episode_id_);
    aeb_reason_ = "costmap is stale";
    return true;
  }

  const auto nearest = std::min_element(
    traj.points.begin(), traj.points.end(), [this](const auto & a, const auto & b) {
      return distance2d(a.pose, odom_->pose.pose) < distance2d(b.pose, odom_->pose.pose);
    });

  // (a) The plan itself.  A freespace path has been seen running straight through
  // occupied cells -- whether the costmap moved under it or the search produced it that
  // way, driving it is not acceptable either way.
  //
  // With one exception, the same one the drivable-area test makes: the ground the robot is
  // ALREADY standing on is not something braking can fix.  Recovery is often called
  // because the robot has ended up half off the road, and then the opening stretch of
  // every plan reads blocked.  Braking there holds the robot in the hole it is trying to
  // climb out of -- measured on the robot as an endless RECOVERY -> REPLAN cycle with the
  // brake on throughout.  So an opening blocked run is tolerated up to
  // containment_grace_m; the moment the plan touches clear ground, everything after it is
  // judged, which is what the brake is actually for: something new in the way.
  double arc = 0.0;
  double travel_sign = 0.0;
  bool reached_clear_ground = false;
  for (auto it = nearest; it != traj.points.end(); ++it) {
    if (it != nearest) {
      arc += distance2d(std::prev(it)->pose, it->pose);
    }
    if (travel_sign == 0.0 && std::abs(it->longitudinal_velocity_mps) > 1e-3) {
      travel_sign = it->longitudinal_velocity_mps > 0.0 ? 1.0 : -1.0;
    }
    if (arc > param_.aeb_lookahead_m) {
      break;
    }
    const bool free =
      footprint_is_free_with_margin(it->pose, param_.aeb_margin_m, param_.aeb_occupancy_threshold);
    if (free) {
      reached_clear_ground = true;
      continue;
    }
    if (arc < param_.aeb_skip_ahead_m) {
      continue;  // the robot is already here; the planner accepted this clearance
    }
    if (!reached_clear_ground && arc <= param_.containment_grace_m) {
      continue;  // still climbing out of where the robot already stands
    }
    // Say exactly where, so a false brake can be checked against the costmap in rviz
    // instead of argued about.
    std::ostringstream why;
    why << "plan is blocked " << std::fixed << std::setprecision(2) << arc << " m along it, at ("
        << it->pose.position.x << ", " << it->pose.position.y << "), footprint margin "
        << param_.aeb_margin_m << " m, threshold " << param_.aeb_occupancy_threshold;
    aeb_reason_ = why.str();
    return true;
  }

  // (b) Is the robot still ON the plan?  (a) only vouches for the path, so if tracking
  // has put the robot somewhere else the plan can read clear while the robot drives into
  // something.
  //
  // This used to sweep a straight line out from the ego pose along the direction of
  // travel, and that was wrong in the one scenario it runs in: a Reeds-Shepp escape is
  // mostly turning, so the straight projection pointed into the very obstacle the
  // manoeuvre was curving around.  It held the brake for an entire 60 s episode until
  // recovery timed out -- braking for the thing recovery exists to get past.
  //
  // Deviation is the honest form of the question, and unlike a straight projection it
  // cannot contradict a plan the planner has already validated.
  if (travel_sign == 0.0) {
    return false;  // nothing is being commanded; nothing to brake for
  }
  const double deviation = distance2d(nearest->pose, odom_->pose.pose);
  if (deviation > param_.aeb_max_deviation_m) {
    aeb_reason_ = "robot is off the plan it is following";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[episode %u] recovery AEB: robot is %.2f m off the plan it is following (limit %.2f m)",
      episode_id_, deviation, param_.aeb_max_deviation_m);
    return true;
  }
  return false;
}

std::optional<geometry_msgs::msg::Pose> StuckRecoverySupervisorNode::compute_escape_goal() const
{
  if (reference_path_.size() < 2) {
    return std::nullopt;
  }

  // Walk OUTWARD along the path and take the first candidate freespace will accept.
  //
  // A single take-it-or-leave-it goal was the wrong shape for this: astar_search
  // rejects a colliding goal outright ("Invalid start or goal pose") and never tries
  // anything else, so one bad candidate killed the whole attempt.  Stepping outward
  // means a goal that happens to sit on the obstacle, on unmapped ground, or outside
  // the costmap window simply gets skipped and the next one along the route is tried,
  // up to max_goal_search_distance_m -- i.e. towards the real mission goal.
  //
  // Nearest-valid-first is deliberate: it is the shortest detour that clears the
  // blockage, and freespace is free to reverse as much as it likes to reach it.
  const bool mirror = path_yaw_opposes_ego();

  double total = 0.0;
  std::vector<double> arc(reference_path_.size(), 0.0);
  for (size_t i = 1; i < reference_path_.size(); ++i) {
    total += distance2d(reference_path_[i - 1], reference_path_[i]);
    arc[i] = total;
  }
  if (total < param_.min_long_distance_to_check) {
    RCLCPP_WARN(
      get_logger(), "[episode %u] only %.1f m of path left, less than min_long_distance_to_check",
      episode_id_, total);
    return std::nullopt;
  }

  size_t tried = 0;
  size_t rejected_offgrid = 0;
  double next_check = param_.min_long_distance_to_check;
  for (size_t k = 1; k < reference_path_.size(); ++k) {
    if (arc[k] < next_check) {
      continue;
    }
    next_check = arc[k] + param_.goal_check_interval_m;
    if (arc[k] > param_.max_goal_search_distance_m) {
      break;
    }

    auto candidate = reference_path_[k];
    if (mirror) {
      // On a reversed route the path yaw is the direction of travel; the robot will be
      // sitting there facing the other way, so hand freespace that heading instead.
      const double yaw = tf2::getYaw(candidate.orientation) + M_PI;
      candidate.orientation.x = 0.0;
      candidate.orientation.y = 0.0;
      candidate.orientation.z = std::sin(yaw * 0.5);
      candidate.orientation.w = std::cos(yaw * 0.5);
    }

    ++tried;
    if (footprint_is_free(candidate)) {
      RCLCPP_INFO(
        get_logger(), "[episode %u] escape goal at %.1f m along the path (%zu candidates tried)%s",
        episode_id_, arc[k], tried, mirror ? ", heading flipped for reversed route" : "");
      return candidate;
    }
    if (!pose_is_on_grid(candidate)) {
      ++rejected_offgrid;
    }
  }

  RCLCPP_WARN(
    get_logger(),
    "[episode %u] no usable escape goal: %zu candidates from %.1f m to %.1f m every %.1f m, "
    "%zu of them outside the recovery costmap window",
    episode_id_, tried, param_.min_long_distance_to_check,
    std::min(total, param_.max_goal_search_distance_m), param_.goal_check_interval_m,
    rejected_offgrid);
  return std::nullopt;
}

bool StuckRecoverySupervisorNode::footprint_within_grid(
  const geometry_msgs::msg::Pose & pose) const
{
  if (!costmap_is_fresh() || grid_->info.resolution <= 0.0) {
    return false;
  }
  // Circumscribed radius: cheap, and erring towards "at the edge" is the safe direction
  // here -- it only stops us judging, it never lets anything through.
  const double reach = std::max(base_to_front_m_, base_to_rear_m_) + vehicle_width_m_ * 0.5 +
                       param_.aeb_margin_m;
  const double res = grid_->info.resolution;
  const double x0 = grid_->info.origin.position.x;
  const double y0 = grid_->info.origin.position.y;
  const double x1 = x0 + static_cast<double>(grid_->info.width) * res;
  const double y1 = y0 + static_cast<double>(grid_->info.height) * res;
  return pose.position.x - x0 > reach && x1 - pose.position.x > reach &&
         pose.position.y - y0 > reach && y1 - pose.position.y > reach;
}

bool StuckRecoverySupervisorNode::pose_is_on_grid(const geometry_msgs::msg::Pose & pose) const
{
  if (!costmap_is_fresh() || grid_->info.resolution <= 0.0) {
    return false;
  }
  const double res = grid_->info.resolution;
  const int col = static_cast<int>(std::floor((pose.position.x - grid_->info.origin.position.x) / res));
  const int row = static_cast<int>(std::floor((pose.position.y - grid_->info.origin.position.y) / res));
  return col >= 0 && row >= 0 && col < static_cast<int>(grid_->info.width) &&
         row < static_cast<int>(grid_->info.height);
}

bool StuckRecoverySupervisorNode::costmap_is_fresh() const
{
  if (!grid_) {
    return false;
  }
  return (this->now() - grid_at_).seconds() <= param_.costmap_timeout_sec;
}

CorridorResult StuckRecoverySupervisorNode::sweep(size_t from_index) const
{
  return sweep_path(reference_path_, from_index);
}

CorridorResult StuckRecoverySupervisorNode::sweep_path(
  const std::vector<geometry_msgs::msg::Pose> & path, size_t from_index) const
{
  if (!costmap_is_fresh() || path.size() < 2) {
    return CorridorResult{};  // never reason on a frozen grid
  }
  CorridorParams params;
  params.step_m = param_.probe_step_m;
  params.max_arc_m = param_.corridor_scan_distance_m;
  params.max_scan_half_width_m = param_.max_scan_half_width_m;
  params.vehicle_width_m = vehicle_width_m_;
  params.lateral_margin_m = param_.lateral_margin_m;
  params.occupancy_threshold = static_cast<int8_t>(param_.occupancy_threshold);
  // Mirroring has to come from the path being swept.  On a reversed route the poses face
  // the direction of travel while the robot faces the other way, so "left of the path" is
  // the robot's right -- and the followed trajectory and the centerline can disagree
  // about that, which is exactly when getting it wrong would matter.
  params.mirror_lateral = false;
  if (odom_) {
    if (const auto nearest = find_nearest_index(path, odom_->pose.pose)) {
      const double path_yaw = tf2::getYaw(path[*nearest].orientation);
      const double ego_yaw = tf2::getYaw(odom_->pose.pose.orientation);
      params.mirror_lateral = std::cos(path_yaw - ego_yaw) < 0.0;
    }
  }
  return sweep_corridor(*grid_, path, from_index, params);
}

std::vector<geometry_msgs::msg::Pose> StuckRecoverySupervisorNode::followed_path() const
{
  std::vector<geometry_msgs::msg::Pose> poses;
  if (!trajectory_ || trajectory_->points.size() < 2 || !odom_) {
    return poses;
  }
  poses.reserve(trajectory_->points.size());
  for (const auto & point : trajectory_->points) {
    poses.push_back(point.pose);
  }
  return poses;
}

bool StuckRecoverySupervisorNode::path_yaw_opposes_ego() const
{
  // On a reversed route the path poses follow the direction of travel while the robot
  // physically faces the other way, so "left of the path" is the robot's right.  Rather
  // than trust either convention, just ask the data: if the path heading near ego and
  // the ego heading point in opposite directions, mirror.
  if (!odom_ || reference_path_.empty()) {
    return false;
  }
  const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose);
  if (!nearest) {
    return false;
  }
  const double path_yaw = tf2::getYaw(reference_path_[*nearest].orientation);
  const double ego_yaw = tf2::getYaw(odom_->pose.pose.orientation);
  return std::cos(path_yaw - ego_yaw) < 0.0;
}

void StuckRecoverySupervisorNode::publish_recovery_arming(bool active)
{
  autoware_internal_planning_msgs::msg::Scenario msg;
  // Our private freespace_planner / costmap_generator instances only test for the
  // string "Parking" in activating_scenarios (freespace utils.cpp is_active).  They
  // listen on our own topic, so nothing upstream sees this and neither node needs a
  // patch.  The system-wide scenario is a separate concern, handled by the
  // scenario_selector force request.
  msg.current_scenario = active ? autoware_internal_planning_msgs::msg::Scenario::PARKING
                                : autoware_internal_planning_msgs::msg::Scenario::EMPTY;
  if (active) {
    msg.activating_scenarios.push_back(autoware_internal_planning_msgs::msg::Scenario::PARKING);
  }
  pub_scenario_->publish(msg);
}

void StuckRecoverySupervisorNode::reset_for_new_mission(const std::string & why)
{
  RCLCPP_INFO(
    get_logger(), "[episode %u] %s -- clearing everything and starting fresh (%s)",
    episode_id_, state_name(state_).c_str(), why.c_str());

  // transition(NOMINAL) already discards the recovery plan and goal, clears the abort
  // reason and reference path, and zeroes attempts_.  What it does NOT clear is the
  // cross-episode bookkeeping below, which is exactly the state that would otherwise
  // leak into the new mission.
  transition(State::NOMINAL, why);
  abort_retries_ = 0;
  episode_stop_source_ = StuckDiagnosis::SOURCE_NONE;
  corridor_clear_ = false;
  last_sweep_ = CorridorResult{};
  cooldown_until_ = this->now();
  entry_pose_ = odom_ ? odom_->pose.pose : geometry_msgs::msg::Pose{};
}

void StuckRecoverySupervisorNode::discard_recovery_state()
{
  freespace_trajectory_.reset();
  freespace_traj_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  recovery_trajectory_.reset();
  recovery_completed_ = false;
  escape_goal_.reset();
  goal_candidates_.clear();
  goal_index_ = 0;
  failed_goal_tries_ = 0;
  replan_attempts_ = 0;
}

void StuckRecoverySupervisorNode::publish_recovery_route(const geometry_msgs::msg::Pose & goal)
{
  // freespace_planner reads nothing but goal_pose off this message (and resets its
  // planner whenever a new one arrives), so a minimal synthetic route is enough --
  // and it means the real mission route is never touched.
  autoware_planning_msgs::msg::LaneletRoute route;
  route.header.stamp = this->now();
  route.header.frame_id = "map";
  if (odom_) {
    route.start_pose = odom_->pose.pose;
  }
  route.goal_pose = goal;
  route.allow_modification = false;

  // Drop the plan we are holding before the new goal goes out: anything freespace
  // produced so far answers the OLD goal, and must never be relayed after this point.
  freespace_trajectory_.reset();
  freespace_traj_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  route_published_at_ = this->now();
  pub_route_->publish(route);

  RCLCPP_INFO(
    get_logger(), "[episode %u] escape goal published at (%.2f, %.2f)", episode_id_,
    goal.position.x, goal.position.y);
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------
void StuckRecoverySupervisorNode::publish_outputs()
{
  const auto now = this->now();

  autoware_stuck_recovery_msgs::msg::RecoveryState state_msg;
  state_msg.stamp = now;
  state_msg.state = static_cast<int32_t>(state_);
  state_msg.state_name = state_name(state_);
  state_msg.episode_id = episode_id_;
  state_msg.attempts = attempts_;
  state_msg.elapsed_in_state_s = static_cast<float>((now - state_since_).seconds());
  state_msg.elapsed_in_episode_s = static_cast<float>((now - episode_since_).seconds());
  // Both fields are kept at zero: the reverse budget they reported is gone, but the
  // fields stay so bags recorded before the removal still decode.
  state_msg.reverse_distance_m = 0.0f;
  state_msg.total_reverse_distance_m = 0.0f;
  state_msg.goal_index = static_cast<uint16_t>(goal_index_);
  state_msg.goal_candidates = static_cast<uint16_t>(goal_candidates_.size());
  state_msg.abort_reason = abort_reason_;
  if (odom_ && !reference_path_.empty()) {
    if (const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose)) {
      state_msg.lateral_excursion_m =
        static_cast<float>(distance2d(reference_path_[*nearest], odom_->pose.pose));
    }
  }
  pub_state_->publish(state_msg);

  autoware_internal_debug_msgs::msg::Int32Stamped state_int;
  state_int.stamp = now;
  state_int.data = static_cast<int32_t>(state_);
  pub_state_int_->publish(state_int);

  StuckDiagnosis diag;
  diag.stamp = now;
  diag.is_stopped = is_stopped_;
  diag.stopped_duration_s = static_cast<float>((now - stopped_since_).seconds());
  diag.commanded_velocity_mps =
    control_cmd_ ? static_cast<float>(control_cmd_->longitudinal.velocity) : 0.0f;
  diag.measured_velocity_mps =
    velocity_ ? static_cast<float>(velocity_->longitudinal_velocity) : 0.0f;
  diag.progress_since_entry_m = static_cast<float>(progress_since_entry_m_);
  diag.stop_source = stop_source_;
  diag.stop_source_module = stop_source_module_;
  diag.legit_stop = legit_stop_;
  diag.legit_reason = legit_reason_;
  diag.path_reversed = path_yaw_opposes_ego();
  diag.free_left_m = static_cast<float>(last_sweep_.free_left_m);
  diag.free_right_m = static_cast<float>(last_sweep_.free_right_m);
  diag.free_width_m = static_cast<float>(last_sweep_.free_width_m);
  diag.chosen_side = static_cast<uint8_t>(last_sweep_.best_side);
  diag.chosen_offset_m = static_cast<float>(last_sweep_.best_offset_m);
  diag.required_width_m = static_cast<float>(last_sweep_.required_width_m);
  diag.corridor_clear = corridor_clear_;
  diag.start_blocked = start_blocked_;
  if (odom_ && route_) {
    diag.remaining_distance_m = static_cast<float>(distance2d(odom_->pose.pose, route_->goal_pose));
  }
  pub_diagnosis_->publish(diag);
}

void StuckRecoverySupervisorNode::record_footprint_debug(
  const autoware_planning_msgs::msg::Trajectory & traj)
{
  footprint_debug_.clear();
  if (!param_.publish_footprint_markers || !costmap_is_fresh()) {
    return;
  }
  // Same walk and the same grace rule the drivable-area test applies, so what appears in
  // rviz is the check's own answer and not a second opinion that could disagree with it.
  double arc = 0.0;
  bool reached_clear_ground = false;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (i > 0) {
      arc += distance2d(traj.points[i - 1].pose, traj.points[i].pose);
    }
    const auto & pose = traj.points[i].pose;
    FootprintVerdict verdict = FootprintVerdict::FREE;
    if (footprint_is_free_with_margin(
          pose, param_.aeb_margin_m, param_.aeb_occupancy_threshold)) {
      reached_clear_ground = true;
    } else if (!reached_clear_ground && arc <= param_.containment_grace_m) {
      verdict = FootprintVerdict::GRACED;
    } else {
      verdict = FootprintVerdict::BLOCKING;
    }
    footprint_debug_.emplace_back(pose, verdict);
  }
}

void StuckRecoverySupervisorNode::append_footprint_markers(
  visualization_msgs::msg::MarkerArray & markers, const rclcpp::Time & now) const
{
  // One closed rectangle per tested pose: the vehicle outline grown by the margin the
  // test used, which is the box the occupancy lookup actually covers.
  const auto box = [&](const geometry_msgs::msg::Pose & pose, double margin) {
    const double yaw = tf2::getYaw(pose.orientation);
    const double c = std::cos(yaw);
    const double sn = std::sin(yaw);
    const double half_w = vehicle_width_m_ * 0.5 + margin;
    const double front = base_to_front_m_ + margin;
    const double rear = base_to_rear_m_ + margin;
    const double lon[5] = {front, front, -rear, -rear, front};
    const double lat[5] = {half_w, -half_w, -half_w, half_w, half_w};
    std::vector<geometry_msgs::msg::Point> pts;
    for (int k = 0; k < 5; ++k) {
      geometry_msgs::msg::Point pt;
      pt.x = pose.position.x + c * lon[k] - sn * lat[k];
      pt.y = pose.position.y + sn * lon[k] + c * lat[k];
      pt.z = pose.position.z;
      pts.push_back(pt);
    }
    return pts;
  };

  auto make = [&](const std::string & ns, int id, const std::vector<geometry_msgs::msg::Point> & pts,
                  float r, float g, float b, float a, double width) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = width;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    m.points = pts;
    return m;
  };

  // Clear the previous frame's boxes: fewer boxes this tick would otherwise leave stale
  // ones on screen, which is worse than no drawing at all.
  visualization_msgs::msg::Marker wipe;
  wipe.header.frame_id = "map";
  wipe.header.stamp = now;
  wipe.ns = "stuck_recovery_plan_footprints";
  wipe.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(wipe);

  int id = 0;
  for (const auto & [pose, verdict] : footprint_debug_) {
    switch (verdict) {
      case FootprintVerdict::FREE:
        markers.markers.push_back(
          make("stuck_recovery_plan_footprints", id++, box(pose, param_.aeb_margin_m), 0.2f, 0.9f,
               0.3f, 0.5f, 0.03));
        break;
      case FootprintVerdict::GRACED:
        // Blocked, but this is the ground the robot already stands on -- drawn so it is
        // obvious the check saw it and deliberately let it pass.
        markers.markers.push_back(
          make("stuck_recovery_plan_footprints", id++, box(pose, param_.aeb_margin_m), 1.0f, 0.7f,
               0.0f, 0.9f, 0.05));
        break;
      case FootprintVerdict::BLOCKING:
        markers.markers.push_back(
          make("stuck_recovery_plan_footprints", id++, box(pose, param_.aeb_margin_m), 1.0f, 0.1f,
               0.1f, 1.0f, 0.08));
        break;
    }
  }

  // The robot's own box, tested the same way.  This is the one that refused ten goals in a
  // row, so it is worth seeing on its own rather than buried among the plan's boxes.
  if (odom_ && costmap_is_fresh()) {
    const bool ego_free = footprint_is_free_with_margin(
      odom_->pose.pose, param_.aeb_margin_m, param_.aeb_occupancy_threshold);
    markers.markers.push_back(make(
      "stuck_recovery_ego_footprint", 0, box(odom_->pose.pose, param_.aeb_margin_m),
      ego_free ? 0.2f : 1.0f, ego_free ? 0.9f : 0.4f, ego_free ? 1.0f : 0.0f, 1.0f, 0.06));
  }

  // The escape goal, grown by the margin GOAL PLACEMENT uses -- a different, larger margin
  // than the brake's, which is why a goal can be rejected on ground the brake would accept.
  if (escape_goal_) {
    markers.markers.push_back(make(
      "stuck_recovery_goal_footprint", 0, box(*escape_goal_, param_.goal_shape_margin_m), 0.2f,
      0.4f, 1.0f, 0.8f, 0.05));
  }
}

void StuckRecoverySupervisorNode::publish_markers()
{
  visualization_msgs::msg::MarkerArray markers;
  const auto now = this->now();

  visualization_msgs::msg::Marker text;
  text.header.frame_id = "map";
  text.header.stamp = now;
  text.ns = "stuck_recovery_state";
  text.id = 0;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  if (odom_) {
    text.pose = odom_->pose.pose;
  }
  text.pose.position.z += 2.0;
  text.scale.z = 0.6;
  text.color.a = 1.0;
  text.color.r = state_ == State::NOMINAL ? 0.4f : 1.0f;
  text.color.g = (state_ == State::NOMINAL || state_ == State::COOLDOWN) ? 1.0f : 0.6f;
  text.color.b = 0.2f;
  text.text = state_name(state_);
  if (state_ != State::NOMINAL) {
    text.text += " | " + stop_source_module_;
    if (last_sweep_.valid) {
      text.text += " | L=" + std::to_string(last_sweep_.free_left_m).substr(0, 4) +
                   " R=" + std::to_string(last_sweep_.free_right_m).substr(0, 4) +
                   " need " + std::to_string(last_sweep_.required_width_m).substr(0, 4) + " m";
    }
  }
  markers.markers.push_back(text);

  // The swept corridor itself: the single most useful thing to look at when the
  // supervisor made a decision you did not expect.
  visualization_msgs::msg::Marker corridor;
  corridor.header.frame_id = "map";
  corridor.header.stamp = now;
  corridor.ns = "stuck_recovery_corridor";
  corridor.id = 0;
  corridor.type = visualization_msgs::msg::Marker::LINE_STRIP;
  corridor.action = reference_path_.empty() ? visualization_msgs::msg::Marker::DELETE
                                            : visualization_msgs::msg::Marker::ADD;
  corridor.scale.x = 0.08;
  corridor.color.a = 0.9;
  corridor.color.r = corridor_clear_ ? 0.1f : 1.0f;
  corridor.color.g = corridor_clear_ ? 1.0f : 0.3f;
  corridor.color.b = 0.1f;
  corridor.pose.orientation.w = 1.0;
  for (const auto & pose : reference_path_) {
    corridor.points.push_back(pose.position);
  }
  markers.markers.push_back(corridor);

  if (escape_goal_) {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = now;
    goal_msg.pose = *escape_goal_;
    pub_escape_goal_->publish(goal_msg);

    visualization_msgs::msg::Marker goal;
    goal.header.frame_id = "map";
    goal.header.stamp = now;
    goal.ns = "stuck_recovery_escape_goal";
    goal.id = 0;
    goal.type = visualization_msgs::msg::Marker::ARROW;
    goal.action = visualization_msgs::msg::Marker::ADD;
    goal.pose = *escape_goal_;
    goal.scale.x = 1.0;
    goal.scale.y = 0.2;
    goal.scale.z = 0.2;
    goal.color.a = 1.0;
    goal.color.r = 0.2f;
    goal.color.g = 0.4f;
    goal.color.b = 1.0f;
    markers.markers.push_back(goal);
  }

  if (param_.publish_footprint_markers) {
    append_footprint_markers(markers, now);
  }

  pub_markers_->publish(markers);
}

void StuckRecoverySupervisorNode::on_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("state", state_name(state_));
  stat.add("mode", param_.mode);
  stat.add("episode_id", episode_id_);
  stat.add("attempts", attempts_);
  stat.add("stop_source_module", stop_source_module_);
  stat.add("legit_reason", legit_reason_);
  stat.add("path_reversed", path_yaw_opposes_ego());
  stat.add("start_blocked", start_blocked_);
  stat.add("goal_candidate", std::to_string(goal_index_ + 1) + "/" +
                               std::to_string(goal_candidates_.size()));
  stat.add("free_left_m", last_sweep_.free_left_m);
  stat.add("free_right_m", last_sweep_.free_right_m);
  stat.add("free_width_m", last_sweep_.free_width_m);
  stat.add("chosen_side", side_name(last_sweep_.best_side));
  stat.add("required_width_m", last_sweep_.required_width_m);
  stat.add("abort_reason", abort_reason_);

  switch (state_) {
    case State::NOMINAL:
    case State::COOLDOWN:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "nominal");
      break;
    case State::SUSPECT:
    case State::PROBE:
    case State::RECOVERY:
    case State::REPLAN:
    case State::TRANSIT:
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "stuck handling: " + state_name(state_));
      break;
    case State::BLOCKED:
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "blocked: " + abort_reason_);
      break;
    case State::ABORT:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "aborted: " + abort_reason_);
      break;
  }
}

void StuckRecoverySupervisorNode::on_force_recovery(
  const autoware_stuck_recovery_msgs::srv::ForceRecovery::Request::SharedPtr request,
  const autoware_stuck_recovery_msgs::srv::ForceRecovery::Response::SharedPtr response)
{
  if (request->enable) {
    if (mode_ != Mode::RECOVERY) {
      response->success = false;
      response->message = "node is in mode '" + param_.mode + "', not 'recovery'";
      return;
    }
    force_requested_ = true;
    response->success = true;
    response->message = "recovery will be attempted at the next tick";
  } else {
    force_requested_ = false;
    transition(State::NOMINAL, "operator cleared: " + request->reason);
    response->success = true;
    response->message = "cleared";
  }
  RCLCPP_WARN(
    get_logger(), "ForceRecovery(enable=%s, reason='%s') -> %s", request->enable ? "true" : "false",
    request->reason.c_str(), response->message.c_str());
}

}  // namespace autoware::stuck_recovery_supervisor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::stuck_recovery_supervisor::StuckRecoverySupervisorNode)
