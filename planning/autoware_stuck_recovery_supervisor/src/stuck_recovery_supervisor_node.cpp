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

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <tf2/utils.h>
// See corridor_checker.cpp: tf2::getYaw needs the fromMsg definition from here, and
// omitting it links fine but dies at the first call.
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
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
    case State::BACKOFF:
      return "BACKOFF";
    case State::REPLAN:
      return "REPLAN";
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
  p.max_goal_search_distance_m = declare_parameter<double>("max_goal_search_distance_m", 30.0);
  p.goal_shape_margin_m = declare_parameter<double>("goal_shape_margin_m", 0.2);
  p.recovery_timeout_sec = declare_parameter<double>("recovery_timeout_sec", 60.0);
  p.clear_hold_sec = declare_parameter<double>("clear_hold_sec", 1.0);
  p.rejoin_lateral_tolerance_m =
    declare_parameter<double>("rejoin_lateral_tolerance_m", 0.5);
  p.aeb_lookahead_m = declare_parameter<double>("aeb_lookahead_m", 1.5);
  p.aeb_margin_m = declare_parameter<double>("aeb_margin_m", 0.05);
  p.aeb_skip_ahead_m = declare_parameter<double>("aeb_skip_ahead_m", 0.3);
  p.max_reverse_distance_m = declare_parameter<double>("max_reverse_distance_m", 10.0);
  p.max_total_reverse_distance_m =
    declare_parameter<double>("max_total_reverse_distance_m", 15.0);
  p.total_reverse_reset_progress_m =
    declare_parameter<double>("total_reverse_reset_progress_m", 5.0);
  p.max_lateral_excursion_m = declare_parameter<double>("max_lateral_excursion_m", 2.0);

  p.recovery_speed_limit_mps = declare_parameter<double>("recovery_speed_limit_mps", 0.278);
  p.min_engageable_stop_distance_m =
    declare_parameter<double>("min_engageable_stop_distance_m", 0.3);
  p.backoff_speed_mps = declare_parameter<double>("backoff_speed_mps", 0.278);
  p.backoff_after_failed_goals =
    static_cast<int>(declare_parameter<int64_t>("backoff_after_failed_goals", 2));
  p.backoff_reach_tolerance_m = declare_parameter<double>("backoff_reach_tolerance_m", 0.3);
  p.backoff_lookback_m = declare_parameter<double>("backoff_lookback_m", 15.0);
  p.goal_retry_sec = declare_parameter<double>("goal_retry_sec", 4.0);
  p.replan_wait_sec = declare_parameter<double>("replan_wait_sec", 5.0);
  p.spent_hold_sec = declare_parameter<double>("spent_hold_sec", 2.0);
  p.backoff_progress_reset_m =
    declare_parameter<double>("backoff_progress_reset_m", 0.5);
  p.freespace_plan_timeout_sec =
    declare_parameter<double>("freespace_plan_timeout_sec", 1.0);
  p.costmap_timeout_sec = declare_parameter<double>("costmap_timeout_sec", 1.0);
  p.max_replan_attempts =
    static_cast<int>(declare_parameter<int64_t>("max_replan_attempts", 3));
  p.max_attempts = static_cast<int>(declare_parameter<int64_t>("max_attempts", 2));
  p.cooldown_sec = declare_parameter<double>("cooldown_sec", 60.0);
  p.max_episode_duration_sec = declare_parameter<double>("max_episode_duration_sec", 180.0);
  p.abort_release_sec = declare_parameter<double>("abort_release_sec", 5.0);

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
  // Backing up a long way is fine as long as it gets the robot past the obstacle --
  // the ceiling is here only to stop a runaway, not to shape the manoeuvre.
  p.max_reverse_distance_m =
    clamp_with_warning(p.max_reverse_distance_m, 0.0, 20.0, "max_reverse_distance_m");
  p.max_total_reverse_distance_m = clamp_with_warning(
    p.max_total_reverse_distance_m, 0.0, 40.0, "max_total_reverse_distance_m");
  p.max_lateral_excursion_m =
    clamp_with_warning(p.max_lateral_excursion_m, 0.0, 3.0, "max_lateral_excursion_m");
  p.corridor_scan_distance_m =
    clamp_with_warning(p.corridor_scan_distance_m, 1.0, 20.0, "corridor_scan_distance_m");
  p.goal_check_interval_m =
    clamp_with_warning(p.goal_check_interval_m, 0.2, 5.0, "goal_check_interval_m");
  p.recovery_timeout_sec =
    clamp_with_warning(p.recovery_timeout_sec, 5.0, 120.0, "recovery_timeout_sec");
  // Crawl speed only: the backoff is a blind reverse manoeuvre, so it is never allowed
  // to go faster than the 1 km/h the recovery plan itself runs at.
  p.backoff_speed_mps = clamp_with_warning(p.backoff_speed_mps, 0.05, 0.4, "backoff_speed_mps");
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
  sub_route_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    param_.topic_route, latched,
    [this](const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) { route_ = msg; });
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
  update_goal_progress();
  classify_stop();

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
    case State::BACKOFF:
      step_backoff();
      break;
    case State::REPLAN:
      step_replan();
      break;
  }

  // Arming is derived from the state rather than latched, so that any exit path --
  // including an exception-driven one -- also disarms.  BLOCKED keeps the costmap
  // alive because that is how we notice the blockage going away.
  const bool arm_costmap =
    mode_ >= Mode::PROBE_ONLY && (state_ == State::PROBE || state_ == State::RECOVERY ||
                                  state_ == State::BLOCKED || state_ == State::BACKOFF ||
                                  state_ == State::REPLAN);
  publish_recovery_arming(arm_costmap);

  std_msgs::msg::Bool force;
  // The backoff drives the robot too, so the scenario has to stay switched over.
  force.data = (mode_ == Mode::RECOVERY &&
                (state_ == State::RECOVERY || state_ == State::BACKOFF ||
                 state_ == State::REPLAN));
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
    if (has_last_pose_) {
      const double delta = distance2d(last_pose_, pose);
      // The reverse budget bounds how far THIS scenario drives the robot backwards --
      // backwards in the vehicle's own frame, because that is the direction it cannot
      // see, which is the actual risk being limited.
      //
      // Two mistakes were made here before.  Measuring against the reference path
      // tangent made the number depend on path geometry, so it could under-count real
      // reverse travel (the hole) as easily as over-count it.  And accumulating in
      // update_motion() regardless of state meant every metre the robot drove in plain
      // lane driving counted too: the budget read 25 m against a 10 m limit, so it was
      // measuring nothing meaningful.  Only count while this scenario is the one driving.
      const bool we_are_driving = state_ == State::RECOVERY || state_ == State::BACKOFF ||
                                  state_ == State::REPLAN;
      if (we_are_driving && delta > 0.0) {
        const double yaw = tf2::getYaw(pose.orientation);
        const double dx = pose.position.x - last_pose_.position.x;
        const double dy = pose.position.y - last_pose_.position.y;
        if ((dx * std::cos(yaw) + dy * std::sin(yaw)) < 0.0) {
          reverse_distance_m_ += delta;
          total_reverse_distance_m_ += delta;
        }
      }
    }
    last_pose_ = pose;
    has_last_pose_ = true;

    // Displacement, not path length: a robot that shuffles back and forth by 10 cm
    // has not escaped anything, and must keep counting as stuck.
    if (state_ != State::NOMINAL) {
      progress_since_entry_m_ = distance2d(entry_pose_, pose);
    }
  }
}

void StuckRecoverySupervisorNode::update_goal_progress()
{
  if (!odom_ || !route_) {
    return;
  }
  const double remaining = distance2d(odom_->pose.pose, route_->goal_pose);
  if (remaining >= best_remaining_to_goal_m_) {
    return;  // no closer than we have already been; nothing has been achieved
  }

  const bool made_progress =
    best_remaining_to_goal_m_ != std::numeric_limits<double>::max() &&
    (best_remaining_to_goal_m_ - remaining) >= param_.total_reverse_reset_progress_m;
  best_remaining_to_goal_m_ = remaining;
  if (made_progress && total_reverse_distance_m_ > 0.0) {
    RCLCPP_INFO(
      get_logger(), "advanced %.1f m toward the goal; clearing the %.2f m cumulative reverse budget",
      param_.total_reverse_reset_progress_m, total_reverse_distance_m_);
    total_reverse_distance_m_ = 0.0;
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
    legit_stop_ = true;
    legit_reason_ = "route_not_set";
    return;
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
  if (next == State::COOLDOWN || next == State::BLOCKED || next == State::ABORT) {
    // The scenario is no longer driving; drop the plan and goal so that re-arming later
    // cannot resurrect them.
    discard_recovery_state();
  }
  if (next == State::ABORT || next == State::BLOCKED) {
    abort_reason_ = reason;
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
    reverse_distance_m_ = 0.0;
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
  reverse_distance_m_ = 0.0;
  progress_since_entry_m_ = 0.0;
  entry_pose_ = odom_->pose.pose;
  pose_at_last_backoff_ = odom_->pose.pose;
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
  const bool triggerable = stop_source_ == StuckDiagnosis::SOURCE_OBSTACLE_STOP ||
                           stop_source_ == StuckDiagnosis::SOURCE_STOP_BEHIND ||
                           stop_source_ == StuckDiagnosis::SOURCE_MAP_STOP;
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
  if (reverse_distance_m_ > param_.max_reverse_distance_m) {
    transition(State::ABORT, "reverse budget exceeded");
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
    RCLCPP_INFO(
      get_logger(), "[episode %u] escape goal passed and none left ahead", episode_id_);
    cooldown_until_ = now + rclcpp::Duration::from_seconds(param_.cooldown_sec);
    transition(State::COOLDOWN, "escape goal passed; manoeuvre complete");
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
      // No goal can help: freespace rejects the START pose because the robot's own
      // footprint is inside the inflated obstacle.  Backing off is the only move.
      begin_backoff("start pose is inside an obstacle; freespace cannot plan from here");
      return;
    }
    ++failed_goal_tries_;
    const int limit = std::max(1, param_.backoff_after_failed_goals);
    if (failed_goal_tries_ >= limit) {
      failed_goal_tries_ = 0;
      begin_backoff(
        "freespace failed " + std::to_string(limit) + " goal attempts in a row");
      return;
    }
    if (goal_index_ + 1 < goal_candidates_.size()) {
      ++goal_index_;
    } else {
      goal_index_ = 0;  // wrap: the grid moves with the robot, so retry from the front
    }
    RCLCPP_WARN(
      get_logger(),
      "[episode %u] no plan for %.1f s (%d/%d before backing off) -> goal candidate %zu/%zu",
      episode_id_, param_.goal_retry_sec, failed_goal_tries_, limit, goal_index_ + 1,
      goal_candidates_.size());
    last_valid_plan_ = now;
    publish_current_goal();
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

  // One direction guard: swapping to a forward lane-driving trajectory while the
  // robot is still rolling backwards means flipping gear under motion.  Freespace
  // already stops at every cusp, so the wait is at most one segment.
  if (current_recovery_segment_is_reverse() && !is_stopped_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[episode %u] corridor clear, finishing the reverse segment before handing back",
      episode_id_);
    return;
  }

  cooldown_until_ = now + rclcpp::Duration::from_seconds(param_.cooldown_sec);
  transition(State::COOLDOWN, "corridor ahead is clear again");
}

void StuckRecoverySupervisorNode::begin_backoff(const std::string & why)
{
  if (!odom_) {
    transition(State::ABORT, "no odometry: " + why);
    return;
  }

  // Retreating is the response to being stuck in the same place, not to falling a little
  // short.  If the robot has covered ground since the last retreat -- it edged past part
  // of the obstacle, or merely paused for AEB -- then the manoeuvre IS working, and
  // backing off would discard that and start over.  Replan from here instead.
  const double progress = distance2d(pose_at_last_backoff_, odom_->pose.pose);
  if (progress >= param_.backoff_progress_reset_m) {
    pose_at_last_backoff_ = odom_->pose.pose;
    failed_goal_tries_ = 0;
    replan_attempts_ = 0;
    goal_index_ = 0;
    RCLCPP_INFO(
      get_logger(),
      "[episode %u] moved %.2f m since the last retreat -- replanning from here instead of "
      "backing off again (%s)",
      episode_id_, progress, why.c_str());
    force_freespace_replan("progress made; retrying from the new pose");
    return;
  }

  if (total_reverse_distance_m_ >= param_.max_total_reverse_distance_m) {
    transition(
      State::ABORT,
      "cumulative reverse budget exhausted (" +
        std::to_string(total_reverse_distance_m_) +
        " m without reaching the goal) -- an advancing obstacle may be pushing the robot "
        "back; this needs an operator: " + why);
    return;
  }
  if (reverse_distance_m_ >= param_.max_reverse_distance_m) {
    transition(State::ABORT, "reverse budget exhausted: " + why);
    return;
  }
  const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose);
  if (!nearest || *nearest == 0) {
    transition(State::ABORT, "nothing behind us to retreat along: " + why);
    return;
  }

  backoff_start_pose_ = odom_->pose.pose;
  pose_at_last_backoff_ = odom_->pose.pose;
  // Give back exactly as much ground as we worked through in the candidate list.
  const double wanted =
    std::max(1, param_.backoff_after_failed_goals) * param_.goal_check_interval_m;
  backoff_target_m_ = std::min(
    std::min(wanted, 5.0), param_.max_reverse_distance_m - reverse_distance_m_);

  // Pin the end index now.  Deriving it from the robot's current position on every
  // tick made the target recede as fast as the robot approached it.
  backoff_end_index_ = 0;
  double arc = 0.0;
  for (size_t k = *nearest; k-- > 0;) {
    arc += distance2d(reference_path_[k + 1], reference_path_[k]);
    if (arc >= backoff_target_m_) {
      backoff_end_index_ = k;
      break;
    }
  }

  RCLCPP_WARN(
    get_logger(), "[episode %u] backing off %.2f m at %.2f m/s -- %s", episode_id_,
    backoff_target_m_, param_.backoff_speed_mps, why.c_str());
  transition(State::BACKOFF, why);
}

void StuckRecoverySupervisorNode::step_backoff()
{
  const auto now = this->now();

  if (!is_operational() || emergency_code_ != 0) {
    transition(State::ABORT, "left autonomous mode or emergency during backoff");
    return;
  }
  if (reverse_distance_m_ > param_.max_reverse_distance_m) {
    transition(State::ABORT, "reverse budget exceeded while backing off");
    return;
  }
  if (total_reverse_distance_m_ > param_.max_total_reverse_distance_m) {
    transition(State::ABORT, "cumulative reverse budget exceeded while backing off");
    return;
  }
  if ((now - episode_since_).seconds() > param_.max_episode_duration_sec) {
    transition(State::ABORT, "episode watchdog expired while backing off");
    return;
  }

  // If something is behind the robot it simply will not move, and none of the other
  // exits fire: distance travelled stays near zero, so the reverse budget never grows
  // either.  Bound it by the time the retreat should physically take, with generous
  // slack for the crawl speed and the smoother's ramp.
  const double expected_sec =
    backoff_target_m_ / std::max(0.05, param_.backoff_speed_mps) * 3.0 + 5.0;
  if ((now - state_since_).seconds() > expected_sec) {
    transition(
      State::BLOCKED,
      "could not retreat -- moved " +
        std::to_string(odom_ ? distance2d(backoff_start_pose_, odom_->pose.pose) : 0.0) +
        " m of " + std::to_string(backoff_target_m_) + " m; something may be behind");
    return;
  }

  // Primary exit: freespace found something.  This has to be checked before anything
  // else -- the whole point of retreating is to reach a pose it can plan from, so the
  // moment it succeeds we stop retreating and follow that plan, whether or not the
  // retreat step had finished.
  if (freespace_plan_is_valid()) {
    last_valid_plan_ = now;
    spent_since_ = now;
    failed_goal_tries_ = 0;
    RCLCPP_INFO(
      get_logger(), "[episode %u] freespace found a plan while backing off; following it",
      episode_id_);
    transition(State::RECOVERY, "freespace produced a valid plan");
    return;
  }

  // The corridor may clear while we retreat (a pedestrian walking on, say) -- in that
  // case there is nothing left to recover from.
  size_t from = 0;
  if (odom_) {
    if (const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose)) {
      from = *nearest;
    }
  }
  last_sweep_ = sweep(from);
  corridor_clear_ = last_sweep_.valid && last_sweep_.clear;
  if (corridor_clear_ && (now - clear_since_).seconds() >= param_.clear_hold_sec) {
    cooldown_until_ = now + rclcpp::Duration::from_seconds(param_.cooldown_sec);
    transition(State::COOLDOWN, "corridor cleared while backing off");
    return;
  }
  if (!corridor_clear_) {
    clear_since_ = now;
  }

  const double moved = odom_ ? distance2d(backoff_start_pose_, odom_->pose.pose) : 0.0;
  const double to_go =
    (odom_ && backoff_end_index_ < reference_path_.size())
      ? distance2d(reference_path_[backoff_end_index_], odom_->pose.pose)
      : 0.0;
  // Safety exit: done as soon as we are within tolerance of the target, measured both
  // ways -- distance actually travelled, and distance still to run to the pinned end.
  const double tol = param_.backoff_reach_tolerance_m;
  const bool reached = moved >= (backoff_target_m_ - tol) || to_go <= tol;
  if (!reached) {
    return;  // keep retreating; the trajectory is published by relay_recovery_trajectory
  }

  // Retreat step done.  Freespace only replans once the vehicle has actually stopped,
  // so wait for that before handing back -- otherwise it just tells us it is waiting.
  if (!is_stopped_) {
    return;
  }

  goal_index_ = 0;
  failed_goal_tries_ = 0;
  last_valid_plan_ = now;
  publish_current_goal();
  RCLCPP_INFO(
    get_logger(), "[episode %u] backed off %.2f m (%.2f m total); replanning from here",
    episode_id_, moved, reverse_distance_m_);
  transition(State::RECOVERY, "retreated a step; letting freespace try again");
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

  double arc = 0.0;
  double next_check = search_from;
  size_t skipped_offgrid = 0;
  for (size_t k = ego_index + 1; k < reference_path_.size(); ++k) {
    arc += distance2d(reference_path_[k - 1], reference_path_[k]);
    if (arc < next_check) {
      continue;
    }
    next_check = arc + param_.goal_check_interval_m;
    if (arc > param_.max_goal_search_distance_m) {
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
    if (footprint_is_free(candidate)) {
      candidates.push_back(candidate);
    } else if (!pose_is_on_grid(candidate)) {
      ++skipped_offgrid;
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "[episode %u] %zu escape-goal candidates from %.1f m (blockage at %.1f m + %.1f m offset), "
    "%zu skipped as off-grid%s",
    episode_id_, candidates.size(), search_from,
    last_sweep_.has_blockage ? last_sweep_.first_blocked_arc_m : -1.0,
    param_.goal_offset_beyond_obstacle_m, skipped_offgrid,
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

autoware_planning_msgs::msg::Trajectory StuckRecoverySupervisorNode::build_backoff_trajectory()
  const
{
  autoware_planning_msgs::msg::Trajectory traj;
  traj.header.frame_id = "map";
  traj.header.stamp = this->now();
  if (!odom_ || reference_path_.size() < 2) {
    return traj;
  }

  const auto nearest = find_nearest_index(reference_path_, odom_->pose.pose);
  if (!nearest || *nearest == 0) {
    return traj;  // nothing behind us to retreat along
  }

  // Vehicle heading: on a reversed route the stored path yaw is the travel direction,
  // so correct it back into the vehicle's own frame first.
  const bool mirror = path_yaw_opposes_ego();
  // Runs to the pinned end index, so the trajectory shortens as the robot approaches
  // and terminates with a zero-velocity point -- which is what makes it stop there.
  const size_t end_index = std::min(backoff_end_index_, *nearest);

  for (size_t k = *nearest + 1; k-- > end_index;) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = reference_path_[k];
    double yaw = tf2::getYaw(point.pose.orientation);
    if (mirror) {
      yaw += M_PI;
      point.pose.orientation.x = 0.0;
      point.pose.orientation.y = 0.0;
      point.pose.orientation.z = std::sin(yaw * 0.5);
      point.pose.orientation.w = std::cos(yaw * 0.5);
    }

    // Sign relative to the vehicle heading, not to the path: retreating along a
    // forward route is reverse gear, but retreating along an already-reversed route is
    // forward gear.  One consistent sign for the whole trajectory, which is what the
    // MPC's single is_forward_shift flag requires.
    const size_t ref = (k == 0) ? 0 : k - 1;  // direction of retreat at this point
    const double dx = reference_path_[ref].position.x - reference_path_[k].position.x;
    const double dy = reference_path_[ref].position.y - reference_path_[k].position.y;
    const double sign = (dx * std::cos(yaw) + dy * std::sin(yaw)) > 0.0 ? 1.0 : -1.0;
    point.longitudinal_velocity_mps =
      static_cast<float>(sign * param_.backoff_speed_mps);
    traj.points.push_back(point);
    if (k == 0) {
      break;
    }
  }

  if (!traj.points.empty()) {
    traj.points.back().longitudinal_velocity_mps = 0.0f;  // stop at the end of the step
  }
  return traj;
}

void StuckRecoverySupervisorNode::relay_recovery_trajectory()
{
  // Single publisher on the topic scenario_selector reads.  Freespace publishes to its
  // own topic and we forward it verbatim -- verbatim matters, because freespace already
  // splits its plan at every direction cusp and waits for the robot to stop there,
  // which is the only reason the MPC can follow a Reeds-Shepp path at all.
  autoware_planning_msgs::msg::Trajectory traj;
  if (state_ == State::BACKOFF) {
    traj = build_backoff_trajectory();
  } else if (state_ == State::RECOVERY && freespace_trajectory_) {
    traj = *freespace_trajectory_;
  } else {
    return;
  }
  if (traj.points.empty()) {
    return;
  }

  clamp_recovery_speed(traj);

  // Emergency brake, applied to EVERYTHING this scenario drives -- the relayed freespace
  // plan and the backoff retreat alike.
  //
  // It has to cover the retreat too.  While this scenario drives, the
  // motion_velocity_planner stop modules are bypassed entirely, and backing up is the
  // one thing here the robot does towards ground it cannot see; a reverse with nothing
  // watching it is the worst case this feature can produce.
  if (recovery_path_is_blocked(traj)) {
    // Zero the velocities but keep the geometry, so the controller still has a reference
    // to hold position against rather than losing its trajectory entirely.
    for (auto & point : traj.points) {
      point.longitudinal_velocity_mps = 0.0f;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[episode %u] recovery AEB in %s: obstacle within %.2f m of the planned path -- holding",
      episode_id_, state_name(state_).c_str(), param_.aeb_lookahead_m);
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

bool StuckRecoverySupervisorNode::plan_is_spent() const
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

  // Less than this ahead of the robot and the smoother will not engage, so the plan
  // cannot restart the robot no matter how long we wait.
  return stop_dist < param_.min_engageable_stop_distance_m;
}

bool StuckRecoverySupervisorNode::plan_is_spent_confirmed()
{
  const auto now = this->now();
  if (!plan_is_spent()) {
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
      RCLCPP_INFO(
        get_logger(), "[episode %u] every escape goal is behind the robot -- manoeuvre done (%s)",
        episode_id_, why.c_str());
      transition(State::COOLDOWN, "drove past the last escape goal");
      return;
    }
    if (!publish_current_goal()) {
      transition(State::COOLDOWN, "no escape goal ahead of the robot is left");
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
  begin_backoff("replanning from this pose keeps failing");
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
  // can.  (Seen in practice: the gear was left in REVERSE after a backoff, forward
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

  // Otherwise ABORT releases itself once the thing that caused it is gone.  Latching it
  // permanently meant that after one failed episode the watchdog was dead for the rest
  // of the session, even with the obstacle long since removed.
  const bool still_blocked = stop_source_ == StuckDiagnosis::SOURCE_OBSTACLE_STOP ||
                             stop_source_ == StuckDiagnosis::SOURCE_STOP_BEHIND;
  const bool moving = !is_stopped_;
  if (still_blocked && !moving) {
    abort_clear_since_ = this->now();
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

  // Keep a stretch BEHIND ego too: that is the geometry the backoff retreats along.
  size_t begin = *nearest;
  double back = 0.0;
  while (begin > 0 && back < param_.backoff_lookback_m) {
    back += distance2d(poses[begin - 1], poses[begin]);
    --begin;
  }

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
  return footprint_is_free_with_margin(pose, param_.goal_shape_margin_m);
}

bool StuckRecoverySupervisorNode::footprint_is_free_with_margin(
  const geometry_msgs::msg::Pose & pose, double margin) const
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
      if (value < 0 || value >= param_.occupancy_threshold) {
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

bool StuckRecoverySupervisorNode::goal_is_ahead(const geometry_msgs::msg::Pose & goal) const
{
  // Compare positions ALONG THE PATH, not against the vehicle heading: on a reversed
  // route the robot faces away from its direction of travel, so a perfectly good goal
  // sits behind its nose.  Arc order is correct in both cases.
  if (!odom_ || reference_path_.size() < 2) {
    return true;
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
    return true;
  }

  const auto nearest = std::min_element(
    traj.points.begin(), traj.points.end(), [this](const auto & a, const auto & b) {
      return distance2d(a.pose, odom_->pose.pose) < distance2d(b.pose, odom_->pose.pose);
    });

  // (a) The plan itself.  A freespace path has been seen running straight through
  // occupied cells -- whether the costmap moved under it or the search produced it that
  // way, driving it is not acceptable either way.
  double arc = 0.0;
  double travel_sign = 0.0;
  for (auto it = nearest; it != traj.points.end(); ++it) {
    if (it != nearest) {
      arc += distance2d(std::prev(it)->pose, it->pose);
    }
    if (travel_sign == 0.0 && std::abs(it->longitudinal_velocity_mps) > 1e-3) {
      travel_sign = it->longitudinal_velocity_mps > 0.0 ? 1.0 : -1.0;
    }
    if (arc < param_.aeb_skip_ahead_m) {
      continue;  // the robot is already here; the planner accepted this clearance
    }
    if (arc > param_.aeb_lookahead_m) {
      break;
    }
    if (!footprint_is_free_with_margin(it->pose, param_.aeb_margin_m)) {
      return true;
    }
  }

  // (b) Where the ROBOT is actually going.  (a) only vouches for the path; if tracking
  // has put the robot off it, the plan can read clear while the robot drives into
  // something.  Sweep straight out from the current pose along the direction of travel,
  // forwards or backwards, taken from the sign of the plan being followed.
  if (travel_sign == 0.0) {
    return false;  // nothing is being commanded; nothing to brake for
  }
  const auto & ego = odom_->pose.pose;
  const double yaw = tf2::getYaw(ego.orientation);
  for (double d = param_.aeb_skip_ahead_m; d <= param_.aeb_lookahead_m; d += 0.1) {
    geometry_msgs::msg::Pose probe = ego;
    probe.position.x += travel_sign * d * std::cos(yaw);
    probe.position.y += travel_sign * d * std::sin(yaw);
    if (!footprint_is_free_with_margin(probe, param_.aeb_margin_m)) {
      return true;
    }
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
  if (!costmap_is_fresh()) {
    return CorridorResult{};  // never reason on a frozen grid
  }
  CorridorParams params;
  params.step_m = param_.probe_step_m;
  params.max_arc_m = param_.corridor_scan_distance_m;
  params.max_scan_half_width_m = param_.max_scan_half_width_m;
  params.vehicle_width_m = vehicle_width_m_;
  params.lateral_margin_m = param_.lateral_margin_m;
  params.occupancy_threshold = static_cast<int8_t>(param_.occupancy_threshold);
  params.mirror_lateral = path_yaw_opposes_ego();
  return sweep_corridor(*grid_, reference_path_, from_index, params);
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

bool StuckRecoverySupervisorNode::current_recovery_segment_is_reverse() const
{
  if (!recovery_trajectory_ || recovery_trajectory_->points.empty()) {
    return false;
  }
  // freespace_planner publishes one single-direction partial trajectory at a time,
  // so the sign of any point in it describes the whole current segment.
  for (const auto & point : recovery_trajectory_->points) {
    if (std::abs(point.longitudinal_velocity_mps) > 1e-3) {
      return point.longitudinal_velocity_mps < 0.0;
    }
  }
  return false;
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
  state_msg.reverse_distance_m = static_cast<float>(reverse_distance_m_);
  state_msg.total_reverse_distance_m = static_cast<float>(total_reverse_distance_m_);
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
  stat.add("reverse_distance_m", reverse_distance_m_);
  stat.add("total_reverse_distance_m", total_reverse_distance_m_);
  stat.add("abort_reason", abort_reason_);

  switch (state_) {
    case State::NOMINAL:
    case State::COOLDOWN:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "nominal");
      break;
    case State::SUSPECT:
    case State::PROBE:
    case State::RECOVERY:
    case State::BACKOFF:
    case State::REPLAN:
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
