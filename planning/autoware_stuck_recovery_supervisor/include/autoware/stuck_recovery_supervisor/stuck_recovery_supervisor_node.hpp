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

#ifndef AUTOWARE__STUCK_RECOVERY_SUPERVISOR__STUCK_RECOVERY_SUPERVISOR_NODE_HPP_
#define AUTOWARE__STUCK_RECOVERY_SUPERVISOR__STUCK_RECOVERY_SUPERVISOR_NODE_HPP_

#include "autoware/stuck_recovery_supervisor/corridor_checker.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_internal_planning_msgs/msg/scenario.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_stuck_recovery_msgs/msg/recovery_state.hpp>
#include <autoware_stuck_recovery_msgs/msg/stuck_diagnosis.hpp>
#include <autoware_stuck_recovery_msgs/srv/force_recovery.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::stuck_recovery_supervisor
{

// ---------------------------------------------------------------------------
// FSM state.
//
// Values are stable -- do NOT renumber.  They are mirrored in RecoveryState.msg,
// plotted straight out of bags, and shown in the RViz overlay.
//
//   NOMINAL  = 0   nothing to see here
//   SUSPECT  = 1   ego is stopped for no legitimate reason; timer running
//   PROBE    = 2   costmap armed; measuring the lateral corridor
//   RECOVERY = 3   recovery scenario active; freespace is driving
//   BLOCKED  = 4   corridor too narrow -- stay stopped, raise diagnostics
//   COOLDOWN = 5   recovered; refuse to re-trigger for a while
//   ABORT    = 6   latched failure; needs an operator
// ---------------------------------------------------------------------------
enum class State : int32_t {
  NOMINAL = 0,
  SUSPECT = 1,
  PROBE = 2,
  RECOVERY = 3,
  BLOCKED = 4,
  COOLDOWN = 5,
  ABORT = 6,
  // Freespace cannot plan from where the robot is standing -- usually because its own
  // footprint is inside the inflated obstacle.  Retreat a step along the path we came
  // from, then let freespace try again from further back.  Alternating BACKOFF and
  // RECOVERY is what lets the robot shuffle back and forth until it gets past.
  BACKOFF = 7,
  // The robot stopped past the live part of freespace's plan, so the nearest point to
  // it already has zero velocity.  Nothing restarts on its own from there: the smoother
  // reads target_vel = 0, and freespace does not consider a replan necessary because
  // the robot is still close to the trajectory (well inside th_course_out_distance_m).
  // Re-publishing the route forces freespace to reset and plan afresh from where the
  // robot actually is.
  REPLAN = 8,
};

/// How far the supervisor is allowed to go.  Set from the `mode` parameter so the
/// rollout stages can be selected from a launch argument without a rebuild.
enum class Mode : uint8_t {
  DETECT_ONLY = 0,  // observe and publish only; never acts
  PROBE_ONLY = 1,   // additionally arms the costmap and measures the corridor
  RECOVERY = 2,     // full behaviour
};

class StuckRecoverySupervisorNode : public rclcpp::Node
{
public:
  explicit StuckRecoverySupervisorNode(const rclcpp::NodeOptions & options);

private:
  // ------------------------------------------------------------------
  // Parameters (loaded once in the constructor, then clamped)
  // ------------------------------------------------------------------
  struct Param
  {
    double update_rate_hz{10.0};
    std::string mode{"detect_only"};

    // detection
    double th_stopped_velocity_mps{0.05};
    double unstuck_progress_m{0.5};
    double suspect_timeout_sec{12.0};
    double goal_reached_dist_m{1.0};
    double map_stop_patience_sec{90.0};
    // A STOP planning factor only counts as a blockage if its control point is this
    // close.  Without it, a robot stopped for some other reason while an obstacle_stop
    // factor sits 20 m ahead would be misclassified as blocked and eventually recover
    // for no reason.  Being stopped already implies we are at the stop point, so this
    // only has to reject the far-away case.
    double th_stop_factor_distance_m{3.0};
    double th_commanded_velocity_mps{0.15};
    double mechanical_confirm_sec{2.0};

    // probe / corridor
    double probe_warmup_sec{1.0};
    double probe_step_m{0.25};
    double lateral_margin_m{0.15};
    double max_scan_half_width_m{4.0};
    int occupancy_threshold{50};

    // recovery
    // How far ahead the corridor question is asked (PROBE and the RECOVERY exit test).
    double corridor_scan_distance_m{8.0};
    // Goal-candidate search: start here, step by this interval, give up at this range.
    // Floor for the goal search.  The search normally starts from the obstacle instead
    // (see goal_offset_beyond_obstacle_m); this is only the minimum.
    double min_long_distance_to_check{3.0};
    // How far PAST the nearest blockage the first goal candidate is placed.  Shooting a
    // fixed distance from the robot could put the first goal on the near side of the
    // obstacle, where freespace has no reason to go around it at all.
    double goal_offset_beyond_obstacle_m{2.0};
    double goal_check_interval_m{1.0};
    // A goal is only placed where the path has already been clear for this long.  A
    // single free footprint is not enough: it can sit wedged a handful of centimetres
    // past the obstacle, which is where the robot was being sent.
    double goal_min_clear_length_m{2.0};
    double max_goal_search_distance_m{30.0};
    // Mirror of vehicle_shape_margin_m in the recovery freespace profile.  The escape
    // goal is validated against the same inflated footprint freespace will use, so we
    // never hand it a goal it is going to reject outright.
    double goal_shape_margin_m{0.2};
    double recovery_timeout_sec{60.0};
    double clear_hold_sec{1.0};
    // Recovery may only hand back once the robot has REJOINED the path.  Judging the exit
    // purely on "the corridor ahead is clear" let it hand back while still standing beside
    // the obstacle: the robot's projection onto the path had already advanced past the
    // blockage, so the sweep looked clear, while lane driving -- which drives the path
    // itself -- still saw the obstacle and stopped again.  That disagreement is what made
    // the state flip between Recovery and LaneDriving indefinitely.
    double rejoin_lateral_tolerance_m{0.5};

    // Recovery's own emergency brake.  The relayed plan was computed against an older
    // costmap; if the world has since put something in it, stop rather than drive it.
    double aeb_lookahead_m{1.5};
    double aeb_margin_m{0.05};
    // Ignore anything this close: the robot is already there, and the planner accepted
    // that clearance when it produced the plan.  Without this the brake latches on the
    // inflation around the robot and it can never move at all.
    double aeb_skip_ahead_m{0.3};
    // How far the robot may be from the plan it is following before the plan stops
    // describing where it is going and the brake goes on.
    double aeb_max_deviation_m{0.6};
    // Occupancy value at which the BRAKE considers a cell solid.  Must match the
    // recovery freespace planner's obstacle_threshold: braking on cells the planner
    // ignores means overruling a plan it just validated on the same grid, which is a
    // deadlock, not a safety net.
    int aeb_occupancy_threshold{100};
    // A brake that has been on this long is not protecting against a passing hazard --
    // the plan is no longer drivable, so replan instead of holding until the episode
    // times out.  That deadlock is exactly what a whole 60 s episode was lost to.
    double aeb_hold_replan_sec{3.0};
    double max_reverse_distance_m{10.0};
    // Cumulative reverse budget across episodes.  The per-episode limit above cannot
    // bound an obstacle that keeps advancing on the robot: every new episode hands out a
    // fresh allowance, so the robot could retreat indefinitely.  This one is cleared only
    // by real forward progress toward the goal, so "retreat without ever getting
    // anywhere" terminates.
    double max_total_reverse_distance_m{15.0};
    double total_reverse_reset_progress_m{5.0};
    double max_lateral_excursion_m{2.0};

    // Backoff: retreat along the path we came from at a crawl.  Stepped rather than
    // continuous, because freespace only replans once the vehicle has stopped
    // (freespace_planner_node.cpp: "Waiting for the vehicle to stop ...").
    // Hard speed ceiling for EVERYTHING this scenario drives -- the relayed freespace
    // plan as much as the backoff.  Applied in the relay because the supervisor is the
    // single publisher there, so it cannot be bypassed; the external velocity-limit
    // channel is no good for this since it can only ever lower a limit, and measurements
    // showed the trajectory reaching control at anything from 0.250 to 0.303 m/s.
    double recovery_speed_limit_mps{0.278};
    // Must stay above the smoother's stop_dist_to_prohibit_engage (0.2 m).  Purely for
    // the warning below -- it explains a robot that refuses to move for no visible
    // reason, which is otherwise a very expensive thing to work out.
    double min_engageable_stop_distance_m{0.3};
    double backoff_speed_mps{0.278};
    // Retreat is triggered after this many failed goal attempts, and its distance is
    // this count x goal_check_interval_m -- i.e. the robot gives back exactly as much
    // ground as it worked through in the candidate list, then tries again from there.
    int backoff_after_failed_goals{2};
    // The retreat counts as done once the robot is within this of the target, rather
    // than demanding it land exactly: a crawl through the smoother never stops on the
    // millimetre, and waiting for that is how a retreat hangs.
    double backoff_reach_tolerance_m{0.3};
    double backoff_lookback_m{15.0};
    // How long to wait for a usable plan before moving on to the next goal candidate.
    double goal_retry_sec{4.0};
    // Forced-replan cycle: how long to wait for freespace to come back with a plan, and
    // how many times to ask before falling back to retreating.
    double replan_wait_sec{5.0};
    // The plan must look spent for THIS long, continuously, before a replan is forced.
    // Evaluating it instantaneously caused a RECOVERY <-> REPLAN oscillation: freespace
    // publishes a short partial for a tick, we declare the plan spent, and the forced
    // replan re-publishes the route -- which makes freespace reset() and throw away the
    // plan it had just produced.  The dwell also gives the robot time to actually drive
    // the segment before we conclude it cannot.
    double spent_hold_sec{2.0};
    // If the robot has covered at least this much ground since the last retreat, the
    // manoeuvre is making progress and the right response to a failure is to replan from
    // here -- NOT to retreat again and throw that progress away.  Without this, coming up
    // a little short of clearing the obstacle, or a brief AEB stop, restarted the whole
    // manoeuvre from the beginning.
    double backoff_progress_reset_m{0.5};
    // A freespace plan is only trusted if it arrived AFTER this episode's route was
    // published, and recently.  Both matter: /planning/recovery/route is transient_local
    // (it has to be -- freespace subscribes latched), so a goal from a previous episode
    // survives on the topic and freespace can answer it the moment it is re-armed.
    // Relaying that answer once sent the robot to a previous scenario's goal.
    double freespace_plan_timeout_sec{1.0};
    // The recovery costmap must be FRESH.  Holding on to the last grid received meant
    // that when the recovery container died the supervisor carried on sweeping a frozen
    // snapshot -- reporting an unchanging clear corridor and exiting recovery instantly,
    // with nothing to indicate the planner was gone.
    double costmap_timeout_sec{1.0};
    int max_replan_attempts{3};

    // budget
    int max_attempts{2};
    double cooldown_sec{60.0};
    double max_episode_duration_sec{180.0};
    // ABORT is not a dead end.  Once the blockage that caused it is gone, the watchdog
    // has to come back to life -- latching it forever left the feature dead for the rest
    // of the session even after the obstacle had been removed.  A mechanical stall is
    // the exception: that one stays latched, because it needs a human to look at it.
    double abort_release_sec{5.0};

    // classification inputs
    std::vector<std::string> pathological_factor_topics{};
    std::vector<std::string> legit_factor_topics{};

    // topics
    std::string topic_odometry;
    std::string topic_velocity_status;
    std::string topic_control_cmd;
    std::string topic_trajectory;
    std::string topic_route;
    std::string topic_route_state;
    std::string topic_operation_mode;
    std::string topic_control_mode;
    std::string topic_gear;
    std::string topic_scenario;
    std::string topic_occupancy_grid;
    std::string topic_recovery_trajectory;
    std::string topic_freespace_trajectory;
    std::string topic_recovery_is_completed;
    std::string topic_road_crossing_state;
    std::string topic_emergency_code;
    std::string topic_robot_fsm;
    std::string topic_out_scenario;
    std::string topic_out_route;
    std::string topic_out_force_recovery;
    std::string topic_out_state;
    std::string topic_out_state_int;
    std::string topic_out_diagnosis;
    std::string topic_out_markers;
    std::string topic_out_escape_goal;
    std::string topic_out_recovery_trajectory;
    std::string service_force_recovery;

    std::vector<int64_t> road_crossing_hold_states{};
  };

  // ------------------------------------------------------------------
  // Lifecycle
  // ------------------------------------------------------------------
  void load_parameters();
  void setup_interfaces();
  void on_timer();

  // FSM steps
  void step_nominal();
  void step_suspect();
  void step_probe();
  void step_recovery();
  void step_blocked();
  void step_cooldown();
  void step_abort();
  void step_backoff();
  void step_replan();
  void force_freespace_replan(const std::string & why);
  /// True when the robot is stopped and there is no usable velocity left ahead of it in
  /// the plan being relayed -- i.e. the plan is spent and only a replan can help.
  bool plan_is_spent() const;
  /// plan_is_spent() held continuously for spent_hold_sec, and long enough after entering
  /// the state that the robot had a fair chance to drive.  Only this may force a replan.
  bool plan_is_spent_confirmed();
  void begin_backoff(const std::string & why);
  void transition(State next, const std::string & reason);

  // Detection helpers
  void update_motion();
  void classify_stop();
  /// Track the closest approach to the goal, and clear the cumulative reverse budget when
  /// the robot has genuinely advanced.
  void update_goal_progress();
  bool is_operational() const;
  bool is_stuck_candidate() const;

  // Recovery helpers
  bool capture_reference_path();
  std::optional<geometry_msgs::msg::Pose> compute_escape_goal() const;
  CorridorResult sweep(size_t from_index) const;
  /// Is the recovery costmap present AND recent?  A stale grid is worse than none.
  bool costmap_is_fresh() const;
  /// True on a reversed route, where the path yaw is the travel direction and the
  /// robot faces the other way -- so lateral left/right has to be mirrored.
  bool path_yaw_opposes_ego() const;
  /// Is the inflated vehicle footprint at `pose` clear of the recovery grid?  Off-grid
  /// counts as blocked, which also keeps the goal inside the costmap window.
  bool footprint_is_free(const geometry_msgs::msg::Pose & pose) const;
  /// Same test with a chosen margin, so the emergency brake can be stricter about what
  /// counts as a collision than the goal search is about where it may aim.
  bool footprint_is_free_with_margin(
    const geometry_msgs::msg::Pose & pose, double margin, int threshold) const;
  /// Lateral distance from the robot to the reference path; the measure of "has rejoined".
  double lateral_offset_from_path() const;
  /// Is `goal` still ahead of the robot ALONG THE PATH?  Measured by arc position, not by
  /// the vehicle heading: on a reversed route the goal is legitimately behind the nose.
  bool goal_is_ahead(const geometry_msgs::msg::Pose & goal) const;
  /// Advance goal_index_ past any candidate the robot has already passed.  Returns false
  /// when none are left ahead.
  bool advance_past_passed_goals();
  /// Would following this plan drive into something the costmap now shows?
  bool recovery_path_is_blocked(const autoware_planning_msgs::msg::Trajectory & traj);
  /// Is the pose inside the recovery costmap window at all?  Used only to explain, in
  /// the log, why candidates were skipped.
  bool pose_is_on_grid(const geometry_msgs::msg::Pose & pose) const;
  void publish_recovery_arming(bool active);
  void publish_recovery_route(const geometry_msgs::msg::Pose & goal);
  bool current_recovery_segment_is_reverse() const;
  /// Ordered escape-goal candidates, nearest usable first.
  std::vector<geometry_msgs::msg::Pose> collect_goal_candidates() const;
  bool publish_current_goal();
  /// Does the relayed freespace output actually move the robot, or is it one of the
  /// stop trajectories freespace emits while it is failing?
  bool freespace_plan_is_valid();
  /// Forget every trace of the recovery scenario: the held plan, the goal, the candidate
  /// list and the relayed trajectory.  Called whenever the scenario is left or a new
  /// route is published, so nothing from one episode can act in another.
  void discard_recovery_state();
  /// Reverse trajectory back along the path we came from, at backoff_speed_mps.
  autoware_planning_msgs::msg::Trajectory build_backoff_trajectory() const;
  void relay_recovery_trajectory();
  /// Hard-limit every point's speed magnitude, keeping its sign.  Applies to the
  /// relayed freespace plan and to the backoff alike.
  void clamp_recovery_speed(autoware_planning_msgs::msg::Trajectory & traj) const;


  // Output
  void publish_outputs();
  void publish_markers();
  void on_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void on_force_recovery(
    const autoware_stuck_recovery_msgs::srv::ForceRecovery::Request::SharedPtr request,
    const autoware_stuck_recovery_msgs::srv::ForceRecovery::Response::SharedPtr response);

  // ------------------------------------------------------------------
  // State
  // ------------------------------------------------------------------
  Param param_{};
  Mode mode_{Mode::DETECT_ONLY};
  State state_{State::NOMINAL};
  std::string abort_reason_{};

  rclcpp::Time state_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time episode_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stopped_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time commanded_but_still_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time clear_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cooldown_until_{0, 0, RCL_ROS_TIME};
  bool is_stopped_{false};
  bool commanded_but_still_{false};
  bool corridor_clear_{false};

  uint32_t episode_id_{0};
  // Stop source this episode was opened on.  STOP_BEHIND needs a different exit test
  // from a normal blockage: its corridor reads clear from the very first tick.
  uint8_t episode_stop_source_{0};
  // When the emergency brake first went on, to tell a passing hazard from a dead plan.
  rclcpp::Time aeb_hold_since_{0, 0, RCL_ROS_TIME};
  bool aeb_holding_{false};
  std::string aeb_reason_;
  uint16_t attempts_{0};

  geometry_msgs::msg::Pose entry_pose_{};
  geometry_msgs::msg::Pose last_pose_{};
  bool has_last_pose_{false};
  double progress_since_entry_m_{0.0};
  double reverse_distance_m_{0.0};
  double total_reverse_distance_m_{0.0};
  /// Closest the robot has ever been to the goal.  Progress is measured against this, so
  /// that reversing and re-approaching the same spot does not count as getting anywhere.
  double best_remaining_to_goal_m_{std::numeric_limits<double>::max()};

  std::vector<geometry_msgs::msg::Pose> reference_path_{};
  std::optional<geometry_msgs::msg::Pose> escape_goal_{};
  std::vector<geometry_msgs::msg::Pose> goal_candidates_{};
  size_t goal_index_{0};
  rclcpp::Time last_valid_plan_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Pose backoff_start_pose_{};
  double backoff_target_m_{0.0};
  /// Index in reference_path_ the retreat ends at.  Pinned once when the backoff
  /// begins: recomputing it from the moving robot each tick made the end point run
  /// away ahead of it, so the robot reversed forever instead of stopping.
  size_t backoff_end_index_{0};
  int failed_goal_tries_{0};
  int replan_attempts_{0};
  bool start_blocked_{false};
  bool abort_was_mechanical_{false};
  rclcpp::Time abort_clear_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time spent_since_{0, 0, RCL_ROS_TIME};
  /// Where the robot was when the last retreat began (or where the episode started).
  /// Distance from here is what "progress since the last retreat" means.
  geometry_msgs::msg::Pose pose_at_last_backoff_{};
  CorridorResult last_sweep_{};

  uint8_t stop_source_{0};
  std::string stop_source_module_{};
  bool legit_stop_{false};
  std::string legit_reason_{};
  rclcpp::Time map_stop_since_{0, 0, RCL_ROS_TIME};
  bool map_stop_active_{false};

  // ------------------------------------------------------------------
  // Latest inputs
  // ------------------------------------------------------------------
  nav_msgs::msg::Odometry::ConstSharedPtr odom_{};
  autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr velocity_{};
  autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_{};
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_{};
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_{};
  autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr route_state_{};
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode_{};
  autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr control_mode_{};
  autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr gear_{};
  autoware_internal_planning_msgs::msg::Scenario::ConstSharedPtr scenario_{};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_{};
  rclcpp::Time grid_at_{0, 0, RCL_ROS_TIME};
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr recovery_trajectory_{};
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr freespace_trajectory_{};
  /// When the current episode's route was published, and when the freespace plan we hold
  /// arrived.  A plan that predates the route belongs to an older goal.
  rclcpp::Time route_published_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time freespace_traj_at_{0, 0, RCL_ROS_TIME};
  bool recovery_completed_{false};
  autoware_internal_debug_msgs::msg::Int32Stamped::ConstSharedPtr road_crossing_state_{};
  uint8_t emergency_code_{0};
  int16_t robot_fsm_{-1};
  bool has_robot_fsm_{false};
  std::map<std::string, autoware_internal_planning_msgs::msg::PlanningFactorArray::ConstSharedPtr>
    planning_factors_{};

  // ------------------------------------------------------------------
  // Interfaces
  // ------------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr timer_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_{};
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_{};
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_cmd_{};
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_{};
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_{};
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_route_state_{};
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_op_mode_{};
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr sub_control_mode_{};
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr sub_gear_{};
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::Scenario>::SharedPtr sub_scenario_{};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_{};
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_recovery_traj_{};
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_freespace_traj_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_recovery_completed_{};
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr
    sub_road_crossing_{};
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_emergency_{};
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_robot_fsm_{};
  std::vector<rclcpp::Subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>::
                SharedPtr>
    sub_factors_{};

  rclcpp::Publisher<autoware_internal_planning_msgs::msg::Scenario>::SharedPtr pub_scenario_{};
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr pub_route_{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_force_recovery_{};
  rclcpp::Publisher<autoware_stuck_recovery_msgs::msg::RecoveryState>::SharedPtr pub_state_{};
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_state_int_{};
  rclcpp::Publisher<autoware_stuck_recovery_msgs::msg::StuckDiagnosis>::SharedPtr pub_diagnosis_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_{};
  /// The goal actually handed to freespace, as a plain PoseStamped so it can be shown
  /// with an RViz Pose display or echoed straight from the command line.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_escape_goal_{};
  /// The recovery trajectory scenario_selector consumes.  The supervisor is the single
  /// publisher here: it relays freespace's plan, or its own backoff trajectory.
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_recovery_traj_{};

  rclcpp::Service<autoware_stuck_recovery_msgs::srv::ForceRecovery>::SharedPtr srv_force_{};

  std::unique_ptr<diagnostic_updater::Updater> diagnostics_{};

  double vehicle_width_m_{0.78};
  double base_to_front_m_{0.9};
  double base_to_rear_m_{0.3};
  bool force_requested_{false};
};

}  // namespace autoware::stuck_recovery_supervisor

#endif  // AUTOWARE__STUCK_RECOVERY_SUPERVISOR__STUCK_RECOVERY_SUPERVISOR_NODE_HPP_
