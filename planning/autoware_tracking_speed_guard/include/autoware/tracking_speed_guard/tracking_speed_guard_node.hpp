// Copyright 2026 IRISA Robot
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

#ifndef AUTOWARE__TRACKING_SPEED_GUARD__TRACKING_SPEED_GUARD_NODE_HPP_
#define AUTOWARE__TRACKING_SPEED_GUARD__TRACKING_SPEED_GUARD_NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>
#include <string>

namespace autoware::tracking_speed_guard
{

/// Lowers the robot's speed limit while it is following its trajectory badly.
///
/// This node sits beside the trajectory follower rather than inside it.  It subscribes to
/// the same trajectory the controller subscribes to and to the same odometry, computes the
/// same lateral and heading errors the MPC computes against the nearest trajectory point,
/// and -- unlike the MPC, which cannot influence speed at all -- turns those two numbers
/// into a speed cap.  It never touches steering, the trajectory, or any planner.
class TrackingSpeedGuardNode : public rclcpp::Node
{
public:
  explicit TrackingSpeedGuardNode(const rclcpp::NodeOptions & options);
  ~TrackingSpeedGuardNode() override;

private:
  struct Param
  {
    double update_rate_hz{10.0};

    // --- error thresholds ---------------------------------------------------
    // Below the _ok value the error costs nothing; at the _bad value the cap is all the
    // way down at v_floor_mps; in between it is interpolated.  The pair defines the ramp,
    // which is what makes the response gradual rather than a cliff the robot falls off.
    double lat_ok_m{0.3};
    double lat_bad_m{1.0};
    double yaw_ok_rad{0.17};   // 10 deg
    double yaw_bad_rad{0.52};  // 30 deg

    // --- speed range --------------------------------------------------------
    // Ceiling. Keep equal to max_vel in common.param.yaml; publishing anything higher is
    // pointless because the selector already caps there.
    double v_free_mps{1.8};
    // Floor.  Deliberately not lower than 1 km/h: below roughly this the velocity smoother
    // refuses to engage at all (stop_dist_to_prohibit_engage), which looks exactly like a
    // robot frozen for no reason -- a failure this stack has already produced once.
    double v_floor_mps{0.278};

    // How long both errors must stay in the clear before the limit is released.  Stops the
    // cap flickering on and off around the threshold.
    double clear_hold_sec{2.0};

    // Inputs older than this are treated as missing: better to release the limit than to
    // hold the robot slow on the strength of a stale measurement.
    double input_timeout_sec{1.0};

    // Act only while Autoware is actually driving the robot.
    bool require_autonomous{true};

    std::string sender{"tracking_speed_guard"};

    std::string topic_trajectory{"/planning/trajectory"};
    std::string topic_odometry{"/localization/kinematic_state"};
    std::string topic_operation_mode{"/api/operation_mode/state"};
    std::string topic_out_velocity_limit{"/planning/scenario_planning/max_velocity_candidates"};
    std::string topic_out_clear_command{"/planning/scenario_planning/clear_velocity_limit"};
    std::string topic_out_debug{"~/debug/tracking_error"};
  };

  /// What the guard made of the current tick.
  struct Assessment
  {
    bool valid{false};       ///< inputs present, fresh, and a usable trajectory
    double lateral_m{0.0};   ///< signed lateral offset from the trajectory
    double yaw_rad{0.0};     ///< signed heading error against the direction of travel
    double mu{0.0};          ///< 0 = tracking fine, 1 = as bad as the thresholds describe
    double cap_mps{0.0};     ///< the speed limit that follows from mu
    bool travelling_back{false};
  };

  void on_timer();
  Assessment assess() const;
  /// Membership of "this error is bad", 0 at ok and 1 at bad, linear in between.
  static double membership(double value, double ok, double bad);
  void publish_limit(double cap_mps);
  void publish_clear();
  void publish_debug(const Assessment & a) const;
  void on_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  Param param_;

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_{};
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    sub_operation_mode_{};

  rclcpp::Publisher<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr pub_limit_{};
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::VelocityLimitClearCommand>::SharedPtr
    pub_clear_{};
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr
    pub_debug_{};

  rclcpp::TimerBase::SharedPtr timer_{};
  diagnostic_updater::Updater diagnostics_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_{};
  rclcpp::Time trajectory_at_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_{};
  rclcpp::Time odometry_at_{0, 0, RCL_ROS_TIME};
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode_{};

  /// Direction of travel, kept from the last tick that had any motion commanded.  The MPC
  /// does the same (m_is_forward_shift): a trajectory of all-zero velocities -- the robot
  /// standing at a stop point -- says nothing about which way it is about to go, and
  /// guessing "forward" there would read a reverse mission as 180 degrees of heading error.
  bool travelling_back_{false};

  /// True while our limit is in force.  The selector holds limits per sender, so it stays
  /// until we clear it; this flag is what stops us clearing a limit we never set.
  bool limit_active_{false};
  /// One clear is sent on the first tick.  A limit we set outlives us -- the selector holds
  /// it per sender until told otherwise -- so after a crash and respawn the robot would
  /// still be capped by a limit nothing is maintaining any more.
  bool startup_clear_sent_{false};
  rclcpp::Time clear_since_{0, 0, RCL_ROS_TIME};
  Assessment last_{};
};

}  // namespace autoware::tracking_speed_guard

#endif  // AUTOWARE__TRACKING_SPEED_GUARD__TRACKING_SPEED_GUARD_NODE_HPP_
