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

#include "autoware/tracking_speed_guard/tracking_speed_guard_node.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/math/normalization.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace autoware::tracking_speed_guard
{
namespace
{
/// Clamp with a warning, so a typo in a yaml cannot quietly produce a nonsense ramp.
double clamped(
  const rclcpp::Logger & logger, double value, double lo, double hi, const char * name)
{
  const double out = std::clamp(value, lo, hi);
  if (std::abs(out - value) > 1e-9) {
    RCLCPP_WARN(logger, "%s = %.3f is out of [%.3f, %.3f]; using %.3f", name, value, lo, hi, out);
  }
  return out;
}
}  // namespace

TrackingSpeedGuardNode::TrackingSpeedGuardNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("tracking_speed_guard", options), diagnostics_(this)
{
  auto & p = param_;
  p.update_rate_hz = declare_parameter<double>("update_rate_hz", 10.0);
  p.lat_ok_m = declare_parameter<double>("lat_ok_m", 0.3);
  p.lat_bad_m = declare_parameter<double>("lat_bad_m", 1.0);
  p.yaw_ok_rad = declare_parameter<double>("yaw_ok_rad", 0.17);
  p.yaw_bad_rad = declare_parameter<double>("yaw_bad_rad", 0.52);
  p.v_free_mps = declare_parameter<double>("v_free_mps", 1.8);
  p.v_floor_mps = declare_parameter<double>("v_floor_mps", 0.278);
  p.clear_hold_sec = declare_parameter<double>("clear_hold_sec", 2.0);
  p.input_timeout_sec = declare_parameter<double>("input_timeout_sec", 1.0);
  p.require_autonomous = declare_parameter<bool>("require_autonomous", true);
  p.sender = declare_parameter<std::string>("sender", p.sender);
  p.topic_trajectory = declare_parameter<std::string>("topic_trajectory", p.topic_trajectory);
  p.topic_odometry = declare_parameter<std::string>("topic_odometry", p.topic_odometry);
  p.topic_operation_mode =
    declare_parameter<std::string>("topic_operation_mode", p.topic_operation_mode);
  p.topic_out_velocity_limit =
    declare_parameter<std::string>("topic_out_velocity_limit", p.topic_out_velocity_limit);
  p.topic_out_clear_command =
    declare_parameter<std::string>("topic_out_clear_command", p.topic_out_clear_command);
  p.topic_out_debug = declare_parameter<std::string>("topic_out_debug", p.topic_out_debug);

  const auto logger = get_logger();
  p.v_floor_mps = clamped(logger, p.v_floor_mps, 0.05, 2.0, "v_floor_mps");
  p.v_free_mps = clamped(logger, p.v_free_mps, p.v_floor_mps, 20.0, "v_free_mps");
  p.lat_ok_m = clamped(logger, p.lat_ok_m, 0.0, 10.0, "lat_ok_m");
  p.lat_bad_m = clamped(logger, p.lat_bad_m, p.lat_ok_m + 0.01, 20.0, "lat_bad_m");
  p.yaw_ok_rad = clamped(logger, p.yaw_ok_rad, 0.0, M_PI, "yaw_ok_rad");
  p.yaw_bad_rad = clamped(logger, p.yaw_bad_rad, p.yaw_ok_rad + 0.01, M_PI, "yaw_bad_rad");
  p.update_rate_hz = clamped(logger, p.update_rate_hz, 1.0, 50.0, "update_rate_hz");

  const rclcpp::QoS qos{1};
  // The selector's inputs are transient_local, so a limit survives for a subscriber that
  // joins late -- which is also why a limit we set outlives us if we go away without
  // clearing it.
  const rclcpp::QoS latched = rclcpp::QoS{10}.transient_local();

  sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    p.topic_trajectory, qos,
    [this](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
      trajectory_ = msg;
      trajectory_at_ = this->now();
    });
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    p.topic_odometry, qos, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      odometry_ = msg;
      odometry_at_ = this->now();
    });
  sub_operation_mode_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    p.topic_operation_mode, rclcpp::QoS{1}.transient_local(),
    [this](const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg) {
      operation_mode_ = msg;
    });

  pub_limit_ = create_publisher<autoware_internal_planning_msgs::msg::VelocityLimit>(
    p.topic_out_velocity_limit, latched);
  pub_clear_ = create_publisher<autoware_internal_planning_msgs::msg::VelocityLimitClearCommand>(
    p.topic_out_clear_command, latched);
  pub_debug_ = create_publisher<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>(
    p.topic_out_debug, qos);

  clear_since_ = this->now();
  diagnostics_.setHardwareID("tracking_speed_guard");
  diagnostics_.add("tracking_speed_guard", this, &TrackingSpeedGuardNode::on_diagnostics);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / p.update_rate_hz), [this]() { on_timer(); });

  RCLCPP_INFO(
    logger,
    "tracking speed guard: lateral %.2f->%.2f m, heading %.0f->%.0f deg, cap %.2f->%.2f m/s, "
    "sender '%s'",
    p.lat_ok_m, p.lat_bad_m, p.yaw_ok_rad * 180.0 / M_PI, p.yaw_bad_rad * 180.0 / M_PI,
    p.v_free_mps, p.v_floor_mps, p.sender.c_str());
}

TrackingSpeedGuardNode::~TrackingSpeedGuardNode()
{
  // Leave nothing behind.  A limit we set stays in the selector's table until someone
  // clears it, so going away quietly would leave the robot slow with no visible cause.
  if (limit_active_) {
    publish_clear();
  }
}

double TrackingSpeedGuardNode::membership(double value, double ok, double bad)
{
  if (bad <= ok) {
    return value > ok ? 1.0 : 0.0;
  }
  return std::clamp((value - ok) / (bad - ok), 0.0, 1.0);
}

TrackingSpeedGuardNode::Assessment TrackingSpeedGuardNode::assess() const
{
  Assessment a;
  a.travelling_back = travelling_back_;

  if (!trajectory_ || !odometry_ || trajectory_->points.size() < 2) {
    return a;
  }
  const auto now = this->now();
  if (
    (now - trajectory_at_).seconds() > param_.input_timeout_sec ||
    (now - odometry_at_).seconds() > param_.input_timeout_sec) {
    return a;  // stale inputs: say nothing rather than hold the robot on old numbers
  }

  const auto & points = trajectory_->points;
  const auto & ego = odometry_->pose.pose;
  const size_t nearest = autoware::motion_utils::findNearestIndex(points, ego.position);

  // Direction of travel, from the velocity sign near the closest point.  This is what the
  // MPC does, and it is what makes one heading-error definition work for both motion
  // modes: in a reverse mission the robot faces away from the way it is going, so the
  // reference heading is the path heading turned around.
  bool back = travelling_back_;
  for (size_t i = nearest; i < points.size(); ++i) {
    if (std::abs(points[i].longitudinal_velocity_mps) > 1e-2) {
      back = points[i].longitudinal_velocity_mps < 0.0;
      break;
    }
  }
  a.travelling_back = back;

  a.lateral_m = autoware::motion_utils::calcLateralOffset(points, ego.position);
  const double ref_yaw = tf2::getYaw(points[nearest].pose.orientation) + (back ? M_PI : 0.0);
  a.yaw_rad = autoware_utils::normalize_radian(tf2::getYaw(ego.orientation) - ref_yaw);

  if (!std::isfinite(a.lateral_m) || !std::isfinite(a.yaw_rad)) {
    return a;  // a.valid stays false
  }

  const double mu_lat = membership(std::abs(a.lateral_m), param_.lat_ok_m, param_.lat_bad_m);
  const double mu_yaw = membership(std::abs(a.yaw_rad), param_.yaw_ok_rad, param_.yaw_bad_rad);
  // The worse of the two decides.  Averaging them would let a robot pointing 30 degrees
  // wrong carry on at speed just because it happens to be sitting on the line.
  a.mu = std::max(mu_lat, mu_yaw);
  a.cap_mps = param_.v_free_mps - a.mu * (param_.v_free_mps - param_.v_floor_mps);
  a.valid = true;
  return a;
}

void TrackingSpeedGuardNode::on_timer()
{
  const auto now = this->now();

  // Clear once on the way up.  If a previous instance of this node set a limit and then
  // died, the selector is still holding it under our sender name and nothing else will
  // ever take it off.
  if (!startup_clear_sent_) {
    startup_clear_sent_ = true;
    publish_clear();
  }

  const bool autonomous =
    !param_.require_autonomous ||
    (operation_mode_ &&
     operation_mode_->mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS &&
     operation_mode_->is_autoware_control_enabled);

  const auto a = assess();
  last_ = a;
  travelling_back_ = a.travelling_back;
  publish_debug(a);

  // Nothing to say: no usable measurement, or we are not the one driving.  Release rather
  // than hold -- a limit we cannot justify any more must not outlive the reason for it.
  if (!a.valid || !autonomous) {
    if (limit_active_) {
      publish_clear();
      RCLCPP_INFO(
        get_logger(), "releasing the speed cap: %s",
        !autonomous ? "not under autonomous control" : "no usable trajectory or odometry");
    }
    clear_since_ = now;
    return;
  }

  if (a.mu > 0.0) {
    clear_since_ = now;
    publish_limit(a.cap_mps);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "tracking error lateral %.2f m, heading %.1f deg -> capping speed at %.2f m/s",
      a.lateral_m, a.yaw_rad * 180.0 / M_PI, a.cap_mps);
    return;
  }

  // Tracking is fine.  Wait out the hold before letting go, so the cap does not flicker
  // while the robot settles around the threshold.
  if (limit_active_ && (now - clear_since_).seconds() >= param_.clear_hold_sec) {
    publish_clear();
    RCLCPP_INFO(
      get_logger(), "tracking is back within tolerance for %.1f s; speed cap released",
      param_.clear_hold_sec);
  }
}

void TrackingSpeedGuardNode::publish_limit(double cap_mps)
{
  autoware_internal_planning_msgs::msg::VelocityLimit msg;
  msg.stamp = this->now();
  msg.max_velocity = static_cast<float>(cap_mps);
  // No constraints: let the smoother brake with its own configured comfort limits.  Naming
  // ourselves matters -- the selector keeps one limit per sender and applies the smallest,
  // so this composes with the obstacle and MRM limits instead of fighting them.
  msg.use_constraints = false;
  msg.sender = param_.sender;
  pub_limit_->publish(msg);
  limit_active_ = true;
}

void TrackingSpeedGuardNode::publish_clear()
{
  autoware_internal_planning_msgs::msg::VelocityLimitClearCommand msg;
  msg.stamp = this->now();
  msg.command = true;
  msg.sender = param_.sender;
  pub_clear_->publish(msg);
  limit_active_ = false;
}

void TrackingSpeedGuardNode::publish_debug(const Assessment & a) const
{
  autoware_internal_debug_msgs::msg::Float64MultiArrayStamped msg;
  msg.stamp = this->now();
  msg.data = {a.lateral_m,           a.yaw_rad, a.mu, a.cap_mps,
              a.valid ? 1.0 : 0.0,   a.travelling_back ? 1.0 : 0.0};
  pub_debug_->publish(msg);
}

void TrackingSpeedGuardNode::on_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("valid", last_.valid);
  stat.add("lateral_error_m", last_.lateral_m);
  stat.add("heading_error_deg", last_.yaw_rad * 180.0 / M_PI);
  stat.add("severity", last_.mu);
  stat.add("speed_cap_mps", limit_active_ ? last_.cap_mps : param_.v_free_mps);
  stat.add("limit_active", limit_active_);
  stat.add("travelling_back", last_.travelling_back);

  if (!last_.valid) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "no measurement");
  } else if (!limit_active_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "tracking within tolerance");
  } else if (last_.mu < 1.0) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "tracking error; speed capped");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "tracking error at the limit; speed at floor");
  }
}

}  // namespace autoware::tracking_speed_guard

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tracking_speed_guard::TrackingSpeedGuardNode)
