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

#include "autoware/turn_in_place_supervisor/executor_logic.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_turn_in_place_msgs/msg/steering_mode_feedback.hpp>
#include <autoware_turn_in_place_msgs/msg/turn_in_place_command.hpp>
#include <autoware_turn_in_place_msgs/msg/turn_in_place_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace autoware::turn_in_place_supervisor
{

/// Runs the spin-in-place executor at a fixed rate: switches the chassis to its spin steering
/// mode, rotates to the commanded heading with a yaw-rate twist, switches back.  It talks to the
/// chassis through the same interface as the real middleware (steering-mode topic, a twist whose
/// angular.z is a yaw rate in the spin mode, steering-mode feedback), so the simulator mock and
/// the robot are driven by the same node.
class TurnInPlaceExecutorNode : public rclcpp::Node
{
public:
  using SteeringModeFeedback = autoware_turn_in_place_msgs::msg::SteeringModeFeedback;
  using TurnInPlaceCommand = autoware_turn_in_place_msgs::msg::TurnInPlaceCommand;
  using TurnInPlaceStatus = autoware_turn_in_place_msgs::msg::TurnInPlaceStatus;

  explicit TurnInPlaceExecutorNode(const rclcpp::NodeOptions & options)
  : Node("turn_in_place_executor", options), executor_(ExecutorParams{})
  {
    constexpr double deg = M_PI / 180.0;
    ExecutorParams p;
    p.spin_mode = static_cast<uint8_t>(declare_parameter<int64_t>("spin_mode"));
    p.driving_mode = static_cast<uint8_t>(declare_parameter<int64_t>("driving_mode"));
    p.kp = declare_parameter<double>("kp");
    p.max_yaw_rate = declare_parameter<double>("max_yaw_rate");
    p.min_yaw_rate = declare_parameter<double>("min_yaw_rate");
    p.done_tolerance = declare_parameter<double>("done_tolerance_deg") * deg;
    p.switch_timeout_sec = declare_parameter<double>("switch_timeout_sec");
    p.rotate_timeout_sec = declare_parameter<double>("rotate_timeout_sec");
    p.request_retry_sec = declare_parameter<double>("request_retry_sec");
    executor_.set_params(p);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec");
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec");
    yaw_rate_command_gain_ = declare_parameter<double>("yaw_rate_command_gain");
    const double rate_hz = declare_parameter<double>("rate_hz");

    sub_command_ = create_subscription<TurnInPlaceCommand>(
      "~/input/command", 1,
      [this](const TurnInPlaceCommand::ConstSharedPtr msg) { command_ = msg; });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odometry", 1,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; });
    sub_feedback_ = create_subscription<SteeringModeFeedback>(
      "~/input/steering_mode_feedback", 1,
      [this](const SteeringModeFeedback::ConstSharedPtr msg) { feedback_ = msg; });

    pub_mode_ = create_publisher<std_msgs::msg::UInt8>("~/output/steering_mode", 1);
    pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("~/output/twist", 1);
    pub_status_ = create_publisher<TurnInPlaceStatus>("~/output/status", 1);

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0));
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { on_timer(); });
  }

private:
  void on_timer()
  {
    const auto stamp = now();
    ExecutorInput in;
    in.now = stamp.seconds();
    // A command the supervisor has stopped sending counts as cancelled.
    in.command_active =
      command_ && command_->active &&
      (stamp - rclcpp::Time(command_->stamp)).seconds() <= command_timeout_sec_;
    if (command_) {
      in.target_yaw = command_->target_yaw;
      in.direction = command_->direction;
    }
    if (odom_) {
      in.yaw = tf2::getYaw(odom_->pose.pose.orientation);
    } else if (in.command_active) {
      in.command_active = false;  // no heading: never rotate blind
    }
    in.feedback_fresh =
      feedback_ && (stamp - rclcpp::Time(feedback_->stamp)).seconds() <= feedback_timeout_sec_;
    if (in.feedback_fresh) {
      in.feedback_mode = feedback_->mode;
      in.feedback_target_mode = feedback_->target_mode;
      in.feedback_switching = feedback_->switching;
    }

    const auto previous = executor_.state();
    const auto out = executor_.step(in);
    if (out.state != previous) {
      RCLCPP_INFO(
        get_logger(), "[TURN-IN-PLACE-EXECUTOR] %d -> %d: %s (yaw error %.1f deg)",
        static_cast<int>(previous), static_cast<int>(out.state), out.reason.c_str(),
        out.yaw_error * 180.0 / M_PI);
    }

    if (out.mode_request) {
      std_msgs::msg::UInt8 mode;
      mode.data = *out.mode_request;
      pub_mode_->publish(mode);
    }
    if (out.publish_twist) {
      // angular.z is the yaw rate the chassis turns at in its spin mode; the gain converts it to
      // the chassis' own unit (the real middleware multiplies by holonomic_yaw_gain).
      geometry_msgs::msg::Twist twist;
      twist.angular.z = out.yaw_rate * yaw_rate_command_gain_;
      pub_twist_->publish(twist);
    }

    TurnInPlaceStatus status;
    status.stamp = stamp;
    status.state = static_cast<uint8_t>(out.state);
    status.yaw_error = out.yaw_error;
    status.reason = out.reason;
    pub_status_->publish(status);
  }

  TurnInPlaceExecutor executor_;
  double command_timeout_sec_{3.0};
  double feedback_timeout_sec_{1.0};
  double yaw_rate_command_gain_{1.0};

  TurnInPlaceCommand::ConstSharedPtr command_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  SteeringModeFeedback::ConstSharedPtr feedback_;

  rclcpp::Subscription<TurnInPlaceCommand>::SharedPtr sub_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<SteeringModeFeedback>::SharedPtr sub_feedback_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_mode_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<TurnInPlaceStatus>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::turn_in_place_supervisor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::turn_in_place_supervisor::TurnInPlaceExecutorNode)
