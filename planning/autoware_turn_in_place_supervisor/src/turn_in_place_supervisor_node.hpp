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

#ifndef TURN_IN_PLACE_SUPERVISOR_NODE_HPP_
#define TURN_IN_PLACE_SUPERVISOR_NODE_HPP_

#include "autoware/turn_in_place_supervisor/decision.hpp"
#include "autoware/turn_in_place_supervisor/geometry.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_active_route_window_msgs/msg/active_route_window.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_turn_in_place_msgs/msg/reverse_to_turn_spot_request.hpp>
#include <autoware_turn_in_place_msgs/msg/turn_in_place_command.hpp>
#include <autoware_turn_in_place_msgs/msg/turn_in_place_status.hpp>
#include <autoware_turn_in_place_msgs/msg/turn_in_place_supervisor_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::turn_in_place_supervisor
{

/// Watches (10 Hz by default) whether the robot faces the way its route runs.  When it does not
/// (|yaw error| above trigger_angle for trigger_confirm_sec), it holds the robot with a zero
/// velocity limit -- from the very first cycle, also before the robot is engaged -- picks a
/// direction whose sweep stays inside the road lanelets and is free of obstacles (no-ground
/// pointcloud cropped around the robot), asks the executor to spin the robot in place, and hands
/// back after a short cooldown.  When the road is too narrow to spin, it has reverse_lane_follow
/// back the robot up along the route (towards the goal) to the first spot wide enough, or all
/// the way to the goal when there is none.  See README.md.
class TurnInPlaceSupervisorNode : public rclcpp::Node
{
public:
  explicit TurnInPlaceSupervisorNode(const rclcpp::NodeOptions & options);

private:
  using ActiveRouteWindow = autoware_active_route_window_msgs::msg::ActiveRouteWindow;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using VelocityLimit = autoware_internal_planning_msgs::msg::VelocityLimit;
  using VelocityLimitClearCommand = autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using ReverseToTurnSpotRequest = autoware_turn_in_place_msgs::msg::ReverseToTurnSpotRequest;
  using TurnInPlaceCommand = autoware_turn_in_place_msgs::msg::TurnInPlaceCommand;
  using TurnInPlaceStatus = autoware_turn_in_place_msgs::msg::TurnInPlaceStatus;
  using TurnInPlaceSupervisorState = autoware_turn_in_place_msgs::msg::TurnInPlaceSupervisorState;

  // A road lanelet's polygon in the map frame, with its bounding box for cheap culling.
  struct RoadPolygon
  {
    Polygon2d points;
    double min_x{0.0};
    double max_x{0.0};
    double min_y{0.0};
    double max_y{0.0};
  };

  struct TurnSpot
  {
    geometry_msgs::msg::Pose pose;       // where the robot should end up (route centreline)
    geometry_msgs::msg::Pose stop_pose;  // what the back-up is asked to stop at (a little past
                                         // pose: the longitudinal controller stops short)
    double distance{0.0};                // [m] along the route from ego to pose
  };

  void on_timer();
  std::optional<double> route_heading_at_ego();
  std::optional<std::vector<Point2d>> obstacle_points_around_rotation_centre();
  void build_road_polygons();
  std::vector<Polygon2d> road_area_near(double x, double y, double radius) const;
  std::vector<Polygon2d> road_area_in_body_frame(double radius) const;
  std::optional<TurnSpot> search_turn_spot();
  void publish_hold(bool hold);
  void publish_reverse_request(const DecisionOutput & out);
  void publish_markers(const DecisionInput & in, const DecisionOutput & out);
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat);
  geometry_msgs::msg::Point rotation_centre_in_map() const;

  // parameters
  DecisionParams decision_params_;
  double decision_rate_hz_{10.0};
  double sweep_margin_{0.2};
  double road_margin_{0.1};
  double sweep_step_rad_{0.087};
  double footprint_sample_spacing_{0.15};
  double rotation_centre_x_{0.0};
  double window_timeout_sec_{2.0};
  double pointcloud_timeout_sec_{1.0};
  double executor_status_timeout_sec_{2.0};
  std::vector<std::string> road_subtypes_;
  double spot_search_step_m_{0.25};
  double spot_search_max_m_{300.0};
  double spot_min_distance_m_{0.6};
  double spot_entry_margin_m_{0.5};
  double spot_reached_tolerance_m_{0.6};
  double spot_research_period_sec_{1.0};
  double spot_stop_overshoot_m_{0.7};
  double spot_settle_sec_{1.0};
  std::optional<double> stopped_since_;
  std::string base_frame_{"base_link"};
  std::string velocity_limit_sender_{"turn_in_place"};
  Footprint footprint_;  // relative to the rotation centre

  TurnInPlaceDecision decision_;
  DecisionOutput last_output_;
  DecisionInput last_input_;
  bool holding_{false};
  std::vector<RoadPolygon> road_polygons_;
  std::optional<TurnSpot> turn_spot_;
  double last_spot_search_time_{-1.0e9};

  // inputs
  autoware::route_handler::RouteHandler route_handler_;
  bool map_ready_{false};
  LaneletMapBin::ConstSharedPtr map_msg_;
  LaneletRoute::ConstSharedPtr route_;
  bool route_changed_{false};
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  ActiveRouteWindow::ConstSharedPtr window_;
  OperationModeState::ConstSharedPtr operation_mode_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_;
  TurnInPlaceStatus::ConstSharedPtr executor_status_;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<ActiveRouteWindow>::SharedPtr sub_window_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Subscription<TurnInPlaceStatus>::SharedPtr sub_executor_status_;

  rclcpp::Publisher<TurnInPlaceCommand>::SharedPtr pub_command_;
  rclcpp::Publisher<TurnInPlaceSupervisorState>::SharedPtr pub_state_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;
  rclcpp::Publisher<ReverseToTurnSpotRequest>::SharedPtr pub_reverse_request_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};
};

}  // namespace autoware::turn_in_place_supervisor

#endif  // TURN_IN_PLACE_SUPERVISOR_NODE_HPP_
