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

#ifndef AUTOWARE__ACTIVE_ROUTE_WINDOW__ACTIVE_ROUTE_WINDOW_NODE_HPP_
#define AUTOWARE__ACTIVE_ROUTE_WINDOW__ACTIVE_ROUTE_WINDOW_NODE_HPP_

#include "autoware/active_route_window/window_calculator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_active_route_window_msgs/msg/active_route_window.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::active_route_window
{

/**
 * Publishes the rolling window of the active route that geometric consumers plan on, and the
 * route segment ego is on.  A route that passes over the same ground twice (an out-and-back on
 * overlapping lanelets) otherwise presents both passes at once, and every lookup that goes by
 * position can land on the wrong one.  One node decides where ego is on the route, so every
 * consumer agrees.
 */
class ActiveRouteWindowNode : public rclcpp::Node
{
public:
  explicit ActiveRouteWindowNode(const rclcpp::NodeOptions & options);

private:
  using ActiveRouteWindow = autoware_active_route_window_msgs::msg::ActiveRouteWindow;

  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void on_route(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
  void on_timer();
  void prepare_route();
  void publish_markers(const std::optional<Window> & window) const;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Publisher<ActiveRouteWindow>::SharedPtr pub_window_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;

  double backward_distance_m_{};
  double forward_distance_m_{};
  double tie_tolerance_m_{};
  double max_drift_m_{};
  double max_ego_distance_m_{};

  lanelet::LaneletMapPtr lanelet_map_;
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;

  // Per route: each segment's preferred lanelet oriented the way the route runs it, its length,
  // and which segments overlap.  Geometry is static, so this is computed once per route.
  std::vector<lanelet::ConstLanelet> segment_lanelets_;
  std::vector<double> segment_lengths_m_;
  ConflictMatrix conflicts_;
  bool route_prepared_{false};

  std::optional<size_t> ego_segment_;
};

}  // namespace autoware::active_route_window

#endif  // AUTOWARE__ACTIVE_ROUTE_WINDOW__ACTIVE_ROUTE_WINDOW_NODE_HPP_
