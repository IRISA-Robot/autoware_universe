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

#ifndef AUTOWARE__LANELET_TAG_PUBLISHER__LANELET_TAG_PUBLISHER_NODE_HPP_
#define AUTOWARE__LANELET_TAG_PUBLISHER__LANELET_TAG_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include <autoware/route_handler/route_handler.hpp>

#include <memory>
#include <string>

namespace autoware::lanelet_tag_publisher
{

// ---------------------------------------------------------------------------
// LaneletTagPublisherNode
//
// Minimal node: watches the vector map + ego pose, and publishes a debounced
// std_msgs/Bool indicating whether ego's current position lies inside ANY
// lanelet tagged with a configurable boolean attribute.
//
// Deliberately does NOT subscribe to the route
// (/planning/mission_planning/route) — RouteHandler::getLaneletMapPtr() only
// requires setMap() to have been called, so route state is not needed for
// this pure point-in-polygon containment check.
// ---------------------------------------------------------------------------
class LaneletTagPublisherNode : public rclcpp::Node
{
public:
  explicit LaneletTagPublisherNode(const rclcpp::NodeOptions & options);

private:
  struct Parameters
  {
    std::string tag_name;
    std::string output_topic;
    std::string topic_map;
    std::string topic_ego_pose;
    double update_rate_hz{10.0};
    int debounce_ticks_on{3};
    int debounce_ticks_off{3};
  };

  // Callbacks
  void onMap(const autoware_map_msgs::msg::LaneletMapBin::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();

  // Core containment check — verified idiom, do NOT use
  // getClosestLaneletWithinRoute() here (wrong tool for a true point-in-
  // polygon question; that call finds the nearest lanelet ALONG THE ROUTE,
  // not necessarily the one ego is actually inside).
  static bool isEgoPositionInTaggedLanelet(
    const route_handler::RouteHandler & route_handler, const geometry_msgs::msg::Point & ego_pos,
    const std::string & tag);

  Parameters param_;

  route_handler::RouteHandler route_handler_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_output_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  bool map_received_{false};

  // Debounce / hysteresis state.
  bool published_state_{false};
  bool published_state_initialized_{false};
  int consecutive_true_ticks_{0};
  int consecutive_false_ticks_{0};
};

}  // namespace autoware::lanelet_tag_publisher

#endif  // AUTOWARE__LANELET_TAG_PUBLISHER__LANELET_TAG_PUBLISHER_NODE_HPP_
