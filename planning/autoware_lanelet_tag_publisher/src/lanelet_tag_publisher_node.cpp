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

#include "autoware/lanelet_tag_publisher/lanelet_tag_publisher_node.hpp"

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace autoware::lanelet_tag_publisher
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
LaneletTagPublisherNode::LaneletTagPublisherNode(const rclcpp::NodeOptions & options)
: Node("lanelet_tag_publisher", options)
{
  // ------------------------------------------------------------------
  // Load parameters
  // ------------------------------------------------------------------
  param_.tag_name = declare_parameter<std::string>("tag_name", "enable_master_local_control");
  param_.output_topic =
    declare_parameter<std::string>("output_topic", "/autoware/master_control_local");
  param_.topic_map = declare_parameter<std::string>("topic_map", "/map/vector_map");
  param_.topic_ego_pose =
    declare_parameter<std::string>("topic_ego_pose", "/localization/kinematic_state");
  param_.update_rate_hz = declare_parameter<double>("update_rate_hz", 10.0);
  param_.debounce_ticks_on = declare_parameter<int>("debounce_ticks_on", 3);
  param_.debounce_ticks_off = declare_parameter<int>("debounce_ticks_off", 3);

  // ------------------------------------------------------------------
  // Subscribers
  // ------------------------------------------------------------------
  // Map: transient_local + reliable (published once at startup) — mirrors
  // road_crossing_fsm_node's exact QoS for this subscription.
  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    param_.topic_map,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&LaneletTagPublisherNode::onMap, this, std::placeholders::_1));

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    param_.topic_ego_pose, rclcpp::QoS(1),
    std::bind(&LaneletTagPublisherNode::onOdom, this, std::placeholders::_1));

  // ------------------------------------------------------------------
  // Publisher
  // ------------------------------------------------------------------
  pub_output_ = create_publisher<std_msgs::msg::Bool>(param_.output_topic, rclcpp::QoS(1));

  // ------------------------------------------------------------------
  // Timer
  // ------------------------------------------------------------------
  const auto period = std::chrono::duration<double>(1.0 / param_.update_rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&LaneletTagPublisherNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "lanelet_tag_publisher started: tag_name='%s', output_topic='%s', "
    "debounce_ticks_on=%d, debounce_ticks_off=%d, update_rate_hz=%.1f",
    param_.tag_name.c_str(), param_.output_topic.c_str(), param_.debounce_ticks_on,
    param_.debounce_ticks_off, param_.update_rate_hz);
}

// ---------------------------------------------------------------------------
// onMap
// ---------------------------------------------------------------------------
void LaneletTagPublisherNode::onMap(const autoware_map_msgs::msg::LaneletMapBin::SharedPtr msg)
{
  route_handler_.setMap(*msg);
  map_received_ = true;
}

// ---------------------------------------------------------------------------
// onOdom
// ---------------------------------------------------------------------------
void LaneletTagPublisherNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

// ---------------------------------------------------------------------------
// isEgoPositionInTaggedLanelet — verified point-in-polygon idiom.
// Mirrors isObjectPositionInNarrowLaneLanelet() in
// autoware_behavior_path_static_obstacle_avoidance_module/src/utils.cpp,
// adapted for ego's own pose instead of an object's position.
// ---------------------------------------------------------------------------
bool LaneletTagPublisherNode::isEgoPositionInTaggedLanelet(
  const route_handler::RouteHandler & route_handler, const geometry_msgs::msg::Point & ego_pos,
  const std::string & tag)
{
  const auto lanelet_map_ptr = route_handler.getLaneletMapPtr();
  if (!lanelet_map_ptr) return false;
  const lanelet::BasicPoint2d point(ego_pos.x, ego_pos.y);
  constexpr double search_margin = 0.1;
  const lanelet::BoundingBox2d bbox(
    lanelet::BasicPoint2d(ego_pos.x - search_margin, ego_pos.y - search_margin),
    lanelet::BasicPoint2d(ego_pos.x + search_margin, ego_pos.y + search_margin));
  for (const auto & ll : lanelet_map_ptr->laneletLayer.search(bbox)) {
    if (
      ll.attributeOr(tag, false) &&
      !boost::geometry::disjoint(point, ll.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// onTimer — main tick: containment check + debounce/hysteresis + publish
// ---------------------------------------------------------------------------
void LaneletTagPublisherNode::onTimer()
{
  // Wait until the map has actually been received. Do NOT require a route —
  // this node never subscribes to /planning/mission_planning/route and never
  // calls setRoute(); getLaneletMapPtr() only needs setMap() to have run.
  if (!map_received_ || !route_handler_.getLaneletMapPtr()) {
    return;
  }
  if (!latest_odom_) {
    return;
  }

  const bool raw_in_tagged_lanelet = isEgoPositionInTaggedLanelet(
    route_handler_, latest_odom_->pose.pose.position, param_.tag_name);

  // -----------------------------------------------------------------
  // Debounce / hysteresis.
  //
  // Guards against localization jitter near a lanelet boundary flickering
  // the gate. Note this fallback mechanism's own trigger depends on the same
  // localization it's meant to backstop — this debounce is the agreed
  // minimal mitigation, not a full redesign.
  // -----------------------------------------------------------------
  if (raw_in_tagged_lanelet) {
    consecutive_true_ticks_++;
    consecutive_false_ticks_ = 0;
  } else {
    consecutive_false_ticks_++;
    consecutive_true_ticks_ = 0;
  }

  if (!published_state_initialized_) {
    // Seed the very first published value directly from the raw reading —
    // no need to wait a full debounce window before the first publish.
    published_state_ = raw_in_tagged_lanelet;
    published_state_initialized_ = true;
  } else if (!published_state_ && consecutive_true_ticks_ >= param_.debounce_ticks_on) {
    published_state_ = true;
  } else if (published_state_ && consecutive_false_ticks_ >= param_.debounce_ticks_off) {
    published_state_ = false;
  }

  std_msgs::msg::Bool out;
  out.data = published_state_;
  pub_output_->publish(out);
}

}  // namespace autoware::lanelet_tag_publisher

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lanelet_tag_publisher::LaneletTagPublisherNode)
