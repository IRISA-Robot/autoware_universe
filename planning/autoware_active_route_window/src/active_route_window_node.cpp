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

#include "autoware/active_route_window/active_route_window_node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <boost/geometry/algorithms/distance.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <chrono>
#include <string>

namespace autoware::active_route_window
{

ActiveRouteWindowNode::ActiveRouteWindowNode(const rclcpp::NodeOptions & options)
: Node("active_route_window", options)
{
  backward_distance_m_ = declare_parameter<double>("backward_distance_m", 10.0);
  forward_distance_m_ = declare_parameter<double>("forward_distance_m", 300.0);
  tie_tolerance_m_ = declare_parameter<double>("tie_tolerance_m", 0.5);
  max_drift_m_ = declare_parameter<double>("max_drift_m", 1.0);
  max_ego_distance_m_ = declare_parameter<double>("max_ego_distance_m", 3.0);
  const double rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);

  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) { on_map(msg); });
  sub_route_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) { on_route(msg); });
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometry_ = msg; });

  pub_window_ = create_publisher<ActiveRouteWindow>("~/output/active_route_window", rclcpp::QoS{1});
  pub_markers_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", rclcpp::QoS{1});

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0)),
    [this]() { on_timer(); });
}

void ActiveRouteWindowNode::on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);
  route_prepared_ = false;
}

void ActiveRouteWindowNode::on_route(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  if (!route_ || route_->uuid != msg->uuid) {
    ego_segment_.reset();  // progress belongs to one route
  }
  route_ = msg;
  route_prepared_ = false;
}

void ActiveRouteWindowNode::prepare_route()
{
  segment_lanelets_.clear();
  segment_lengths_m_.clear();
  conflicts_.clear();
  route_prepared_ = false;
  if (!lanelet_map_ || !route_ || route_->segments.empty()) {
    return;
  }
  for (const auto & segment : route_->segments) {
    const auto id = segment.preferred_primitive.id;
    if (!lanelet_map_->laneletLayer.exists(id)) {
      RCLCPP_WARN(get_logger(), "route lanelet %ld is not in the map", id);
      return;
    }
    const lanelet::ConstLanelet forward = lanelet_map_->laneletLayer.get(id);
    const auto llt = segment.is_reversed ? forward.invert() : forward;
    segment_lanelets_.push_back(llt);
    segment_lengths_m_.push_back(lanelet::geometry::length2d(llt));
  }
  conflicts_ = compute_conflicts(segment_lanelets_);

  std::string pairs;
  for (size_t i = 0; i < conflicts_.size(); ++i) {
    for (size_t j = i + 1; j < conflicts_.size(); ++j) {
      if (conflicts_.at(i).at(j)) {
        pairs += " " + std::to_string(segment_lanelets_.at(i).id()) + "/" +
                 std::to_string(segment_lanelets_.at(j).id());
      }
    }
  }
  RCLCPP_INFO(
    get_logger(), "route of %zu segments; overlapping segment pairs:%s", segment_lanelets_.size(),
    pairs.empty() ? " none (the window will cover the whole route ahead)" : pairs.c_str());
  route_prepared_ = true;
}

void ActiveRouteWindowNode::on_timer()
{
  if (!route_prepared_) {
    prepare_route();
  }
  if (!route_prepared_ || !odometry_) {
    return;
  }

  ActiveRouteWindow msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.route_uuid = route_->uuid;

  const auto & position = odometry_->pose.pose.position;
  const lanelet::BasicPoint2d ego(position.x, position.y);
  std::vector<double> distances;
  distances.reserve(segment_lanelets_.size());
  for (const auto & llt : segment_lanelets_) {
    distances.push_back(boost::geometry::distance(ego, llt.polygon2d().basicPolygon()));
  }

  const auto ego_segment = select_ego_segment(
    distances, ego_segment_, tie_tolerance_m_, max_drift_m_, max_ego_distance_m_);
  if (!ego_segment) {
    msg.valid = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[NO-ACTIVE-WINDOW] ego (%.2f, %.2f) is not within %.1f m of any route lanelet; consumers "
      "use the full route",
      position.x, position.y, max_ego_distance_m_);
    pub_window_->publish(msg);
    publish_markers(std::nullopt);
    return;
  }
  if (ego_segment_ != ego_segment) {
    RCLCPP_INFO(
      get_logger(), "ego on segment %zu (lanelet %ld)", *ego_segment,
      segment_lanelets_.at(*ego_segment).id());
  }
  ego_segment_ = ego_segment;

  const auto & centerline = segment_lanelets_.at(*ego_segment).centerline2d();
  const double ego_arc = lanelet::geometry::toArcCoordinates(centerline, ego).length;
  const auto window = compute_window(
    *ego_segment, ego_arc, segment_lengths_m_, conflicts_, backward_distance_m_,
    forward_distance_m_);

  msg.valid = true;
  msg.begin_segment = static_cast<uint32_t>(window.begin);
  msg.end_segment = static_cast<uint32_t>(window.end);
  msg.ego_segment = static_cast<uint32_t>(*ego_segment);
  msg.ego_lanelet_id = segment_lanelets_.at(*ego_segment).id();
  pub_window_->publish(msg);
  publish_markers(window);
}

void ActiveRouteWindowNode::publish_markers(const std::optional<Window> & window) const
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  if (window) {
    for (size_t i = window->begin; i <= window->end && i < segment_lanelets_.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = "active_route_window";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = (ego_segment_ && *ego_segment_ == i) ? 0.35 : 0.2;
      m.color.r = 0.1F;
      m.color.g = 0.9F;
      m.color.b = 0.9F;
      m.color.a = 0.9F;
      for (const auto & p : segment_lanelets_.at(i).centerline3d()) {
        geometry_msgs::msg::Point q;
        q.x = p.x();
        q.y = p.y();
        q.z = p.z() + 0.1;
        m.points.push_back(q);
      }
      markers.markers.push_back(m);
    }
  }
  pub_markers_->publish(markers);
}

}  // namespace autoware::active_route_window

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::active_route_window::ActiveRouteWindowNode)
