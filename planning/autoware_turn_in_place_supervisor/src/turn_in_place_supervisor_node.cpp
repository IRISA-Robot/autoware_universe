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

#include "turn_in_place_supervisor_node.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::turn_in_place_supervisor
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

bool same_uuid(
  const unique_identifier_msgs::msg::UUID & a, const unique_identifier_msgs::msg::UUID & b)
{
  return a.uuid == b.uuid;
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}
}  // namespace

TurnInPlaceSupervisorNode::TurnInPlaceSupervisorNode(const rclcpp::NodeOptions & options)
: Node("turn_in_place_supervisor", options), decision_(DecisionParams{})
{
  decision_params_.trigger_angle = declare_parameter<double>("trigger_angle_deg") * kDegToRad;
  decision_params_.done_angle = declare_parameter<double>("done_angle_deg") * kDegToRad;
  decision_params_.trigger_confirm_sec = declare_parameter<double>("trigger_confirm_sec");
  decision_params_.stop_speed = declare_parameter<double>("stop_speed_mps");
  decision_params_.cooldown_sec = declare_parameter<double>("cooldown_sec");
  decision_params_.stop_timeout_sec = declare_parameter<double>("stop_timeout_sec");
  decision_params_.clearance_timeout_sec = declare_parameter<double>("clearance_timeout_sec");
  decision_params_.turn_timeout_sec = declare_parameter<double>("turn_timeout_sec");
  decision_params_.enable_reverse_to_turn_spot =
    declare_parameter<bool>("enable_reverse_to_turn_spot");
  decision_.set_params(decision_params_);

  decision_rate_hz_ = declare_parameter<double>("decision_rate_hz");
  sweep_margin_ = declare_parameter<double>("sweep_margin_m");
  road_margin_ = declare_parameter<double>("road_margin_m");
  sweep_step_rad_ = declare_parameter<double>("sweep_step_deg") * kDegToRad;
  footprint_sample_spacing_ = declare_parameter<double>("footprint_sample_spacing_m");
  rotation_centre_x_ = declare_parameter<double>("rotation_centre_x_m");
  window_timeout_sec_ = declare_parameter<double>("active_route_window_timeout_sec");
  pointcloud_timeout_sec_ = declare_parameter<double>("pointcloud_timeout_sec");
  executor_status_timeout_sec_ = declare_parameter<double>("executor_status_timeout_sec");
  road_subtypes_ = declare_parameter<std::vector<std::string>>("road_subtypes");
  no_turn_lanelet_tag_ = declare_parameter<std::string>("no_turn_lanelet_tag");
  spot_search_step_m_ = declare_parameter<double>("turn_spot.search_step_m");
  spot_search_max_m_ = declare_parameter<double>("turn_spot.search_max_m");
  spot_min_distance_m_ = declare_parameter<double>("turn_spot.min_distance_m");
  spot_entry_margin_m_ = declare_parameter<double>("turn_spot.entry_margin_m");
  spot_reached_tolerance_m_ = declare_parameter<double>("turn_spot.reached_tolerance_m");
  spot_research_period_sec_ = declare_parameter<double>("turn_spot.research_period_sec");
  spot_stop_overshoot_m_ = declare_parameter<double>("turn_spot.stop_overshoot_m");
  spot_settle_sec_ = declare_parameter<double>("turn_spot.settle_sec");
  base_frame_ = declare_parameter<std::string>("base_frame");

  // The body rectangle about the point it spins around.  vehicle_info offsets are from
  // base_link; rotation_centre_x_m moves the spin centre along the body.
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  footprint_.front = vehicle_info.max_longitudinal_offset_m - rotation_centre_x_;
  footprint_.rear = -vehicle_info.min_longitudinal_offset_m + rotation_centre_x_;
  footprint_.left = vehicle_info.max_lateral_offset_m;
  footprint_.right = -vehicle_info.min_lateral_offset_m;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  const auto latched = rclcpp::QoS{1}.transient_local();
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", latched, [this](const LaneletMapBin::ConstSharedPtr msg) {
      route_handler_.setMap(*msg);
      map_ready_ = true;
      build_road_polygons();
      if (route_) {
        route_handler_.setRoute(*route_);
      }
    });
  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", latched, [this](const LaneletRoute::ConstSharedPtr msg) {
      if (!route_ || !same_uuid(route_->uuid, msg->uuid)) {
        route_changed_ = true;
      }
      route_ = msg;
      if (map_ready_) {
        route_handler_.setRoute(*msg);
      }
    });
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; });
  sub_window_ = create_subscription<ActiveRouteWindow>(
    "~/input/active_route_window", 1,
    [this](const ActiveRouteWindow::ConstSharedPtr msg) { window_ = msg; });
  sub_operation_mode_ = create_subscription<OperationModeState>(
    "~/input/operation_mode_state", latched,
    [this](const OperationModeState::ConstSharedPtr msg) { operation_mode_ = msg; });
  sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { pointcloud_ = msg; });
  sub_executor_status_ = create_subscription<TurnInPlaceStatus>(
    "~/input/executor_status", 1,
    [this](const TurnInPlaceStatus::ConstSharedPtr msg) { executor_status_ = msg; });

  pub_command_ = create_publisher<TurnInPlaceCommand>("~/output/command", 1);
  pub_state_ = create_publisher<TurnInPlaceSupervisorState>("~/output/state", 1);
  // Same QoS as the external velocity limit selector's internal inputs.
  pub_velocity_limit_ = create_publisher<VelocityLimit>(
    "~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
  pub_clear_velocity_limit_ = create_publisher<VelocityLimitClearCommand>(
    "~/output/clear_velocity_limit", rclcpp::QoS{1}.transient_local());
  pub_reverse_request_ =
    create_publisher<ReverseToTurnSpotRequest>("~/output/reverse_request", 1);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 1);

  diagnostics_.setHardwareID("turn_in_place_supervisor");
  diagnostics_.add("turn_in_place", this, &TurnInPlaceSupervisorNode::diagnose);

  const auto period = std::chrono::duration<double>(1.0 / std::max(decision_rate_hz_, 0.1));
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { on_timer(); });
}

void TurnInPlaceSupervisorNode::build_road_polygons()
{
  // Every road lanelet of the map counts as space the body may sweep while it turns -- the
  // neighbouring and opposite lanes and the shoulders too, not only the route.
  road_polygons_.clear();
  no_turn_polygons_.clear();
  const auto map = route_handler_.getLaneletMapPtr();
  if (!map) {
    return;
  }
  for (const auto & llt : map->laneletLayer) {
    // [NO-TURN LANELETS 2026-11-09] A lanelet tagged no_turn_lanelet_tag=yes (any subtype) is
    // off limits to the turning sweep, even where another lanelet overlaps it.
    const std::string no_turn = llt.attributeOr(no_turn_lanelet_tag_, "no");
    const bool is_no_turn = no_turn == "yes" || no_turn == "true";
    const std::string subtype = llt.attributeOr(lanelet::AttributeName::Subtype, "");
    const bool is_road =
      std::find(road_subtypes_.begin(), road_subtypes_.end(), subtype) != road_subtypes_.end();
    if (!is_road && !is_no_turn) {
      continue;
    }
    RoadPolygon polygon;
    polygon.min_x = polygon.min_y = std::numeric_limits<double>::max();
    polygon.max_x = polygon.max_y = std::numeric_limits<double>::lowest();
    for (const auto & p : llt.polygon2d().basicPolygon()) {
      polygon.points.push_back(Point2d{p.x(), p.y()});
      polygon.min_x = std::min(polygon.min_x, p.x());
      polygon.max_x = std::max(polygon.max_x, p.x());
      polygon.min_y = std::min(polygon.min_y, p.y());
      polygon.max_y = std::max(polygon.max_y, p.y());
    }
    if (polygon.points.size() >= 3) {
      if (is_no_turn) {
        no_turn_polygons_.push_back(polygon);
      }
      if (is_road) {
        road_polygons_.push_back(std::move(polygon));
      }
    }
  }
  RCLCPP_INFO(
    get_logger(), "[TURN-IN-PLACE] road area: %zu lanelets, no-turn (%s=yes): %zu lanelets",
    road_polygons_.size(), no_turn_lanelet_tag_.c_str(), no_turn_polygons_.size());
}

std::vector<Polygon2d> TurnInPlaceSupervisorNode::road_area_near(
  const double x, const double y, const double radius) const
{
  return polygons_near(road_polygons_, x, y, radius);
}

std::vector<Polygon2d> TurnInPlaceSupervisorNode::no_turn_area_near(
  const double x, const double y, const double radius) const
{
  return polygons_near(no_turn_polygons_, x, y, radius);
}

std::vector<Polygon2d> TurnInPlaceSupervisorNode::polygons_near(
  const std::vector<RoadPolygon> & polygons, const double x, const double y, const double radius)
{
  std::vector<Polygon2d> area;
  for (const auto & polygon : polygons) {
    if (
      polygon.max_x < x - radius || polygon.min_x > x + radius || polygon.max_y < y - radius ||
      polygon.min_y > y + radius) {
      continue;
    }
    area.push_back(polygon.points);
  }
  return area;
}

geometry_msgs::msg::Point TurnInPlaceSupervisorNode::rotation_centre_in_map() const
{
  geometry_msgs::msg::Point c;
  if (!odom_) {
    return c;
  }
  const auto & pose = odom_->pose.pose;
  const double yaw = tf2::getYaw(pose.orientation);
  c.x = pose.position.x + rotation_centre_x_ * std::cos(yaw);
  c.y = pose.position.y + rotation_centre_x_ * std::sin(yaw);
  c.z = pose.position.z;
  return c;
}

std::vector<Polygon2d> TurnInPlaceSupervisorNode::to_body_frame(
  const std::vector<Polygon2d> & polygons) const
{
  // Map-frame polygons in the frame the sweep is checked in: origin at the rotation centre, x
  // along the body.
  std::vector<Polygon2d> area;
  if (!odom_) {
    return area;
  }
  const auto c = rotation_centre_in_map();
  const double yaw = tf2::getYaw(odom_->pose.pose.orientation);
  const double cs = std::cos(-yaw);
  const double sn = std::sin(-yaw);
  for (const auto & polygon : polygons) {
    Polygon2d local;
    local.reserve(polygon.size());
    for (const auto & p : polygon) {
      const double dx = p.x - c.x;
      const double dy = p.y - c.y;
      local.push_back(Point2d{cs * dx - sn * dy, sn * dx + cs * dy});
    }
    area.push_back(std::move(local));
  }
  return area;
}

std::optional<double> TurnInPlaceSupervisorNode::route_heading_at_ego()
{
  // The route heading comes from the active route window's ego lanelet, oriented the way the
  // route runs it (getLaneletsFromId re-applies an inverted traversal), at the centreline point
  // nearest ego by position alone.  Deliberately not the trajectory: with the robot facing the
  // wrong way it is not trustworthy, and on an out-and-back route a nearest search picks the
  // wrong leg -- the window is what knows which pass ego is on.
  if (!map_ready_ || !route_ || !odom_ || !window_ || !window_->valid) {
    return std::nullopt;
  }
  if ((now() - rclcpp::Time(window_->header.stamp)).seconds() > window_timeout_sec_) {
    return std::nullopt;
  }
  if (!same_uuid(window_->route_uuid, route_->uuid)) {
    return std::nullopt;
  }
  const auto map = route_handler_.getLaneletMapPtr();
  if (!map || !map->laneletLayer.exists(window_->ego_lanelet_id)) {
    return std::nullopt;
  }
  const auto lanelet = route_handler_.getLaneletsFromId(window_->ego_lanelet_id);
  return lanelet::utils::getLaneletAngle(lanelet, odom_->pose.pose.position);
}

std::optional<std::vector<Point2d>>
TurnInPlaceSupervisorNode::obstacle_points_around_rotation_centre()
{
  // nullopt = no usable cloud: the turning area is then treated as blocked on both sides.
  if (!pointcloud_) {
    return std::nullopt;
  }
  if ((now() - rclcpp::Time(pointcloud_->header.stamp)).seconds() > pointcloud_timeout_sec_) {
    return std::nullopt;
  }
  tf2::Transform to_base;
  if (pointcloud_->header.frame_id == base_frame_) {
    to_base.setIdentity();
  } else {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, pointcloud_->header.frame_id, tf2::TimePointZero);
      tf2::fromMsg(tf.transform, to_base);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "[TURN-IN-PLACE] pointcloud transform failed: %s",
        e.what());
      return std::nullopt;
    }
  }

  std::vector<Point2d> points;
  points.reserve(pointcloud_->width * pointcloud_->height);
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*pointcloud_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*pointcloud_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*pointcloud_, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {
      continue;
    }
    const tf2::Vector3 p = to_base * tf2::Vector3(*it_x, *it_y, *it_z);
    points.push_back(Point2d{p.x() - rotation_centre_x_, p.y()});
  }
  return points;
}

std::optional<TurnInPlaceSupervisorNode::TurnSpot> TurnInPlaceSupervisorNode::search_turn_spot()
{
  // Walk the route centreline from ego towards the goal and return the first place where a
  // full spin fits inside the road lanelets (a disk of the body's corner radius + road margin).
  // Obstacles there are not known yet (the cloud is cropped around the robot); they are checked
  // again once the robot stands on the spot.
  if (!odom_ || !route_ || !map_ready_ || !window_ || !window_->valid) {
    return std::nullopt;
  }
  const auto map = route_handler_.getLaneletMapPtr();
  if (!map || !map->laneletLayer.exists(window_->ego_lanelet_id)) {
    return std::nullopt;
  }
  const auto & ego_pose = odom_->pose.pose;
  const auto ego_lanelet = route_handler_.getLaneletsFromId(window_->ego_lanelet_id);
  const auto sequence =
    route_handler_.getLaneletSequence(ego_lanelet, ego_pose, 0.0, spot_search_max_m_ + 50.0);
  if (sequence.empty()) {
    return std::nullopt;
  }

  // The route centreline as a polyline, in the order the route runs it.
  std::vector<Point2d> line;
  for (const auto & llt : sequence) {
    for (const auto & p : llt.centerline2d()) {
      if (
        line.empty() || std::hypot(p.x() - line.back().x, p.y() - line.back().y) > 1e-3) {
        line.push_back(Point2d{p.x(), p.y()});
      }
    }
  }
  if (line.size() < 2) {
    return std::nullopt;
  }
  std::vector<double> s(line.size(), 0.0);
  for (size_t i = 1; i < line.size(); ++i) {
    s.at(i) = s.at(i - 1) + std::hypot(line.at(i).x - line.at(i - 1).x, line.at(i).y - line.at(i - 1).y);
  }
  const auto project = [&](const double x, const double y, const size_t from = 0) {
    double best_d2 = std::numeric_limits<double>::max();
    double best_s = 0.0;
    for (size_t i = from; i + 1 < line.size(); ++i) {
      const auto & a = line.at(i);
      const auto & b = line.at(i + 1);
      const double vx = b.x - a.x;
      const double vy = b.y - a.y;
      const double len2 = vx * vx + vy * vy;
      const double t =
        len2 < 1e-9 ? 0.0 : std::clamp(((x - a.x) * vx + (y - a.y) * vy) / len2, 0.0, 1.0);
      const double px = a.x + t * vx;
      const double py = a.y + t * vy;
      const double d2 = (x - px) * (x - px) + (y - py) * (y - py);
      if (d2 < best_d2) {
        best_d2 = d2;
        best_s = s.at(i) + t * std::sqrt(len2);
      }
    }
    return best_s;
  };
  const auto point_at = [&](const double target) {
    const auto it = std::upper_bound(s.begin(), s.end(), target);
    const size_t i = std::clamp<size_t>(
      static_cast<size_t>(std::distance(s.begin(), it)), 1, line.size() - 1);
    const auto & a = line.at(i - 1);
    const auto & b = line.at(i);
    const double seg = s.at(i) - s.at(i - 1);
    const double t = seg < 1e-9 ? 0.0 : std::clamp((target - s.at(i - 1)) / seg, 0.0, 1.0);
    return std::make_pair(
      Point2d{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)}, std::atan2(b.y - a.y, b.x - a.x));
  };

  const double s_ego = project(ego_pose.position.x, ego_pose.position.y);
  // Never past the goal: beyond it the route ends.
  const auto goal = route_handler_.getGoalPose();
  double s_end = std::min(s.back(), s_ego + spot_search_max_m_);
  const double s_goal = project(goal.position.x, goal.position.y);
  if (s_goal > s_ego) {
    s_end = std::min(s_end, s_goal);
  }

  const double radius = sweep_radius(footprint_, road_margin_);
  std::optional<double> run_start;
  for (double si = s_ego + spot_min_distance_m_; si <= s_end; si += spot_search_step_m_) {
    const auto [c, yaw] = point_at(si);
    const auto area = road_area_near(c.x, c.y, radius + 0.5);
    const auto no_turn = no_turn_area_near(c.x, c.y, radius + 0.5);
    const bool fits = disk_inside_area(area, c, radius, 10.0 * kDegToRad, no_turn);
    if (fits && !run_start) {
      run_start = si;
    }
    if (!run_start) {
      continue;
    }
    // Stop a little inside the wide stretch so stopping short or long still leaves room; a
    // stretch shorter than that is aimed at its middle.
    const bool reached_margin = si - *run_start >= spot_entry_margin_m_;
    if (reached_margin || !fits || si + spot_search_step_m_ > s_end) {
      const double target = reached_margin && fits ? si : 0.5 * (*run_start + std::max(*run_start, si - spot_search_step_m_));
      const auto [tc, tyaw] = point_at(target);
      TurnSpot spot;
      spot.pose.position.x = tc.x;
      spot.pose.position.y = tc.y;
      spot.pose.position.z = ego_pose.position.z;
      spot.pose.orientation = yaw_to_quaternion(tyaw);
      spot.distance = target - s_ego;
      // The back-up stops short of what it is asked for (the longitudinal controller brakes to
      // a stop once inside its stopping distance): ask for a point that much further on, so the
      // robot comes to rest on the spot itself.
      const auto [sc, syaw] = point_at(std::min(target + spot_stop_overshoot_m_, s.back()));
      spot.stop_pose = spot.pose;
      spot.stop_pose.position.x = sc.x;
      spot.stop_pose.position.y = sc.y;
      spot.stop_pose.orientation = yaw_to_quaternion(syaw);
      return spot;
    }
  }
  return std::nullopt;
}

void TurnInPlaceSupervisorNode::publish_hold(const bool hold)
{
  const auto stamp = now();
  if (hold) {
    // Re-published every cycle while holding (cheap, and survives a selector restart).
    VelocityLimit limit;
    limit.stamp = stamp;
    limit.max_velocity = 0.0F;
    limit.use_constraints = false;
    limit.sender = velocity_limit_sender_;
    pub_velocity_limit_->publish(limit);
  } else if (holding_) {
    VelocityLimitClearCommand clear;
    clear.stamp = stamp;
    clear.command = true;
    clear.sender = velocity_limit_sender_;
    pub_clear_velocity_limit_->publish(clear);
  }
  holding_ = hold;
}

void TurnInPlaceSupervisorNode::publish_reverse_request(const DecisionOutput & out)
{
  ReverseToTurnSpotRequest request;
  request.stamp = now();
  request.active = out.reverse_active;
  request.has_stop_pose = out.reverse_active && out.reverse_to_spot && turn_spot_.has_value();
  if (request.has_stop_pose) {
    request.stop_pose = turn_spot_->stop_pose;
  }
  pub_reverse_request_->publish(request);
}

void TurnInPlaceSupervisorNode::on_timer()
{
  DecisionInput in;
  in.now = now().seconds();
  in.autonomous = operation_mode_ &&
                  operation_mode_->mode == OperationModeState::AUTONOMOUS &&
                  operation_mode_->is_autoware_control_enabled;
  in.route_changed = route_changed_;
  route_changed_ = false;
  if (in.route_changed) {
    turn_spot_.reset();
  }
  in.speed = odom_ ? std::abs(odom_->twist.twist.linear.x) : 0.0;

  const double ego_yaw = odom_ ? tf2::getYaw(odom_->pose.pose.orientation) : 0.0;
  const auto route_yaw = route_heading_at_ego();
  in.route_valid = route_yaw.has_value() && odom_ != nullptr;
  if (in.route_valid) {
    in.route_yaw = *route_yaw;
    in.yaw_error = normalize_angle(*route_yaw - ego_yaw);
  }

  // The sweep checks only when a turn is on the table (they cost a few thousand point tests).
  const auto state = decision_.state();
  const bool turn_relevant = state == SupervisorState::CHECK_CLEARANCE ||
                             state == SupervisorState::TURNING ||
                             state == SupervisorState::REQUEST_STOP;
  bool ccw_inside_road = false;
  bool cw_inside_road = false;
  if (in.route_valid && turn_relevant) {
    const double ccw = rotation_in_direction(ego_yaw, in.route_yaw, kDirectionCcw);
    const double cw = -rotation_in_direction(ego_yaw, in.route_yaw, kDirectionCw);
    const auto c = rotation_centre_in_map();
    const double near_radius = sweep_radius(footprint_, road_margin_) + 0.5;
    const auto road = to_body_frame(road_area_near(c.x, c.y, near_radius));
    const auto no_turn = to_body_frame(no_turn_area_near(c.x, c.y, near_radius));
    ccw_inside_road = sweep_inside_area(
      road, footprint_, road_margin_, ccw, sweep_step_rad_, footprint_sample_spacing_, no_turn);
    cw_inside_road = sweep_inside_area(
      road, footprint_, road_margin_, cw, sweep_step_rad_, footprint_sample_spacing_, no_turn);
    bool ccw_free = false;
    bool cw_free = false;
    if (const auto points = obstacle_points_around_rotation_centre()) {
      ccw_free = sweep_is_clear(*points, footprint_, sweep_margin_, ccw, sweep_step_rad_);
      cw_free = sweep_is_clear(*points, footprint_, sweep_margin_, cw, sweep_step_rad_);
    }
    in.ccw_clear = ccw_inside_road && ccw_free;
    in.cw_clear = cw_inside_road && cw_free;
    in.road_blocks_both = !ccw_inside_road && !cw_inside_road;
  }

  // Turn spot: searched when the road blocks the turn here, and again now and then while
  // backing up towards the goal (the road may widen on the way).  Once the robot is backing
  // up to a spot, that spot stays fixed.
  const bool reversing = state == SupervisorState::REVERSING_TO_TURN_SPOT;
  const bool search_due = in.now - last_spot_search_time_ >= spot_research_period_sec_;
  if (
    search_due && ((state == SupervisorState::CHECK_CLEARANCE && in.road_blocks_both) ||
                   (reversing && !last_output_.reverse_to_spot))) {
    turn_spot_ = search_turn_spot();
    last_spot_search_time_ = in.now;
  }
  if (state == SupervisorState::MONITOR) {
    turn_spot_.reset();
    last_spot_search_time_ = -1.0e9;
  }
  in.turn_spot_found = turn_spot_.has_value();
  // On the spot = near it and at rest for spot_settle_sec (a back-up decelerating through the
  // stop threshold for one cycle is not at rest yet).
  if (reversing && in.speed < decision_params_.stop_speed) {
    if (!stopped_since_) {
      stopped_since_ = in.now;
    }
  } else {
    stopped_since_.reset();
  }
  // Stopping while backing up is not precise (short of the stop point inside the controller's
  // stopping distance, past it through a curve), so the robot is not required to be on the spot:
  // once it comes to rest, the turning area is judged where it actually stands, and if the spin
  // still does not fit, the next spot is searched from there.
  if (turn_spot_ && odom_ && stopped_since_ && in.now - *stopped_since_ >= spot_settle_sec_) {
    in.at_turn_spot = true;
    const auto c = rotation_centre_in_map();
    const double d =
      std::hypot(c.x - turn_spot_->pose.position.x, c.y - turn_spot_->pose.position.y);
    if (d > spot_reached_tolerance_m_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[TURN-IN-PLACE] back-up came to rest %.2f m from the turn spot; judging the turn here", d);
    }
  }

  in.executor_fresh =
    executor_status_ &&
    (now() - rclcpp::Time(executor_status_->stamp)).seconds() <= executor_status_timeout_sec_;
  if (in.executor_fresh) {
    in.executor_state = static_cast<ExecutorState>(executor_status_->state);
  }

  const auto previous_state = decision_.state();
  const auto out = decision_.step(in);
  if (out.state != previous_state) {
    RCLCPP_INFO(
      get_logger(),
      "[TURN-IN-PLACE] %s -> %s: %s (yaw error %.1f deg, ccw %s/%s, cw %s/%s, turn spot %s)",
      to_string(previous_state), to_string(out.state), out.reason.c_str(),
      in.yaw_error / kDegToRad, ccw_inside_road ? "road ok" : "off road",
      in.ccw_clear ? "clear" : "blocked", cw_inside_road ? "road ok" : "off road",
      in.cw_clear ? "clear" : "blocked",
      turn_spot_ ? (std::to_string(turn_spot_->distance) + " m ahead").c_str() : "none");
    // A new phase searches afresh.
    last_spot_search_time_ = -1.0e9;
  }
  last_output_ = out;
  last_input_ = in;

  publish_hold(out.hold_vehicle);
  publish_reverse_request(out);

  TurnInPlaceCommand command;
  command.stamp = now();
  command.active = out.command_active;
  command.target_yaw = out.target_yaw;
  command.direction = static_cast<int8_t>(out.direction);
  pub_command_->publish(command);

  TurnInPlaceSupervisorState state_msg;
  state_msg.stamp = command.stamp;
  state_msg.state = static_cast<uint8_t>(out.state);
  state_msg.yaw_error = in.yaw_error;
  state_msg.target_yaw = out.target_yaw;
  state_msg.direction = static_cast<int8_t>(out.direction);
  state_msg.ccw_clear = in.ccw_clear;
  state_msg.cw_clear = in.cw_clear;
  state_msg.ccw_inside_road = ccw_inside_road;
  state_msg.cw_inside_road = cw_inside_road;
  state_msg.turn_spot_found = turn_spot_.has_value();
  state_msg.turn_spot_distance = turn_spot_ ? turn_spot_->distance : 0.0;
  state_msg.holding_vehicle = out.hold_vehicle;
  state_msg.reason = out.reason;
  pub_state_->publish(state_msg);

  publish_markers(in, out);
  diagnostics_.force_update();
}

void TurnInPlaceSupervisorNode::publish_markers(
  const DecisionInput & in, const DecisionOutput & out)
{
  if (!odom_ || pub_markers_->get_subscription_count() == 0) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  const auto & pose = odom_->pose.pose;
  const auto c = rotation_centre_in_map();
  const auto stamp = now();

  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = "map";
  circle.header.stamp = stamp;
  circle.ns = "sweep_circle";
  circle.id = 0;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.action = visualization_msgs::msg::Marker::ADD;
  circle.pose.orientation.w = 1.0;
  circle.scale.x = 0.04;
  const bool any_clear = in.ccw_clear || in.cw_clear;
  circle.color.a = 0.8;
  circle.color.r = out.state == SupervisorState::MONITOR ? 0.3 : (any_clear ? 0.1 : 1.0);
  circle.color.g = out.state == SupervisorState::MONITOR ? 0.3 : (any_clear ? 0.9 : 0.1);
  circle.color.b = out.state == SupervisorState::MONITOR ? 0.3 : 0.1;
  const double r = sweep_radius(footprint_, std::max(sweep_margin_, road_margin_));
  for (int i = 0; i <= 48; ++i) {
    geometry_msgs::msg::Point p;
    const double a = 2.0 * M_PI * i / 48.0;
    p.x = c.x + r * std::cos(a);
    p.y = c.y + r * std::sin(a);
    p.z = pose.position.z;
    circle.points.push_back(p);
  }
  markers.markers.push_back(circle);

  visualization_msgs::msg::Marker arrow;
  arrow.header = circle.header;
  arrow.ns = "route_heading";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = in.route_valid ? visualization_msgs::msg::Marker::ADD
                                : visualization_msgs::msg::Marker::DELETE;
  arrow.pose.position = c;
  arrow.pose.position.z += 0.1;
  arrow.pose.orientation = yaw_to_quaternion(in.route_yaw);
  arrow.scale.x = r;
  arrow.scale.y = 0.08;
  arrow.scale.z = 0.08;
  arrow.color.a = 0.9;
  arrow.color.b = 1.0;
  markers.markers.push_back(arrow);

  visualization_msgs::msg::Marker spot;
  spot.header = circle.header;
  spot.ns = "turn_spot";
  spot.id = 0;
  spot.type = visualization_msgs::msg::Marker::CYLINDER;
  spot.action = turn_spot_ ? visualization_msgs::msg::Marker::ADD
                           : visualization_msgs::msg::Marker::DELETE;
  if (turn_spot_) {
    spot.pose = turn_spot_->pose;
    spot.scale.x = spot.scale.y = 2.0 * sweep_radius(footprint_, road_margin_);
    spot.scale.z = 0.05;
    spot.color.a = 0.35;
    spot.color.g = 0.8;
    spot.color.b = 1.0;
  }
  markers.markers.push_back(spot);

  visualization_msgs::msg::Marker text;
  text.header = circle.header;
  text.ns = "state";
  text.id = 0;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.position = c;
  text.pose.position.z += 1.5;
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.3;
  text.color.a = 1.0;
  text.color.r = text.color.g = text.color.b = 1.0;
  text.text = std::string(to_string(out.state)) + " " +
              std::to_string(static_cast<int>(std::round(in.yaw_error / kDegToRad))) + "deg";
  markers.markers.push_back(text);

  pub_markers_->publish(markers);
}

void TurnInPlaceSupervisorNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto level = last_output_.state == SupervisorState::ERROR ? DiagnosticStatus::ERROR
                     : last_output_.state == SupervisorState::MONITOR ? DiagnosticStatus::OK
                                                                     : DiagnosticStatus::WARN;
  stat.summary(level, std::string(to_string(last_output_.state)) + ": " + last_output_.reason);
  stat.add("holding_vehicle", last_output_.hold_vehicle);
  stat.add("direction", last_output_.direction);
  stat.add("reverse_active", last_output_.reverse_active);
}

}  // namespace autoware::turn_in_place_supervisor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::turn_in_place_supervisor::TurnInPlaceSupervisorNode)
