// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware/freespace_planner/freespace_planner_node.hpp"

#include "autoware/freespace_planner/utils.hpp"
#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::freespace_planner
{
FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
    p.update_rate = declare_parameter<double>("update_rate");
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m");
    p.th_obstacle_time_sec = declare_parameter<double>("th_obstacle_time_sec");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
    // Defaulted, unlike the rest: the parking profile does not know about them, and without
    // them the planner behaves exactly as upstream.
    p.escape_vehicle_shape_margin_m =
      declare_parameter<double>("escape.vehicle_shape_margin_m", p.vehicle_shape_margin_m);
    p.escape_radius_m = declare_parameter<double>("escape.radius_m", 0.0);
    p.escape_max_start_overlap_ratio =
      declare_parameter<double>("escape.max_start_overlap_ratio", 0.0);
    p.replan_at_cusp_tolerance_m = declare_parameter<double>("replan_at_cusp_tolerance_m", -1.0);
    p.continuous_replan_enable = declare_parameter<bool>("continuous_replan.enable", false);
    p.continuous_replan_period_sec =
      declare_parameter<double>("continuous_replan.period_sec", 0.5);
    p.replan_when_obstacle_found = declare_parameter<bool>("replan_when_obstacle_found");
    p.replan_when_course_out = declare_parameter<bool>("replan_when_course_out");
  }

  // set vehicle_info
  {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base_length = vehicle_info.wheel_base_m;
    vehicle_shape_.max_steering = vehicle_info.max_steer_angle_rad;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
  }

  // Planning
  initializePlanningAlgorithm();

  // Subscribers
  route_sub_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&FreespacePlannerNode::onRoute, this, _1));

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    full_trajectory_pub_ = create_publisher<Trajectory>("~/output/full_trajectory", qos);
    debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos);
    debug_partial_pose_array_pub_ = create_publisher<PoseArray>("~/debug/partial_pose_array", qos);
    parking_state_pub_ = create_publisher<std_msgs::msg::Bool>("is_completed", qos);
    processing_time_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&FreespacePlannerNode::onTimer, this));
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
}

PlannerCommonParam FreespacePlannerNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.direction_change_weight = declare_parameter<double>("direction_change_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");
  p.max_turning_ratio = declare_parameter<double>("max_turning_ratio");
  p.turning_steps = declare_parameter<int>("turning_steps");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    return true;
  }

  // [STUCK-RECOVERY] partial_trajectory_ can legitimately be empty here -- for example
  // when get_partial_trajectory() collapsed to nothing because prev_target_index_ and
  // target_index_ ended up equal after a reset.  checkCurrentTrajectoryCollision() calls
  // findNearestIndex() on it, whose validateNonEmpty() THROWS, and nothing catches it:
  // the exception propagates out of onTimer() and std::terminate() takes down the whole
  // component container (costmap_generator included).  Observed as
  // "process has died [exit code -6]" with validateNonEmpty at the top of the stack.
  //
  // No partial trajectory to follow simply means a plan is required.
  if (partial_trajectory_.points.empty()) {
    return true;
  }

  if (replan_at_cusp_requested_) {
    replan_at_cusp_requested_ = false;
    return true;
  }

  if (node_param_.replan_when_obstacle_found && checkCurrentTrajectoryCollision()) {
    RCLCPP_DEBUG(get_logger(), "Found obstacle");
    return true;
  }

  if (node_param_.replan_when_course_out) {
    const bool is_course_out = utils::calc_distance_2d(trajectory_, current_pose_.pose) >
                               node_param_.th_course_out_distance_m;
    if (is_course_out) {
      RCLCPP_INFO(get_logger(), "Course out");
      return true;
    }
  }

  return false;
}

bool FreespacePlannerNode::checkCurrentTrajectoryCollision()
{
  algo_->setMap(*occupancy_grid_);

  const size_t nearest_index_partial = autoware::motion_utils::findNearestIndex(
    partial_trajectory_.points, current_pose_.pose.position);
  const size_t end_index_partial = partial_trajectory_.points.size() - 1;
  const auto forward_trajectory = utils::get_partial_trajectory(
    partial_trajectory_, nearest_index_partial, end_index_partial, get_clock());

  const bool is_obs_found =
    algo_->hasObstacleOnTrajectory(utils::trajectory_to_pose_array(forward_trajectory));

  if (!is_obs_found) {
    obs_found_time_ = {};
    return false;
  }

  if (!obs_found_time_) obs_found_time_ = get_clock()->now();

  return (get_clock()->now() - obs_found_time_.get()).seconds() > node_param_.th_obstacle_time_sec;
}

void FreespacePlannerNode::updateTargetIndex()
{
  if (!utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps)) {
    return;
  }

  const auto is_near_target = utils::is_near_target(
    trajectory_.points.at(target_index_).pose, current_pose_.pose,
    node_param_.th_arrived_distance_m);

  if (!is_near_target) return;

  const auto new_target_index =
    utils::get_next_target_index(trajectory_.points.size(), reversing_indices_, target_index_);

  // [STUCK-RECOVERY] The next segment was planned to start AT this cusp.  Handing it over
  // with the robot stopped short of the cusp -- anywhere inside th_arrived_distance_m --
  // makes the controller chase a path that begins somewhere the robot is not, with a heading
  // it does not have, which it does with a hard steer.  Measured: stopped 0.50 m before the
  // cusp of a 1.49 m reverse leg, then turned sharply into the forward leg.  Plan the rest
  // afresh from where the robot actually stands instead; it is stopped, which is all the
  // planner needs.
  if (new_target_index != target_index_ && node_param_.replan_at_cusp_tolerance_m >= 0.0) {
    const double miss = autoware_utils::calc_distance2d(
      trajectory_.points.at(target_index_).pose, current_pose_.pose);
    if (miss > node_param_.replan_at_cusp_tolerance_m) {
      RCLCPP_WARN(
        get_logger(),
        "stopped %.2f m from the cusp (tolerance %.2f m); replanning from here instead of "
        "starting the next segment",
        miss, node_param_.replan_at_cusp_tolerance_m);
      replan_at_cusp_requested_ = true;
      return;
    }
  }

  if (new_target_index == target_index_) {
    // Finished publishing all partial trajectories
    is_completed_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace planning completed");
    std_msgs::msg::Bool is_completed_msg;
    is_completed_msg.data = is_completed_;
    parking_state_pub_->publish(is_completed_msg);
  } else {
    // Switch to next partial trajectory
    prev_target_index_ = target_index_;
    target_index_ = new_target_index;
  }
}

void FreespacePlannerNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  route_ = msg;

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;

  // [STUCK-RECOVERY] In continuous mode a new goal is just a reason to plan again now.
  // reset() here is what made every goal change stop the robot: it dropped the plan and made
  // the next one wait for the vehicle to stand still.
  if (node_param_.continuous_replan_enable) {
    continuous_replan_requested_ = true;
    return;
  }

  is_new_parking_cycle_ = true;

  reset();
}

void FreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

void FreespacePlannerNode::updateData()
{
  occupancy_grid_ = occupancy_grid_sub_.take_data();

  {
    auto msgs = odom_sub_.take_data();
    for (const auto & msg : msgs) {
      onOdometry(msg);
    }
  }
}

bool FreespacePlannerNode::isDataReady()
{
  bool is_ready = true;

  if (!route_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for route data.");
    is_ready = false;
  }

  if (!occupancy_grid_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for occupancy grid.");
    is_ready = false;
  }

  if (!odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry.");
    is_ready = false;
  }

  return is_ready;
}

void FreespacePlannerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  scenario_ = scenario_sub_.take_data();
  if (!utils::is_active(scenario_)) {
    reset();
    return;
  }

  updateData();

  if (!isDataReady()) {
    return;
  }

  if (is_completed_) {
    partial_trajectory_.header = odom_->header;
    const auto stop_trajectory = utils::create_stop_trajectory(partial_trajectory_);
    trajectory_pub_->publish(stop_trajectory);
    return;
  }

  // Get current pose
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  if (current_pose_.header.frame_id == "") {
    return;
  }

  if (node_param_.continuous_replan_enable) {
    onTimerContinuous();
    return;
  }

  // Must stop before replanning any new trajectory
  const bool is_reset_required = !reset_in_progress_ && isPlanRequired();
  if (is_reset_required) {
    // Stop before planning new trajectory, except in a new parking cycle as the vehicle already
    // stops.
    if (!is_new_parking_cycle_) {
      const auto stop_trajectory = partial_trajectory_.points.empty()
                                     ? utils::create_stop_trajectory(current_pose_, get_clock())
                                     : utils::create_stop_trajectory(partial_trajectory_);
      trajectory_pub_->publish(stop_trajectory);
      debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(stop_trajectory));
      debug_partial_pose_array_pub_->publish(utils::trajectory_to_pose_array(stop_trajectory));
    }

    reset();

    reset_in_progress_ = true;
  }

  if (reset_in_progress_) {
    const auto is_ego_stopped =
      utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps);
    if (is_ego_stopped) {
      // Plan new trajectory
      planTrajectory();
      reset_in_progress_ = false;
    } else {
      // Will keep current stop trajectory
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for the vehicle to stop before generating a new trajectory.");
    }
  }

  // StopTrajectory
  if (trajectory_.points.size() <= 1) {
    is_new_parking_cycle_ = false;
    return;
  }

  // Update partial trajectory
  updateTargetIndex();
  partial_trajectory_ =
    utils::get_partial_trajectory(trajectory_, prev_target_index_, target_index_, get_clock());

  // Publish messages
  // [STUCK-RECOVERY] The full plan goes out first, and in the same tick as the segment cut
  // from it.  ~/output/trajectory only ever carries one direction segment, so a consumer
  // judging the manoeuvre from that alone sees the first leg, accepts it, and only learns
  // at the cusp that the next leg goes somewhere it would never have agreed to -- with the
  // robot already committed to half a manoeuvre.  The header stamp is the planner's own and
  // stays fixed for the life of one plan, so it identifies the plan.
  full_trajectory_pub_->publish(trajectory_);
  trajectory_pub_->publish(partial_trajectory_);
  debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(trajectory_));
  debug_partial_pose_array_pub_->publish(utils::trajectory_to_pose_array(partial_trajectory_));

  is_new_parking_cycle_ = false;

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}

void FreespacePlannerNode::onTimerContinuous()
{
  // [STUCK-RECOVERY] Planner server, Nav2-style.  None of the stop-before-replan machinery runs
  // here -- no stop trajectories, no waiting for the vehicle to stand still, no segments: the
  // consumer (the stuck-recovery supervisor) keeps driving the plan it has, cuts its own
  // segments, and decides whether each new plan is worth switching to.  This node only keeps
  // the plan fresh: from wherever the robot is now, every period, and at once on a new goal.
  // A failed search publishes nothing, which leaves the consumer on the plan it already has.
  const auto now = get_clock()->now();
  const bool due = continuous_replan_requested_ || !last_continuous_plan_time_ ||
                   (now - *last_continuous_plan_time_).seconds() >=
                     node_param_.continuous_replan_period_sec;
  if (!due) {
    return;
  }
  continuous_replan_requested_ = false;
  last_continuous_plan_time_ = now;

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  if (planTrajectory()) {
    full_trajectory_pub_->publish(trajectory_);
    debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(trajectory_));
  }

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}

bool FreespacePlannerNode::planTrajectory()
{
  if (occupancy_grid_ == nullptr) {
    return false;
  }

  // Provide robot shape and map for the planner
  algo_->setMap(*occupancy_grid_);

  // Calculate poses in costmap frame
  const auto current_pose_in_costmap_frame = utils::transform_pose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = utils::transform_pose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute planning
  const rclcpp::Time start = get_clock()->now();
  std::string error_msg;
  bool result = false;
  try {
    result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  } catch (const std::exception & e) {
    error_msg = e.what();
  }
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_DEBUG(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());

  if (result) {
    RCLCPP_DEBUG(get_logger(), "Found goal!");
    if (algo_->isStartEscapeActive()) {
      RCLCPP_WARN(
        get_logger(),
        "start pose is inside the %.2f m margin; planned out of it with the %.2f m escape "
        "margin for the first %.2f m (footprint already on %zu obstacle cell(s))",
        node_param_.vehicle_shape_margin_m, node_param_.escape_vehicle_shape_margin_m,
        node_param_.escape_radius_m, algo_->getStartEscapeOverlap());
    }
    trajectory_ = utils::create_trajectory(
      current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
    reversing_indices_ = utils::get_reversing_indices(trajectory_);
    prev_target_index_ = 0;
    target_index_ = utils::get_next_target_index(
      trajectory_.points.size(), reversing_indices_, prev_target_index_);
    return true;
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "Can't find goal: %s", error_msg.c_str());
  // In continuous mode keep the last plan: the consumer is still driving it, and this node
  // never republishes it anyway.  Otherwise the upstream behaviour: drop everything.
  if (!node_param_.continuous_replan_enable) {
    reset();
  }
  return false;
}

void FreespacePlannerNode::reset()
{
  replan_at_cusp_requested_ = false;
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
  parking_state_pub_->publish(is_completed_msg);
  obs_found_time_ = {};
}

TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  // Extend robot shape
  autoware::freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;
  extended_vehicle_shape.setMinMaxDimension();

  const auto planner_common_param = getPlannerCommonParam();

  const auto algo_name = node_param_.planning_algorithm;

  // initialize specified algorithm
  if (algo_name == "astar") {
    algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);
  } else if (algo_name == "rrtstar") {
    algo_ = std::make_unique<RRTStar>(planner_common_param, extended_vehicle_shape, *this);
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);

  // [STUCK-RECOVERY] Start escape.  A robot that stopped closer to an obstacle than
  // vehicle_shape_margin_m has a start pose that "collides", and the planner refuses it
  // outright ("Invalid start or goal pose") -- in precisely the situation where the only
  // way out is a plan that backs away.  Within escape.radius_m of such a start the robot is
  // checked with escape.vehicle_shape_margin_m instead; beyond it, and for every start that
  // is clear anyway, the full margin applies unchanged.
  const double escape_margin =
    std::clamp(node_param_.escape_vehicle_shape_margin_m, 0.0, margin);
  if (node_param_.escape_radius_m > 0.0 && escape_margin < margin) {
    autoware::freespace_planning_algorithms::VehicleShape escape_shape = vehicle_shape_;
    escape_shape.length += escape_margin;
    escape_shape.width += escape_margin;
    escape_shape.base2back += escape_margin / 2;
    escape_shape.setMinMaxDimension();
    algo_->setStartEscape(
      escape_shape, node_param_.escape_radius_m, node_param_.escape_max_start_overlap_ratio);
    RCLCPP_INFO(
      get_logger(),
      "start escape: margin %.2f m (instead of %.2f m) within %.2f m of a start that collides, "
      "ignoring up to %.0f%% of the footprint already on obstacle cells",
      escape_margin, margin, node_param_.escape_radius_m,
      100.0 * node_param_.escape_max_start_overlap_ratio);
  }
}
}  // namespace autoware::freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::freespace_planner::FreespacePlannerNode)
