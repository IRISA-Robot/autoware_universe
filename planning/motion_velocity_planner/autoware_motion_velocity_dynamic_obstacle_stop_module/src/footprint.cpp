// Copyright 2023 TIER IV, Inc.
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

#include "footprint.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/envelope.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
autoware_utils::MultiPolygon2d make_forward_footprints(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & obstacles,
  const PlannerParam & params, const double hysteresis)
{
  autoware_utils::MultiPolygon2d forward_footprints;
  for (const auto & obstacle : obstacles)
    forward_footprints.push_back(project_to_pose(
      make_forward_footprint(obstacle, params, hysteresis),
      obstacle.kinematics.initial_pose_with_covariance.pose));
  return forward_footprints;
}

autoware_utils::Polygon2d make_forward_footprint(
  const autoware_perception_msgs::msg::PredictedObject & obstacle, const PlannerParam & params,
  const double hysteresis)
{
  const auto & shape = obstacle.shape.dimensions;
  const auto longitudinal_offset =
    shape.x / 2.0 +
    obstacle.kinematics.initial_twist_with_covariance.twist.linear.x * params.time_horizon;
  const auto lateral_offset = (shape.y + params.extra_object_width) / 2.0 + hysteresis;
  return autoware_utils::Polygon2d{
    {{longitudinal_offset, -lateral_offset},
     {longitudinal_offset, lateral_offset},
     {shape.x / 2.0, lateral_offset},
     {shape.x / 2.0, -lateral_offset},
     {longitudinal_offset, -lateral_offset}},
    {}};
}

autoware_utils::Polygon2d project_to_pose(
  const autoware_utils::Polygon2d & base_footprint, const geometry_msgs::msg::Pose & pose)
{
  const auto angle = tf2::getYaw(pose.orientation);
  const auto rotated_footprint = autoware_utils::rotate_polygon(base_footprint, angle);
  autoware_utils::Polygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.outer().emplace_back(p.x() + pose.position.x, p.y() + pose.position.y);
  return footprint;
}

void make_ego_footprint_rtree(EgoData & ego_data, const PlannerParam & params)
{
  // Each trajectory point's footprint only extends towards ego's LEADING edge (the direction it
  // is about to sweep into), relying on consecutive trajectory points' footprints to overlap and
  // cover the rest of the vehicle body along the path. When driving forward that leading edge is
  // the front bumper (extend from base_link towards +x, i.e. base_to_front); when reversing it is
  // the rear bumper (extend towards -x, i.e. base_to_rear), since ego then sweeps backwards
  // relative to the pose's heading. Using the front-only extension unconditionally while
  // reversing would build a footprint pointing away from ego's actual direction of travel and
  // miss collisions behind ego.
  const auto base_to_front = ego_data.is_driving_forward ? params.ego_longitudinal_offset : 0.0;
  const auto base_to_rear =
    ego_data.is_driving_forward ? 0.0 : params.ego_rear_longitudinal_offset;
  for (const auto & p : ego_data.trajectory)
    ego_data.trajectory_footprints.push_back(
      autoware_utils::to_footprint(
        p.pose, base_to_front, base_to_rear, params.ego_lateral_offset * 2.0));
  std::vector<BoxIndexPair> rtree_nodes;
  rtree_nodes.reserve(ego_data.trajectory_footprints.size());
  for (auto i = 0UL; i < ego_data.trajectory_footprints.size(); ++i) {
    const auto box =
      boost::geometry::return_envelope<autoware_utils::Box2d>(ego_data.trajectory_footprints[i]);
    rtree_nodes.emplace_back(box, i);
  }
  ego_data.rtree = Rtree(rtree_nodes);
}

}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
