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

#include "delete_nearest_object.hpp"

#include <rviz_common/display_context.hpp>

#include <cmath>
#include <limits>
#include <string>

namespace rviz_plugins
{
DeleteNearestObjectTool::DeleteNearestObjectTool()
{
  shortcut_key_ = 'x';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Pose Topic", "/simulation/dummy_perception_publisher/object_info",
    "The topic on which to publish dummy object info.", getPropertyContainer(), SLOT(updateTopic()),
    this);
  objects_topic_property_ = new rviz_common::properties::StringProperty(
    "Objects Topic", "/simulation/dummy_perception_publisher/output/debug/ground_truth_objects",
    "Ground-truth objects to pick from.  These carry the same uuid the delete is keyed on, "
    "so the publisher must be launched with publish_ground_truth:=true.",
    getPropertyContainer(), SLOT(updateTopic()), this);
  max_pick_distance_ = new rviz_common::properties::FloatProperty(
    "Max pick distance", 3.0, "Clicks further than this from every object delete nothing [m]",
    getPropertyContainer());
  max_pick_distance_->setMin(0.1);
}

void DeleteNearestObjectTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Delete Nearest Object");
  updateTopic();
}

void DeleteNearestObjectTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  dummy_object_info_pub_ = raw_node->create_publisher<tier4_simulation_msgs::msg::DummyObject>(
    topic_property_->getStdString(), 1);
  objects_sub_ = raw_node->create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
    objects_topic_property_->getStdString(), rclcpp::QoS{1},
    [this](const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg) {
      objects_ = msg;
    });
  clock_ = raw_node->get_clock();
}

// cppcheck-suppress unusedFunction
void DeleteNearestObjectTool::onPoseSet(double x, double y, [[maybe_unused]] double theta)
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  if (!objects_ || objects_->objects.empty()) {
    // Say which of the two it is.  Silence here is the difference between "nothing to
    // delete" and "the ground-truth topic was never switched on", and guessing wrong
    // costs a long time.
    RCLCPP_WARN(
      raw_node->get_logger(),
      "Delete Nearest Object: no objects on '%s'. Either the scene is empty, or the dummy "
      "perception publisher was not launched with publish_ground_truth:=true.",
      objects_topic_property_->getStdString().c_str());
    return;
  }

  const auto * nearest = &objects_->objects.front();
  double best = std::numeric_limits<double>::max();
  for (const auto & object : objects_->objects) {
    const auto & p = object.kinematics.pose_with_covariance.pose.position;
    const double distance = std::hypot(p.x - x, p.y - y);
    if (distance < best) {
      best = distance;
      nearest = &object;
    }
  }

  if (best > max_pick_distance_->getFloat()) {
    RCLCPP_INFO(
      raw_node->get_logger(),
      "Delete Nearest Object: nearest object is %.2f m away, beyond the %.2f m pick distance; "
      "nothing deleted.",
      best, max_pick_distance_->getFloat());
    return;
  }

  tier4_simulation_msgs::msg::DummyObject output_msg;
  output_msg.header.frame_id = context_->getFixedFrame().toStdString();
  output_msg.header.stamp = clock_->now();
  output_msg.action = tier4_simulation_msgs::msg::DummyObject::DELETE;
  // The publisher deletes by uuid, and the uuid it gave this object is the one it copied
  // into the ground-truth message we picked from.
  output_msg.id = nearest->object_id;
  dummy_object_info_pub_->publish(output_msg);

  RCLCPP_INFO(
    raw_node->get_logger(), "Delete Nearest Object: deleted the object %.2f m from the click.",
    best);
}

}  // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DeleteNearestObjectTool, rviz_common::Tool)
