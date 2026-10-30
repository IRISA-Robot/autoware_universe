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

#include "relocate_teleop_object.hpp"

#include <rviz_common/display_context.hpp>

#include <tf2/LinearMath/Quaternion.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace rviz_plugins
{
RelocateTeleopObjectTool::RelocateTeleopObjectTool()
{
  // Delete All Objects' key; this tool took its place in the toolbar.
  shortcut_key_ = 'd';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Pose Topic", "/simulation/dummy_teleop/relocate",
    "The topic the Teleop Dummy Object panel takes the new pose from.", getPropertyContainer(),
    SLOT(updateTopic()), this);
}

void RelocateTeleopObjectTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Relocate Teleop Object");
  updateTopic();
}

void RelocateTeleopObjectTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pose_pub_ = raw_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), 1);
  clock_ = raw_node->get_clock();
}

// cppcheck-suppress unusedFunction
void RelocateTeleopObjectTool::onPoseSet(double x, double y, double theta)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = clock_->now();
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  msg.pose.orientation = tf2::toMsg(q);
  pose_pub_->publish(msg);
}

}  // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::RelocateTeleopObjectTool, rviz_common::Tool)
