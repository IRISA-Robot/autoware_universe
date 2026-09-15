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

#ifndef TOOLS__DELETE_NEAREST_OBJECT_HPP_
#define TOOLS__DELETE_NEAREST_OBJECT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <tier4_simulation_msgs/msg/dummy_object.hpp>

namespace rviz_plugins
{

/// Deletes the one dummy object you click nearest to, leaving the rest of the scene alone.
///
/// The existing single-object delete lives inside each spawning tool: it is alt+right-click
/// in interactive mode, and it looks the object up in a registry that tool keeps for itself.
/// That makes it unusable for building a scene -- interactive mode replaces ordinary
/// click-drag placement, and a tool can only delete what it personally spawned.
///
/// This tool takes the other route.  The dummy perception publisher copies each object's
/// uuid straight into the ground-truth TrackedObjects it publishes, so subscribing to those
/// gives the position AND the uuid of every object currently in the scene, whichever tool
/// created it.  Clicking picks the nearest and deletes it by uuid.
///
/// It needs `publish_ground_truth: true` on the dummy perception publisher; without that
/// the topic is silent and the tool says so rather than deleting something at random.
class DeleteNearestObjectTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  DeleteNearestObjectTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:  // NOLINT for Qt
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<tier4_simulation_msgs::msg::DummyObject>::SharedPtr dummy_object_info_pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr objects_sub_;
  autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr objects_;

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::StringProperty * objects_topic_property_;
  /// Clicks further than this from every object delete nothing.  Without it a stray click
  /// on empty map would remove whatever happened to be closest, possibly metres away.
  rviz_common::properties::FloatProperty * max_pick_distance_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__DELETE_NEAREST_OBJECT_HPP_
