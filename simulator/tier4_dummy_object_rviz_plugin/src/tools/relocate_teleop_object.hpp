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

#ifndef TOOLS__RELOCATE_TELEOP_OBJECT_HPP_
#define TOOLS__RELOCATE_TELEOP_OBJECT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rviz_plugins
{

/// Click-drag on the map to put the TeleopDummyObjectPanel's pedestrian there, facing the
/// drag, and stopped -- for when driving it over with the keyboard is not worth it.  Spawns
/// it if there is none.  The panel owns the object; this only sends it the pose.
class RelocateTeleopObjectTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  RelocateTeleopObjectTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:  // NOLINT for Qt
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rviz_common::properties::StringProperty * topic_property_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__RELOCATE_TELEOP_OBJECT_HPP_
