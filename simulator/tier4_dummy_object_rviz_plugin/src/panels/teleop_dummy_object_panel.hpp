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

#ifndef PANELS__TELEOP_DUMMY_OBJECT_PANEL_HPP_
#define PANELS__TELEOP_DUMMY_OBJECT_PANEL_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QButtonGroup>
#include <QElapsedTimer>
#include <QLabel>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_simulation_msgs/msg/dummy_object.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

namespace rviz_plugins
{
using tier4_simulation_msgs::msg::DummyObject;

/// One pedestrian driven from the keyboard, for experiments that need an object to move the
/// way you want right now rather than in a straight line set up beforehand.
///
/// The dummy perception publisher replaces an object wholesale on MODIFY (matched by uuid)
/// and moves it straight along its heading between messages.  So the panel integrates the
/// pose itself and re-sends it at 20 Hz; the object goes through the same pointcloud ->
/// detection -> tracking -> prediction chain as any other dummy object.
///
/// It lives in a panel and not in a terminal script because driving needs real key-release
/// events: a terminal only sees auto-repeat, so it can neither tell a short tap from a hold
/// nor hold W and A together.  Qt gives press and release, and the panel needs the keyboard
/// focus (click it) to get them.
///
///   hold W / S   throttle forward / backward (the other one brakes first)
///   release      coasts to a stop
///   hold A / D   turn left / right
///   1 / 2 / 3    max speed preset
///
/// The RelocateTeleopObjectTool moves it (and stops it) with a click-drag on the map.
class TeleopDummyObjectPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TeleopDummyObjectPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

protected:
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;
  void focusInEvent(QFocusEvent * event) override;
  void focusOutEvent(QFocusEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;

private Q_SLOTS:
  void onSpawn();
  void onRemove();
  void onTick();
  void onMaxSpeed(int index);

private:  // NOLINT for Qt
  enum Key { kW, kS, kA, kD, kKeyCount };

  void onObjectInfo(const DummyObject::ConstSharedPtr msg);
  void onRelocate(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  // Pose in map of `pose` given in `frame`; nullopt (with the reason in the status) without tf.
  std::optional<std::array<double, 3>> toMap(
    const std::string & frame, const geometry_msgs::msg::Pose & pose);
  bool held(Key key, qint64 now_ms) const;
  void releaseAll();
  DummyObject message(uint8_t action) const;
  void place(double x, double y, double yaw);
  void setStatus(const QString & text, bool error = false);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<DummyObject>::SharedPtr pub_;
  rclcpp::Subscription<DummyObject>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr relocate_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Whether the publisher has the object, as seen on object_info: our own ADD turns it on,
  // a DELETE of our uuid or a DELETEALL (Delete Nearest, the scenario panel's Clear) off.
  std::atomic<bool> spawned_{false};
  std::mutex relocate_mutex_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pending_relocate_;

  // State in map, integrated here.
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double v_{0.0};
  double max_speed_{1.4};

  std::array<bool, kKeyCount> down_{};
  // When a key was released, for the debounce; -1 when it was not.
  std::array<qint64, kKeyCount> released_ms_{-1, -1, -1, -1};
  QElapsedTimer clock_;
  qint64 last_tick_ms_{0};

  QButtonGroup * speed_group_;
  QLabel * focus_label_;
  QLabel * state_label_;
  QLabel * status_label_;
  QTimer * tick_;
};

}  // namespace rviz_plugins

#endif  // PANELS__TELEOP_DUMMY_OBJECT_PANEL_HPP_
