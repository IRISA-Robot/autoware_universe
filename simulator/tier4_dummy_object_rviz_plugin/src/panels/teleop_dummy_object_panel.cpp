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

#include "teleop_dummy_object_panel.hpp"

#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace rviz_plugins
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

constexpr const char * kObjectInfoTopic = "/simulation/dummy_perception_publisher/object_info";
constexpr const char * kRelocateTopic = "/simulation/dummy_teleop/relocate";

constexpr int kTickMs = 50;
constexpr double kAccel = 1.0;                   // holding W / S [m/s^2]
constexpr double kDecel = 1.5;                   // nothing held: coasting to a stop [m/s^2]
constexpr double kTurnRate = 45.0 * M_PI / 180;  // holding A / D [rad/s]
constexpr double kSpawnAhead = 5.0;              // [m] in front of the robot
constexpr std::array<double, 3> kMaxSpeeds{0.5, 1.4, 3.0};
constexpr int kDefaultSpeedIndex = 1;
// A release followed by a press this soon is auto-repeat that the X server (or NoMachine)
// did not mark as such; treat the key as held through it.
constexpr qint64 kReleaseDebounceMs = 40;

// Fixed, so there is only ever one teleop object and a respawn replaces it.
const std::array<uint8_t, 16> kUuid{'t', 'e', 'l', 'e', 'o', 'p', '-', 'd',
                                    'u', 'm', 'm', 'y', '-', 'o', 'b', 'j'};

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

TeleopDummyObjectPanel::TeleopDummyObjectPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * spawn_button = new QPushButton("Spawn");
  spawn_button->setToolTip(
    QString("(Re)spawn the pedestrian %1 m ahead of the robot, facing it").arg(kSpawnAhead));
  auto * remove_button = new QPushButton("Remove");
  auto * button_layout = new QHBoxLayout;
  button_layout->addWidget(spawn_button);
  button_layout->addWidget(remove_button);

  speed_group_ = new QButtonGroup(this);
  speed_group_->setExclusive(true);
  auto * speed_layout = new QHBoxLayout;
  speed_layout->addWidget(new QLabel("Max speed"));
  for (size_t i = 0; i < kMaxSpeeds.size(); ++i) {
    auto * button = new QPushButton(QString("%1: %2 m/s").arg(i + 1).arg(kMaxSpeeds[i]));
    button->setCheckable(true);
    button->setFocusPolicy(Qt::NoFocus);
    speed_group_->addButton(button, static_cast<int>(i));
    speed_layout->addWidget(button);
  }
  speed_group_->button(kDefaultSpeedIndex)->setChecked(true);
  max_speed_ = kMaxSpeeds[kDefaultSpeedIndex];

  // The keyboard only reaches the panel while it has the focus; clicking a button must not
  // take it away.
  spawn_button->setFocusPolicy(Qt::NoFocus);
  remove_button->setFocusPolicy(Qt::NoFocus);
  setFocusPolicy(Qt::StrongFocus);

  focus_label_ = new QLabel;
  state_label_ = new QLabel;
  status_label_ = new QLabel;
  status_label_->setWordWrap(true);
  auto * help_label = new QLabel(
    "Hold W/S: throttle fwd/back (release coasts to a stop)   Hold A/D: turn   1/2/3: max speed\n"
    "Relocate Teleop Object tool: click-drag on the map to move it (it stops)");
  help_label->setWordWrap(true);

  auto * layout = new QVBoxLayout;
  layout->addLayout(button_layout);
  layout->addLayout(speed_layout);
  layout->addWidget(focus_label_);
  layout->addWidget(state_label_);
  layout->addWidget(help_label);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(spawn_button, &QPushButton::clicked, this, &TeleopDummyObjectPanel::onSpawn);
  connect(remove_button, &QPushButton::clicked, this, &TeleopDummyObjectPanel::onRemove);
  connect(speed_group_, &QButtonGroup::idClicked, this, &TeleopDummyObjectPanel::onMaxSpeed);

  tick_ = new QTimer(this);
  connect(tick_, &QTimer::timeout, this, &TeleopDummyObjectPanel::onTick);
  releaseAll();
  focus_label_->setText("Keyboard: click this panel to drive");
}

void TeleopDummyObjectPanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pub_ = raw_node_->create_publisher<DummyObject>(kObjectInfoTopic, rclcpp::QoS{10});
  sub_ = raw_node_->create_subscription<DummyObject>(
    kObjectInfoTopic, rclcpp::QoS{100},
    [this](const DummyObject::ConstSharedPtr msg) { onObjectInfo(msg); });
  relocate_sub_ = raw_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    kRelocateTopic, rclcpp::QoS{1},
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { onRelocate(msg); });
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(raw_node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  clock_.start();
  last_tick_ms_ = clock_.elapsed();
  tick_->start(kTickMs);
}

void TeleopDummyObjectPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Max Speed Index", speed_group_->checkedId());
}

void TeleopDummyObjectPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  int index{};
  if (
    config.mapGetInt("Max Speed Index", &index) && index >= 0 &&
    index < static_cast<int>(kMaxSpeeds.size())) {
    speed_group_->button(index)->setChecked(true);
    onMaxSpeed(index);
  }
}

void TeleopDummyObjectPanel::keyPressEvent(QKeyEvent * event)
{
  if (event->isAutoRepeat()) {
    return;
  }
  const auto press = [&](Key key) {
    down_[key] = true;
    released_ms_[key] = -1;
  };
  switch (event->key()) {
    case Qt::Key_W:
      press(kW);
      return;
    case Qt::Key_S:
      press(kS);
      return;
    case Qt::Key_A:
      press(kA);
      return;
    case Qt::Key_D:
      press(kD);
      return;
    case Qt::Key_1:
    case Qt::Key_2:
    case Qt::Key_3: {
      const int index = event->key() - Qt::Key_1;
      speed_group_->button(index)->setChecked(true);
      onMaxSpeed(index);
      return;
    }
    default:
      rviz_common::Panel::keyPressEvent(event);
  }
}

void TeleopDummyObjectPanel::keyReleaseEvent(QKeyEvent * event)
{
  if (event->isAutoRepeat()) {
    return;
  }
  const auto release = [&](Key key) {
    down_[key] = false;
    released_ms_[key] = clock_.elapsed();
  };
  switch (event->key()) {
    case Qt::Key_W:
      release(kW);
      return;
    case Qt::Key_S:
      release(kS);
      return;
    case Qt::Key_A:
      release(kA);
      return;
    case Qt::Key_D:
      release(kD);
      return;
    default:
      rviz_common::Panel::keyReleaseEvent(event);
  }
}

void TeleopDummyObjectPanel::focusInEvent(QFocusEvent * event)
{
  rviz_common::Panel::focusInEvent(event);
  focus_label_->setText("Keyboard: <b style='color:#2e7d32'>active</b>");
}

void TeleopDummyObjectPanel::focusOutEvent(QFocusEvent * event)
{
  rviz_common::Panel::focusOutEvent(event);
  // The release of a key held while the focus left never reaches us; without this the
  // object would keep accelerating.
  releaseAll();
  focus_label_->setText("Keyboard: click this panel to drive");
}

void TeleopDummyObjectPanel::mousePressEvent(QMouseEvent * event)
{
  setFocus(Qt::MouseFocusReason);
  rviz_common::Panel::mousePressEvent(event);
}

void TeleopDummyObjectPanel::releaseAll()
{
  down_.fill(false);
  released_ms_.fill(-1);
}

bool TeleopDummyObjectPanel::held(Key key, qint64 now_ms) const
{
  return down_[key] ||
         (released_ms_[key] >= 0 && now_ms - released_ms_[key] < kReleaseDebounceMs);
}

void TeleopDummyObjectPanel::onMaxSpeed(int index)
{
  max_speed_ = kMaxSpeeds.at(index);
}

void TeleopDummyObjectPanel::onObjectInfo(const DummyObject::ConstSharedPtr msg)
{
  if (msg->action == DummyObject::DELETEALL) {
    spawned_ = false;
    return;
  }
  if (msg->id.uuid != kUuid) {
    return;
  }
  if (msg->action == DummyObject::ADD || msg->action == DummyObject::PREDICT) {
    spawned_ = true;
  } else if (msg->action == DummyObject::DELETE) {
    spawned_ = false;
  }
}

void TeleopDummyObjectPanel::onRelocate(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // Applied on the Qt side in onTick, where the rest of the state lives.
  std::lock_guard<std::mutex> lock(relocate_mutex_);
  pending_relocate_ = msg;
}

std::optional<std::array<double, 3>> TeleopDummyObjectPanel::toMap(
  const std::string & frame, const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    setStatus(QString("No tf map <- %1: %2").arg(frame.c_str()).arg(e.what()), true);
    return std::nullopt;
  }
  geometry_msgs::msg::Pose in_map;
  tf2::doTransform(pose, in_map, transform);
  return std::array<double, 3>{in_map.position.x, in_map.position.y, tf2::getYaw(in_map.orientation)};
}

DummyObject TeleopDummyObjectPanel::message(uint8_t action) const
{
  DummyObject object{};
  object.header.frame_id = "map";
  object.header.stamp = raw_node_->now();
  object.id.uuid = kUuid;
  object.action = action;
  if (action == DummyObject::DELETE) {
    return object;
  }

  // Same pedestrian as the 2D Dummy Pedestrian tool.
  object.classification.label = ObjectClassification::PEDESTRIAN;
  object.classification.probability = 1.0;
  object.shape.type = Shape::CYLINDER;
  object.shape.dimensions.x = 0.6;
  object.shape.dimensions.y = 0.6;
  object.shape.dimensions.z = 2.0;

  auto & pose = object.initial_state.pose_covariance.pose;
  pose.position.x = x_;
  pose.position.y = y_;
  pose.position.z = 1.0;  // the publisher puts it on the robot's ground anyway
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  pose.orientation = tf2::toMsg(q);
  auto & cov = object.initial_state.pose_covariance.covariance;
  cov[0] = cov[7] = cov[14] = 0.03 * 0.03;
  cov[35] = std::pow(5.0 * M_PI / 180.0, 2);

  // The publisher carries it on at this speed until the next message.
  object.initial_state.twist_covariance.twist.linear.x = v_;
  object.max_velocity = 33.3;
  object.min_velocity = -33.3;  // backing up is a negative speed
  return object;
}

void TeleopDummyObjectPanel::place(double x, double y, double yaw)
{
  x_ = x;
  y_ = y;
  yaw_ = normalizeAngle(yaw);
  v_ = 0.0;
  // DELETE + ADD rather than MODIFY: it also brings back an object something else deleted.
  pub_->publish(message(DummyObject::DELETE));
  pub_->publish(message(DummyObject::ADD));
  setStatus(QString("Placed at (%1, %2), yaw %3 deg")
              .arg(x_, 0, 'f', 2)
              .arg(y_, 0, 'f', 2)
              .arg(yaw_ * 180.0 / M_PI, 0, 'f', 0));
}

void TeleopDummyObjectPanel::onSpawn()
{
  const auto robot = toMap("base_link", geometry_msgs::msg::Pose{});
  if (!robot) {
    return;
  }
  const auto [bx, by, byaw] = *robot;
  place(bx + kSpawnAhead * std::cos(byaw), by + kSpawnAhead * std::sin(byaw), byaw + M_PI);
}

void TeleopDummyObjectPanel::onRemove()
{
  pub_->publish(message(DummyObject::DELETE));
  v_ = 0.0;
  setStatus("Removed");
}

void TeleopDummyObjectPanel::onTick()
{
  const qint64 now = clock_.elapsed();
  const double dt = std::min(static_cast<double>(now - last_tick_ms_) / 1000.0, 0.2);
  last_tick_ms_ = now;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr relocate;
  {
    std::lock_guard<std::mutex> lock(relocate_mutex_);
    std::swap(relocate, pending_relocate_);
  }
  if (relocate) {
    if (const auto pose = toMap(relocate->header.frame_id, relocate->pose)) {
      place((*pose)[0], (*pose)[1], (*pose)[2]);
    }
  }

  if (!spawned_) {
    v_ = 0.0;
    state_label_->setText("No teleop object -- press Spawn or use the Relocate tool");
    return;
  }

  // Throttle: W and S push the speed up or down (so the other one brakes first); with
  // neither, or both, it coasts to a stop.
  const bool forward = held(kW, now);
  const bool backward = held(kS, now);
  if (forward != backward) {
    v_ += (forward ? kAccel : -kAccel) * dt;
  } else if (std::abs(v_) <= kDecel * dt) {
    v_ = 0.0;
  } else {
    v_ -= std::copysign(kDecel * dt, v_);
  }
  // Over the max (just lowered, or throttling into it): come down to it, not snap.
  if (std::abs(v_) > max_speed_) {
    v_ = std::copysign(std::max(std::abs(v_) - kDecel * dt, max_speed_), v_);
  }

  const int turn = (held(kA, now) ? 1 : 0) - (held(kD, now) ? 1 : 0);
  yaw_ = normalizeAngle(yaw_ + turn * kTurnRate * dt);
  x_ += v_ * std::cos(yaw_) * dt;
  y_ += v_ * std::sin(yaw_) * dt;

  pub_->publish(message(DummyObject::MODIFY));
  state_label_->setText(QString("v %1 m/s (max %2)   x %3  y %4  yaw %5 deg")
                          .arg(v_, 0, 'f', 2)
                          .arg(max_speed_, 0, 'f', 1)
                          .arg(x_, 0, 'f', 2)
                          .arg(y_, 0, 'f', 2)
                          .arg(yaw_ * 180.0 / M_PI, 0, 'f', 0));
}

void TeleopDummyObjectPanel::setStatus(const QString & text, bool error)
{
  status_label_->setStyleSheet(error ? "color: #c62828;" : "");
  status_label_->setText(text);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TeleopDummyObjectPanel, rviz_common::Panel)
