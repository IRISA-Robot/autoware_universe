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

#ifndef PANELS__DUMMY_OBJECT_SCENARIO_PANEL_HPP_
#define PANELS__DUMMY_OBJECT_SCENARIO_PANEL_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <tier4_simulation_msgs/msg/dummy_object.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace rviz_plugins
{
using tier4_simulation_msgs::msg::DummyObject;

// Saves the dummy objects placed with the 2D Dummy * tools to a YAML file and spawns them
// again on Load, so a test scene is laid out once and replayed on every re-run.
//
// The scene is rebuilt by listening to the same object_info topic the tools publish on:
// ADD/PREDICT add an object, MODIFY moves it, DELETE/DELETEALL remove it.  What gets saved
// is therefore each object's *initial* state, which is what a re-run wants.  Objects placed
// before the panel existed are not seen, which is why it lives in the default rviz config.
class DummyObjectScenarioPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DummyObjectScenarioPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onBrowseDir();
  void onRefresh();
  void onSave();
  void onLoad();
  void onClear();
  void onTick();

private:  // NOLINT for Qt
  void onObjectInfo(const DummyObject::ConstSharedPtr msg);
  std::string scenarioPath(const QString & name) const;
  void setStatus(const QString & text, bool error = false);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<DummyObject>::SharedPtr pub_;
  rclcpp::Subscription<DummyObject>::SharedPtr sub_;

  mutable std::mutex mutex_;
  std::vector<DummyObject> scene_;

  QLineEdit * dir_edit_;
  QComboBox * scenario_combo_;
  QCheckBox * clear_before_load_;
  QLabel * count_label_;
  QLabel * status_label_;
  QTimer * tick_;
};

}  // namespace rviz_plugins

#endif  // PANELS__DUMMY_OBJECT_SCENARIO_PANEL_HPP_
