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
#include <QTableWidget>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_road_crossing_msgs/msg/pedestrian_light_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tier4_simulation_msgs/msg/dummy_object.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rviz_plugins
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_road_crossing_msgs::msg::PedestrianLightState;
using tier4_simulation_msgs::msg::DummyObject;

// Saves the dummy objects placed with the 2D Dummy * tools to a YAML file and spawns them
// again on Load, so a test scene is laid out once and replayed on every re-run.
//
// The scene is rebuilt by listening to the same object_info topic the tools publish on:
// ADD/PREDICT add an object, MODIFY moves it, DELETE/DELETEALL remove it.  What gets saved
// is therefore each object's *initial* state, which is what a re-run wants.  Objects placed
// before the panel existed are not seen, which is why it lives in the default rviz config.
//
// The same file carries a traffic light routine (pedestrian green -> green flashing -> red ->
// red flashing per light), played by the traffic_light_sequencer node.  Load starts it
// together with the objects so every re-run sees the same signal timing.
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
  void onAddLight();
  void onRemoveLight();
  void onStartLights();
  void onStopLights();

private:  // NOLINT for Qt
  void onObjectInfo(const DummyObject::ConstSharedPtr msg);
  void onMap(const LaneletMapBin::ConstSharedPtr msg);
  void onSignals(const TrafficLightGroupArray::ConstSharedPtr msg);
  std::string scenarioPath(const QString & name) const;
  void setStatus(const QString & text, bool error = false);

  // durations: green, green flashing, red, red flashing [s]
  void addLightRow(int64_t id, const std::array<double, 4> & durations, int start, bool loop);
  // Table rows plus the hand-written lights the table cannot show; nullopt on a bad cell.
  std::optional<YAML::Node> lightsToYaml();
  void lightsFromYaml(const YAML::Node & lights);
  void sendRoutine(const YAML::Node & lights);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<DummyObject>::SharedPtr pub_;
  rclcpp::Subscription<DummyObject>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr routine_pub_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr signals_sub_;
  rclcpp::Subscription<PedestrianLightState>::SharedPtr pedestrian_light_sub_;

  mutable std::mutex mutex_;
  std::vector<DummyObject> scene_;
  // Lights the map offers, id -> what it is (tf_pedestrian way / crosswalk regulatory element)
  std::map<int64_t, std::string> map_lights_;
  bool map_lights_changed_{false};
  PedestrianLightState::ConstSharedPtr pedestrian_light_;
  std::map<int64_t, std::string> live_signals_;
  rclcpp::Time live_signals_stamp_;

  // Lights from a loaded file that are not the pedestrian pattern; kept verbatim.
  std::vector<YAML::Node> custom_lights_;

  QLineEdit * dir_edit_;
  QComboBox * scenario_combo_;
  QCheckBox * clear_before_load_;
  QLabel * count_label_;
  QLabel * status_label_;
  QComboBox * map_light_combo_;
  QTableWidget * light_table_;
  QLabel * custom_label_;
  QLabel * pedestrian_light_label_;
  QTimer * tick_;
};

}  // namespace rviz_plugins

#endif  // PANELS__DUMMY_OBJECT_SCENARIO_PANEL_HPP_
