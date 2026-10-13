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

#include "dummy_object_scenario_panel.hpp"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QVBoxLayout>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rviz_common/display_context.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace rviz_plugins
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

using autoware_perception_msgs::msg::TrafficLightElement;

constexpr const char * kObjectInfoTopic = "/simulation/dummy_perception_publisher/object_info";
constexpr const char * kRoutineTopic = "/simulation/traffic_light_sequencer/input/routine";
constexpr const char * kSignalsTopic = "/perception/traffic_light_recognition/traffic_signals";
constexpr const char * kMapTopic = "/map/vector_map";
constexpr const char * kPedestrianLightTopic = "/perception/road_crossing/pedestrian_light";

enum LightColumn {
  kColId,
  kColGreen,
  kColGreenFlashing,
  kColRed,
  kColRedFlashing,
  kColStart,
  kColLoop,
  kColNow
};

// The pedestrian light cycle a table row stands for, one phase per duration column.
// A phase can be 0 s long (e.g. no red flashing at this crossing): it is then skipped.
struct PatternPhase
{
  int column;
  const char * color;
  const char * status;
  const char * name;  // in the Start combo
};
const std::array<PatternPhase, 4> kPattern{{
  {kColGreen, "GREEN", "SOLID_ON", "Green"},
  {kColGreenFlashing, "GREEN", "FLASHING", "Green flash"},
  {kColRed, "RED", "SOLID_ON", "Red"},
  {kColRedFlashing, "RED", "FLASHING", "Red flash"},
}};

const std::map<uint8_t, std::string> kLabelNames = {
  {ObjectClassification::UNKNOWN, "UNKNOWN"},       {ObjectClassification::CAR, "CAR"},
  {ObjectClassification::TRUCK, "TRUCK"},           {ObjectClassification::BUS, "BUS"},
  {ObjectClassification::TRAILER, "TRAILER"},       {ObjectClassification::MOTORCYCLE, "MOTORCYCLE"},
  {ObjectClassification::BICYCLE, "BICYCLE"},       {ObjectClassification::PEDESTRIAN, "PEDESTRIAN"},
};

const std::map<uint8_t, std::string> kShapeNames = {
  {Shape::BOUNDING_BOX, "BOUNDING_BOX"},
  {Shape::CYLINDER, "CYLINDER"},
  {Shape::POLYGON, "POLYGON"},
};

// Names keep the file readable and hand-editable; numbers are accepted too.
template <typename T>
T fromName(const YAML::Node & node, const std::map<uint8_t, std::string> & names)
{
  const auto text = node.as<std::string>();
  for (const auto & [value, name] : names) {
    if (name == text) {
      return value;
    }
  }
  return static_cast<T>(node.as<int>());
}

std::string toName(uint8_t value, const std::map<uint8_t, std::string> & names)
{
  const auto it = names.find(value);
  return it != names.end() ? it->second : std::to_string(value);
}

void randomUuid(DummyObject & object)
{
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(object.id.uuid.begin(), object.id.uuid.end(), bit_eng);
}

YAML::Node toYaml(const DummyObject & object)
{
  const auto & pose = object.initial_state.pose_covariance.pose;
  const auto & cov = object.initial_state.pose_covariance.covariance;
  const auto & q = pose.orientation;
  double roll{}, pitch{}, yaw{};
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);

  YAML::Node node;
  node["frame_id"] = object.header.frame_id;
  node["action"] = object.action == DummyObject::PREDICT ? "PREDICT" : "ADD";
  node["label"] = toName(object.classification.label, kLabelNames);
  node["probability"] = object.classification.probability;

  YAML::Node shape;
  shape["type"] = toName(object.shape.type, kShapeNames);
  shape["dimensions"] = std::vector<double>{
    object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  for (const auto & p : object.shape.footprint.points) {
    shape["footprint"].push_back(std::vector<double>{p.x, p.y});
  }
  node["shape"] = shape;

  node["pose"]["x"] = pose.position.x;
  node["pose"]["y"] = pose.position.y;
  node["pose"]["z"] = pose.position.z;
  node["pose"]["yaw"] = yaw;

  node["std_dev"]["x"] = std::sqrt(cov[0]);
  node["std_dev"]["y"] = std::sqrt(cov[7]);
  node["std_dev"]["z"] = std::sqrt(cov[14]);
  node["std_dev"]["yaw"] = std::sqrt(cov[35]);

  node["velocity"] = object.initial_state.twist_covariance.twist.linear.x;
  node["accel"] = object.initial_state.accel_covariance.accel.linear.x;
  node["max_velocity"] = object.max_velocity;
  node["min_velocity"] = object.min_velocity;
  return node;
}

DummyObject fromYaml(const YAML::Node & node)
{
  DummyObject object{};
  object.header.frame_id = node["frame_id"].as<std::string>("map");
  object.action = node["action"].as<std::string>("ADD") == "PREDICT" ? DummyObject::PREDICT
                                                                      : DummyObject::ADD;
  object.classification.label = fromName<uint8_t>(node["label"], kLabelNames);
  object.classification.probability = node["probability"].as<float>(1.0f);

  const auto & shape = node["shape"];
  object.shape.type = fromName<uint8_t>(shape["type"], kShapeNames);
  const auto dimensions = shape["dimensions"].as<std::vector<double>>();
  object.shape.dimensions.x = dimensions.at(0);
  object.shape.dimensions.y = dimensions.at(1);
  object.shape.dimensions.z = dimensions.at(2);
  if (shape["footprint"]) {
    for (const auto & p : shape["footprint"]) {
      geometry_msgs::msg::Point32 point;
      point.x = p[0].as<float>();
      point.y = p[1].as<float>();
      object.shape.footprint.points.push_back(point);
    }
  }

  auto & pose = object.initial_state.pose_covariance.pose;
  pose.position.x = node["pose"]["x"].as<double>();
  pose.position.y = node["pose"]["y"].as<double>();
  pose.position.z = node["pose"]["z"].as<double>(0.0);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, node["pose"]["yaw"].as<double>(0.0));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  auto & cov = object.initial_state.pose_covariance.covariance;
  const auto & std_dev = node["std_dev"];
  const auto sq = [](double v) { return v * v; };
  cov[0] = sq(std_dev["x"].as<double>(0.03));
  cov[7] = sq(std_dev["y"].as<double>(0.03));
  cov[14] = sq(std_dev["z"].as<double>(0.03));
  cov[35] = sq(std_dev["yaw"].as<double>(5.0 * M_PI / 180.0));

  object.initial_state.twist_covariance.twist.linear.x = node["velocity"].as<double>(0.0);
  object.initial_state.accel_covariance.accel.linear.x = node["accel"].as<double>(0.0);
  object.max_velocity = node["max_velocity"].as<float>(33.3f);
  object.min_velocity = node["min_velocity"].as<float>(-33.3f);
  return object;
}
}  // namespace


DummyObjectScenarioPanel::DummyObjectScenarioPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  dir_edit_ = new QLineEdit(QDir::home().filePath("dummy_object_scenarios"));
  auto * browse_button = new QPushButton("...");
  browse_button->setMaximumWidth(30);
  auto * dir_layout = new QHBoxLayout;
  dir_layout->addWidget(new QLabel("Dir"));
  dir_layout->addWidget(dir_edit_);
  dir_layout->addWidget(browse_button);

  scenario_combo_ = new QComboBox;
  scenario_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  auto * refresh_button = new QPushButton("Refresh");
  auto * scenario_layout = new QHBoxLayout;
  scenario_layout->addWidget(scenario_combo_);
  scenario_layout->addWidget(refresh_button);

  auto * save_button = new QPushButton("Save as...");
  auto * load_button = new QPushButton("Load");
  auto * clear_button = new QPushButton("Clear scene");
  auto * button_layout = new QHBoxLayout;
  button_layout->addWidget(save_button);
  button_layout->addWidget(load_button);
  button_layout->addWidget(clear_button);

  clear_before_load_ = new QCheckBox("Clear scene before load");
  clear_before_load_->setChecked(true);
  count_label_ = new QLabel;
  status_label_ = new QLabel;
  status_label_->setWordWrap(true);

  // Traffic light routine: one row per light, green -> green flashing -> red -> red flashing.
  map_light_combo_ = new QComboBox;
  map_light_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  map_light_combo_->setToolTip(
    "Lights in the lanelet map: tf_pedestrian ways (read by the road_crossing FSM) and "
    "traffic light regulatory elements on crosswalks (read by the crosswalk module)");
  auto * add_light_button = new QPushButton("Add");
  add_light_button->setToolTip("Add the selected light, or an empty row to type an ID");
  auto * map_light_layout = new QHBoxLayout;
  map_light_layout->addWidget(new QLabel("Light"));
  map_light_layout->addWidget(map_light_combo_);
  map_light_layout->addWidget(add_light_button);

  light_table_ = new QTableWidget(0, 8);
  light_table_->setHorizontalHeaderLabels(
    {"ID", "Green [s]", "G flash [s]", "Red [s]", "R flash [s]", "Start", "Loop", "Now"});
  light_table_->horizontalHeaderItem(kColGreenFlashing)->setToolTip("Green flashing");
  light_table_->horizontalHeaderItem(kColRedFlashing)
    ->setToolTip("Red flashing; leave 0 where the light has none");
  light_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  light_table_->horizontalHeader()->setStretchLastSection(true);
  light_table_->verticalHeader()->setVisible(false);
  light_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  light_table_->setMaximumHeight(140);

  auto * remove_light_button = new QPushButton("Remove");
  auto * start_lights_button = new QPushButton("Start / Restart");
  start_lights_button->setToolTip("Restart the routine at time zero with the table as it is now");
  auto * stop_lights_button = new QPushButton("Stop");
  auto * light_button_layout = new QHBoxLayout;
  light_button_layout->addWidget(remove_light_button);
  light_button_layout->addWidget(start_lights_button);
  light_button_layout->addWidget(stop_lights_button);

  custom_label_ = new QLabel;
  custom_label_->setWordWrap(true);
  custom_label_->setVisible(false);
  pedestrian_light_label_ = new QLabel;
  pedestrian_light_label_->setToolTip(QString("Last %1").arg(kPedestrianLightTopic));

  auto * light_layout = new QVBoxLayout;
  light_layout->addLayout(map_light_layout);
  light_layout->addWidget(light_table_);
  light_layout->addLayout(light_button_layout);
  light_layout->addWidget(custom_label_);
  light_layout->addWidget(pedestrian_light_label_);
  auto * light_group = new QGroupBox("Traffic lights");
  light_group->setLayout(light_layout);

  auto * layout = new QVBoxLayout;
  layout->addLayout(dir_layout);
  layout->addLayout(scenario_layout);
  layout->addLayout(button_layout);
  layout->addWidget(clear_before_load_);
  layout->addWidget(count_label_);
  layout->addWidget(light_group);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(browse_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onBrowseDir);
  connect(dir_edit_, &QLineEdit::editingFinished, this, &DummyObjectScenarioPanel::onRefresh);
  connect(refresh_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onRefresh);
  connect(save_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onSave);
  connect(load_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onLoad);
  connect(clear_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onClear);
  connect(add_light_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onAddLight);
  connect(
    remove_light_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onRemoveLight);
  connect(
    start_lights_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onStartLights);
  connect(stop_lights_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onStopLights);

  // The subscription callback runs on the ROS executor; the label is refreshed from the
  // Qt side instead of touching widgets from there.
  tick_ = new QTimer(this);
  connect(tick_, &QTimer::timeout, this, &DummyObjectScenarioPanel::onTick);
}

void DummyObjectScenarioPanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  live_signals_stamp_ = raw_node_->now();  // same clock type as the ages taken from it
  // Load publishes a burst (DELETEALL + every object); a depth-1 queue would drop most of it.
  pub_ = raw_node_->create_publisher<DummyObject>(kObjectInfoTopic, rclcpp::QoS{100});
  sub_ = raw_node_->create_subscription<DummyObject>(
    kObjectInfoTopic, rclcpp::QoS{100},
    [this](const DummyObject::ConstSharedPtr msg) { onObjectInfo(msg); });
  // transient_local so the sequencer still gets the routine if it starts after the Load.
  routine_pub_ = raw_node_->create_publisher<std_msgs::msg::String>(
    kRoutineTopic, rclcpp::QoS{1}.transient_local());
  map_sub_ = raw_node_->create_subscription<LaneletMapBin>(
    kMapTopic, rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { onMap(msg); });
  signals_sub_ = raw_node_->create_subscription<TrafficLightGroupArray>(
    kSignalsTopic, rclcpp::QoS{10},
    [this](const TrafficLightGroupArray::ConstSharedPtr msg) { onSignals(msg); });
  pedestrian_light_sub_ = raw_node_->create_subscription<PedestrianLightState>(
    kPedestrianLightTopic, rclcpp::QoS{1}, [this](const PedestrianLightState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      pedestrian_light_ = msg;
    });
  onRefresh();
  onTick();
  tick_->start(500);
}

void DummyObjectScenarioPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Scenario Dir", dir_edit_->text());
  config.mapSetValue("Scenario", scenario_combo_->currentText());
  config.mapSetValue("Clear Before Load", clear_before_load_->isChecked());
}

void DummyObjectScenarioPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString text;
  if (config.mapGetString("Scenario Dir", &text) && !text.isEmpty()) {
    dir_edit_->setText(text);
  }
  bool checked{};
  if (config.mapGetBool("Clear Before Load", &checked)) {
    clear_before_load_->setChecked(checked);
  }
  onRefresh();
  if (config.mapGetString("Scenario", &text)) {
    scenario_combo_->setCurrentText(text);
  }
}

void DummyObjectScenarioPanel::onObjectInfo(const DummyObject::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto same_id = [&](const DummyObject & o) { return o.id.uuid == msg->id.uuid; };
  switch (msg->action) {
    case DummyObject::ADD:
    case DummyObject::PREDICT:
      scene_.erase(std::remove_if(scene_.begin(), scene_.end(), same_id), scene_.end());
      scene_.push_back(*msg);
      break;
    case DummyObject::MODIFY: {
      const auto it = std::find_if(scene_.begin(), scene_.end(), same_id);
      if (it != scene_.end()) {
        // Keep whether it was spawned as ADD or PREDICT; take everything else from the move.
        const auto action = it->action;
        *it = *msg;
        it->action = action;
      }
      break;
    }
    case DummyObject::DELETE:
      scene_.erase(std::remove_if(scene_.begin(), scene_.end(), same_id), scene_.end());
      break;
    case DummyObject::DELETEALL:
      scene_.clear();
      break;
  }
}

std::string DummyObjectScenarioPanel::scenarioPath(const QString & name) const
{
  QString file = name;
  if (!file.endsWith(".yaml")) {
    file += ".yaml";
  }
  return QDir(dir_edit_->text()).filePath(file).toStdString();
}

void DummyObjectScenarioPanel::setStatus(const QString & text, bool error)
{
  status_label_->setStyleSheet(error ? "color: red;" : "");
  status_label_->setText(text);
  if (raw_node_) {
    if (error) {
      RCLCPP_WARN(raw_node_->get_logger(), "DummyObjectScenario: %s", text.toStdString().c_str());
    } else {
      RCLCPP_INFO(raw_node_->get_logger(), "DummyObjectScenario: %s", text.toStdString().c_str());
    }
  }
}

void DummyObjectScenarioPanel::onBrowseDir()
{
  const auto dir =
    QFileDialog::getExistingDirectory(this, "Scenario directory", dir_edit_->text());
  if (!dir.isEmpty()) {
    dir_edit_->setText(dir);
    onRefresh();
    Q_EMIT configChanged();
  }
}

void DummyObjectScenarioPanel::onRefresh()
{
  const auto current = scenario_combo_->currentText();
  scenario_combo_->clear();
  const auto files =
    QDir(dir_edit_->text()).entryInfoList(QStringList{"*.yaml"}, QDir::Files, QDir::Name);
  for (const auto & info : files) {
    scenario_combo_->addItem(info.completeBaseName());
  }
  if (!current.isEmpty()) {
    scenario_combo_->setCurrentText(current);
  }
}

void DummyObjectScenarioPanel::onSave()
{
  std::vector<DummyObject> scene;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scene = scene_;
  }
  const auto lights = lightsToYaml();
  if (!lights) {
    return;
  }
  if (scene.empty() && lights->size() == 0) {
    setStatus(
      "Nothing to save: no dummy objects placed since the panel started and no traffic lights.",
      true);
    return;
  }

  bool ok{};
  const auto name = QInputDialog::getText(
                      this, "Save scenario", "Scenario name:", QLineEdit::Normal,
                      scenario_combo_->currentText(), &ok)
                      .trimmed();
  if (!ok || name.isEmpty()) {
    return;
  }

  const auto path = scenarioPath(name);
  if (
    QFileInfo::exists(QString::fromStdString(path)) &&
    QMessageBox::question(
      this, "Overwrite?", QString("'%1' exists. Overwrite it?").arg(name)) != QMessageBox::Yes) {
    return;
  }
  QDir().mkpath(dir_edit_->text());

  YAML::Node root;
  for (const auto & object : scene) {
    root["objects"].push_back(toYaml(object));
  }
  if (lights->size() > 0) {
    root["traffic_lights"] = *lights;
  }
  // 10 significant digits: short numbers stay short (0.6, not 0.59999999999999998) while
  // map coordinates in the 1e5 m range still keep sub-millimetre resolution.
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(10);
  emitter.SetFloatPrecision(7);
  emitter << root;

  std::ofstream out(path);
  out << "# Dummy object scenario, saved by the DummyObjectScenarioPanel.\n"
         "# Hand-editable: yaw in rad, velocity/accel along the object's heading.\n"
         "# traffic_lights: id is the traffic light regulatory element id; phases play in\n"
         "# order from start_phase, colors RED/AMBER/GREEN, status SOLID_ON/FLASHING/SOLID_OFF.\n"
      << emitter.c_str() << "\n";
  if (!out) {
    setStatus(QString("Failed to write %1").arg(QString::fromStdString(path)), true);
    return;
  }

  onRefresh();
  scenario_combo_->setCurrentText(QFileInfo(QString::fromStdString(path)).completeBaseName());
  Q_EMIT configChanged();
  setStatus(QString("Saved %1 object(s) and %2 traffic light(s) to %3")
              .arg(scene.size())
              .arg(lights->size())
              .arg(QString::fromStdString(path)));
}

void DummyObjectScenarioPanel::onLoad()
{
  const auto name = scenario_combo_->currentText();
  if (name.isEmpty()) {
    setStatus("No scenario selected.", true);
    return;
  }
  const auto path = scenarioPath(name);

  std::vector<DummyObject> objects;
  try {
    const auto root = YAML::LoadFile(path);
    if (root["objects"]) {
      for (const auto & node : root["objects"]) {
        objects.push_back(fromYaml(node));
      }
    }
    lightsFromYaml(root["traffic_lights"] ? root["traffic_lights"] : YAML::Node{});
  } catch (const std::exception & e) {
    setStatus(QString("Failed to load %1: %2").arg(QString::fromStdString(path), e.what()), true);
    return;
  }

  if (clear_before_load_->isChecked()) {
    DummyObject clear_msg;
    clear_msg.header.frame_id = getDisplayContext()->getFixedFrame().toStdString();
    clear_msg.header.stamp = raw_node_->now();
    clear_msg.action = DummyObject::DELETEALL;
    pub_->publish(clear_msg);
  }

  // Stamped now: the publisher moves objects by the time elapsed since the stamp.  Fresh
  // uuids so loading twice without clearing gives two independent copies.
  const auto now = raw_node_->now();
  for (auto & object : objects) {
    object.header.stamp = now;
    randomUuid(object);
    pub_->publish(object);
  }

  // The routine restarts at zero together with the objects, so signal timing relative to
  // the scene is the same on every run.  A file without lights stops the previous routine.
  const auto lights = lightsToYaml();
  const size_t light_count = lights ? lights->size() : 0;
  if (lights) {
    sendRoutine(*lights);
  }

  Q_EMIT configChanged();
  if (lights) {
    setStatus(QString("Loaded %1 object(s) and %2 traffic light(s) from '%3'")
                .arg(objects.size())
                .arg(light_count)
                .arg(name));
  }
}

void DummyObjectScenarioPanel::onClear()
{
  DummyObject clear_msg;
  clear_msg.header.frame_id = getDisplayContext()->getFixedFrame().toStdString();
  clear_msg.header.stamp = raw_node_->now();
  clear_msg.action = DummyObject::DELETEALL;
  pub_->publish(clear_msg);
  setStatus("Cleared all dummy objects.");
}

void DummyObjectScenarioPanel::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map);
  std::map<int64_t, std::string> lights;
  // The IRISA pedestrian lights: ways tagged tf_pedestrian=true, as the
  // pedestrian_light_detector reads them.
  for (const auto & way : map->lineStringLayer) {
    if (way.attributeOr("tf_pedestrian", "") == std::string("true")) {
      lights[way.id()] = "pedestrian light";
    }
  }
  // Name the road_crossing each one is paired with (forward_pedestrian_tf), so the right
  // light is picked for the crossing under test.
  for (const auto & lanelet : map->laneletLayer) {
    std::stringstream ss(lanelet.attributeOr("forward_pedestrian_tf", ""));
    std::string token;
    while (std::getline(ss, token, ',')) {
      int64_t id = 0;
      try {
        id = std::stoll(token);
      } catch (const std::exception &) {
        continue;
      }
      const auto it = lights.find(id);
      if (it != lights.end()) {
        it->second += ", crossing " + std::to_string(lanelet.id());
      }
    }
  }
  // Same lookup the crosswalk module does: the traffic lights regulating the crosswalk
  // lanelet are the pedestrian signals it obeys.
  std::map<int64_t, std::vector<int64_t>> crosswalk_lights;
  const auto crosswalks =
    lanelet::utils::query::crosswalkLanelets(lanelet::utils::query::laneletLayer(map));
  for (const auto & crosswalk : crosswalks) {
    for (const auto & light : crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>()) {
      crosswalk_lights[light->id()].push_back(crosswalk.id());
    }
  }
  for (const auto & [light_id, crosswalk_ids] : crosswalk_lights) {
    std::string text = "crosswalk";
    for (const auto crosswalk_id : crosswalk_ids) {
      text += " " + std::to_string(crosswalk_id);
    }
    lights[light_id] = text;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  map_lights_ = lights;
  map_lights_changed_ = true;
}

void DummyObjectScenarioPanel::onSignals(const TrafficLightGroupArray::ConstSharedPtr msg)
{
  const auto color_name = [](uint8_t color) -> std::string {
    switch (color) {
      case TrafficLightElement::RED:
        return "RED";
      case TrafficLightElement::AMBER:
        return "AMBER";
      case TrafficLightElement::GREEN:
        return "GREEN";
      case TrafficLightElement::WHITE:
        return "WHITE";
      default:
        return "UNKNOWN";
    }
  };
  std::map<int64_t, std::string> states;
  for (const auto & group : msg->traffic_light_groups) {
    std::string text;
    for (const auto & element : group.elements) {
      text += (text.empty() ? "" : " + ") + color_name(element.color);
      if (element.status == TrafficLightElement::FLASHING) {
        text += " (flashing)";
      } else if (element.status == TrafficLightElement::SOLID_OFF) {
        text += " (off)";
      }
    }
    states[group.traffic_light_group_id] = text;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  live_signals_ = states;
  live_signals_stamp_ = raw_node_->now();
}

void DummyObjectScenarioPanel::addLightRow(
  int64_t id, const std::array<double, 4> & durations, int start, bool loop)
{
  const int row = light_table_->rowCount();
  light_table_->insertRow(row);
  light_table_->setItem(row, kColId, new QTableWidgetItem(QString::number(id)));
  auto * start_combo = new QComboBox;
  for (size_t i = 0; i < kPattern.size(); ++i) {
    light_table_->setItem(
      row, kPattern[i].column, new QTableWidgetItem(QString::number(durations[i])));
    start_combo->addItem(kPattern[i].name);
  }
  start_combo->setCurrentIndex(start);
  light_table_->setCellWidget(row, kColStart, start_combo);

  auto * loop_item = new QTableWidgetItem;
  loop_item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
  loop_item->setCheckState(loop ? Qt::Checked : Qt::Unchecked);
  light_table_->setItem(row, kColLoop, loop_item);

  auto * now_item = new QTableWidgetItem("-");
  now_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
  light_table_->setItem(row, kColNow, now_item);
}

std::optional<YAML::Node> DummyObjectScenarioPanel::lightsToYaml()
{
  YAML::Node lights(YAML::NodeType::Sequence);
  for (int row = 0; row < light_table_->rowCount(); ++row) {
    bool ok{};
    const auto id = light_table_->item(row, kColId)->text().toLongLong(&ok);
    if (!ok || id <= 0) {
      setStatus(
        QString(
          "Traffic light row %1: '%2' is not a valid ID. Double-click the ID cell and type the "
          "traffic light regulatory element id from the lanelet map.")
          .arg(row + 1)
          .arg(light_table_->item(row, kColId)->text()),
        true);
      return std::nullopt;
    }

    std::array<double, kPattern.size()> durations{};
    double total = 0.0;
    for (size_t i = 0; i < kPattern.size(); ++i) {
      durations[i] = light_table_->item(row, kPattern[i].column)->text().toDouble(&ok);
      if (!ok || durations[i] < 0.0) {
        setStatus(
          QString("Traffic light %1: durations must be numbers >= 0 [s].").arg(id), true);
        return std::nullopt;
      }
      total += durations[i];
    }
    if (total <= 0.0) {
      setStatus(QString("Traffic light %1: all durations are zero.").arg(id), true);
      return std::nullopt;
    }

    YAML::Node light;
    light["id"] = id;
    light["start_phase"] =
      static_cast<QComboBox *>(light_table_->cellWidget(row, kColStart))->currentIndex();
    light["loop"] = light_table_->item(row, kColLoop)->checkState() == Qt::Checked;
    for (size_t i = 0; i < kPattern.size(); ++i) {
      YAML::Node phase;
      phase.SetStyle(YAML::EmitterStyle::Flow);
      phase["color"] = kPattern[i].color;
      phase["status"] = kPattern[i].status;
      phase["duration"] = durations[i];
      light["phases"].push_back(phase);
    }
    lights.push_back(light);
  }
  for (const auto & light : custom_lights_) {
    lights.push_back(light);
  }
  return lights;
}

void DummyObjectScenarioPanel::lightsFromYaml(const YAML::Node & lights)
{
  light_table_->setRowCount(0);
  custom_lights_.clear();
  if (lights) {
    for (const auto & light : lights) {
      // The table only shows the plain pedestrian pattern; anything else (vehicle lights,
      // offsets, extra phases) is kept as written and still played and saved.
      // Files from before red flashing existed have only the first three phases; they
      // load with red flashing at 0.
      const auto & phases = light["phases"];
      const auto is = [&](size_t i) {
        const auto & p = phases[i];
        return p["color"].as<std::string>("") == kPattern[i].color &&
               p["status"].as<std::string>("SOLID_ON") == kPattern[i].status &&
               p["shape"].as<std::string>("CIRCLE") == "CIRCLE";
      };
      const auto start = light["start_phase"].as<int>(0);
      bool matches = phases.IsSequence() && phases.size() >= kPattern.size() - 1 &&
                     phases.size() <= kPattern.size() &&
                     light["offset"].as<double>(0.0) == 0.0 && start >= 0 &&
                     start < static_cast<int>(phases.size());
      for (size_t i = 0; matches && i < phases.size(); ++i) {
        matches = is(i);
      }
      if (matches) {
        std::array<double, kPattern.size()> durations{};
        for (size_t i = 0; i < phases.size(); ++i) {
          durations[i] = phases[i]["duration"].as<double>();
        }
        addLightRow(light["id"].as<int64_t>(), durations, start, light["loop"].as<bool>(true));
      } else {
        custom_lights_.push_back(light);
      }
    }
  }
  custom_label_->setVisible(!custom_lights_.empty());
  custom_label_->setText(
    QString("+ %1 custom light(s) from the file, played as written (edit them in the YAML).")
      .arg(custom_lights_.size()));
}

void DummyObjectScenarioPanel::sendRoutine(const YAML::Node & lights)
{
  YAML::Node root;
  root["traffic_lights"] = lights;
  std_msgs::msg::String msg;
  msg.data = YAML::Dump(root);
  routine_pub_->publish(msg);
}

void DummyObjectScenarioPanel::onAddLight()
{
  // No map light picked: an empty row whose ID is typed in by hand.
  const auto id = map_light_combo_->currentData().isValid()
                    ? map_light_combo_->currentData().toLongLong()
                    : 0;
  addLightRow(id, {10.0, 5.0, 15.0, 0.0}, 0, true);
  if (id == 0) {
    // Say why the row is blank: a map without lights is the usual cause, and nothing
    // reacts to an id that is not a light in the map.
    setStatus(
      map_light_combo_->count() == 0
        ? "The loaded map has no tf_pedestrian lights or crosswalk traffic lights. Type an ID "
          "into the new row; only ids of lights in the map have any effect."
        : "Type the light id (tf_pedestrian way or traffic light regulatory element) into the "
          "new row.",
      true);
  }
}

void DummyObjectScenarioPanel::onRemoveLight()
{
  const auto rows = light_table_->selectionModel()->selectedRows();
  std::vector<int> indices;
  for (const auto & index : rows) {
    indices.push_back(index.row());
  }
  std::sort(indices.rbegin(), indices.rend());
  for (const auto row : indices) {
    light_table_->removeRow(row);
  }
}

void DummyObjectScenarioPanel::onStartLights()
{
  const auto lights = lightsToYaml();
  if (!lights) {
    return;
  }
  if (lights->size() == 0) {
    setStatus("No traffic lights to start. Add a row first.", true);
    return;
  }
  sendRoutine(*lights);
  setStatus(QString("Traffic light routine started for %1 light(s).").arg(lights->size()));
}

void DummyObjectScenarioPanel::onStopLights()
{
  sendRoutine(YAML::Node(YAML::NodeType::Sequence));
  setStatus("Traffic light routine stopped.");
}

void DummyObjectScenarioPanel::onTick()
{
  size_t count{};
  std::optional<std::map<int64_t, std::string>> map_lights;
  std::map<int64_t, std::string> live_signals;
  PedestrianLightState::ConstSharedPtr pedestrian_light;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    count = scene_.size();
    if (map_lights_changed_) {
      map_lights = map_lights_;
      map_lights_changed_ = false;
    }
    pedestrian_light = pedestrian_light_;
    // Older than the planner's own timeout means nobody is publishing any more.
    if (raw_node_ && (raw_node_->now() - live_signals_stamp_).seconds() < 1.0) {
      live_signals = live_signals_;
    }
  }
  count_label_->setText(QString("Objects in scene: %1").arg(count));

  if (map_lights) {
    map_light_combo_->clear();
    for (const auto & [light_id, text] : *map_lights) {
      map_light_combo_->addItem(
        QString("%1 (%2)").arg(light_id).arg(QString::fromStdString(text)),
        QVariant::fromValue<qlonglong>(light_id));
    }
    if (map_lights->empty()) {
      map_light_combo_->setPlaceholderText("no lights in the map");
    }
  }

  // What the road_crossing FSM is being told right now.
  QString pedestrian_text = "-";
  if (
    pedestrian_light && raw_node_ &&
    (raw_node_->now() - rclcpp::Time(pedestrian_light->header.stamp, RCL_ROS_TIME)).seconds() <
      1.0) {
    static const char * names[] = {"NONE", "RED", "GREEN", "COUNTDOWN_GREEN", "COUNTDOWN_BLANK"};
    pedestrian_text = pedestrian_light->state < 5 ? names[pedestrian_light->state] : "?";
    if (pedestrian_light->light_id != 0) {
      pedestrian_text += QString(" (light %1)").arg(pedestrian_light->light_id);
    }
  }
  pedestrian_light_label_->setText(QString("Road-crossing light: %1").arg(pedestrian_text));

  for (int row = 0; row < light_table_->rowCount(); ++row) {
    const auto id = light_table_->item(row, kColId)->text().toLongLong();
    const auto it = live_signals.find(id);
    light_table_->item(row, kColNow)
      ->setText(it != live_signals.end() ? QString::fromStdString(it->second) : "-");
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DummyObjectScenarioPanel, rviz_common::Panel)
