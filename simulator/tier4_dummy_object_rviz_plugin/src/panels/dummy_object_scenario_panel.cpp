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
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace rviz_plugins
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

constexpr const char * kObjectInfoTopic = "/simulation/dummy_perception_publisher/object_info";

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

  auto * layout = new QVBoxLayout;
  layout->addLayout(dir_layout);
  layout->addLayout(scenario_layout);
  layout->addLayout(button_layout);
  layout->addWidget(clear_before_load_);
  layout->addWidget(count_label_);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(browse_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onBrowseDir);
  connect(dir_edit_, &QLineEdit::editingFinished, this, &DummyObjectScenarioPanel::onRefresh);
  connect(refresh_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onRefresh);
  connect(save_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onSave);
  connect(load_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onLoad);
  connect(clear_button, &QPushButton::clicked, this, &DummyObjectScenarioPanel::onClear);

  // The subscription callback runs on the ROS executor; the label is refreshed from the
  // Qt side instead of touching widgets from there.
  tick_ = new QTimer(this);
  connect(tick_, &QTimer::timeout, this, &DummyObjectScenarioPanel::onTick);
}

void DummyObjectScenarioPanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  // Load publishes a burst (DELETEALL + every object); a depth-1 queue would drop most of it.
  pub_ = raw_node_->create_publisher<DummyObject>(kObjectInfoTopic, rclcpp::QoS{100});
  sub_ = raw_node_->create_subscription<DummyObject>(
    kObjectInfoTopic, rclcpp::QoS{100},
    [this](const DummyObject::ConstSharedPtr msg) { onObjectInfo(msg); });
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
  if (scene.empty()) {
    setStatus("Nothing to save: no dummy objects have been placed since the panel started.", true);
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
  // 10 significant digits: short numbers stay short (0.6, not 0.59999999999999998) while
  // map coordinates in the 1e5 m range still keep sub-millimetre resolution.
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(10);
  emitter.SetFloatPrecision(7);
  emitter << root;

  std::ofstream out(path);
  out << "# Dummy object scenario, saved by the DummyObjectScenarioPanel.\n"
         "# Hand-editable: yaw in rad, velocity/accel along the object's heading.\n"
      << emitter.c_str() << "\n";
  if (!out) {
    setStatus(QString("Failed to write %1").arg(QString::fromStdString(path)), true);
    return;
  }

  onRefresh();
  scenario_combo_->setCurrentText(QFileInfo(QString::fromStdString(path)).completeBaseName());
  Q_EMIT configChanged();
  setStatus(QString("Saved %1 object(s) to %2").arg(scene.size()).arg(QString::fromStdString(path)));
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
    for (const auto & node : root["objects"]) {
      objects.push_back(fromYaml(node));
    }
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

  Q_EMIT configChanged();
  setStatus(QString("Loaded %1 object(s) from '%2'").arg(objects.size()).arg(name));
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

void DummyObjectScenarioPanel::onTick()
{
  size_t count{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    count = scene_.size();
  }
  count_label_->setText(QString("Objects in scene: %1").arg(count));
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DummyObjectScenarioPanel, rviz_common::Panel)
