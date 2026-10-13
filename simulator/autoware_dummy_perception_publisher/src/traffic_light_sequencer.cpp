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

// Fake traffic light recognition for the planning simulator: plays a timed routine of
// signal phases (e.g. pedestrian green -> green flashing -> red) per traffic light
// and publishes it where the planner reads traffic signals.
//
// Two kinds of light ids are understood:
//  - traffic light regulatory element ids: published on ~/output/traffic_signals, which
//    the crosswalk module reads.
//  - ids of `tf_pedestrian=true` ways (the IRISA pedestrian lights): additionally, the
//    light the road_crossing FSM is waiting for (~/input/target_light, paired with the
//    crossing by its forward_pedestrian_tf tag) is published as a PedestrianLightState on
//    ~/output/pedestrian_light, standing in for the pedestrian_light_detector that the
//    FSM listens to.  Green flashing alternates COUNTDOWN_GREEN and COUNTDOWN_BLANK, red
//    flashing RED and COUNTDOWN_BLANK, every half `flashing_period`.  Every light is also published on
//    ~/output/traffic_signals so the rviz panel can show it; the planner ignores ids that
//    are not regulatory elements.
//
// The routine arrives as a YAML string on ~/input/routine (the DummyObjectScenarioPanel
// sends the `traffic_lights:` section of a scenario on Load/Start) or from the
// `routine_file` parameter for headless runs.  Receiving a routine restarts the clock at
// zero, so every run of a scenario sees the same signal timing.  An empty routine stops
// publishing.
//
//   traffic_lights:
//     - id: 1234            # regulatory element id, or tf_pedestrian way id
//       start_phase: 0      # phase the routine starts in
//       offset: 0.0         # [s] extra time into start_phase at start
//       loop: true          # false: hold the last phase once the routine has run through
//       phases:
//         - {color: GREEN, status: SOLID_ON, duration: 10.0}
//         - {color: GREEN, status: FLASHING, duration: 5.0}
//         - {color: RED, status: SOLID_ON, duration: 15.0}
//         - {color: RED, status: FLASHING, duration: 0.0}   # 0 s phases are skipped
//
// shape defaults to CIRCLE, which is what the crosswalk module checks for red.

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_road_crossing_msgs/msg/pedestrian_light_state.hpp>
#include <autoware_road_crossing_msgs/msg/pedestrian_light_target.hpp>
#include <std_msgs/msg/string.hpp>

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_core/LaneletMap.h>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::dummy_perception_publisher
{
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_road_crossing_msgs::msg::PedestrianLightState;
using autoware_road_crossing_msgs::msg::PedestrianLightTarget;

namespace
{
const std::map<std::string, uint8_t> kColors = {
  {"UNKNOWN", TrafficLightElement::UNKNOWN}, {"RED", TrafficLightElement::RED},
  {"AMBER", TrafficLightElement::AMBER},     {"GREEN", TrafficLightElement::GREEN},
  {"WHITE", TrafficLightElement::WHITE},
};
const std::map<std::string, uint8_t> kShapes = {
  {"UNKNOWN", TrafficLightElement::UNKNOWN},
  {"CIRCLE", TrafficLightElement::CIRCLE},
  {"LEFT_ARROW", TrafficLightElement::LEFT_ARROW},
  {"RIGHT_ARROW", TrafficLightElement::RIGHT_ARROW},
  {"UP_ARROW", TrafficLightElement::UP_ARROW},
  {"CROSS", TrafficLightElement::CROSS},
};
const std::map<std::string, uint8_t> kStatuses = {
  {"UNKNOWN", TrafficLightElement::UNKNOWN},
  {"SOLID_OFF", TrafficLightElement::SOLID_OFF},
  {"SOLID_ON", TrafficLightElement::SOLID_ON},
  {"FLASHING", TrafficLightElement::FLASHING},
};

uint8_t lookup(
  const YAML::Node & node, const std::map<std::string, uint8_t> & table, const std::string & def)
{
  const auto name = node ? node.as<std::string>() : def;
  const auto it = table.find(name);
  if (it == table.end()) {
    throw std::runtime_error("unknown value '" + name + "'");
  }
  return it->second;
}

struct Phase
{
  TrafficLightElement element;
  double duration{};
};

struct Light
{
  int64_t id{};
  std::vector<Phase> phases;
  double start{};  // [s] position in the cycle at routine time zero
  double cycle{};  // [s] sum of phase durations
  bool loop{true};

  const TrafficLightElement & elementAt(double elapsed) const
  {
    double t = start + elapsed;
    if (loop) {
      t = std::fmod(t, cycle);
    }
    for (const auto & phase : phases) {
      if (t < phase.duration) {
        return phase.element;
      }
      t -= phase.duration;
    }
    return phases.back().element;  // not looping and past the end: hold the last phase
  }
};

std::vector<Light> parseRoutine(const std::string & text)
{
  std::vector<Light> lights;
  const auto root = YAML::Load(text);
  if (!root || !root["traffic_lights"]) {
    return lights;
  }
  for (const auto & node : root["traffic_lights"]) {
    Light light;
    light.id = node["id"].as<int64_t>();
    light.loop = node["loop"].as<bool>(true);
    for (const auto & p : node["phases"]) {
      Phase phase;
      phase.element.color = lookup(p["color"], kColors, "UNKNOWN");
      phase.element.shape = lookup(p["shape"], kShapes, "CIRCLE");
      phase.element.status = lookup(p["status"], kStatuses, "SOLID_ON");
      phase.element.confidence = 1.0f;
      phase.duration = p["duration"].as<double>();
      if (phase.duration < 0.0) {
        throw std::runtime_error("negative phase duration for light " + std::to_string(light.id));
      }
      light.cycle += phase.duration;
      light.phases.push_back(phase);
    }
    if (light.phases.empty() || light.cycle <= 0.0) {
      throw std::runtime_error("light " + std::to_string(light.id) + " has no timed phases");
    }
    const auto start_phase = node["start_phase"].as<size_t>(0);
    if (start_phase >= light.phases.size()) {
      throw std::runtime_error("start_phase out of range for light " + std::to_string(light.id));
    }
    for (size_t i = 0; i < start_phase; ++i) {
      light.start += light.phases[i].duration;
    }
    light.start += node["offset"].as<double>(0.0);
    lights.push_back(light);
  }
  return lights;
}
}  // namespace

class TrafficLightSequencer : public rclcpp::Node
{
public:
  TrafficLightSequencer() : Node("traffic_light_sequencer")
  {
    const double rate = declare_parameter<double>("publish_rate", 10.0);
    const auto routine_file = declare_parameter<std::string>("routine_file", "");
    flashing_period_ = declare_parameter<double>("flashing_period", 1.0);

    signals_pub_ = create_publisher<TrafficLightGroupArray>("~/output/traffic_signals", 1);
    pedestrian_light_pub_ =
      create_publisher<PedestrianLightState>("~/output/pedestrian_light", 1);
    map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
      [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) { onMap(msg); });
    target_light_sub_ = create_subscription<PedestrianLightTarget>(
      "~/input/target_light", 1,
      [this](const PedestrianLightTarget::ConstSharedPtr msg) { target_light_ = msg; });
    // transient_local: a routine sent before this node came up is still picked up.
    routine_sub_ = create_subscription<std_msgs::msg::String>(
      "~/input/routine", rclcpp::QoS{1}.transient_local(),
      [this](const std_msgs::msg::String::ConstSharedPtr msg) { setRoutine(msg->data, "topic"); });

    if (!routine_file.empty()) {
      try {
        setRoutine(YAML::Dump(YAML::LoadFile(routine_file)), routine_file);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to read %s: %s", routine_file.c_str(), e.what());
      }
    }

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this]() { onTimer(); });
  }

private:
  void setRoutine(const std::string & text, const std::string & source)
  {
    std::vector<Light> lights;
    try {
      lights = parseRoutine(text);
    } catch (const std::exception & e) {
      // Keep the current routine rather than going dark on a typo.
      RCLCPP_ERROR(get_logger(), "Rejected routine from %s: %s", source.c_str(), e.what());
      return;
    }
    lights_ = lights;
    start_time_ = now();
    RCLCPP_INFO(
      get_logger(), "Routine from %s: %zu traffic light(s)%s", source.c_str(), lights_.size(),
      lights_.empty() ? ", stopped" : ", started");
  }

  void onTimer()
  {
    if (lights_.empty()) {
      return;
    }
    const double elapsed = (now() - start_time_).seconds();
    TrafficLightGroupArray msg;
    msg.stamp = now();
    for (const auto & light : lights_) {
      TrafficLightGroup group;
      group.traffic_light_group_id = light.id;
      group.elements.push_back(light.elementAt(elapsed));
      msg.traffic_light_groups.push_back(group);
    }
    signals_pub_->publish(msg);
    publishPedestrianLight(elapsed);
  }

  void onMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*msg, map);
    // Same selection as the pedestrian_light_detector's map parser: a way tagged
    // tf_pedestrian=true is a pedestrian light, located at its midpoint.
    pedestrian_lights_.clear();
    for (const auto & way : map->lineStringLayer) {
      if (way.attributeOr("tf_pedestrian", "") != std::string("true") || way.empty()) {
        continue;
      }
      const auto & a = way.front().basicPoint();
      const auto & b = way.back().basicPoint();
      pedestrian_lights_[way.id()] = {(a.x() + b.x()) / 2.0, (a.y() + b.y()) / 2.0};
    }
    RCLCPP_INFO(get_logger(), "Map: %zu tf_pedestrian light(s)", pedestrian_lights_.size());
  }

  // The detector reports the light the FSM asked for (the one paired with the active
  // crossing).  NONE when there is no target or the routine does not play it — what the
  // detector says when no light is in view, and what makes the FSM fall back after its
  // timeout.
  void publishPedestrianLight(double elapsed)
  {
    const bool any_pedestrian_light =
      std::any_of(lights_.begin(), lights_.end(), [this](const Light & light) {
        return pedestrian_lights_.count(light.id) > 0;
      });
    if (!any_pedestrian_light) {
      return;  // routine has no pedestrian lights: leave the topic to whoever else owns it
    }

    const Light * target = nullptr;
    if (target_light_) {
      for (const auto id : target_light_->light_ids) {
        const auto it = std::find_if(
          lights_.begin(), lights_.end(), [id](const Light & light) { return light.id == id; });
        if (it != lights_.end()) {
          target = &*it;
          break;
        }
      }
    }

    PedestrianLightState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_front";
    msg.state = PedestrianLightState::NONE;
    if (target) {
      const auto & element = target->elementAt(elapsed);
      msg.confidence = 1.0f;
      // A flashing light is dark half of the time; the detector reports that as BLANK.
      const bool lit = element.status != TrafficLightElement::FLASHING ||
                       std::fmod(elapsed, flashing_period_) < 0.5 * flashing_period_;
      if (element.status == TrafficLightElement::SOLID_OFF) {
        msg.state = PedestrianLightState::NONE;
      } else if (!lit) {
        msg.state = PedestrianLightState::COUNTDOWN_BLANK;
      } else if (element.color == TrafficLightElement::RED) {
        msg.state = PedestrianLightState::RED;
      } else if (element.color == TrafficLightElement::GREEN) {
        msg.state = element.status == TrafficLightElement::FLASHING
                      ? PedestrianLightState::COUNTDOWN_GREEN
                      : PedestrianLightState::GREEN;
      }
      msg.light_id = msg.state == PedestrianLightState::NONE ? 0 : target->id;
    }
    pedestrian_light_pub_->publish(msg);
  }

  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr signals_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr routine_sub_;
  rclcpp::Publisher<PedestrianLightState>::SharedPtr pedestrian_light_pub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<PedestrianLightTarget>::SharedPtr target_light_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Light> lights_;
  rclcpp::Time start_time_;
  double flashing_period_{};
  std::map<int64_t, std::pair<double, double>> pedestrian_lights_;  // way id -> map x, y
  PedestrianLightTarget::ConstSharedPtr target_light_;
};

}  // namespace autoware::dummy_perception_publisher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::dummy_perception_publisher::TrafficLightSequencer>());
  rclcpp::shutdown();
  return 0;
}
