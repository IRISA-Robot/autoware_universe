// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::behavior_velocity_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

class RoadCrossingModule : public SceneModuleInterface
{
public:
  struct PlannerParam
  {
    // State machine
    double stop_distance_threshold{5.0};
    double stop_arrival_threshold{0.2};
    double stop_state_speed{0.05};
    double sustained_clear_sec{1.5};
    double timeout_sec{30.0};

    // Prediction check
    double predict_horizon_sec{5.0};
    double min_confidence{0.5};
    bool enable_dynamic_horizon{true};
    double crossing_speed_for_horizon{0.833};

    // Object class filter
    bool block_on_car{true};
    bool block_on_truck{true};
    bool block_on_bus{true};
    bool block_on_trailer{true};
    bool block_on_motorcycle{true};
    bool block_on_bicycle{false};
    bool block_on_pedestrian{false};
    bool block_on_unknown{false};
  };

  enum class State { APPROACHING, STOPPING, CHECKING, CROSSING, DONE };

  RoadCrossingModule(
    const int64_t module_id, const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(PathWithLaneId * path) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  PlannerParam planner_param_;
  State state_{State::APPROACHING};
};

}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
