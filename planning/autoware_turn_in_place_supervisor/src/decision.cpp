// Copyright 2026 azzamwildan462
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

#include "autoware/turn_in_place_supervisor/decision.hpp"

#include "autoware/turn_in_place_supervisor/geometry.hpp"

#include <cmath>
#include <string>

namespace autoware::turn_in_place_supervisor
{

const char * to_string(const SupervisorState state)
{
  switch (state) {
    case SupervisorState::MONITOR:
      return "MONITOR";
    case SupervisorState::REQUEST_STOP:
      return "REQUEST_STOP";
    case SupervisorState::CHECK_CLEARANCE:
      return "CHECK_CLEARANCE";
    case SupervisorState::TURNING:
      return "TURNING";
    case SupervisorState::COOLDOWN:
      return "COOLDOWN";
    case SupervisorState::ERROR:
      return "ERROR";
    case SupervisorState::REVERSING_TO_TURN_SPOT:
      return "REVERSING_TO_TURN_SPOT";
  }
  return "UNKNOWN";
}

void TurnInPlaceDecision::enter(const SupervisorState next, const double now)
{
  if (next != state_) {
    state_ = next;
    state_since_ = now;
  }
  if (next != SupervisorState::TURNING) {
    direction_ = 0;
  }
  if (next == SupervisorState::MONITOR) {
    over_trigger_since_.reset();
  }
}

DecisionOutput TurnInPlaceDecision::output(const bool hold, const std::string & reason) const
{
  DecisionOutput out;
  out.state = state_;
  out.hold_vehicle = hold;
  out.command_active = state_ == SupervisorState::TURNING;
  out.target_yaw = target_yaw_;
  out.direction = out.command_active ? direction_ : 0;
  out.reverse_active = state_ == SupervisorState::REVERSING_TO_TURN_SPOT;
  out.reverse_to_spot = out.reverse_active && reverse_to_spot_;
  out.reason = reason;
  return out;
}

DecisionOutput TurnInPlaceDecision::step(const DecisionInput & in)
{
  const double abs_error = std::abs(in.yaw_error);
  const bool heading_fine = in.route_valid && abs_error < params_.done_angle;
  const bool heading_off = in.route_valid && abs_error > params_.trigger_angle;
  if (in.route_valid) {
    target_yaw_ = in.route_yaw;
  }

  // Leaving autonomous driving or getting a new route ends whatever was going on.  The robot is
  // still held whenever its heading is off the route -- also before it is engaged -- so that it
  // never moves the wrong way while the turn is being confirmed.
  if (!in.autonomous) {
    enter(SupervisorState::MONITOR, in.now);
    return output(
      heading_off, heading_off ? "not autonomous; heading off the route, holding" : "not autonomous");
  }
  if (in.route_changed && state_ != SupervisorState::MONITOR) {
    enter(SupervisorState::MONITOR, in.now);
    return output(heading_off, "route changed");
  }

  switch (state_) {
    case SupervisorState::MONITOR: {
      if (!heading_off) {
        over_trigger_since_.reset();
        return output(false, "heading ok");
      }
      if (!over_trigger_since_) {
        over_trigger_since_ = in.now;
      }
      if (in.now - *over_trigger_since_ >= params_.trigger_confirm_sec) {
        enter(SupervisorState::REQUEST_STOP, in.now);
        return output(true, "heading off the route; stopping before turning");
      }
      // Held from the first cycle: confirming must not cost any motion the wrong way.
      return output(true, "heading off the route; confirming");
    }

    case SupervisorState::REQUEST_STOP: {
      if (heading_fine) {
        enter(SupervisorState::MONITOR, in.now);
        return output(false, "heading recovered while stopping");
      }
      if (in.speed < params_.stop_speed) {
        enter(SupervisorState::CHECK_CLEARANCE, in.now);
        return output(true, "standing still; checking the turning area");
      }
      if (elapsed(in.now) > params_.stop_timeout_sec) {
        enter(SupervisorState::ERROR, in.now);
        return output(true, "timed out waiting for the robot to stop");
      }
      return output(true, "waiting for the robot to stop");
    }

    case SupervisorState::CHECK_CLEARANCE: {
      if (heading_fine) {
        enter(SupervisorState::COOLDOWN, in.now);
        return output(true, "heading already fine");
      }
      // Turn the short way when it is free, the long way when only that one is.
      const int preferred = in.yaw_error >= 0.0 ? kDirectionCcw : kDirectionCw;
      const bool preferred_clear = preferred == kDirectionCcw ? in.ccw_clear : in.cw_clear;
      const bool other_clear = preferred == kDirectionCcw ? in.cw_clear : in.ccw_clear;
      if (in.route_valid && (preferred_clear || other_clear)) {
        enter(SupervisorState::TURNING, in.now);
        direction_ = preferred_clear ? preferred : -preferred;
        return output(
          true, preferred_clear ? "turning the short way"
                                : "short way blocked; turning the other way");
      }
      // The road itself is too narrow here: waiting cannot help.  Back up along the route --
      // that is towards the goal, the robot facing away from it -- to the first spot wide
      // enough, or all the way to the goal when there is none.
      if (in.route_valid && in.road_blocks_both && params_.enable_reverse_to_turn_spot) {
        reverse_to_spot_ = in.turn_spot_found;
        enter(SupervisorState::REVERSING_TO_TURN_SPOT, in.now);
        return output(
          false, reverse_to_spot_ ? "road too narrow to turn here; backing up to a turn spot"
                                  : "road too narrow to turn anywhere ahead; backing up to the goal");
      }
      if (elapsed(in.now) > params_.clearance_timeout_sec) {
        enter(SupervisorState::ERROR, in.now);
        return output(true, "timed out waiting for a free turning area");
      }
      return output(
        true, in.route_valid ? "both turning directions blocked; waiting"
                             : "route heading unknown; waiting");
    }

    case SupervisorState::REVERSING_TO_TURN_SPOT: {
      if (heading_fine) {
        enter(SupervisorState::MONITOR, in.now);
        return output(false, "heading fine; stopping the back-up");
      }
      if (reverse_to_spot_ && in.at_turn_spot) {
        enter(SupervisorState::CHECK_CLEARANCE, in.now);
        return output(true, "reached the turn spot; checking the turning area");
      }
      // A spot appeared ahead (e.g. the road widens) while backing up towards the goal.
      if (!reverse_to_spot_ && in.turn_spot_found) {
        reverse_to_spot_ = true;
        return output(false, "turn spot found while backing up; stopping there");
      }
      return output(
        false, reverse_to_spot_ ? "backing up to the turn spot" : "backing up towards the goal");
    }

    case SupervisorState::TURNING: {
      const bool executor_done = in.executor_fresh && in.executor_state == ExecutorState::DONE;
      if (in.executor_fresh && in.executor_state == ExecutorState::FAILED) {
        enter(SupervisorState::ERROR, in.now);
        return output(true, "executor failed");
      }
      if (executor_done) {
        if (heading_fine) {
          enter(SupervisorState::COOLDOWN, in.now);
          return output(true, "turn done");
        }
        enter(SupervisorState::CHECK_CLEARANCE, in.now);
        return output(true, "turn ended short of the route heading; re-checking");
      }
      const bool still_clear = direction_ == kDirectionCcw ? in.ccw_clear : in.cw_clear;
      if (!still_clear) {
        enter(SupervisorState::CHECK_CLEARANCE, in.now);
        return output(true, "turning area became blocked; stopping the turn");
      }
      if (elapsed(in.now) > params_.turn_timeout_sec) {
        enter(SupervisorState::ERROR, in.now);
        return output(true, "turn timed out");
      }
      return output(true, "turning");
    }

    case SupervisorState::COOLDOWN: {
      if (elapsed(in.now) >= params_.cooldown_sec) {
        enter(SupervisorState::MONITOR, in.now);
        return output(false, "cooldown over; handing back");
      }
      return output(true, "cooling down after the turn");
    }

    case SupervisorState::ERROR: {
      if (heading_fine) {
        enter(SupervisorState::MONITOR, in.now);
        return output(false, "heading fine again; leaving ERROR");
      }
      return output(true, "error: holding the robot");
    }
  }
  return output(false, "");
}

}  // namespace autoware::turn_in_place_supervisor
