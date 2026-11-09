# autoware_turn_in_place_supervisor

Turns the robot around **in place** whenever its body heading is wrong for its route, instead of
driving backwards. Meant for a chassis that can spin in place (GAMA v2: `STEER_MODE_HOLONOMIC`) but
must not reverse far.

Lanelets the route traverses against their lanelet2 direction (`bidirectional_driving=yes`) are
driven **nose-first** by the ordinary lane following path (`reverse_lane_follow` runs with
`activation: external_request` and only backs up when this supervisor asks). The robot only has to
face the right way, at the start of a mission and at the apex of an out-and-back route. That is
this package's job. Where the road is too narrow to spin, the robot backs up along the route
(towards the goal) to the first spot wide enough, or all the way to the goal when there is none.

## Nodes

### `turn_in_place_supervisor` (decision, 10 Hz)

**Route heading.** The heading of the route at ego is the tangent of the active route window's ego
lanelet, oriented the way the route runs it, at the centreline point nearest ego by position. The
trajectory is not used: with the robot facing the wrong way it cannot be trusted.

**Turning area.** The body rectangle (from `vehicle_info`) is swept through the rotation in each
direction, sampled every `sweep_step_deg`. A direction is safe only when the sweep
- stays inside the road lanelets (all lanelets of `road_subtypes`, any lane or direction), inflated
  by `road_margin_m`, and
- touches no point of the no-ground pointcloud cropped around the robot, inflated by
  `sweep_margin_m`. If the cloud is missing or stale, both directions count as blocked.

**No-turn lanelets.** A lanelet tagged `disable_turn_in_place=yes` (tag name:
`no_turn_lanelet_tag`; any subtype) is off limits to the turning sweep, even where another
lanelet overlaps it. A robot that has to turn while in or next to one does not turn there: it
backs up along the route until a spin fits clear of it (or to the goal).

**Turn spot.** When both sweeps leave the road lanelets, the route centreline ahead (towards the
goal) is searched for the first place where a full spin (a disk of the body's corner radius +
`road_margin_m`) fits in the lanelets. `reverse_lane_follow` is asked to back up to it
(`/planning/turn_in_place/reverse_request`), or to the goal when there is none. Stopping while
backing up is not precise, so once the robot comes to rest the turn is judged where it stands; if
it still does not fit, the next spot is searched from there.

**States** (`TurnInPlaceSupervisorState`):

```text
MONITOR --(|err| > trigger_angle for trigger_confirm_sec, autonomous)--> REQUEST_STOP
REQUEST_STOP --(|v| < stop_speed)--> CHECK_CLEARANCE
CHECK_CLEARANCE --(short way clear, else the other way)--> TURNING
CHECK_CLEARANCE --(both sweeps leave the road lanelets)--> REVERSING_TO_TURN_SPOT
CHECK_CLEARANCE --(blocked by obstacles only)--> wait (robot held)
REVERSING_TO_TURN_SPOT --(at rest for settle_sec)--> CHECK_CLEARANCE
TURNING --(executor DONE and |err| < done_angle)--> COOLDOWN --(cooldown_sec)--> MONITOR
TURNING --(chosen side becomes blocked)--> CHECK_CLEARANCE
any phase --(timeout)--> ERROR   (robot held; leaves ERROR when the heading is fine again)
leaving autonomous / new route --> MONITOR
```

The robot is held by a zero velocity limit (sender `turn_in_place`) on
`/planning/scenario_planning/max_velocity_candidates` from the first cycle its heading is off the
route -- also before it is engaged, so engaging never moves it the wrong way -- through COOLDOWN,
and in ERROR. It is released while backing up to a turn spot and on return to MONITOR.

### `turn_in_place_executor` (20 Hz)

Talks to the chassis through the same interface as `middleware_controller_manager`:

1. Publishes `steering_mode` = `spin_mode` (4), then waits for `SteeringModeFeedback`.
2. Publishes a yaw-rate twist (`angular.z`). It always turns the way the supervisor chose; the only
   exception is a small overshoot, which it corrects back the short way.
3. Restores the pre-turn steering mode, waits for the feedback, then reports `DONE`.

Every phase has a timeout that leads to `FAILED`.

## Topics (`launch/turn_in_place.launch.xml`)

| Topic | Type | Direction |
| --- | --- | --- |
| `/planning/turn_in_place/command` | `TurnInPlaceCommand` | supervisor → executor |
| `/planning/turn_in_place/executor/status` | `TurnInPlaceStatus` | executor → supervisor |
| `/planning/turn_in_place/supervisor/state` | `TurnInPlaceSupervisorState` | also read by `stuck_recovery_supervisor` |
| `/planning/turn_in_place/pointcloud_near` | `PointCloud2` | crop box output |
| `/gama/cmd/steering_mode` | `std_msgs/UInt8` | executor → chassis |
| `/gama/feedback/steering_mode` | `SteeringModeFeedback` | chassis / simulator mock → executor |
| `/control/turn_in_place/cmd_vel` | `geometry_msgs/Twist` | executor → chassis (yaw rate) |
| `/planning/turn_in_place/reverse_request` | `ReverseToTurnSpotRequest` | supervisor → `reverse_lane_follow` |

## Enabling (autoware_launch)

`launch_turn_in_place:=true` does all of the following:

- launches both nodes;
- enables the simulator's steering-mode mock in the planning simulator;
- sets the mission planner's `reverse_start_heading_check:=any` (a route may start facing either
  way);
- sets the behavior path planner's `fold_handling:=truncate` (stop at the apex of an out-and-back
  path);
- sets `reverse_lane_follow.activation:=external_request` (backs up only when asked);
- sets `stuck_recovery_supervisor`'s `mission_direction:=forward_only`.

Combine it with `launch_reverse_lane_follow_module:=true` (needed for backing up to a turn spot)
and `allow_reverse_route:=true`.

## Not done yet (hardware phase)

See **[docs/hardware_integration.md](docs/hardware_integration.md)** for the full picture and the
recommended way (master executes the turn).


- On the real robot, multiplex `/control/turn_in_place/cmd_vel` into the middleware's `/cmd_vel`
  while the executor is active, and suppress Autoware's control command.
- Have the middleware publish `SteeringModeFeedback`.
- Set `yaw_rate_command_gain` = 1 / `holonomic_yaw_gain`.
