# Turn-in-place: hardware integration (TODO for the integration agent)

Status (2026-11-09): the Autoware side is done and tested in the planning simulator (with the
steering-mode mock in `autoware_simple_planning_simulator`). **On the real robot the turn cannot
run yet**: the executor's spin command is not connected to the chassis and the chassis gives no
steering-mode feedback. This document lists exactly what is missing and the recommended way to
connect it. Read `../README.md` first for what the supervisor does.

Paths are relative to the workspace `src/`:
- `UNI` = `universe/autoware_universe/planning/autoware_turn_in_place_supervisor`
- `MASTER` = `hardware/autoware_vehicle_interfaces/autoware_master/master`
- `MW` = `hardware/gama_hardware/middleware_controller_manager`

## 1. What the Autoware side does today

| Node | Rate | Talks to |
| --- | --- | --- |
| `turn_in_place_supervisor` | 10 Hz | decides; holds the robot with a velocity limit; publishes `/planning/turn_in_place/command` (`TurnInPlaceCommand{active, target_yaw, direction}`); reads `/planning/turn_in_place/executor/status` (`TurnInPlaceStatus`) |
| `turn_in_place_executor` | 20 Hz | the only node that touches the chassis (see below) |

The executor (`UNI/src/turn_in_place_executor_node.cpp`, logic in `UNI/src/executor_logic.cpp`):

1. Publishes `std_msgs/UInt8` **4** (HOLONOMIC) on **`/gama/cmd/steering_mode`**. This topic *is*
   what `MW` subscribes (`MW/config/middleware_controller_manager.yaml`: `steering_mode_topic`), so
   this part already reaches the hardware, directly and not through master.
2. Waits for `autoware_turn_in_place_msgs/SteeringModeFeedback` on
   **`/gama/feedback/steering_mode`** (`mode`, `target_mode`, `switching`). **Nothing on the robot
   publishes this.** Today, on hardware, the executor times out after `switch_timeout_sec` (5 s),
   reports FAILED, and the supervisor goes to ERROR. That is safe: the robot is held still.
3. Publishes `geometry_msgs/Twist` (`linear` 0, `angular.z` = yaw rate [rad/s] ×
   `yaw_rate_command_gain`) on **`/control/turn_in_place/cmd_vel`**, turning in the commanded
   direction (CCW = +1 / CW = −1: the supervisor picks the side that is free of obstacles and road
   edges, so **the direction must be honoured**, not replaced by the shortest way). **Nothing on
   the robot consumes this topic**; only the simulator mock does.
4. Publishes the pre-turn steering mode back on `/gama/cmd/steering_mode`, waits for feedback, then
   reports DONE.

During the whole turn the supervisor also publishes a 0 velocity limit, so Autoware's own control
command is "stand still" (but the MPC may still command steering angles).

## 2. What exists on the hardware side (verified)

- **`MW` `/cmd_vel`** is the only motion input. Its `angular.z` is **overloaded**
  (`MW/include/middleware_controller_manager/chassis_math.hpp`, header comment): in Ackermann /
  FWS / 4WCS it is a *steering angle*; in HOLONOMIC (mode 4) it is a *yaw rate* scaled by
  `holonomic_yaw_gain` (3.14 in `MW/config/middleware_controller_manager.yaml`). Sending a yaw
  rate while the chassis is still in Ackermann would command a steering angle. Order matters:
  mode switch, settle, then the rate.
- **`MW` steering mode** is set by topic `/gama/cmd/steering_mode` (UInt8) or service
  `/gama/cmd/set_steering_mode`, and read by service `/gama/cmd/get_steering_mode`. There is no
  feedback topic and no echo on mode change.
- **Master owns `/cmd_vel`**: `MASTER/src/velocity_mux.cpp` publishes it every tick, choosing the
  source by `robot_fsm`: AUTONOMOUS → Autoware's `auto_linear_x` / `auto_angular_z`, DEMO_MODE →
  MotionController output, RC modes → RC. It also hard-zeros `/cmd_vel` for
  `change_motion_vel_cooldown_s` whenever `steering_mode_target` changes. A second publisher on
  `/cmd_vel` would fight it, so do not let the executor publish `/cmd_vel` directly.
- **Master already has a spin-in-place controller**: `MotionController::request_spin()`
  (`MASTER/src/motion_controller.cpp`, around lines 370-425 and the `is_spin_` branch of `step()`).
  It switches to HOLONOMIC through `SteeringModeClient` (service `/gama/cmd/set_steering_mode`),
  hard-zeros for `spin_mode_switch_settle_s` (1.0 s, `MASTER/config/master.yaml`), runs a P
  controller (`spin_angular_gain`, `max_spin_angular_rate_rad_s`), and always restores mode 2 via
  `end_holonomic_mode()`. It is exposed as service **`spin_to_angle`**
  (`master_msgs/srv/SpinToAngle`: `target_angle_rad`, `theta_tolerance` →
  `accepted, success, result_code, final_theta_error`), but:
  - it is accepted **only when `robot_fsm == READY`** (`MASTER/src/master.cpp` ~line 430), and the
    turn happens while AUTONOMOUS;
  - it turns the **short way** (`delta_angle_rad`), with no direction argument;
  - its output goes to the mux only in DEMO_MODE (`demo_mode_cmd_vel`).

## 3. Recommended integration: master executes the turn

Reuse master's proven spin machinery instead of the Autoware-side executor, so that master stays
the single owner of `/cmd_vel` and of the steering mode.

1. **Master subscribes `/planning/turn_in_place/command`** (`TurnInPlaceCommand`) and publishes
   **`/planning/turn_in_place/executor/status`** (`TurnInPlaceStatus`). Add
   `autoware_turn_in_place_msgs` as a dependency of `master`.
2. **Accept the spin while AUTONOMOUS.** When `command.active` arrives (fresh, < 3 s old) and
   `robot_fsm` is AUTONOMOUS, start the spin through the same machinery as `request_spin()`
   (mode 4, settle, P control, restore). Suggested shape: a new robot_fsm state or a flag (like
   `AUTOWARE_TO_POINT` / `autoware_to_point_requested`) so that `VelocityMux` sends
   MotionController's output instead of Autoware's control command while the turn runs, and goes
   back to Autoware's command when it ends. RC override and emergency must keep priority.
3. **Honour `command.direction`.** Add a direction to the spin (do not take `delta_angle_rad`'s
   short way when `direction` says otherwise). Within ~15 deg past the target, correcting back the
   short way is fine (that is what the Autoware executor does: `UNI/src/executor_logic.cpp`,
   `compute_yaw_rate()`).
4. **Tolerance**: stop within 7 deg (`done_tolerance_deg` of the Autoware executor); the supervisor
   accepts < 10 deg and the MPC corrects the rest.
5. **Cancel**: `command.active == false` (or no command for > 3 s) must stop the rotation and
   restore the driving mode at once.
6. **Status mapping** for `TurnInPlaceStatus.state`:
   `SWITCHING_MODE` while settling into mode 4, `ROTATING` while turning, `RESTORING_MODE` while
   switching back, `DONE` once the driving mode is back and the heading is within tolerance (keep
   DONE while `command.active` stays true), `FAILED` on timeout / RC / emergency / HW fault,
   `IDLE` otherwise. Fill `yaw_error` (target − current, normalized) and `reason`.
7. **Do not launch the Autoware executor on the robot**: add a launch arg
   `launch_executor` (default true) to `UNI/launch/turn_in_place.launch.xml` and to
   `launcher/autoware_launch/autoware_launch/launch/components/tier4_turn_in_place_component.launch.xml`,
   set it false in the robot run script. Keep it true for the planning simulator (it drives the
   steering-mode mock).

### Alternative (if master should not change much)

Keep the Autoware executor and add to master only (a) a mux input: while a fresh
`/control/turn_in_place/cmd_vel` exists, `VelocityMux` forwards its `angular.z` (× 1 /
`holonomic_yaw_gain` → set `yaw_rate_command_gain` in `turn_in_place_executor.param.yaml`
accordingly) instead of Autoware's command, and (b) a `SteeringModeFeedback` publisher on
`/gama/feedback/steering_mode` built from the existing `get_steering_mode` polling plus a settle
time after each change (`switching` = true until settled). This duplicates the spin logic master
already has, so the recommended path is preferred.

## 4. Robot launch settings

In the robot run script (`run_aw_gama.sh`, meta repo), for v2:

```
launch_turn_in_place:=true
launch_reverse_lane_follow_module:=true   # backs up only on the supervisor's request (narrow roads)
allow_reverse_route:=true
```

`launch_turn_in_place:=true` also sets: mission planner `reverse_start_heading_check:=any`, BPP
`fold_handling:=truncate`, `reverse_lane_follow.activation:=external_request`, stuck recovery
`mission_direction:=forward_only` (see `launcher/autoware_launch/.../autoware.launch.xml`).
The steering-mode mock is only enabled by `planning_simulator.launch.xml`; nothing to disable on
the robot.

Pointcloud for the turning-area check: `/perception/obstacle_segmentation/pointcloud` cropped to
±2 m around `base_link` (`UNI/launch/turn_in_place.launch.xml`). Check that this topic exists and
covers the area around the robot (front and rear) on the real sensor setup.

Rotation centre: `rotation_centre_x_m` (default 0.0 = base_link) in
`launcher/autoware_launch/autoware_launch/config/planning/turn_in_place/turn_in_place_supervisor.param.yaml`.
GAMA's base_link is the body centre (`.../gama_vehicle_description/config/self_crop.param.yaml`
explains why); confirm the chassis really spins about it in HOLONOMIC.

## 5. How to test on the robot (in order)

1. Robot lifted or in an empty area, Autoware not engaged: call master's spin path / publish a
   `TurnInPlaceCommand` by hand and check mode 4 → rotation in the commanded direction → mode back
   → status DONE. Also cancel mid-turn, and test RC override and emergency during the turn.
2. Autoware engaged, robot facing against a short route: expect REQUEST_STOP → CHECK_CLEARANCE →
   TURNING → COOLDOWN → driving nose-first. The robot must not move before turning (it is held
   from the first cycle, also before engage).
3. Put a box in the turning circle on one side: the supervisor must pick the other direction; on
   both sides it must wait (robot held).
4. Watch `/planning/turn_in_place/supervisor/state` (reason text) and
   `/planning/turn_in_place/supervisor/debug/markers` in RViz.

## 6. Related Autoware-side pieces (already done, for reference)

- `autoware_turn_in_place_msgs`: all messages above.
- `reverse_lane_follow` `activation: external_request` + `/planning/turn_in_place/reverse_request`
  (backing up to a turn spot on narrow roads).
- `stuck_recovery_supervisor`: stands down while the supervisor is turning or backing up.
- Lanelet tag `disable_turn_in_place=yes`: no turning there; the robot backs up out of it first.
