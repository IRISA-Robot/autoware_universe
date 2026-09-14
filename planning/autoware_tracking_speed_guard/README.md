# autoware_tracking_speed_guard

Slows the robot down while it is following its trajectory badly, and lets it go again once
it is back on track.

## Why it exists

Nothing else in the stack does this:

- The **MPC cannot influence speed at all**. Its only channel to the longitudinal
  controller is a single `is_steer_converged` bool (`sync_data.hpp`), and that consumer is
  disabled in this configuration (`pid.param.yaml`,
  `enable_keep_stopped_until_steer_convergence: false`). Speed belongs entirely to the PID
  longitudinal controller, which reads it from the trajectory.
- The **MPC has no error threshold**. A large error is just a large `x0` in the QP; the
  only validity check is for NaN/Inf. It keeps trying, indefinitely, at full speed.
- `enable_large_tracking_error_emergency` is declared and stored by the PID controller and
  then **never read anywhere**. It looks like a protection and is not one.
- The **planning validator's** `distance_deviation` and `yaw_deviation` checks are
  configured with thresholds that never trip (100 m) and `handling_type: 0`.

So a robot that has been left far from its path -- after a recovery manoeuvre, say --
chases it at full speed. This node is the missing piece.

## What it does

It runs beside the trajectory follower. It takes the same trajectory the controller
follows (`/planning/trajectory`) and the same odometry, and computes the same two numbers
the MPC computes: the lateral offset and the heading error against the nearest trajectory
point. Each is turned into a 0..1 severity by a linear ramp between its `*_ok` and `*_bad`
thresholds, the worse of the two wins, and the speed cap is interpolated between
`v_free_mps` and `v_floor_mps`.

**Forward and reverse missions are treated identically.** The heading error is measured
against the *direction of travel*, taken from the sign of the trajectory's velocity exactly
as the MPC does, so a healthy reverse mission reads near-zero error rather than 180
degrees. There is no branch anywhere that distinguishes the two motion modes.

The cap goes out on the external velocity limit channel under its own `sender` name. The
selector keeps one limit per sender and applies the smallest, so this composes with the
obstacle, surround-check and MRM limits instead of overriding them.

## Failure direction

If this node dies while a cap is in force, the selector keeps holding that cap and the
robot stays **slow**. That is the right direction to fail in, but it is not self-healing:
nothing else will take the limit off. The node clears its cap on a clean shutdown, and
sends one clear on its first tick so that a restart releases whatever a previous instance
left behind.

To release it by hand:

```sh
ros2 topic pub --once /planning/scenario_planning/clear_velocity_limit \
  autoware_internal_planning_msgs/msg/VelocityLimitClearCommand \
  '{command: true, sender: "tracking_speed_guard"}'
```

## Tuning

The thresholds shipped here are first guesses. `~/debug/tracking_error` publishes
`[lateral_m, yaw_rad, severity, cap_mps, valid, travelling_back]` every tick -- calibrate
from a bag before tightening them.

`v_floor_mps` should not go below about 1 km/h: under roughly that the velocity smoother
refuses to engage at all (`stop_dist_to_prohibit_engage`), which presents as a robot frozen
for no visible reason.
