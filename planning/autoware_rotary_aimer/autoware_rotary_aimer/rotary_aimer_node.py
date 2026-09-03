# Copyright 2024 azzamwildan462
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
rotary_aimer_node — rotate the sensor platform to face the nearest
tf_pedestrian traffic light so the front camera sees it.

Angle convention
----------------
rotary 0 = platform faces FRONT (front camera looking along +X of base_link).
desired angle = atan2(light_base.y, light_base.x) in the base_link frame.

Sign reasoning:
  - Light to the LEFT of the robot: light_base.y > 0
    => atan2(+y, +x) > 0  =>  positive rotary angle
    => platform rotates counter-clockwise (left) to face the light.  CORRECT.
  - Light to the RIGHT of the robot: light_base.y < 0
    => atan2(-y, +x) < 0  =>  negative rotary angle
    => platform rotates clockwise (right).  CORRECT.

If the physical front camera is NOT aligned with platform +X, set
`front_cam_offset_rad` (subtracted from the bearing).  If the entire
sign is inverted mechanically, set `invert_sign: true`.

Chain (verified working in Fase 1):
  /control/rotary_angle_cmd  (Float32, rad)
  -> irisa_aio (AUTO mode) -> /gama/cmd/rotary_target
  -> gama_mw PID -> /gama/feedback/rotary

Feedback convention
-------------------
  /gama/feedback/rotary (std_msgs/Float32, rad) is the position feedback
  consumed by gama_mw_interface as the PID process variable for the rotary
  motor.  It is in the SAME sign convention and units as /control/rotary_angle_cmd
  — no inversion or offset is applied between them in gama_mw_interface.
  Therefore _last_published_angle is seeded from the raw feedback value on
  activation (no transform needed).

Services
--------
  ~/enable  (std_srvs/SetBool)
    data=true  -> start aiming timer
    data=false -> if zero_on_disable: publish 0.0 N times then stop

Parameters
----------
  map_path             str    -- path to lanelet2 .osm
  base_frame           str    -- robot base frame (default: base_link)
  map_frame            str    -- map frame (default: map)
  output_topic         str    -- angle command topic (default: /control/rotary_angle_cmd)
  feedback_topic       str    -- rotary position feedback topic (default: /gama/feedback/rotary)
  enable_service       str    -- service name (default: ~/enable)
  rate_hz              float  -- control loop rate (default: 10.0)
  front_cam_offset_rad float  -- subtract from bearing if cam not aligned w/ +X (default: 0.0)
  invert_sign          bool   -- flip sign of output angle (default: false)
  start_enabled        bool   -- start in enabled state (default: false)
  zero_on_disable      bool   -- publish 0.0 on disable to re-zero platform (default: true)
  zero_repeat_count    int    -- how many times to publish 0.0 on disable (default: 5)
  zero_repeat_period_s float  -- interval between zero pulses in seconds (default: 0.1)
  target_topic         str    -- PointStamped topic for external aim target from FSM
                                 (default: ~/aim_target).  When a fresh point is
                                 received (within target_fresh_sec) it overrides
                                 the nearest tf_pedestrian light.
  target_fresh_sec     float  -- max age (s) for an external target to be considered
                                 fresh (default: 0.5).  Older targets are ignored and
                                 the node falls back to the internal light.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime

from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

try:
    import tf2_ros
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False

from autoware_rotary_aimer.osm_parser import parse_tf_pedestrian_lights


def _wrap_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RotaryAimerNode(Node):
    """Rotate the sensor platform to face the nearest tf_pedestrian light."""

    def __init__(self):
        super().__init__('rotary_aimer')

        # ------------------------------------------------------------------ params
        # Default kosong: path sebenarnya di-pass dari launch (map_path global).
        # Jangan hardcode path mesin lain.
        self.declare_parameter('map_path', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('output_topic', '/control/rotary_angle_cmd')
        self.declare_parameter('feedback_topic', '/gama/feedback/rotary')
        self.declare_parameter('enable_service', '~/enable')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('front_cam_offset_rad', 0.0)
        self.declare_parameter('invert_sign', False)
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('zero_on_disable', True)
        self.declare_parameter('zero_repeat_count', 5)
        self.declare_parameter('zero_repeat_period_s', 0.1)
        self.declare_parameter('max_angle_rate_rad_s', 0.1)
        self.declare_parameter('zero_rate_rad_s', 0.04)
        self.declare_parameter('target_topic', '~/aim_target')
        self.declare_parameter('target_fresh_sec', 0.5)

        self._map_path = str(self.get_parameter('map_path').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._feedback_topic = str(self.get_parameter('feedback_topic').value)
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._front_cam_offset_rad = float(
            self.get_parameter('front_cam_offset_rad').value)
        self._invert_sign = bool(self.get_parameter('invert_sign').value)
        self._start_enabled = bool(self.get_parameter('start_enabled').value)
        self._zero_on_disable = bool(self.get_parameter('zero_on_disable').value)
        self._zero_repeat_count = int(self.get_parameter('zero_repeat_count').value)
        self._zero_repeat_period_s = float(
            self.get_parameter('zero_repeat_period_s').value)
        self._max_angle_rate_rad_s = float(
            self.get_parameter('max_angle_rate_rad_s').value)
        self._zero_rate_rad_s = float(
            self.get_parameter('zero_rate_rad_s').value)
        self._target_topic = str(self.get_parameter('target_topic').value)
        self._target_fresh_sec = float(
            self.get_parameter('target_fresh_sec').value)

        # ------------------------------------------------------------------ state
        self._enabled = False
        self._zero_remaining = 0      # countdown for re-zero pulses
        self._zero_timer = None       # timer used for re-zero sequence
        self._zero_done = False       # flag: ramp reached 0 — safe to destroy from outside
        self._last_published_angle = None  # slew-rate tracking; None = not yet started
        self._latest_feedback_angle = None  # latest /gama/feedback/rotary value (rad)
        self._feedback_received = False     # True once at least one feedback msg arrived

        # External aim target from FSM (road_crossing approach phase)
        self._ext_target_point = None   # (x, y, z) in map frame, or None
        self._ext_target_time = None    # rclpy.time.Time when the msg was received

        # ------------------------------------------------------------------ tf2
        if _TF2_AVAILABLE:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(
                self._tf_buffer, self)
        else:
            self._tf_buffer = None
            self._tf_listener = None
            self.get_logger().warn(
                'tf2_ros not available — rotary_aimer will not function.')

        # ------------------------------------------------------------------ map
        self._lights = []
        try:
            self._lights = parse_tf_pedestrian_lights(self._map_path)
            self.get_logger().info(
                f'Parsed {len(self._lights)} tf_pedestrian light(s) from {self._map_path}')
            for light in self._lights:
                self.get_logger().info(
                    f'  Light way_id={light.way_id} midpoint={light.midpoint}')
        except IOError as exc:
            self.get_logger().error(f'Failed to parse OSM map: {exc}')

        if not self._lights:
            self.get_logger().warn(
                'No tf_pedestrian lights found in map — node will idle.')

        # ------------------------------------------------------------------ pub
        self._pub = self.create_publisher(Float32, self._output_topic, 10)

        # ------------------------------------------------------------------ sub (feedback)
        self._feedback_sub = self.create_subscription(
            Float32,
            self._feedback_topic,
            self._feedback_cb,
            10,
        )
        self.get_logger().info(
            f'Subscribed to rotary feedback: {self._feedback_topic}')

        # ------------------------------------------------------------------ sub (external target)
        self._ext_target_sub = self.create_subscription(
            PointStamped,
            self._target_topic,
            self._ext_target_cb,
            10,
        )
        self.get_logger().info(
            f'Subscribed to external aim target: {self._target_topic} '
            f'(fresh window: {self._target_fresh_sec}s)')

        # ------------------------------------------------------------------ service
        svc_name = self.get_parameter('enable_service').value
        self._enable_srv = self.create_service(
            SetBool, svc_name, self._handle_enable)
        self.get_logger().info(f'Enable service ready at: {svc_name}')

        # ------------------------------------------------------------------ timer
        self._aim_timer = None
        if self._start_enabled:
            self._start_aiming()

        # ------------------------------------------------------------------ live-param callback
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f'rotary_aimer started. enabled={self._start_enabled} '
            f'lights={len(self._lights)} rate={self._rate_hz}Hz '
            f'output={self._output_topic} feedback={self._feedback_topic} '
            f'target_topic={self._target_topic} '
            f'target_fresh_sec={self._target_fresh_sec}')

    # ---------------------------------------------------------------------- service

    def _handle_enable(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if request.data:
            self._start_aiming()
            response.success = True
            response.message = 'rotary_aimer enabled'
            self.get_logger().info('rotary_aimer ENABLED')
        else:
            self._stop_aiming()
            response.success = True
            response.message = 'rotary_aimer disabled'
            self.get_logger().info('rotary_aimer DISABLED')
        return response

    # ---------------------------------------------------------------------- lifecycle

    def _start_aiming(self):
        """Start the aim control loop.

        Fix 2 — IDEMPOTENT: if already enabled with a live aim timer, do nothing.
        Repeated enable calls (FSM spam every 0.3 s) must not re-seed the slew
        tracker or reset the timer; that would pin the output near the seed and
        prevent the ramp from advancing.
        """
        # Destroy any completed zero-timer safely (outside its own callback).
        # _cancel_zero_timer is also safe here because we are in a service-callback
        # context, never inside _zero_tick itself.
        self._cancel_zero_timer()
        self._zero_remaining = 0

        # --- Fix 2: idempotency guard ---
        if self._enabled and self._aim_timer is not None:
            # Already aiming — repeated enable is a no-op.
            return

        # Seed the slew tracker from the ACTUAL current rotary position so that
        # the first _aim_tick ramps FROM here toward the desired angle rather than
        # snapping instantly to it.  Fall back to 0.0 if feedback has not yet arrived.
        if self._feedback_received and self._latest_feedback_angle is not None:
            self._last_published_angle = self._latest_feedback_angle
            self.get_logger().info(
                f'rotary_aimer ENABLE: seeding slew from feedback '
                f'{math.degrees(self._latest_feedback_angle):.1f}deg '
                f'({self._latest_feedback_angle:.4f}rad)')
        else:
            self._last_published_angle = 0.0
            self.get_logger().warn(
                'rotary_aimer ENABLE: no feedback received yet, '
                'seeding slew from 0.0 rad (fallback)')

        self._enabled = True
        if self._aim_timer is None:
            period = 1.0 / max(self._rate_hz, 0.1)
            self._aim_timer = self.create_timer(period, self._aim_tick)

    def _stop_aiming(self):
        """Stop the aim loop; if zero_on_disable, slew-ramp back to 0.0."""
        self._enabled = False

        # Destroy the aim timer
        if self._aim_timer is not None:
            self._aim_timer.cancel()
            self._aim_timer.destroy()
            self._aim_timer = None

        if self._zero_on_disable:
            # Ensure _last_published_angle is seeded so _zero_tick ramps from
            # the real current position (not from a stale None).
            if self._last_published_angle is None:
                seed = (self._latest_feedback_angle
                        if self._feedback_received and self._latest_feedback_angle is not None
                        else 0.0)
                self._last_published_angle = seed
                self.get_logger().info(
                    f'Re-zero seed from feedback: {math.degrees(seed):.1f}deg')

            # Start the slew-limited ramp-to-zero timer at the same rate_hz
            # as the aim loop so the zero_rate_rad_s param is consistent.
            period = 1.0 / max(self._rate_hz, 0.1)
            self._zero_timer = self.create_timer(period, self._zero_tick)
        else:
            self.get_logger().info(
                'rotary_aimer disabled, zero_on_disable=false — no re-zero.')
            self._last_published_angle = None

    def _zero_tick(self):
        """Timer callback: slew _last_published_angle toward 0.0 each tick.

        Fix 1 — SAFE SELF-STOP: do NOT call destroy() from inside this callback
        (rclpy executor corruption).  When the ramp reaches zero we only cancel()
        the timer (safe) and set _zero_done=True so subsequent spurious ticks
        no-op.  The actual destroy() happens lazily in _cancel_zero_timer(), which
        is always called from a service-callback or _start_aiming() context — never
        from within _zero_tick itself.

        Fix 3 — EXCEPTION GUARD: wrap the body so an unhandled error logs and
        returns rather than propagating into the executor and killing the node.
        """
        try:
            # Guard: if the ramp already completed (spurious re-fire), do nothing.
            if self._zero_done:
                return

            _EPS = 0.01  # rad — close enough to zero
            dt = 1.0 / max(self._rate_hz, 0.1)
            max_step = self._zero_rate_rad_s * dt

            current = self._last_published_angle if self._last_published_angle is not None else 0.0
            delta = _wrap_pi(0.0 - current)

            if abs(delta) <= _EPS:
                # Close enough — publish exact 0, finish ramp.
                self._publish_angle(0.0)
                self._last_published_angle = None
                # Fix 1: cancel only — NEVER destroy from within the callback.
                if self._zero_timer is not None:
                    self._zero_timer.cancel()
                self._zero_done = True
                self.get_logger().info('Re-zero complete — platform back to front.')
                return

            step = max(-max_step, min(max_step, delta))
            out = _wrap_pi(current + step)
            self._publish_angle(out)
            self._last_published_angle = out

        except Exception as exc:  # Fix 3
            self.get_logger().error(
                f'_zero_tick unhandled exception: {exc}',
                throttle_duration_sec=2.0,
            )

    def _cancel_zero_timer(self):
        """Cancel and destroy the zero-ramp timer.

        MUST be called from outside _zero_tick (service-callback or
        _start_aiming context) — never from within the timer's own callback.
        """
        if self._zero_timer is not None:
            self._zero_timer.cancel()
            self._zero_timer.destroy()
            self._zero_timer = None
        self._zero_done = False  # reset flag for the next re-zero cycle

    # ---------------------------------------------------------------------- control loop

    def _aim_tick(self):
        """Main control loop: compute bearing to chosen target, publish angle.

        Target selection (orthogonal to enable/slew):
          1. If an external PointStamped was received within target_fresh_sec,
             use its map-frame (x, y, z) as the target point.
          2. Otherwise fall back to the nearest tf_pedestrian light from the OSM
             (original behavior — identical when no external target is present).

        The SAME transform pipeline (map -> base_frame via tf2, bearing =
        atan2(y, x), front_cam_offset_rad, invert_sign, slew) is applied to
        whichever map-point is chosen.

        Fix 3 — EXCEPTION GUARD: the entire body is wrapped so any unhandled
        error logs (throttled) and returns rather than propagating into the
        rclpy executor and killing the node.
        """
        try:
            if not self._enabled:
                return
            if self._tf_buffer is None:
                return

            # Look up transform: map -> base_frame  (i.e., base expressed in map coords,
            # which we use to express the light in base_frame via inverse)
            # We need: light point in base_frame.
            # tf2 lookup_transform(target, source): converts points FROM source TO target.
            # We want points from map frame expressed in base_frame:
            #   transform = lookup_transform(base_frame, map_frame)
            try:
                tf_stamped = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._map_frame,
                    RclpyTime(),  # latest available
                )
            except Exception as exc:
                self.get_logger().warn(
                    f'TF lookup {self._map_frame}->{self._base_frame} failed: {exc}',
                    throttle_duration_sec=2.0,
                )
                return

            # Extract rotation (quaternion) and translation from tf_stamped
            tr = tf_stamped.transform.translation
            ro = tf_stamped.transform.rotation

            # Rotation matrix: map -> base_frame
            R = _quat_to_matrix(ro.x, ro.y, ro.z, ro.w)
            t = (tr.x, tr.y, tr.z)

            import numpy as np
            t_vec = np.array(t, dtype=float)

            # ------------------------------------------------------------------
            # TARGET SELECTION
            # ------------------------------------------------------------------
            # Check whether the external target (from FSM, published as
            # PointStamped in map frame) is still fresh.
            use_external = False
            if (self._ext_target_point is not None
                    and self._ext_target_time is not None):
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                recv_sec = self._ext_target_time.nanoseconds * 1e-9
                if (now_sec - recv_sec) <= self._target_fresh_sec:
                    use_external = True

            if use_external:
                # --- EXTERNAL target (road_crossing approach centroid) ---
                ex, ey, ez = self._ext_target_point
                p_map = np.array([ex, ey, ez], dtype=float)
                p_base = R @ p_map + t_vec
                light_x, light_y = p_base[0], p_base[1]

                bearing = math.atan2(light_y, light_x)
                desired = bearing - self._front_cam_offset_rad
                desired = _wrap_pi(desired)
                if self._invert_sign:
                    desired = -desired

                target_label = f'EXTERNAL ({ex:.2f},{ey:.2f})'
                dist_m = math.sqrt(light_x ** 2 + light_y ** 2)

                self.get_logger().info(
                    f'[rotary_aimer] aim target: {target_label} '
                    f'dist={dist_m:.2f}m '
                    f'base_xy=({light_x:.2f},{light_y:.2f}) '
                    f'bearing={math.degrees(bearing):.1f}deg '
                    f'desired={math.degrees(desired):.1f}deg',
                    throttle_duration_sec=1.0,
                )

            else:
                # --- INTERNAL target: nearest tf_pedestrian light (original behavior) ---
                if not self._lights:
                    return

                # Find nearest light by distance from robot origin to light midpoint
                # expressed in base_frame.
                best_light = None
                best_dist_sq = float('inf')

                for light in self._lights:
                    mx, my, mz = light.midpoint
                    # Transform midpoint from map to base_frame: p_base = R @ p_map + t
                    p_map = np.array([mx, my, mz], dtype=float)
                    p_base = R @ p_map + t_vec

                    # 2D distance in horizontal plane
                    dist_sq = p_base[0] ** 2 + p_base[1] ** 2
                    if dist_sq < best_dist_sq:
                        best_dist_sq = dist_sq
                        best_light = (light, p_base)

                if best_light is None:
                    return

                light, p_base = best_light
                light_x, light_y = p_base[0], p_base[1]

                # Bearing to light in base_frame.
                # atan2(y, x):
                #   light LEFT  -> y > 0 -> angle > 0 -> platform CCW (left)  -> CORRECT
                #   light RIGHT -> y < 0 -> angle < 0 -> platform CW  (right) -> CORRECT
                bearing = math.atan2(light_y, light_x)
                desired = bearing - self._front_cam_offset_rad
                desired = _wrap_pi(desired)
                if self._invert_sign:
                    desired = -desired

                dist_m = math.sqrt(best_dist_sq)
                self.get_logger().info(
                    f'[rotary_aimer] aim target: light way_id={light.way_id} '
                    f'dist={dist_m:.2f}m '
                    f'base_xy=({light_x:.2f},{light_y:.2f}) '
                    f'bearing={math.degrees(bearing):.1f}deg '
                    f'desired={math.degrees(desired):.1f}deg',
                    throttle_duration_sec=1.0,
                )
            # ------------------------------------------------------------------
            # END TARGET SELECTION
            # ------------------------------------------------------------------

            # ------ slew-rate limit ------
            dt = 1.0 / max(self._rate_hz, 0.1)

            # _last_published_angle is always seeded in _start_aiming before this
            # timer fires, so it should never be None here.  Guard defensively: if
            # somehow None (e.g. restarted in an unexpected order), seed from the
            # latest feedback rather than snapping to desired.
            if self._last_published_angle is None:
                seed = (self._latest_feedback_angle
                        if self._feedback_received and self._latest_feedback_angle is not None
                        else 0.0)
                self.get_logger().warn(
                    f'_aim_tick: _last_published_angle unexpectedly None, '
                    f'seeding from {math.degrees(seed):.1f}deg')
                self._last_published_angle = seed

            delta = _wrap_pi(desired - self._last_published_angle)
            max_step = self._max_angle_rate_rad_s * dt
            step = max(-max_step, min(max_step, delta))
            out = _wrap_pi(self._last_published_angle + step)

            self._publish_angle(out)
            self._last_published_angle = out
            # ------ end slew ------

            self.get_logger().info(
                f'[rotary_aimer] cmd={math.degrees(out):.1f}deg ({out:.4f}rad)',
                throttle_duration_sec=1.0,
            )

        except Exception as exc:  # Fix 3
            self.get_logger().error(
                f'_aim_tick unhandled exception: {exc}',
                throttle_duration_sec=2.0,
            )

    # ---------------------------------------------------------------------- helpers

    def _feedback_cb(self, msg: Float32):
        """Store latest rotary position feedback (rad, same convention as cmd)."""
        self._latest_feedback_angle = float(msg.data)
        self._feedback_received = True

    def _ext_target_cb(self, msg: PointStamped):
        """Store the latest external aim target published by the FSM.

        Expected frame: map (param map_frame).  If the message header.frame_id
        differs from map_frame, a warning is emitted and the message is stored
        anyway with the assumption it is map-relative (the FSM always publishes
        in map).  No tf2 lookup is performed here — the transform happens in
        _aim_tick along the exact same path as the light midpoints.
        """
        if msg.header.frame_id and msg.header.frame_id != self._map_frame:
            self.get_logger().warn(
                f'External aim target arrived in frame '
                f'"{msg.header.frame_id}" (expected "{self._map_frame}"); '
                f'treating as map-relative.',
                throttle_duration_sec=5.0,
            )
        self._ext_target_point = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )
        self._ext_target_time = self.get_clock().now()

    def _publish_angle(self, angle_rad: float):
        msg = Float32()
        msg.data = float(angle_rad)
        self._pub.publish(msg)

    # ---------------------------------------------------------------------- live-param callback

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Apply runtime parameter updates to the corresponding member variables.

        Live params (take effect immediately without restart):
          max_angle_rate_rad_s, zero_rate_rad_s, front_cam_offset_rad,
          invert_sign, zero_on_disable.

        rate_hz is intentionally excluded — changing it would require
        recreating the timer; accepted silently with a warning.
        All other declared params (map_path, base_frame, etc.) are
        accepted but left unchanged (they require a node restart).
        """
        for param in params:
            name = param.name

            if name == 'max_angle_rate_rad_s':
                val = float(param.value)
                if val <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='max_angle_rate_rad_s must be > 0',
                    )
                self._max_angle_rate_rad_s = val
                self.get_logger().info(
                    f'param max_angle_rate_rad_s updated -> {val}')

            elif name == 'zero_rate_rad_s':
                val = float(param.value)
                if val <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='zero_rate_rad_s must be > 0',
                    )
                self._zero_rate_rad_s = val
                self.get_logger().info(
                    f'param zero_rate_rad_s updated -> {val}')

            elif name == 'front_cam_offset_rad':
                val = float(param.value)
                self._front_cam_offset_rad = val
                self.get_logger().info(
                    f'param front_cam_offset_rad updated -> {val}')

            elif name == 'invert_sign':
                val = bool(param.value)
                self._invert_sign = val
                self.get_logger().info(
                    f'param invert_sign updated -> {val}')

            elif name == 'zero_on_disable':
                val = bool(param.value)
                self._zero_on_disable = val
                self.get_logger().info(
                    f'param zero_on_disable updated -> {val}')

            elif name == 'rate_hz':
                # Changing rate_hz requires recreating the timer — not live.
                # Accept the parameter store update but leave the member (and
                # running timer period) unchanged until next restart.
                self.get_logger().warn(
                    'param rate_hz accepted in store but NOT applied at runtime; '
                    'restart the node for the new rate to take effect.')

            # All other params (map_path, base_frame, map_frame, output_topic,
            # feedback_topic, enable_service, start_enabled, zero_repeat_count,
            # zero_repeat_period_s) are accepted into the parameter store but
            # not hot-applied; they require a node restart.

        return SetParametersResult(successful=True)


# ---------------------------------------------------------------------------
# Internal math (no scipy/transforms3d dependency)
# ---------------------------------------------------------------------------

def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion (x,y,z,w) to 3x3 numpy rotation matrix."""
    import numpy as np
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz),
         2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),
         1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),
         2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)
    return R


def main(args=None):
    rclpy.init(args=args)
    node = RotaryAimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
