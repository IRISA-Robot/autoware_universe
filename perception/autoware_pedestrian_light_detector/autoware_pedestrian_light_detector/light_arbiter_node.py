# Copyright 2026 IRISA
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
pedestrian_light_arbiter: detector -> [AUTO | operator RED | operator GREEN] -> FSM.

    input_topic   (PedestrianLightState, from pedestrian_light_detector)
    command_topic (PedestrianLightOverride, from mock RC web / a remote node)
    fsm_state_topic (Int32Stamped, road_crossing_fsm: a crossing end uses up a GREEN)
 -> output_topic  (PedestrianLightState, what road_crossing_fsm reads)
    status_topic  (PedestrianLightOverrideStatus, for the web page and the recorder)

AUTO: every detector message is forwarded unchanged (source AUTO).  RED / GREEN: detector
messages are held back and the operator's state is sent at publish_rate_hz with source
MANUAL (the FSM skips light pairing and the light-sequence rules for it).  Back to AUTO: one
NONE (source AUTO) right away so the FSM drops the override without waiting for its timeout.
Rules: light_arbiter.py.
"""

import rclpy
from rclpy.node import Node

from autoware_internal_debug_msgs.msg import Int32Stamped
from autoware_road_crossing_msgs.msg import (
    PedestrianLightOverride,
    PedestrianLightOverrideStatus,
    PedestrianLightState,
)

from autoware_pedestrian_light_detector.light_arbiter import GREEN, LightArbiter


class LightArbiterNode(Node):

    def __init__(self):
        super().__init__('pedestrian_light_arbiter')
        p = self.declare_parameter
        self._in_topic = p('input_topic', '/perception/road_crossing/pedestrian_light_auto').value
        out_topic = p('output_topic', '/perception/road_crossing/pedestrian_light').value
        cmd_topic = p('command_topic', '/road_crossing/light_override').value
        status_topic = p('status_topic', '/road_crossing/light_override/status').value
        fsm_topic = p('fsm_state_topic', '/planning/road_crossing/fsm_state').value
        lease = float(p('green_lease_sec', 2.0).value)
        rate = float(p('publish_rate_hz', 10.0).value)
        self._det_timeout = float(p('detector_timeout_sec', 1.0).value)

        self._arb = LightArbiter(green_lease_sec=lease)
        self._last_det = None
        self._last_det_t = None
        self._last_out_state = PedestrianLightState.NONE
        self._refused_logged = {}

        self._pub = self.create_publisher(PedestrianLightState, out_topic, 10)
        self._pub_status = self.create_publisher(PedestrianLightOverrideStatus, status_topic, 10)
        self.create_subscription(PedestrianLightState, self._in_topic, self._on_detector, 10)
        self.create_subscription(PedestrianLightOverride, cmd_topic, self._on_command, 10)
        self.create_subscription(Int32Stamped, fsm_topic, self._on_fsm, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._on_timer)
        self.get_logger().info(
            f'arbiter: {self._in_topic} -> {out_topic}, perintah {cmd_topic}, '
            f'GREEN lease {lease:.1f} s, mulai AUTO')

    def _t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _log(self, line):
        if line:
            self.get_logger().warn(line)

    # ---- inputs ------------------------------------------------------------------------
    def _on_detector(self, msg):
        self._last_det, self._last_det_t = msg, self._t()
        if not self._arb.manual():
            msg.source = PedestrianLightState.SOURCE_AUTO
            self._publish(msg)

    def _on_command(self, msg):
        was_manual = self._arb.manual()
        line = self._arb.command(msg.mode, msg.request_id, msg.requester, self._t())
        if self._arb.refused:
            # the page keeps re-sending a used-up GREEN: say it once per request id
            if self._refused_logged.get(msg.requester) != msg.request_id:
                self._refused_logged[msg.requester] = msg.request_id
                self._log(line)
        else:
            self._log(line)
        self._after_change(was_manual)

    def _on_fsm(self, msg):
        was_manual = self._arb.manual()
        self._log(self._arb.fsm_state(msg.data, self._t()))
        self._after_change(was_manual)

    def _on_timer(self):
        was_manual = self._arb.manual()
        self._log(self._arb.tick(self._t()))
        self._after_change(was_manual)
        if self._arb.manual():
            out = PedestrianLightState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.state = (PedestrianLightState.GREEN if self._arb.mode == GREEN
                         else PedestrianLightState.RED)
            out.confidence = 1.0
            out.light_id = 0
            out.source = PedestrianLightState.SOURCE_MANUAL
            self._publish(out)
        self._publish_status()

    # ---- outputs -----------------------------------------------------------------------
    def _after_change(self, was_manual):
        if was_manual and not self._arb.manual():
            # end the override in the FSM now (AUTO NONE carries no light for the interlock)
            out = PedestrianLightState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.state = PedestrianLightState.NONE
            out.source = PedestrianLightState.SOURCE_AUTO
            self._publish(out)

    def _publish(self, msg):
        self._last_out_state = msg.state
        self._pub.publish(msg)

    def _publish_status(self):
        t = self._t()
        s = PedestrianLightOverrideStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.mode = self._arb.mode
        s.request_id = self._arb.request_id
        s.requester = self._arb.requester
        s.green_lease_remaining_sec = float(self._arb.lease_remaining(t))
        s.last_used_green_id = self._arb.last_used_green_id
        fresh = self._last_det_t is not None and (t - self._last_det_t) <= self._det_timeout
        s.detected_fresh = fresh
        s.detected_state = self._last_det.state if (fresh and self._last_det) else 0
        s.output_state = (self._last_out_state if self._arb.manual() or fresh
                          else PedestrianLightState.NONE)
        s.reason = self._arb.reason
        self._pub_status.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = LightArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
