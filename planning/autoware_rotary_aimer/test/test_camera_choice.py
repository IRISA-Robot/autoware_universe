# Copyright 2026 azzamwildan462
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

"""Camera choice of rotary_aimer: the camera with the smallest error faces the light."""

import math
from types import SimpleNamespace

from autoware_rotary_aimer.rotary_aimer_node import RotaryAimerNode
import pytest


class _Logger:
    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass


def _quat_yaw(yaw):
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class _TfBuffer:
    """body_link -> camera_<name>_link with the given yaws [deg]."""

    def __init__(self, yaws_deg):
        self._yaws = yaws_deg

    def lookup_transform(self, target, source, _stamp):
        assert target == 'body_link'
        name = source[len('camera_'):-len('_link')]
        if name not in self._yaws:
            raise RuntimeError('no tf')
        return SimpleNamespace(
            transform=SimpleNamespace(rotation=_quat_yaw(math.radians(self._yaws[name]))))


def _aimer(cameras=('front', 'left', 'rear', 'right'), tf_yaws=None, invert=False):
    n = RotaryAimerNode.__new__(RotaryAimerNode)  # no rclpy node: only the pure logic
    n.get_logger = _Logger
    n._tf_buffer = _TfBuffer(tf_yaws) if tf_yaws is not None else None
    n._cam_yaw_tf = {}
    n._cam_yaw_missing_logged = False
    n._cameras = list(cameras)
    n._camera_frame_template = 'camera_{}_link'
    n._body_frame = 'body_link'
    n._front_cam_offset_rad = 0.0
    n._invert_sign = invert
    n._camera_hysteresis_rad = math.radians(15.0)
    n._feedback_received = False
    n._latest_feedback_angle = None
    n._last_published_angle = 0.0
    n._aim_camera = None
    return n


def _aim(n, bearing_deg, body_deg=0.0):
    n._last_published_angle = math.radians(body_deg)
    target, cam, err = n._camera_desired(math.radians(bearing_deg))
    return math.degrees(target), cam, math.degrees(err)


@pytest.mark.parametrize('bearing, camera, target', [
    (-60.0, 'right', 30.0),   # the example: right camera, body +30
    (100.0, 'left', 10.0),
    (180.0, 'rear', 0.0),
    (30.0, 'front', 30.0),
    (-150.0, 'rear', 30.0),
    (60.0, 'left', -30.0),
])
def test_smallest_error_camera_faces_the_light(bearing, camera, target):
    t, cam, _ = _aim(_aimer(), bearing)
    assert cam == camera
    assert t == pytest.approx(target, abs=1e-6)


def test_platform_turns_at_most_45_deg():
    n = _aimer()
    for b in range(-180, 180, 5):
        n._aim_camera = None
        t, _, _ = _aim(n, b)
        assert abs(t) <= 45.0 + 1e-6


def test_hysteresis_keeps_the_camera_in_use():
    n = _aimer()
    n._aim_camera = 'front'
    # left is closer (-40 vs +50) but not by more than 15 deg: stay on front
    assert _aim(n, 50.0)[1] == 'front'
    # left closer by 50 deg: switch
    assert _aim(n, 70.0)[1] == 'left'


def test_error_is_measured_from_the_current_body_angle():
    n = _aimer()
    # body at +40: front axis at +40, so a light at +60 is 20 deg off front, 70 off left
    t, cam, err = _aim(n, 60.0, body_deg=40.0)
    assert cam == 'front' and err == pytest.approx(20.0) and t == pytest.approx(60.0)


def test_front_only_is_the_old_behaviour():
    t, cam, _ = _aim(_aimer(cameras=['front']), -120.0)
    assert cam == 'front' and t == pytest.approx(-120.0)


def test_yaw_from_tf_relative_to_the_front_camera():
    # lidar_top turned 180 deg on body_link: every camera is turned with it, and the yaws
    # relative to the front camera stay 0/90/180/-90.
    n = _aimer(tf_yaws={'front': 180.0, 'left': -90.0, 'rear': 0.0, 'right': 90.0})
    t, cam, _ = _aim(n, -60.0)
    assert cam == 'right' and t == pytest.approx(30.0)


def test_calibrated_yaw_is_followed():
    # right camera mounted at -80 deg instead of -90: body +20 puts it on a light at -60
    n = _aimer(tf_yaws={'front': 0.0, 'left': 90.0, 'rear': 180.0, 'right': -80.0})
    t, cam, _ = _aim(n, -60.0)
    assert cam == 'right' and t == pytest.approx(20.0)


def test_invert_sign_reads_the_body_angle_in_the_physical_convention():
    n = _aimer(invert=True)
    # command +40 = physical -40: front axis at -40, a light at -60 is 20 deg off front
    _, cam, err = _aim(n, -60.0, body_deg=40.0)
    assert cam == 'front' and err == pytest.approx(-20.0)
