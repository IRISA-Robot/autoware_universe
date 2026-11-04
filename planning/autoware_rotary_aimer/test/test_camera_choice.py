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


def _aimer(cameras=('front', 'left', 'rear', 'right'), tf_yaws=None, invert=False,
           tol_deg=0.0, enter_deg=0.0):
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
    n._aim_tol_rad = math.radians(tol_deg)
    n._aim_enter_rad = math.radians(enter_deg)
    n._aim_rotating = True
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


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


@pytest.mark.parametrize('cameras', [
    ('front', 'right', 'left', 'rear'),   # deployed order (2026-11-03)
    ('front', 'left', 'rear', 'right'),
])
def test_reverse_mirrors_forward_with_the_rear_camera(cameras):
    # Reversing with the body at 0 puts the light behind the robot: the rear camera takes it
    # with the same small body turn the front camera needs when driving forward.
    for b in range(-40, 41, 5):
        n = _aimer(cameras=cameras)
        t_f, cam_f, err_f = _aim(n, float(b))
        n = _aimer(cameras=cameras)
        t_r, cam_r, err_r = _aim(n, _wrap_deg(b + 180.0))
        assert cam_f == 'front' and cam_r == 'rear', b
        assert t_r == pytest.approx(t_f, abs=1e-6), b
        assert err_r == pytest.approx(err_f, abs=1e-6), b


# ---------------------------------------------------------------- aim tolerance (20 / 15 deg)

def _tol_aimer(cameras=('front', 'right', 'left', 'rear')):
    return _aimer(cameras=cameras, tol_deg=15.0, enter_deg=20.0)


def test_tolerance_turns_only_to_its_edge_on_enable():
    # just enabled, light 25 deg off the front camera: turn to leave it 15 deg off (10 deg turn)
    n = _tol_aimer()
    t, cam, _ = _aim(n, 25.0)
    assert cam == 'front' and t == pytest.approx(10.0)
    n = _tol_aimer()
    t, _, _ = _aim(n, -25.0)
    assert t == pytest.approx(-10.0)


def test_inside_the_enter_band_after_settling_does_not_turn():
    n = _tol_aimer()
    assert _aim(n, 25.0)[0] == pytest.approx(10.0)
    # body reached 10 deg: the light is 15 deg off -> settled, hold
    assert _aim(n, 25.0, body_deg=10.0)[0] == pytest.approx(10.0)
    # light drifts to 28 deg (18 off): inside 20 -> still hold
    assert _aim(n, 28.0, body_deg=10.0)[0] == pytest.approx(10.0)
    # 32 deg (22 off): beyond 20 -> turn again, to 15 off
    assert _aim(n, 32.0, body_deg=10.0)[0] == pytest.approx(17.0)


def test_on_enable_turns_even_between_tolerance_and_enter():
    # 18 deg off right after enable: the image said "edge" -> turn to 15 off anyway
    n = _tol_aimer()
    assert _aim(n, 18.0)[0] == pytest.approx(3.0)


def test_within_tolerance_on_enable_holds():
    n = _tol_aimer()
    assert _aim(n, 12.0)[0] == pytest.approx(0.0)


def test_tolerance_cases_of_2026_11_03():
    # reverse, light 1331: 43 deg off the left camera -> 28 deg turn instead of 43
    n = _tol_aimer()
    t, cam, _ = _aim(n, 133.0)
    assert cam == 'left' and abs(t) == pytest.approx(28.0)
    # forward, light 892: 37 deg off the left camera (bearing 53) -> 22 deg turn instead of 37
    n = _tol_aimer()
    t, cam, _ = _aim(n, 53.0)
    assert cam == 'left' and abs(t) == pytest.approx(22.0)


def test_tolerance_reverse_mirrors_forward_with_the_rear_camera():
    # (not at exactly 45 deg: a tie between two cameras, decided by the camera list order)
    for b in range(-40, 41, 5):
        n = _tol_aimer()
        t_f, cam_f, _ = _aim(n, float(b))
        n = _tol_aimer()
        t_r, cam_r, _ = _aim(n, _wrap_deg(b + 180.0))
        if cam_f == 'front':
            assert cam_r == 'rear', b
            assert t_r == pytest.approx(t_f, abs=1e-6), b


def test_zero_tolerance_is_the_old_behaviour():
    n = _aimer(tol_deg=0.0, enter_deg=0.0)
    assert _aim(n, 25.0)[0] == pytest.approx(25.0)
