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

"""roi_projector.select(): the camera that sees the light most centred is chosen."""

import math
from types import SimpleNamespace

import autoware_pedestrian_light_detector.roi_projector as rp
import numpy as np
import pytest

CAM_YAW_DEG = {'front': 0.0, 'left': 90.0, 'rear': 180.0, 'right': -90.0}


def _quat_from_matrix(m):
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0,
                      m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0,
                      m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0,
                      m[1, 0] - m[0, 1])
    return SimpleNamespace(x=x, y=y, z=z, w=w)


class _TfBuffer:
    """map -> camera_<name>_optical_frame; cameras at the map origin, yawed per CAM_YAW_DEG."""

    def lookup_transform(self, target, source, _stamp):
        assert source == 'map'
        a = math.radians(CAM_YAW_DEG[target.split('_')[1]])
        r_link = np.array([[math.cos(a), math.sin(a), 0.0],
                           [-math.sin(a), math.cos(a), 0.0],
                           [0.0, 0.0, 1.0]])
        r_opt = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]) @ r_link
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0), rotation=_quat_from_matrix(r_opt)))


def _cameras(hfov_deg):
    f = 320.0 / math.tan(math.radians(hfov_deg / 2.0))
    k = np.array([[f, 0.0, 320.0], [0.0, f, 240.0], [0.0, 0.0, 1.0]])
    return [SimpleNamespace(name=n, K=k, frame_id=f'camera_{n}_optical_frame',
                            width=640, height=480) for n in CAM_YAW_DEG]


def _light(bearing_deg, dist=10.0, way_id='1'):
    a = math.radians(bearing_deg)
    x, y = dist * math.cos(a), dist * math.sin(a)
    return SimpleNamespace(way_id=way_id, midpoint=(x, y, 0.0),
                           points=[(x, y, -0.3), (x, y, 0.3)], height_m=None)


@pytest.fixture(autouse=True)
def _tf_available(monkeypatch):
    monkeypatch.setattr(rp, '_TF2_AVAILABLE', True)


@pytest.mark.parametrize('bearing, camera', [
    (-60.0, 'right'), (-30.0, 'front'), (20.0, 'front'), (50.0, 'left'),
    (100.0, 'left'), (170.0, 'rear'), (-120.0, 'right'),
])
def test_most_centred_camera_with_overlapping_fov(bearing, camera):
    # 120 deg FOV: the light is in two cameras' view; the one nearer its axis wins.  (The
    # old smallest-depth rule took the other one: depth = range * cos(off-axis angle).)
    sel = rp.select([_light(bearing)], _cameras(120.0), _TfBuffer())
    assert sel is not None and sel[0] == camera


def test_nearest_light_first_then_its_most_centred_camera():
    near = _light(-60.0, dist=5.0, way_id='near')
    far = _light(5.0, dist=30.0, way_id='far')
    sel = rp.select([far, near], _cameras(120.0), _TfBuffer())
    assert sel[3].way_id == 'near' and sel[0] == 'right'


def test_no_camera_sees_the_light():
    # 40 deg FOV cameras leave 45 deg gaps
    assert rp.select([_light(45.0)], _cameras(40.0), _TfBuffer()) is None


def _pinned_light(bearing_deg, camera):
    lt = _light(bearing_deg)
    lt.camera = camera
    return lt


def test_hysteresis_keeps_the_camera_in_use():
    cams, tf = _cameras(120.0), _TfBuffer()
    h = math.radians(15.0)
    # light at 50 deg: left is 40 deg off, front 50 -- front in use stays (within 15 deg)
    assert rp.select([_light(50.0)], cams, tf, current_cam='front', hysteresis_rad=h)[0] == 'front'
    # light at 60 deg: left 30 off, front 60 -- beyond 15 deg, switch
    assert rp.select([_light(60.0)], cams, tf, current_cam='front', hysteresis_rad=h)[0] == 'left'
    # no camera in use yet / no hysteresis: the most centred one, as before
    assert rp.select([_light(50.0)], cams, tf)[0] == 'left'
    assert rp.select([_light(50.0)], cams, tf, current_cam='front')[0] == 'left'


def test_hysteresis_stops_flip_flop_at_the_boundary():
    # light wobbling around the front/left boundary (45 deg): the camera never changes
    cams, tf = _cameras(120.0), _TfBuffer()
    cur = 'front'
    for b in (44.0, 46.0, 43.0, 47.5, 45.0, 48.0, 42.0):
        cur = rp.select([_light(b)], cams, tf, current_cam=cur,
                        hysteresis_rad=math.radians(15.0))[0]
        assert cur == 'front', b


def test_map_tag_pins_the_camera():
    cams, tf = _cameras(120.0), _TfBuffer()
    # pinned to front although left sees it more centred
    assert rp.select([_pinned_light(50.0, 'front')], cams, tf)[0] == 'front'
    # pin wins over the camera in use as well
    assert rp.select([_pinned_light(50.0, 'front')], cams, tf, current_cam='left',
                     hysteresis_rad=math.radians(15.0))[0] == 'front'
    # pinned camera does not see the light: fall back to the automatic choice
    assert rp.select([_pinned_light(50.0, 'rear')], cams, tf)[0] == 'left'


def test_map_tag_is_parsed(tmp_path):
    osm = tmp_path / 'm.osm'
    osm.write_text(
        '<osm>'
        '<node id="1" lat="0" lon="0"><tag k="local_x" v="1"/><tag k="local_y" v="2"/></node>'
        '<node id="2" lat="0" lon="0"><tag k="local_x" v="1"/><tag k="local_y" v="3"/></node>'
        '<way id="10"><nd ref="1"/><nd ref="2"/><tag k="tf_pedestrian" v="true"/>'
        '<tag k="pedestrian_light_camera" v="left"/></way>'
        '<way id="11"><nd ref="1"/><nd ref="2"/><tag k="tf_pedestrian" v="true"/></way>'
        '</osm>')
    lights = {lt.way_id: lt for lt in rp.parse_tf_pedestrian_lights(str(osm))}
    assert lights['10'].camera == 'left' and lights['11'].camera is None



# ---- 2026-11-17: liveness + sticky camera -------------------------------------------------

def test_liveness_dead_after_timeout_and_revive_delay():
    lv = rp.CameraLiveness(dead_sec=2.0, revive_sec=0.5)
    lv.rx('left', 0.0)
    assert lv.is_alive('left', 0.0)              # never dead: alive from the first message
    assert lv.is_alive('left', 1.9)              # gap 1.9 s < 2 s: still alive
    assert not lv.is_alive('left', 2.1)          # gap > 2 s: dead
    lv.rx('left', 3.0)                           # first message after the gap
    assert not lv.is_alive('left', 3.0)          # not before 0.5 s of data
    lv.rx('left', 3.3)
    lv.rx('left', 3.5)
    assert lv.is_alive('left', 3.5)
    assert not lv.is_alive('front', 3.5)         # never received


def test_liveness_short_gaps_of_2026_11_16_do_not_kill():
    # inactive camera_info gaps of ~1.5 s (detector log 2026-11-16) must not count as dead
    lv = rp.CameraLiveness(dead_sec=2.0, revive_sec=0.5)
    t = 0.0
    for gap in (0.033, 1.5, 0.033, 1.4, 0.033, 1.6):
        t += gap
        lv.rx('right', t)
        assert lv.is_alive('right', t + 0.3)


def test_liveness_off():
    lv = rp.CameraLiveness(dead_sec=0.0)
    assert lv.is_alive('front', 100.0)


def test_sticky_keeps_current_camera_while_it_sees_the_light():
    # light 40 deg left: front 40 off-axis, left 50 -> best is front; left in use stays
    cams, tf = _cameras(120.0), _TfBuffer()
    res, why = rp.select_sticky([_light(40.0)], cams, tf, current_cam='left')
    assert res[0] == 'left' and why == 'tetap'
    # even when another camera is much more centred (light 55 deg: left 35, front 55 -- beyond
    # the old 15 deg hysteresis)
    res, _ = rp.select_sticky([_light(55.0)], cams, tf, current_cam='front')
    assert res[0] == 'front'


def test_sticky_initial_pick_is_the_best_or_pinned():
    cams, tf = _cameras(120.0), _TfBuffer()
    assert rp.select_sticky([_light(40.0)], cams, tf)[0][0] == 'front'
    assert rp.select_sticky([_pinned_light(40.0, 'left')], cams, tf)[0][0] == 'left'


def test_sticky_switches_when_current_dead():
    cams, tf = _cameras(120.0), _TfBuffer()
    alive = [c for c in cams if c.name != 'front']
    res, why = rp.select_sticky([_light(40.0)], alive, tf, current_cam='front')
    assert res[0] == 'left' and 'mati' in why


def test_sticky_switches_when_light_leaves_the_fov():
    # 40 deg FOV cameras: a light at 30 deg is outside front (half-FOV 20) but... left is 60 off,
    # also outside -> nobody; use 120 deg: light at 70 deg is outside front (60) -> left
    cams, tf = _cameras(120.0), _TfBuffer()
    res, why = rp.select_sticky([_light(70.0)], cams, tf, current_cam='front')
    assert res[0] == 'left' and 'keluar' in why


def test_sticky_switch_away_only_when_asked_and_another_camera_sees_it():
    cams, tf = _cameras(120.0), _TfBuffer()
    res, why = rp.select_sticky([_light(40.0)], cams, tf, current_cam='front', switch_away=True)
    assert res[0] == 'left' and 'NONE' in why
    # light at 0 deg: only front sees it (left 90 off) -> stays
    res, why = rp.select_sticky([_light(0.0)], cams, tf, current_cam='front', switch_away=True)
    assert res[0] == 'front' and why.startswith('tetap')


def _run(lights, alive_at, none_at=lambda t: False, moving_at=lambda t: False,
         none_sec=1.5, dt=0.1, n=120):
    """Node loop: liveness + sticky + NONE timer (reset on a camera change)."""
    cams, tf = _cameras(120.0), _TfBuffer()
    lv = rp.CameraLiveness(2.0, 0.5)
    cur, none_since, out = '', None, []
    for i in range(n):
        t = round(i * dt, 6)
        for name in alive_at(t):
            lv.rx(name, t)
        none_for = t - none_since if none_since is not None else 0.0
        res, _ = rp.select_sticky(lights, lv.alive(cams, t), tf, current_cam=cur,
                                  switch_away=moving_at(t) and none_for >= none_sec)
        new = res[0] if res else cur
        if new != cur:
            none_since = None
        cur = new
        if none_at(t):
            none_since = t if none_since is None else none_since
        else:
            none_since = None
        out.append((t, cur))
    return out


def _cam(out, t):
    return next(c for tt, c in out if tt >= t - 1e-9)


ALL = ['front', 'left', 'right', 'rear']


def test_stationary_covered_camera_never_switches():
    # all cameras normal, robot stands still, the camera in use is covered (NONE) for 8 s
    out = _run([_light(40.0)], lambda t: ALL, none_at=lambda t: 2.0 <= t < 10.0)
    assert {c for _, c in out} == {'front'}


def test_moving_none_switches_after_none_sec():
    out = _run([_light(40.0)], lambda t: ALL, none_at=lambda t: t >= 2.0,
               moving_at=lambda t: True)
    assert _cam(out, 3.4) == 'front' and _cam(out, 3.6) == 'left'


def test_front_dies_left_then_front_back_is_not_taken_back():
    # front dead 3..6 s: left after 2 s without data; front back -> sticky stays on left
    out = _run([_light(40.0)], lambda t: ALL if not 3.0 <= t < 6.0 else ['left', 'right', 'rear'])
    assert _cam(out, 2.9) == 'front'
    assert _cam(out, 4.8) == 'front'     # last front frame 2.9 s: gap 1.9 s, not dead yet
    assert _cam(out, 5.0) == 'left'      # gap 2.1 s > 2.0 s
    assert _cam(out, 11.9) == 'left'


def test_flapping_inactive_cameras_cause_no_switch():
    # 2026-11-16: left/right camera_info gaps ~1.5 s while right is in use
    def alive_at(t):
        names = ['right', 'rear']
        if (t % 3.0) < 1.5:
            names.append('left')
        return names
    out = _run([_light(-40.0)], alive_at)
    assert {c for _, c in out} == {'right'}
