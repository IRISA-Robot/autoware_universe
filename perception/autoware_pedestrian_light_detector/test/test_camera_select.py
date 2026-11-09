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
