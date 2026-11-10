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

"""lidar_check: a box that shows something near the robot is rejected, the light is kept."""

import numpy as np

from autoware_pedestrian_light_detector import lidar_check as lc

# Camera at the map origin looking along +x (cloud frame = map).  Optical frame:
# z forward = map x, x right = -map y, y down = -map z.
R_CAM_MAP = np.array([[0.0, -1.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [1.0, 0.0, 0.0]])
T_CAM = lc.transform_matrix(R_CAM_MAP, [0.0, 0.0, 0.0])
T_MAP = np.eye(4)
K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])

LIGHT = np.array([[15.0, -0.2, 2.0], [15.0, 0.2, 2.0]])   # linestring, 15 m ahead, 2 m up
LIGHT_DEPTH = 15.0
# light projects to u=320, v = 240 - 500*2/15 = 173
BOX = (300, 150, 40, 50)


def _on_ray(scale, n=6, spread=0.02):
    """n points along the camera ray to the light centre, at scale * light distance."""
    c = np.array([15.0, 0.0, 2.0]) * scale
    offs = np.linspace(-spread, spread, n)
    return np.array([[c[0], c[1] + o, c[2] + o] for o in offs])


def _check(pts, **kw):
    return lc.check(pts, T_CAM, T_MAP, K, BOX, LIGHT, LIGHT_DEPTH, **kw)


def test_person_in_front_of_the_camera_is_rejected():
    v = _check(_on_ray(0.1))            # 1.5 m from the camera, in the box
    assert not v.ok
    assert v.reason == 'far_from_light'
    assert v.min_dist_m > 5.0


def test_light_hit_by_the_lidar_is_kept():
    v = _check(_on_ray(1.0))
    assert v.ok and v.reason == 'near_light'
    assert v.min_dist_m < 0.5


def test_smallest_distance_wins_when_the_light_is_also_hit():
    v = _check(np.vstack([_on_ray(0.1), _on_ray(1.0)]))
    assert v.ok and v.min_dist_m < 0.5


def test_background_behind_the_light_is_ignored():
    # light not hit (thin housing); only a building 15 m behind it in the box
    v = _check(_on_ray(2.0))
    assert v.ok and v.reason == 'few_points'
    assert v.n_in_box == 6 and v.n_front == 0


def test_object_within_threshold_of_the_light_is_kept():
    # something 3 m in front of the light (e.g. the pole of a nearby sign): < 5 m -> accept
    v = _check(_on_ray(0.8))
    assert v.ok and v.min_dist_m < 5.0


def test_threshold_is_configurable():
    assert not _check(_on_ray(0.8), max_dist_m=2.0).ok


def test_no_cloud_or_too_few_points_cannot_judge():
    assert _check(None).reason == 'no_cloud'
    assert _check(np.zeros((0, 3))).ok
    v = _check(_on_ray(0.1, n=2))      # 2 stray points < min_points 3
    assert v.ok and v.reason == 'few_points'


def test_points_outside_the_box_or_behind_the_camera_do_not_count():
    beside = _on_ray(0.1) + np.array([0.0, 1.0, 0.0])   # 1 m to the left at 1.5 m: off-box
    behind = -_on_ray(0.1)                               # behind the camera
    v = _check(np.vstack([beside, behind]))
    assert v.ok and v.n_in_box == 0


def test_polyline_distance():
    line = np.array([[0.0, 0.0], [10.0, 0.0]])
    assert lc.min_xy_dist_to_polyline(np.array([[5.0, 3.0]]), line) == 3.0
    assert lc.min_xy_dist_to_polyline(np.array([[-4.0, 3.0]]), line) == 5.0
    assert lc.min_xy_dist_to_polyline(np.array([[2.0, 0.0]]), np.array([[2.0, 1.0]])) == 1.0
