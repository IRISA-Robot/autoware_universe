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

"""lens: plumb_bob projection equals OpenCV, and never folds far points into the image."""

import cv2
import numpy as np

from autoware_pedestrian_light_detector import lens

# left camera calibration (calibration/camera_left.yaml)
K = np.array([[313.66054, 0.0, 331.24072], [0.0, 313.66054, 240.0], [0.0, 0.0, 1.0]])
D = [-0.286963, 0.060986, -0.000266, -0.002720, 0.0]


def test_matches_opencv_inside_the_image():
    dist = lens.Distortion(D)
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(-1.0, 1.0, 200), rng.uniform(-0.7, 0.7, 200),
                           np.ones(200)]) * rng.uniform(5.0, 40.0, (200, 1))
    ref, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K, np.array(D))
    for p, r in zip(pts, ref.reshape(-1, 2)):
        uv = lens.project_camera_point(p[0], p[1], p[2], K, dist)
        assert uv is not None
        assert abs(uv[0] - r[0]) < 1e-6 and abs(uv[1] - r[1]) < 1e-6


def test_edge_of_the_image_moves_inwards():
    # a light 38 deg off axis: pinhole puts it ~575 px, the real (barrel) lens ~535 px
    dist = lens.Distortion(D)
    X, Z = np.tan(np.radians(38.0)), 1.0
    u_pin = lens.project_camera_point(X, 0.0, Z, K, None)[0]
    u_real = lens.project_camera_point(X, 0.0, Z, K, dist)[0]
    assert u_pin > 570
    assert 30 < u_pin - u_real < 60


def test_far_outside_the_model_is_not_projected():
    dist = lens.Distortion(D)
    assert dist.r_max > 1.3      # beyond the image corners (r ~ 1.3); this lens stays monotonic
    # 80 deg off axis: the polynomial would fold it back near the centre -> refused
    X = np.tan(np.radians(80.0))
    assert lens.project_camera_point(X, 0.0, 1.0, K, dist) is None
    assert lens.project_camera_point(0.1, 0.0, -1.0, K, dist) is None  # behind


def test_no_distortion_is_pinhole():
    assert lens.Distortion.from_camera_info('plumb_bob', [0, 0, 0, 0, 0]) is None
    assert lens.Distortion.from_camera_info('equidistant', D[:4]) is None
    assert lens.Distortion.from_camera_info('plumb_bob', D) is not None
    uv = lens.project_camera_point(1.0, 0.5, 10.0, K, None)
    assert abs(uv[0] - (313.66054 * 0.1 + 331.24072)) < 1e-9


def test_folding_lens_is_cut_at_its_monotonic_radius():
    # strong barrel without a k2 to bend it back: r*(1 + k1 r^2) peaks at r = 1/sqrt(3|k1|)
    dist = lens.Distortion([-0.3, 0.0, 0.0, 0.0, 0.0])
    assert abs(dist.r_max - 1.0 / np.sqrt(0.9)) < 0.01
    assert lens.project_camera_point(1.2, 0.0, 1.0, K, dist) is None   # past the fold
    assert lens.project_camera_point(0.9, 0.0, 1.0, K, dist) is not None
