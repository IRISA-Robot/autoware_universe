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

"""roi_refine tolerance: a light near the edge of the rough ROI (projection error) is kept."""

from autoware_pedestrian_light_detector import roi_refine as rr

ROUGH = (100, 100, 200, 220)          # x1, y1, x2, y2
CENTER = (150.0, 160.0)
EDGE_DET = (190, 150, 16, 30, 0.9)    # sticks 6 px out of the right edge, centre inside


def test_full_inside_rule_drops_a_light_at_the_edge():
    assert rr.select_detection([EDGE_DET], ROUGH, CENTER, 30.0) is None


def test_centre_inside_rule_keeps_it():
    box = rr.select_detection([EDGE_DET], ROUGH, CENTER, 30.0, require_full_inside=False)
    assert box == (190, 150, 16, 30)


def test_centre_outside_is_still_dropped():
    far = (205, 150, 16, 30, 0.9)     # centre at x=213 > 200
    assert rr.select_detection([far], ROUGH, CENTER, 30.0, require_full_inside=False) is None


def test_refine_and_hold_follow_the_rule():
    hold = rr.DetectionHold(1.0)
    box, src = rr.refine([EDGE_DET], ROUGH, CENTER, (14.0, 30.0), (640, 480), hold, 0.0,
                         require_full_inside=False)
    assert src == 'detection'
    box, src = rr.refine([], ROUGH, CENTER, (14.0, 30.0), (640, 480), hold, 0.5,
                         require_full_inside=False)
    assert src == 'hold' and box == (190, 150, 16, 30)
    box, src = rr.refine([], ROUGH, CENTER, (14.0, 30.0), (640, 480), hold, 0.6)
    assert src == 'fallback'          # full-inside rule: the held box is outside


def test_pad_box_grows_and_clips():
    assert rr.pad_box((100, 50, 5, 10), 2, 640, 480) == (98, 48, 9, 14)
    assert rr.pad_box((0, 1, 5, 10), 2, 640, 480) == (0, 0, 7, 13)
    assert rr.pad_box((636, 470, 4, 10), 2, 640, 480) == (634, 468, 6, 12)
    assert rr.pad_box((10, 10, 5, 10), 0, 640, 480) == (10, 10, 5, 10)


def test_centered_roi_is_shifted_inside_like_the_reference():
    # traffic_light_infer.py: x0 = int(min(max(cx - rw/2, 0), max(0, W - rw)))
    assert rr.centered_roi(277, 198, 120, 96, 640, 480) == (217, 150, 337, 246)
    assert rr.centered_roi(10, 5, 120, 96, 640, 480) == (0, 0, 120, 96)
    assert rr.centered_roi(635, 478, 120, 96, 640, 480) == (520, 384, 640, 480)
    assert rr.centered_roi(100, 100, 800, 96, 640, 480) == (0, 52, 640, 148)


def test_majority_needs_more_than_half():
    N = 0
    assert rr.majority([1, 1, 3, 1, 1], 5, N) == 1
    assert rr.majority([0, 3, 0, 3, 0], 5, N) == 0      # single greens between NONE: no green
    assert rr.majority([3, 3, 0, 1, 1], 5, N) == N      # no majority -> NONE
    assert rr.majority([2, 2, 3, 3, 3, 3], 5, N) == 3   # only the last 5 count
    assert rr.majority([3], 5, N) == 3
    assert rr.majority([], 5, N) == N
    assert rr.majority([1, 3], 1, N) == 3               # window 1 = off


def test_grow_box_right_bottom_matches_reference_crop():
    # reference crop of a float box (29.4, 147.2, 34.7, 158.9) is [147:158, 29:34] = 5x11;
    # TensorRT gives 5x10 at the same corner -> +1 bottom (and +1 right) restores it
    assert rr.grow_box((29, 147, 4, 10), (0, 0, 1, 1), 640, 480) == (29, 147, 5, 11)
    assert rr.grow_box((636, 470, 4, 10), (0, 0, 1, 1), 640, 480) == (636, 470, 4, 10)
    assert rr.grow_box((0, 0, 5, 5), (2, 2, 2, 2), 640, 480) == (0, 0, 7, 7)
