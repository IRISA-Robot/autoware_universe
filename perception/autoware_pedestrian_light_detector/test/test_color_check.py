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

"""color_check: top-window colour judges glare-washed lamps and overrides the CNN."""

import numpy as np

from autoware_pedestrian_light_detector import color_check


def lamp(top_bgr, bottom_bgr, h=20, w=10):
    img = np.zeros((h, w, 3), np.uint8)
    img[: h // 2] = top_bgr
    img[h // 2:] = bottom_bgr
    return img


def test_red_man_lit_is_red():
    # like the red phase under glare: top strongly red, bottom washed out
    f = color_check.top_window_features(lamp((110, 120, 245), (150, 200, 220)))
    assert f.redness_p90 > 100 and f.sat_mean > 72
    assert color_check.judge(f) == 'red'


def test_washed_out_top_is_green():
    # green phase under glare: top pale (redness ~50, low saturation), bottom pale yellow
    f = color_check.top_window_features(lamp((175, 180, 225), (157, 211, 216)))
    assert f.redness_p90 < 65 and f.sat_mean < 70
    assert color_check.judge(f) == 'green'


def test_in_between_has_no_opinion():
    f = color_check.top_window_features(lamp((140, 150, 222), (150, 200, 220)))
    assert color_check.judge(f) is None
    assert color_check.judge(None) is None
    assert color_check.top_window_features(np.zeros((1, 5, 3), np.uint8)) is None


def test_override_only_red_green():
    assert color_check.override('red', 'green') == 'green'
    assert color_check.override('green', 'red') == 'red'
    assert color_check.override('red', None) == 'red'
    assert color_check.override('none', 'green') == 'none'


def test_crop_image_msg_bgr_and_rgb_with_padding():
    h, w = 6, 5
    img = np.arange(h * w * 3, dtype=np.uint8).reshape(h, w, 3)
    step = w * 3 + 4                       # row padding
    buf = np.zeros((h, step), np.uint8)
    buf[:, : w * 3] = img.reshape(h, -1)
    c = color_check.crop_image_msg(buf.tobytes(), w, h, step, 'bgr8', (1, 2, 3, 2))
    assert np.array_equal(c, img[2:4, 1:4])
    c = color_check.crop_image_msg(buf.tobytes(), w, h, step, 'rgb8', (1, 2, 3, 2))
    assert np.array_equal(c, img[2:4, 1:4, ::-1])
    # clipped at the border, unsupported encoding
    assert color_check.crop_image_msg(buf.tobytes(), w, h, step, 'bgr8', (3, 4, 9, 9)).shape == (2, 2, 3)
    assert color_check.crop_image_msg(buf.tobytes(), w, h, step, 'mono8', (0, 0, 2, 2)) is None
    assert color_check.crop_image_msg(buf.tobytes(), w, h, step, 'bgr8', (9, 9, 2, 2)) is None


def test_max_redness_and_full_lamp_gate_640():
    p = color_check.Params(red_min=55, red_sat_min=0, green_max=45, green_sat_max=255,
                           min_box_h=10, use_max=True, min_aspect=1.7)
    # one lit red pixel in a washed-out top window (640x480 lamp, ~5x11 box)
    img = np.full((11, 6, 3), (200, 205, 210), np.uint8)
    img[2, 3] = (120, 130, 200)                        # R - max(G,B) = 70
    assert color_check.judge(color_check.top_window_features(img), p) == 'red'
    img[2, 3] = (200, 205, 210)                        # not lit
    assert color_check.judge(color_check.top_window_features(img), p) == 'green'
    # full lamp 6x11 judges; half a lamp (5x7) or a short box does not
    assert color_check.full_lamp_box(6, 11, p)
    assert not color_check.full_lamp_box(5, 7, p)
    assert not color_check.full_lamp_box(6, 9, p)
