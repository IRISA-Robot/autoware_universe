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

"""ae_feed: image topic choice with fallback, padded box -> driver AE ROI."""

from autoware_pedestrian_light_detector import ae_feed


def test_image_topic():
    assert ae_feed.image_topic('front', 'ae/image_raw') == '/camera/front/ae/image_raw'
    assert ae_feed.image_topic('left', '/image_raw') == '/camera/left/image_raw'
    assert ae_feed.image_topic('rear', 'ae/roi') == '/camera/rear/ae/roi'


def test_bbox_to_roi_inside():
    assert ae_feed.bbox_to_roi((100, 50, 180, 210), 640, 480) == (100, 50, 80, 160)


def test_bbox_to_roi_clamped_to_the_image():
    # padded box sticking out of the frame on the left and bottom
    assert ae_feed.bbox_to_roi((-30, 400, 40, 520), 640, 480) == (0, 400, 40, 80)


def test_bbox_to_roi_empty_or_outside_is_none():
    assert ae_feed.bbox_to_roi(None, 640, 480) is None
    assert ae_feed.bbox_to_roi((700, 10, 800, 90), 640, 480) is None
    assert ae_feed.bbox_to_roi((10, 10, 11, 90), 640, 480) is None
    assert ae_feed.bbox_to_roi((10, 10, 50, 90), 0, 0) is None


def test_fallback_after_silence_only_once_and_per_camera():
    src = ae_feed.ImageSourceFallback('ae/image_raw', 'image_raw', 3.0)
    assert src.suffix_for('front') == 'ae/image_raw'
    src.subscribed(10.0, 'front')
    assert not src.check(12.9)
    assert src.check(13.0)                     # 3 s without a frame
    assert src.suffix_for('front') == 'image_raw'
    assert not src.check(20.0)                 # already on the fallback
    assert src.suffix_for('left') == 'ae/image_raw'  # other cameras still try AE


def test_frames_keep_the_preferred_source():
    src = ae_feed.ImageSourceFallback('ae/image_raw', 'image_raw', 3.0)
    src.subscribed(0.0, 'front')
    for t in (1.0, 2.0, 3.5, 5.0, 7.9):
        src.image(t)
        assert not src.check(t + 0.1)
    # stream stops mid-episode
    assert src.check(7.9 + 3.0)


def test_reset_retries_ae_next_episode():
    src = ae_feed.ImageSourceFallback('ae/image_raw', 'image_raw', 3.0)
    src.subscribed(0.0, 'front')
    assert src.check(4.0)
    src.reset()
    assert src.suffix_for('front') == 'ae/image_raw'


def test_preferred_equal_to_fallback_never_switches():
    src = ae_feed.ImageSourceFallback('image_raw', 'image_raw', 3.0)
    src.subscribed(0.0, 'front')
    assert not src.check(100.0)
    assert src.suffix_for('front') == 'image_raw'


def test_returns_to_ae_when_it_delivers_again():
    src = ae_feed.ImageSourceFallback('ae/image_raw', 'image_raw', 3.0, return_frames=3)
    src.subscribed(0.0, 'front')
    assert src.check(3.0)                      # AE switched off -> fallback
    assert src.on_fallback('front')
    assert not src.should_return('front')
    src.preferred_frame(10.0, 'front')
    src.preferred_frame(10.1, 'front')
    assert not src.should_return('front')      # 2 frames are not enough
    src.preferred_frame(10.2, 'front')
    assert src.should_return('front')          # AE enabled again -> back
    assert src.suffix_for('front') == 'ae/image_raw'
    assert not src.on_fallback('front')
    # and if it dies again, the fallback works again
    src.subscribed(11.0, 'front')
    assert src.check(14.0)


def test_scattered_preferred_frames_do_not_count():
    src = ae_feed.ImageSourceFallback('ae/image_raw', 'image_raw', 3.0, return_frames=3)
    src.subscribed(0.0, 'front')
    assert src.check(3.0)
    for t in (10.0, 20.0, 30.0):              # one stray frame every 10 s
        src.preferred_frame(t, 'front')
        assert not src.should_return('front')


def test_preferred_frames_for_a_camera_not_on_fallback_are_ignored():
    src = ae_feed.ImageSourceFallback('ae/image_raw', 'image_raw', 3.0)
    src.preferred_frame(1.0, 'left')
    assert not src.should_return('left')
