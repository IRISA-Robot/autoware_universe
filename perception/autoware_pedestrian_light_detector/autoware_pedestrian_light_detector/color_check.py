"""Colour check of the pedestrian light's TOP window (red man) to confirm/override the CNN.

Why: under sun glare (lamp 892, left camera, 2026-11-11) both windows of the lamp saturate and
the lit green window shows as pale yellow; the MobileNetV2 ped classifier then says RED for
295/298 green-phase frames (same model + preprocessing as traffic_light_infer.py).  What still
separates the phases is the top window:

                       red phase (538)        green phase (298)
  redness p90          med 119, p5 110        med 50, p95 53       redness = R - max(G, B)
  saturation mean      med 86,  p5 81         med 58, p95 66       HSV S of the top half

judge():  redness > red_min AND sat > red_sat_min      -> 'red'
          redness < green_max AND sat < green_sat_max  -> 'green'
          otherwise                                     -> None (no opinion)
On those crops: red phase 531 red / 0 green / 7 none; green phase 294 green / 1 red / 3 none.

override(): the colour check wins over the classifier when it has an opinion, so RED read on a
green lamp becomes GREEN and a GREEN read while the red man is clearly lit becomes RED.  With no
opinion the classifier result stays.  Pure numpy/cv2, tested in test/test_color_check.py.
"""

from typing import NamedTuple, Optional, Tuple

import cv2
import numpy as np


class Params(NamedTuple):
    red_min: float = 80.0
    red_sat_min: float = 72.0
    green_max: float = 65.0
    green_sat_max: float = 70.0
    min_box_h: int = 8
    # 2026-11-12, 640x480: the lamp window is 1-2 px, so the MAX redness (the lit pixel) is the
    # feature; saturation does not separate across lamps (set the sat limits to 0 / 255).
    use_max: bool = False
    # Only a FULL lamp box may judge: a box on half a lamp (4x6 instead of 5x11, person or truck
    # in front) has no red window in its top half and would read "not red" on a red lamp.
    min_aspect: float = 0.0


class Features(NamedTuple):
    redness_p90: float
    sat_mean: float
    redness_max: float = 0.0


def top_window_features(box_bgr: np.ndarray) -> Optional[Features]:
    """Features of the top half of the (tight) lamp box, BGR uint8."""
    if box_bgr is None or box_bgr.ndim != 3 or box_bgr.shape[0] < 2 or box_bgr.shape[1] < 1:
        return None
    top = box_bgr[: box_bgr.shape[0] // 2]
    t = top.astype(np.float32)
    redness = t[..., 2] - np.maximum(t[..., 1], t[..., 0])
    sat = cv2.cvtColor(np.ascontiguousarray(top), cv2.COLOR_BGR2HSV)[..., 1]
    return Features(float(np.percentile(redness, 90)), float(sat.astype(np.float32).mean()),
                    float(redness.max()))


def full_lamp_box(w: int, h: int, p: Params = Params()) -> bool:
    """Box big and tall enough to hold both windows (see Params.min_aspect)."""
    return h >= p.min_box_h and w > 0 and h >= p.min_aspect * w


def judge(feat: Optional[Features], p: Params = Params()) -> Optional[str]:
    if feat is None:
        return None
    redness = feat.redness_max if p.use_max else feat.redness_p90
    if redness > p.red_min and feat.sat_mean > p.red_sat_min:
        return 'red'
    if redness < p.green_max and feat.sat_mean < p.green_sat_max:
        return 'green'
    return None


def crop_image_msg(data, width: int, height: int, step: int, encoding: str,
                   box: Tuple[int, int, int, int]) -> Optional[np.ndarray]:
    """BGR crop (x, y, w, h) out of a sensor_msgs/Image buffer without decoding the whole frame.
    Supports bgr8 / rgb8 / bgra8 / rgba8; anything else -> None."""
    ch = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}.get(encoding)
    if ch is None:
        return None
    x, y, w, h = (int(v) for v in box)
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(int(width), x + w), min(int(height), y + h)
    if x1 <= x0 or y1 <= y0:
        return None
    buf = np.frombuffer(data, dtype=np.uint8)
    if buf.size < int(step) * int(height):
        return None
    img = buf[: int(step) * int(height)].reshape(int(height), int(step))
    crop = img[y0:y1, x0 * ch: x1 * ch].reshape(y1 - y0, x1 - x0, ch)[..., :3]
    if encoding.startswith('rgb'):
        crop = crop[..., ::-1]
    return np.ascontiguousarray(crop)


def override_safe(clf_color: str, clf_conf: float, verdict: Optional[str],
                  keep_conf: float = 0.9, red_to_green: bool = False) -> str:
    """
    Safe override (2026-11-23).

    The colour check never creates a GREEN by default:
      classifier sure (conf >= keep_conf)       -> classifier (lamp 886: orange leaves at the
                                                   box edge read 'red' on a GREEN 0.97)
      classifier GREEN unsure + verdict 'red'   -> 'red'   (safe side: the robot waits)
      classifier RED unsure + verdict 'green'   -> 'none'  (a false red no longer cancels a green
                                                   phase; lamp 895 in the sun: RED 0.8 on green)
                                                   ('green' only with red_to_green=True)
      anything else                             -> classifier (UNKNOWN / NONE never changed)
    """
    if clf_color not in ('red', 'green') or verdict is None:
        return clf_color
    if clf_conf >= keep_conf:
        return clf_color
    if clf_color == 'green' and verdict == 'red':
        return 'red'
    if clf_color == 'red' and verdict == 'green':
        return 'green' if red_to_green else 'none'
    return clf_color


def override(clf_color: str, verdict: Optional[str]) -> str:
    """clf_color / result in {'red', 'green', 'none'}; only a red/green classifier result is
    overridden (a NONE / UNKNOWN classifier result stays NONE)."""
    if clf_color in ('red', 'green') and verdict is not None:
        return verdict
    return clf_color
