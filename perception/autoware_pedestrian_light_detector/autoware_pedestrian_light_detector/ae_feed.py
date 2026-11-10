"""Link to the leopard_cam_driver software AE (pure logic, no ROS; tested in test/test_ae_feed.py).

Two directions:

* image: the classified camera is read from ``/camera/<cam>/<camera_image_suffix>``
  (default ``ae/image_raw``: the driver's AE-corrected copy, same size and stamp as image_raw).
  When that topic gives nothing for ``fallback_sec`` the node falls back to ``image_raw`` for
  that camera, keeps listening to the AE topic meanwhile, and goes back to it as soon as it
  delivers ``return_frames`` frames again (no gap longer than ``fallback_sec`` between them).
* ROI: the coloured padded box of the selected light (what the debug image draws) is sent to
  ``/camera/<cam>/ae/roi`` so the driver exposes for the light, not the whole scene.  Width 0 =
  "no target" (the driver drops back to its default ROI at once).
"""

from typing import Optional, Tuple


def image_topic(cam_name: str, suffix: str) -> str:
    return f'/camera/{cam_name}/{suffix.strip("/")}'


def bbox_to_roi(bbox, img_w: int, img_h: int) -> Optional[Tuple[int, int, int, int]]:
    """(x1, y1, x2, y2) -> (x, y, w, h) clamped to the image; None when empty/degenerate."""
    if bbox is None or img_w <= 0 or img_h <= 0:
        return None
    x1, y1, x2, y2 = (int(round(v)) for v in bbox)
    x1 = max(0, min(x1, img_w))
    x2 = max(0, min(x2, img_w))
    y1 = max(0, min(y1, img_h))
    y2 = max(0, min(y2, img_h))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    return x1, y1, x2 - x1, y2 - y1


class ImageSourceFallback:
    """Which image suffix the active camera uses, with a fallback after a silence.

    ``subscribed(t, cam)`` when the active subscription is (re)created, ``image(t)`` on every
    frame, ``check(t)`` once per tick -> True exactly when the node must switch to the fallback.
    While on the fallback the node also listens to the preferred topic: ``preferred_frame(t,
    cam)`` per frame there, and ``should_return(cam)`` -> True when it must switch back.
    """

    def __init__(self, preferred: str, fallback: str, fallback_sec: float,
                 return_frames: int = 3):
        self.preferred = preferred.strip('/')
        self.fallback = fallback.strip('/')
        self.fallback_sec = float(fallback_sec)
        self.return_frames = max(1, int(return_frames))
        self._fell_back = set()   # cameras on the fallback
        self._cam = ''
        self._since = None        # time of the last frame, or of the subscription
        self._back_count = {}     # cam -> consecutive preferred frames seen while on fallback
        self._back_last = {}      # cam -> time of the last of them

    def on_fallback(self, cam_name: str) -> bool:
        return cam_name in self._fell_back

    def suffix_for(self, cam_name: str) -> str:
        if cam_name in self._fell_back or self.preferred == self.fallback:
            return self.fallback
        return self.preferred

    def subscribed(self, t: float, cam_name: str):
        self._cam = cam_name
        self._since = t

    def image(self, t: float):
        self._since = t

    def check(self, t: float) -> bool:
        if not self._cam or self._since is None or self.fallback_sec <= 0.0:
            return False
        if self.suffix_for(self._cam) == self.fallback:
            return False
        if t - self._since < self.fallback_sec:
            return False
        self._fell_back.add(self._cam)
        self._back_count[self._cam] = 0
        return True

    def preferred_frame(self, t: float, cam_name: str):
        """A frame on the preferred topic while ``cam_name`` is on the fallback."""
        if cam_name not in self._fell_back:
            return
        last = self._back_last.get(cam_name)
        if last is None or t - last > self.fallback_sec:
            self._back_count[cam_name] = 0
        self._back_count[cam_name] = self._back_count.get(cam_name, 0) + 1
        self._back_last[cam_name] = t

    def should_return(self, cam_name: str) -> bool:
        """Preferred topic alive again -> leave the fallback (the caller resubscribes)."""
        if cam_name not in self._fell_back:
            return False
        if self._back_count.get(cam_name, 0) < self.return_frames:
            return False
        self._fell_back.discard(cam_name)
        self._back_count.pop(cam_name, None)
        self._back_last.pop(cam_name, None)
        return True

    def reset(self):
        self._fell_back.clear()
        self._back_count.clear()
        self._back_last.clear()
        self._cam = ''
        self._since = None
