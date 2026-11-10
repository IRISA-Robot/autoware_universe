"""Lens distortion for projecting map points into the RAW (distorted) camera image.

The detector crops the raw image, so a light projected with the pinhole K alone lands where an
undistorted camera would see it: with the wide lenses here (k1 ~ -0.29) that is ~40 px off near
the image edge.  plumb_bob model (OpenCV / ROS camera_info.D = k1, k2, p1, p2, k3) applied to the
normalized coordinates x = X/Z, y = Y/Z:

    r2 = x^2 + y^2
    radial = 1 + k1 r2 + k2 r2^2 + k3 r2^3
    xd = x radial + 2 p1 x y + p2 (r2 + 2 x^2)
    yd = y radial + p1 (r2 + 2 y^2) + 2 p2 x y

The polynomial is only meaningful where r * radial(r) still grows with r; beyond that it folds
back and would put points far outside the field of view INTO the image.  Such points are
reported as not projectable (None).

Pure numpy, tested in test/test_lens.py.
"""

from typing import Optional, Sequence, Tuple

import numpy as np


class Distortion:
    """plumb_bob coefficients + the radius up to which the model is monotonic."""

    def __init__(self, d: Sequence[float]):
        c = list(d) + [0.0] * (5 - len(d))
        self.k1, self.k2, self.p1, self.p2, self.k3 = (float(v) for v in c[:5])
        self.r_max = self._monotonic_radius()

    @staticmethod
    def from_camera_info(model: str, d: Sequence[float]) -> Optional['Distortion']:
        """None when there is nothing to apply (no / unsupported model, all zeros)."""
        if model not in ('plumb_bob', '') or d is None or len(d) < 4:
            return None
        if all(abs(float(v)) < 1e-12 for v in d):
            return None
        return Distortion(d)

    def _radial(self, r2):
        return 1.0 + self.k1 * r2 + self.k2 * r2 * r2 + self.k3 * r2 * r2 * r2

    def _monotonic_radius(self, r_limit: float = 4.0, steps: int = 4000) -> float:
        r = np.linspace(0.0, r_limit, steps)
        rd = r * self._radial(r * r)
        dr = np.diff(rd)
        bad = np.nonzero(dr <= 0.0)[0]
        return float(r[bad[0]]) if len(bad) else r_limit

    def distort(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        r2 = x * x + y * y
        if r2 > self.r_max * self.r_max:
            return None
        radial = self._radial(r2)
        xd = x * radial + 2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x * x)
        yd = y * radial + self.p1 * (r2 + 2.0 * y * y) + 2.0 * self.p2 * x * y
        return xd, yd

    def distort_array(self, x: np.ndarray, y: np.ndarray):
        """Vectorised; returns (xd, yd, valid)."""
        r2 = x * x + y * y
        valid = r2 <= self.r_max * self.r_max
        radial = self._radial(r2)
        xd = x * radial + 2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x * x)
        yd = y * radial + self.p1 * (r2 + 2.0 * y * y) + 2.0 * self.p2 * x * y
        return xd, yd, valid


def project_camera_point(
    X: float, Y: float, Z: float, K: np.ndarray, dist: Optional[Distortion] = None
) -> Optional[Tuple[float, float]]:
    """Optical-frame point -> pixel (u, v); None behind the camera or outside the model."""
    if Z <= 0.0:
        return None
    x, y = X / Z, Y / Z
    if dist is not None:
        res = dist.distort(x, y)
        if res is None:
            return None
        x, y = res
    return K[0, 0] * x + K[0, 1] * y + K[0, 2], K[1, 1] * y + K[1, 2]
