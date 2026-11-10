"""Lidar check of a pedestrian-light detection (pure numpy, tested in test/test_lidar_check.py).

Question: is what the camera sees inside the classified box really the light from the map, or
something closer to the robot (a person in a green shirt standing in front of the camera)?

1. Lidar points (obstacle_segmentation cloud, ground removed) are projected into the camera;
   keep those inside the box the classifier gets (YOLOX bbox / hold / map fallback box).
2. Drop points BEHIND the light (camera depth > light depth + behind_margin_m): the background
   behind a light (buildings, trees) is not what the camera sees as the light, and a light that
   the lidar does not hit at all (thin housing on a pole) would otherwise be judged by its
   background alone.
3. Fewer than ``min_points`` left: no evidence either way -> accept (the lidar cannot judge).
4. Otherwise: SMALLEST horizontal (x, y) distance of those points, in the map frame, to the
   light's linestring.  > ``max_dist_m`` -> reject: everything in front of the camera in that box
   is far from the light, i.e. the box shows something else.
"""

from typing import NamedTuple, Optional, Sequence, Tuple

import numpy as np


class Verdict(NamedTuple):
    ok: bool
    reason: str               # 'no_cloud' | 'few_points' | 'near_light' | 'far_from_light'
    n_in_box: int             # cloud points inside the box (any depth)
    n_front: int              # ... not behind the light
    min_dist_m: Optional[float]


def transform_matrix(R: np.ndarray, t: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def min_xy_dist_to_polyline(pts_xy: np.ndarray, line_xy: np.ndarray) -> float:
    """Smallest distance from any of ``pts_xy`` (N,2) to the polyline ``line_xy`` (P,2)."""
    if len(pts_xy) == 0 or len(line_xy) == 0:
        return float('inf')
    if len(line_xy) == 1:
        return float(np.min(np.linalg.norm(pts_xy - line_xy[0], axis=1)))
    best = float('inf')
    for a, b in zip(line_xy[:-1], line_xy[1:]):
        ab = b - a
        L2 = float(ab @ ab)
        if L2 < 1e-12:
            d = np.linalg.norm(pts_xy - a, axis=1)
        else:
            u = np.clip(((pts_xy - a) @ ab) / L2, 0.0, 1.0)
            d = np.linalg.norm(pts_xy - (a + u[:, None] * ab), axis=1)
        best = min(best, float(np.min(d)))
    return best


def check(
    pts_src: Optional[np.ndarray],
    T_cam_src: np.ndarray,
    T_map_src: np.ndarray,
    K: np.ndarray,
    box: Tuple[int, int, int, int],
    light_line_map: np.ndarray,
    light_depth_m: float,
    max_dist_m: float = 5.0,
    behind_margin_m: float = 1.0,
    min_points: int = 3,
    min_depth_m: float = 0.3,
    dist=None,
) -> Verdict:
    """See the module docstring.  box = (x, y, w, h) in image pixels; pts_src (N,3) in the
    cloud frame; T_* map cloud-frame points into the camera optical frame / map."""
    if pts_src is None or len(pts_src) == 0:
        return Verdict(True, 'no_cloud', 0, 0, None)
    pc = apply(T_cam_src, pts_src)
    z = pc[:, 2]
    valid = z > min_depth_m
    if not np.any(valid):
        return Verdict(True, 'few_points', 0, 0, None)
    pc = pc[valid]
    src = pts_src[valid]
    xn = pc[:, 0] / pc[:, 2]
    yn = pc[:, 1] / pc[:, 2]
    if dist is not None:
        # same lens model as the ROI (lens.py): the box is in the raw, distorted image
        xn, yn, ok = dist.distort_array(xn, yn)
        pc, src, xn, yn = pc[ok], src[ok], xn[ok], yn[ok]
    u = K[0, 0] * xn + K[0, 2]
    v = K[1, 1] * yn + K[1, 2]
    x, y, w, h = box
    inside = (u >= x) & (u < x + w) & (v >= y) & (v < y + h)
    n_in_box = int(np.count_nonzero(inside))
    front = inside & (pc[:, 2] <= light_depth_m + behind_margin_m)
    n_front = int(np.count_nonzero(front))
    if n_front < max(1, int(min_points)):
        return Verdict(True, 'few_points', n_in_box, n_front, None)
    pm = apply(T_map_src, src[front])
    d = min_xy_dist_to_polyline(pm[:, :2], np.asarray(light_line_map, dtype=float)[:, :2])
    if d > max_dist_m:
        return Verdict(False, 'far_from_light', n_in_box, n_front, d)
    return Verdict(True, 'near_light', n_in_box, n_front, d)
