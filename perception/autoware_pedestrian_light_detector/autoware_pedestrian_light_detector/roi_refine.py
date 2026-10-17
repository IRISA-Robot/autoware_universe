"""
ROI lampu untuk classifier: deteksi YOLOX full-frame yang jatuh di ROI peta, atau fallback peta.

Dipakai node (backend autoware, autoware_roi_refiner='fullframe') dan juga harness evaluasi
rosbag, supaya yang dievaluasi offline persis sama dengan yang jalan di robot.

Alur per frame:
  1. YOLOX full-frame (model car_ped_tl_detector, sama dengan tools/ped_tl_test) mendeteksi
     semua lampu di gambar.  Labelnya keluar UNKNOWN semua, jadi lampu mobil vs pejalan kaki
     tidak bisa dibedakan dari kelasnya.
  2. select_detection(): hanya deteksi yang SELURUHNYA di dalam ROI kasar dari peta ("AND" dengan
     mask peta), bentuknya tegak (lampu pejalan kaki: tinggi > lebar; lampu mobil horizontal) dan
     tingginya sebanding housing yang diharapkan.  Dari sisanya dipilih yang paling dekat ke
     titik proyeksi peta.
  3. DetectionHold: kalau frame ini miss, pakai bbox terakhir selama hold_sec (lampu statis,
     robot diam menunggu) asal masih di dalam ROI kasar sekarang.
  4. fallback_box(): kalau tetap tidak ada, kotak tinggi di titik proyeksi peta (menutup lampu
     merah atas + hijau bawah).  Evaluasi rosbag 2026-10-17: classifier 100% benar di kotak ini
     selama proyeksi meleset <= ~6 px.

Semua bbox di sini (x, y, w, h) dalam piksel; region/ROI kasar (x1, y1, x2, y2).
"""

from typing import List, Optional, Tuple

Box = Tuple[int, int, int, int]            # x, y, w, h
Region = Tuple[int, int, int, int]         # x1, y1, x2, y2
Detection = Tuple[int, int, int, int, float]  # x, y, w, h, score


def box_inside(box: Box, region: Region) -> bool:
    x, y, w, h = box
    return region[0] <= x and region[1] <= y and x + w <= region[2] and y + h <= region[3]


def select_detection(
    detections: List[Detection],
    rough: Region,
    center: Tuple[float, float],
    expect_h_px: float,
    min_aspect: float = 1.0,
    min_h_ratio: float = 0.3,
    max_h_ratio: float = 3.0,
) -> Optional[Box]:
    """Deteksi yang paling mungkin lampu pejalan kaki target, atau None.

    detections   : [(x, y, w, h, score)] dari YOLOX full-frame
    rough        : ROI kasar dari peta (x1, y1, x2, y2) — deteksi harus seluruhnya di dalamnya
    center       : titik proyeksi lampu dari peta (u, v)
    expect_h_px  : tinggi housing yang diharapkan [px] (dari ukuran fisik + jarak)
    min_aspect   : tinggi/lebar minimum (lampu pejalan kaki tegak; lampu mobil horizontal)
    min/max_h_ratio : rentang tinggi deteksi relatif expect_h_px (lampu menyala saja ~0.6,
                      housing penuh ~1.2)
    """
    best = None
    for x, y, w, h, score in detections:
        if w <= 0 or h <= 0:
            continue
        if not box_inside((x, y, w, h), rough):
            continue
        if h / float(w) < min_aspect:
            continue
        if expect_h_px > 0 and not (min_h_ratio * expect_h_px <= h <= max_h_ratio * expect_h_px):
            continue
        d2 = (x + w / 2.0 - center[0]) ** 2 + (y + h / 2.0 - center[1]) ** 2
        key = (d2, -score)
        if best is None or key < best[0]:
            best = (key, (int(x), int(y), int(w), int(h)))
    return None if best is None else best[1]


def fallback_box(
    center: Tuple[float, float],
    expect_wh_px: Tuple[float, float],
    img_wh: Tuple[int, int],
    scale_w: float = 2.0,
    scale_h: float = 2.0,
    min_w: int = 6,
    min_h: int = 16,
) -> Box:
    """Kotak di titik proyeksi peta, expect_wh x scale (tinggi menutup lampu merah + hijau)."""
    w = max(float(min_w), expect_wh_px[0] * scale_w)
    h = max(float(min_h), expect_wh_px[1] * scale_h)
    x1 = max(0, int(round(center[0] - w / 2.0)))
    y1 = max(0, int(round(center[1] - h / 2.0)))
    x2 = min(int(img_wh[0]), int(round(center[0] + w / 2.0)))
    y2 = min(int(img_wh[1]), int(round(center[1] + h / 2.0)))
    return (x1, y1, max(1, x2 - x1), max(1, y2 - y1))


class DetectionHold:
    """bbox deteksi terakhir, dipakai ulang selama hold_sec kalau YOLOX miss."""

    def __init__(self, hold_sec: float = 1.0):
        self.hold_sec = float(hold_sec)
        self.reset()

    def reset(self):
        self._box: Optional[Box] = None
        self._t: Optional[float] = None

    def update(self, box: Optional[Box], now_sec: float):
        if box is not None:
            self._box = box
            self._t = now_sec

    def get(self, now_sec: float, rough: Region) -> Optional[Box]:
        if self._box is None or self._t is None or self.hold_sec <= 0.0:
            return None
        if now_sec - self._t > self.hold_sec:
            return None
        return self._box if box_inside(self._box, rough) else None


def refine(
    detections: List[Detection],
    rough: Region,
    center: Tuple[float, float],
    expect_wh_px: Tuple[float, float],
    img_wh: Tuple[int, int],
    hold: DetectionHold,
    now_sec: float,
    min_aspect: float = 1.0,
    min_h_ratio: float = 0.3,
    max_h_ratio: float = 3.0,
    fallback_scale_w: float = 2.0,
    fallback_scale_h: float = 2.0,
) -> Tuple[Box, str]:
    """ROI untuk classifier + sumbernya: 'detection' | 'hold' | 'fallback'."""
    det = select_detection(detections, rough, center, expect_wh_px[1],
                           min_aspect, min_h_ratio, max_h_ratio)
    if det is not None:
        hold.update(det, now_sec)
        return det, 'detection'
    held = hold.get(now_sec, rough)
    if held is not None:
        return held, 'hold'
    return fallback_box(center, expect_wh_px, img_wh, fallback_scale_w, fallback_scale_h), 'fallback'
