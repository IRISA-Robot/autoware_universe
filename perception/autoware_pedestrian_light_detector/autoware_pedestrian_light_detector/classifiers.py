# Copyright 2024 azzamwildan462
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

"""
Fase 1 + Gating Geometris: Classifier interface + HSV color classifier + geometric gate.

Hierarki:
  Classifier          <- base interface
    HsvClassifier     <- klasifikasi merah/hijau lewat HSV mask + contour area

Gate Geometris (baru):
  geometric_gate(mask, tight_box_in_roi, roi_origin, geom_params)
      -> (accepted: bool, reason: str, centroid_in_roi: tuple|None)

  Memeriksa apakah blob warna terbesar:
  1. Centroid-nya berada di dalam tight_box (kotak ketat proyeksi lampu).
  2. Luas blob-nya tidak melebihi max_blob_fraction dari luas tight_box
     (bus mengisi seluruh kotak; lampu hanya mengisi sebagian kecil).
  3. Luas blob-nya memenuhi min_blob_fraction dari luas tight_box
     (ada objek berwarna yang signifikan di lokasi lampu).

  Jika gate dinonaktifkan (geom_gate_enabled=False) atau tight_box=None,
  fungsi selalu return accepted=True (perilaku lama dipertahankan).

Factory:
  make_classifier(backend, params) -> Classifier instance
"""

import cv2
import numpy as np

# State constants — sama persis dengan msg constants di node.
STATE_NONE = 0
STATE_RED = 1
STATE_GREEN = 2
STATE_COUNTDOWN_GREEN = 3
STATE_COUNTDOWN_BLANK = 4


# ---------------------------------------------------------------------------
# Geometric gate — filter blob berdasarkan posisi & ukuran relatif tight_box
# ---------------------------------------------------------------------------

def geometric_gate(
    mask: np.ndarray,
    tight_box_in_roi: tuple,
    geom_params: dict,
) -> tuple:
    """
    Validasi geometris blob warna terbesar terhadap kotak ketat proyeksi lampu.

    Gating mencegah objek berwarna (mis. bus hijau yang melintas) memicu deteksi
    palsu hanya karena sebagian warnanya masuk ke ROI padded yang longgar.
    Hanya blob yang centroid-nya berada di dalam tight_box DAN ukurannya sesuai
    ekspektasi lampu yang diterima — sisanya ditolak.

    Parameters
    ----------
    mask : np.ndarray (uint8, single-channel)
        Mask biner warna (output cv2.inRange) dalam koordinat ROI (bukan image).
    tight_box_in_roi : tuple (tx1_r, ty1_r, tx2_r, ty2_r)
        Kotak ketat proyeksi lampu dalam koordinat ROI lokal
        (sudah dikonversi oleh pemanggil: tight_box_image - roi_origin).
        Jika None atau tuple kosong: gate dilewati (return accepted=True).
    geom_params : dict
        Kunci yang dikenali:
        - geom_gate_enabled        (bool, default True)
        - geom_require_centroid    (bool, default True)
          Jika True, centroid blob terbesar HARUS di dalam tight_box.
        - geom_max_blob_fraction   (float, default 0.75)
          Fraksi maksimum luas tight_box yang boleh ditempati blob.
          Bus biasanya mengisi >> 0.75; lampu bulat ~0.1–0.4.
        - geom_min_blob_fraction   (float, default 0.02)
          Fraksi minimum. Di bawah ini dianggap noise.

    Returns
    -------
    (accepted, reason, centroid_in_roi)
        accepted        : bool — True = blob lolos gate.
        reason          : str  — penjelasan singkat keputusan (untuk debug image).
        centroid_in_roi : (cx, cy) dalam koordinat ROI, atau None jika tidak ada blob.
    """
    # Baca parameter
    enabled = bool(geom_params.get('geom_gate_enabled', True))
    require_centroid = bool(geom_params.get('geom_require_centroid', True))
    max_frac = float(geom_params.get('geom_max_blob_fraction', 0.75))
    min_frac = float(geom_params.get('geom_min_blob_fraction', 0.02))

    # Gate dinonaktifkan atau tight_box tidak tersedia → selalu lolos
    if not enabled or tight_box_in_roi is None:
        return True, 'gate-off', None

    tx1_r, ty1_r, tx2_r, ty2_r = tight_box_in_roi

    # Luas tight_box dalam piksel
    tight_w = max(1, tx2_r - tx1_r)
    tight_h = max(1, ty2_r - ty1_r)
    tight_area = float(tight_w * tight_h)

    # Temukan blob terbesar dari mask
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        # Tidak ada blob sama sekali — tidak ada gating yang perlu dilakukan;
        # classifier akan return STATE_NONE dengan area=0 < min_area.
        return True, 'no-blob', None

    # Pilih contour terbesar
    best_cnt = max(contours, key=cv2.contourArea)
    blob_area = cv2.contourArea(best_cnt)

    if blob_area < 1.0:
        return True, 'no-blob', None

    # Hitung centroid blob
    M = cv2.moments(best_cnt)
    if M['m00'] < 1e-6:
        return True, 'no-blob', None

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    centroid_in_roi = (cx, cy)

    # Fraksi luas blob relatif terhadap tight_box
    blob_frac = blob_area / tight_area

    # --- Cek fraksi minimum ---
    # Blob terlalu kecil relatif tight_box: mungkin noise di luar lampu
    # CATATAN: ini tidak menolak; classifier sudah punya min_area absolut.
    # Tapi kita informasikan lewat reason agar debug berguna.

    # --- Cek fraksi maksimum ---
    # Blob terlalu besar → bukan lampu (mis. bus, objek besar)
    if blob_frac > max_frac:
        return (
            False,
            f'blob-too-large({blob_frac:.2f}>{max_frac:.2f})',
            centroid_in_roi,
        )

    # --- Cek centroid di dalam tight_box ---
    if require_centroid:
        centroid_inside = (tx1_r <= cx < tx2_r) and (ty1_r <= cy < ty2_r)
        if not centroid_inside:
            return (
                False,
                f'centroid-outside({cx},{cy} not in [{tx1_r}-{tx2_r},{ty1_r}-{ty2_r}])',
                centroid_in_roi,
            )

    # Semua cek lolos
    return True, f'ok(frac={blob_frac:.2f})', centroid_in_roi


# ---------------------------------------------------------------------------
# Base interface
# ---------------------------------------------------------------------------

class Classifier:
    """
    Interface dasar classifier.

    Setiap subclass HARUS mengoverride classify().
    """

    def classify(self, bgr_roi: np.ndarray) -> tuple:
        """
        Klasifikasikan ROI frame BGR.

        Parameters
        ----------
        bgr_roi : np.ndarray
            Region-of-interest dalam format BGR (output cv2).

        Returns
        -------
        (state, confidence) : tuple[int, float]
            state   — salah satu STATE_* constant di atas.
            confidence — float [0.0, 1.0].
        """
        raise NotImplementedError


# ---------------------------------------------------------------------------
# HSV Classifier
# ---------------------------------------------------------------------------

class HsvClassifier(Classifier):
    """
    Klasifikasi warna lampu (merah vs hijau) menggunakan HSV masking.

    Logika:
    - Konversi BGR -> HSV.
    - Buat mask MERAH: dua rentang hue (hue rendah [0..red_hue_lo_max]
      + hue tinggi [red_hue_hi_min..179]) AND s >= s_min AND v >= v_min.
    - Buat mask HIJAU: satu rentang hue [green_hue_lo..green_hue_hi]
      AND s >= s_min AND v >= v_min.
    - Temukan blob terbesar (largest contour) per warna; syarat area >= min_area.
    - Warna dengan blob qualifying yang lebih besar -> output state.
    - Jika tidak ada yang qualify -> STATE_NONE.
    - confidence = area_blob_qualifying / total_area_roi (clamped 0..1).
    """

    def __init__(self, params: dict):
        """
        Parameters
        ----------
        params : dict
            Kunci yang dikenali (dengan default):
            - red_hue_lo_max  (int, default 10)  : batas atas hue merah rentang rendah
            - red_hue_hi_min  (int, default 170) : batas bawah hue merah rentang tinggi
            - green_hue_lo    (int, default 40)  : batas bawah hue hijau
            - green_hue_hi    (int, default 90)  : batas atas hue hijau
            - s_min           (int, default 80)  : saturasi minimum (0-255)
            - v_min           (int, default 80)  : value minimum (0-255)
            - min_area        (int, default 20)  : luas blob minimum (piksel^2)
        """
        self._red_hue_lo_max = int(params.get('red_hue_lo_max', 10))
        self._red_hue_hi_min = int(params.get('red_hue_hi_min', 170))
        self._green_hue_lo = int(params.get('green_hue_lo', 40))
        self._green_hue_hi = int(params.get('green_hue_hi', 90))
        self._s_min = int(params.get('s_min', 80))
        self._v_min = int(params.get('v_min', 80))
        self._min_area = int(params.get('min_area', 20))

    def classify(self, bgr_roi: np.ndarray) -> tuple:
        """
        Lihat docstring kelas untuk detail logika.

        Wrapper tipis yang memanggil classify_detailed() dan hanya mengembalikan
        (state, confidence) untuk kompatibilitas API lama.
        """
        state, confidence, _, _, _ = self.classify_detailed(bgr_roi)
        return state, confidence

    def classify_detailed(self, bgr_roi: np.ndarray) -> tuple:
        """
        Klasifikasi penuh dengan informasi intermediate untuk geometric gate.

        Returns
        -------
        (state, confidence, winning_mask, red_mask, green_mask)
            state         : int, STATE_* constant
            confidence    : float [0..1]
            winning_mask  : np.ndarray uint8 — mask warna pemenang (merah atau hijau),
                            atau array zeros jika STATE_NONE
            red_mask      : np.ndarray uint8 — mask merah mentah
            green_mask    : np.ndarray uint8 — mask hijau mentah
        """
        if bgr_roi is None or bgr_roi.size == 0:
            empty = np.zeros((1, 1), dtype=np.uint8)
            return STATE_NONE, 0.0, empty, empty, empty

        h_roi, w_roi = bgr_roi.shape[:2]
        roi_area = float(h_roi * w_roi)
        if roi_area < 1.0:
            empty = np.zeros((h_roi, w_roi), dtype=np.uint8)
            return STATE_NONE, 0.0, empty, empty, empty

        hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)

        s_min = self._s_min
        v_min = self._v_min

        # --- Mask MERAH ---
        # Merah di HSV wrap-around: hue 0..red_hue_lo_max  U  red_hue_hi_min..179
        mask_red_lo = cv2.inRange(
            hsv,
            np.array([0, s_min, v_min], dtype=np.uint8),
            np.array([self._red_hue_lo_max, 255, 255], dtype=np.uint8),
        )
        mask_red_hi = cv2.inRange(
            hsv,
            np.array([self._red_hue_hi_min, s_min, v_min], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        mask_red = cv2.bitwise_or(mask_red_lo, mask_red_hi)

        # --- Mask HIJAU ---
        mask_green = cv2.inRange(
            hsv,
            np.array([self._green_hue_lo, s_min, v_min], dtype=np.uint8),
            np.array([self._green_hue_hi, 255, 255], dtype=np.uint8),
        )

        red_area = self._largest_blob_area(mask_red)
        green_area = self._largest_blob_area(mask_green)

        red_qualifies = red_area >= self._min_area
        green_qualifies = green_area >= self._min_area

        zeros = np.zeros((h_roi, w_roi), dtype=np.uint8)

        if not red_qualifies and not green_qualifies:
            return STATE_NONE, 0.0, zeros, mask_red, mask_green

        if red_qualifies and (not green_qualifies or red_area >= green_area):
            winning_area = red_area
            state = STATE_RED
            winning_mask = mask_red
        else:
            winning_area = green_area
            state = STATE_GREEN
            winning_mask = mask_green

        confidence = min(1.0, winning_area / roi_area)
        return state, float(confidence), winning_mask, mask_red, mask_green

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _largest_blob_area(self, mask: np.ndarray) -> float:
        """
        Kembalikan luas contour terbesar dari mask biner.
        Return 0.0 jika tidak ada contour.
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return 0.0
        return float(max(cv2.contourArea(c) for c in contours))


# ---------------------------------------------------------------------------
# Autoware Classifier (cache holder — async result from external CNN node)
# ---------------------------------------------------------------------------

class AutowareClassifier(Classifier):
    """
    Cache holder untuk hasil klasifikasi dari Autoware traffic_light_classifier.

    Tidak melakukan inferensi in-process.  Node menjalankan loop:
      publish image + ROI  →  external classifier  →  subscribe TrafficLightArray
      →  update_result()   →  classify_cached()    →  publish PedestrianLightState

    Thread-safety: semua akses dari satu ROS executor thread — tidak perlu lock.
    """

    def __init__(self, params: dict):
        """
        Parameters
        ----------
        params : dict
            Kunci yang dikenali:
            - result_timeout_sec (float, default 1.0)
              Waktu maksimum (detik) cache dianggap valid sejak update terakhir.
              Jika melebihi batas ini, classify_cached() mengembalikan STATE_NONE.
        """
        self._result_timeout_sec = float(params.get('result_timeout_sec', 1.0))
        self._cached_state: int = STATE_NONE
        self._cached_conf: float = 0.0
        self._last_update_time: float = None  # monotonic seconds via node clock

    def update_result(self, state: int, confidence: float, now_sec: float):
        """
        Simpan state + confidence hasil dari subscription TrafficLightArray.

        Dipanggil oleh node dari callback _on_clf_signals setiap kali pesan
        baru diterima dari external classifier.

        Parameters
        ----------
        state      : int   — STATE_RED / STATE_GREEN / STATE_NONE
        confidence : float — nilai confidence dari TrafficLightElement
        now_sec    : float — waktu sekarang dalam detik (dari node clock)
        """
        self._cached_state = state
        self._cached_conf = confidence
        self._last_update_time = now_sec

    def classify_cached(self, now_sec: float) -> tuple:
        """
        Kembalikan cache dengan pengecekan staleness.

        Jika belum pernah ada update atau cache kadaluarsa (> result_timeout_sec),
        kembalikan (STATE_NONE, 0.0) sehingga FSM WAIT_GREEN counter tidak
        bertambah saat sinyal hilang.

        Parameters
        ----------
        now_sec : float — waktu sekarang dalam detik (dari node clock)

        Returns
        -------
        (state, confidence) : tuple[int, float]
        """
        if self._last_update_time is None:
            return STATE_NONE, 0.0
        age = now_sec - self._last_update_time
        if age > self._result_timeout_sec:
            return STATE_NONE, 0.0
        return self._cached_state, self._cached_conf

    def classify(self, bgr_roi) -> tuple:
        """
        Kembalikan raw cache tanpa cek staleness.

        Untuk kompatibilitas interface Classifier.  Untuk autoware backend,
        node HARUS memanggil classify_cached(now_sec) agar staleness dicek.
        """
        return self._cached_state, self._cached_conf

    def classify_detailed(self, bgr_roi) -> tuple:
        """
        Kembalikan raw cache — winning_mask=None karena tidak ada mask HSV.

        Node melewati geometric_gate untuk backend autoware (gate hanya berlaku
        untuk HSV path).  Nilai None pada winning_mask aman karena gate
        tidak dipanggil sama sekali.
        """
        return self._cached_state, self._cached_conf, None, None, None


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_classifier(backend: str, params: dict) -> Classifier:
    """
    Factory function: buat Classifier berdasarkan nama backend.

    Parameters
    ----------
    backend : str
        'hsv'      -> HsvClassifier (Fase 1)
        'autoware' -> AutowareClassifier (delegate ke external CNN node)
        'lytnet'   -> belum tersedia (Fase 3), raise NotImplementedError
        'force'    -> bukan classifier; ditangani di node, jangan panggil factory ini.

    params : dict
        Parameter diteruskan ke constructor classifier.

    Returns
    -------
    Classifier instance.
    """
    if backend == 'hsv':
        return HsvClassifier(params)
    elif backend == 'autoware':
        return AutowareClassifier(params)
    elif backend == 'lytnet':
        raise NotImplementedError(
            "LytNet classifier belum diimplementasikan (Fase 3). "
            "Gunakan backend='hsv' untuk Fase 1."
        )
    else:
        raise ValueError(
            f"Backend classifier tidak dikenal: '{backend}'. "
            "Pilihan yang valid: 'hsv', 'autoware', 'force'."
        )
