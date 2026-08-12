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
Fase 2: Map-based ROI projector untuk pedestrian-light detector.

Modul ini melakukan:
1. Parsing OSM lanelet map untuk menemukan lampu pejalan kaki
   (way dengan tag tf_pedestrian="true").
2. Proyeksi titik 3D (frame map) ke piksel kamera via tf2 + pinhole projection.
3. Seleksi kamera yang paling dekat/relevan dari beberapa kamera.
4. (Neu) Kalkulasi kotak ketat (tight box) dari ukuran fisik lampu — basis
   gating geometris untuk menolak objek hijau/merah yang bukan lampu.

Tidak ada dependency lanelet2 Python binding — hanya xml.etree.

Public API:
-----------
parse_tf_pedestrian_lights(osm_path)   -> list[Light]
project_point(p_map, K, cam_frame, tf_buffer, stamp) -> (u,v,depth)|None
light_roi_in_camera(light, K, cam_frame, w, h, tf_buffer, margin_px,
                    light_radius_m, tight_box_height_scale)
    -> (padded_bbox, tight_bbox) | (None, None)
    padded_bbox: (x1,y1,x2,y2) untuk crop ROI ke classifier
    tight_bbox:  (x1,y1,x2,y2) kotak ketat proyeksi extent lampu fisik
                 (digunakan geometric gate — relatif ke koordinat IMAGE, bukan ROI)
select(lights, cameras_state, tf_buffer, margin_px, light_radius_m,
       tight_box_height_scale)
    -> (cam_name, padded_bbox, tight_bbox, light) | None

Light namedtuple:
  .points    list[(x,y,z)]  — koordinat node dalam frame map
  .midpoint  (x,y,z)        — rata-rata semua points
  .height_m  float|None     — tinggi fisik fixture dalam meter dari tag OSM 'height';
                               None jika tag tidak ada (gunakan tight_box_height_scale)
"""

import xml.etree.ElementTree as ET
from typing import Dict, List, NamedTuple, Optional, Tuple

import numpy as np

# tf2 imports — digunakan untuk lookup transform map->camera
try:
    import tf2_ros
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

class Light(NamedTuple):
    """Representasi satu pedestrian traffic light dari peta."""
    way_id: str                         # ID way OSM, untuk identifikasi
    points: List[Tuple[float, float, float]]  # daftar titik (x,y,z) map frame
    midpoint: Tuple[float, float, float]      # centroid semua titik
    height_m: Optional[float] = None    # tinggi fisik fixture (meter) dari tag OSM 'height';
                                        # None jika tag tidak ada — pakai tight_box_height_scale


class CameraState(NamedTuple):
    """State satu kamera: intrinsic K + frame_id + resolusi."""
    name: str                 # nama kamera, mis. 'front'
    K: np.ndarray             # 3x3 intrinsic matrix
    frame_id: str             # optical frame dari camera_info.header.frame_id
    width: int                # lebar image (piksel)
    height: int               # tinggi image (piksel)


# ---------------------------------------------------------------------------
# OSM parser
# ---------------------------------------------------------------------------

def parse_tf_pedestrian_lights(osm_path: str) -> List[Light]:
    """
    Parse file OSM lanelet2 dan kembalikan semua pedestrian traffic light.

    Way yang dianggap lampu pejalan kaki: memiliki tag tf_pedestrian="true".
    Koordinat node diambil dari tag local_x/local_y/ele (koordinat map frame).

    Parameters
    ----------
    osm_path : str
        Path absolut ke file .osm.

    Returns
    -------
    list[Light]
        List light yang ditemukan. Kosong jika tidak ada.
    """
    try:
        tree = ET.parse(osm_path)
    except Exception as e:
        raise IOError(f'Gagal parse OSM: {osm_path}: {e}')

    root = tree.getroot()

    # Bangun dict node_id -> (x, y, z) dari semua node yang punya local_x/ele
    node_coords: Dict[str, Tuple[float, float, float]] = {}
    for nd in root.findall('node'):
        nid = nd.get('id', '')
        tags = {t.get('k', ''): t.get('v', '') for t in nd.findall('tag')}
        if 'local_x' in tags and 'local_y' in tags:
            try:
                x = float(tags['local_x'])
                y = float(tags['local_y'])
                z = float(tags.get('ele', '0.0'))
                node_coords[nid] = (x, y, z)
            except ValueError:
                pass

    # Cari way dengan tf_pedestrian="true"
    lights: List[Light] = []
    for way in root.findall('way'):
        wid = way.get('id', '')
        tags = {t.get('k', ''): t.get('v', '') for t in way.findall('tag')}

        if tags.get('tf_pedestrian') != 'true':
            continue

        # Kumpulkan koordinat node-node dari way ini
        points: List[Tuple[float, float, float]] = []
        for nd_ref in way.findall('nd'):
            ref = nd_ref.get('ref', '')
            if ref in node_coords:
                points.append(node_coords[ref])

        if not points:
            # Way tidak memiliki node dengan koordinat map — lewati
            continue

        # Hitung midpoint (centroid)
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        mid = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))

        # Baca tag 'height' opsional (meter) — tinggi fisik fixture lampu pejalan kaki.
        # Dipakai oleh light_roi_in_camera untuk membangun tight_bbox yang lebih tinggi
        # agar mencakup lampu merah (atas) dan hijau (bawah) sekaligus.
        # Jika tag tidak ada (mis. way 2261), height_m=None → pakai tight_box_height_scale.
        height_m: Optional[float] = None
        if 'height' in tags:
            try:
                h_val = float(tags['height'])
                if h_val > 0.0:
                    height_m = h_val
            except ValueError:
                pass

        lights.append(Light(way_id=wid, points=points, midpoint=mid, height_m=height_m))

    return lights


# ---------------------------------------------------------------------------
# Projection helpers
# ---------------------------------------------------------------------------

def project_point(
    p_map: Tuple[float, float, float],
    K: np.ndarray,
    cam_frame: str,
    tf_buffer,
    stamp,
) -> Optional[Tuple[float, float, float]]:
    """
    Proyeksikan titik 3D dalam frame map ke piksel kamera.

    Langkah:
      1. tf2 lookup: transform dari frame 'map' ke cam_frame (optical frame).
      2. Transformasikan titik p_map ke koordinat kamera.
      3. Tolak jika Z <= 0 (di belakang kamera).
      4. Proyeksi pinhole: u = fx*X/Z+cx, v = fy*Y/Z+cy.

    Parameters
    ----------
    p_map : (x, y, z)
        Koordinat titik dalam frame 'map'.
    K : np.ndarray shape (3,3)
        Matriks intrinsic kamera (row-major dari camera_info.k).
    cam_frame : str
        Frame ID kamera optik (dari camera_info.header.frame_id).
    tf_buffer : tf2_ros.Buffer
        Buffer tf2 yang sudah berjalan.
    stamp : builtin_interfaces.msg.Time | rclpy.time.Time
        Timestamp untuk lookup (gunakan Time(seconds=0) untuk lookup terbaru).

    Returns
    -------
    (u, v, depth) atau None
        None jika tf gagal atau titik di belakang kamera.
    """
    if not _TF2_AVAILABLE or tf_buffer is None:
        return None

    try:
        # Gunakan rclpy Time(0) = ambil transform terbaru yang tersedia
        from rclpy.time import Time as RclpyTime
        t = tf_buffer.lookup_transform(
            cam_frame,
            'map',
            RclpyTime(),
        )
    except Exception:
        # LookupException, ExtrapolationException, ConnectivityException
        return None

    # Ambil rotasi (quaternion) dan translasi dari TransformStamped
    tr = t.transform.translation
    ro = t.transform.rotation

    tx, ty, tz = tr.x, tr.y, tr.z
    qx, qy, qz, qw = ro.x, ro.y, ro.z, ro.w

    # Konversi quaternion ke rotation matrix 3x3
    R = _quat_to_matrix(qx, qy, qz, qw)

    # Titik dalam frame map
    p = np.array([p_map[0], p_map[1], p_map[2]], dtype=np.float64)

    # Transform ke frame kamera: p_cam = R @ p + t
    p_cam = R @ p + np.array([tx, ty, tz], dtype=np.float64)

    X, Y, Z = p_cam[0], p_cam[1], p_cam[2]

    # Tolak jika di belakang kamera (Z <= 0)
    if Z <= 0.0:
        return None

    # Proyeksi pinhole
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    u = fx * X / Z + cx
    v = fy * Y / Z + cy

    return (u, v, Z)


def light_roi_in_camera(
    light: Light,
    K: np.ndarray,
    cam_frame: str,
    w: int,
    h: int,
    tf_buffer,
    margin_px: int = 40,
    light_radius_m: float = 0.15,
    tight_box_height_scale: float = 2.0,
) -> Tuple[Optional[Tuple[int, int, int, int]], Optional[Tuple[int, int, int, int]]]:
    """
    Hitung bounding box piksel dari satu light dalam satu kamera.

    Mengembalikan DUA bbox:
      1. padded_bbox  — bbox longgar (node-to-node + margin_px) untuk crop ROI.
      2. tight_bbox   — bbox ketat proyeksi extent fisik lampu
                        (endpoint-to-endpoint ± light_radius_m di semua sumbu).
                        Tinggi tight_bbox diperluas agar mencakup KEDUA posisi lampu
                        (merah atas + hijau bawah) pada fixture 2-lamp:
                          - Jika light.height_m diset (dari tag OSM 'height'):
                              v_half_px = fy * (light.height_m / 2.0) / depth
                              → box tingginya = ukuran fisik fixture sebenarnya.
                          - Jika light.height_m tidak ada (None):
                              tinggi tight_box dikalikan tight_box_height_scale
                              (default 2.0) dari extent node-to-node + radius.
                        Lebar horizontal tidak berubah.
                        Ini adalah SATU-SATUNYA region yang boleh mengandung warna
                        lampu; dipakai oleh geometric gate di classifier.

    Jika lampu hanya satu titik (midpoint saja, tidak ada endpoint yang valid),
    tight_bbox dibuat dari radius lampu saja di sekitar midpoint.

    Parameters
    ----------
    light : Light
    K : np.ndarray (3,3)
    cam_frame : str
    w, h : int  — lebar/tinggi image dalam piksel
    tf_buffer  — tf2_ros.Buffer
    margin_px  — margin perluasan padded_bbox dalam piksel (lihat param roi_margin_px)
    light_radius_m — radius fisik lampu dalam meter (default 0.15 m = 15 cm).
                     Dipakai untuk mengembangkan tight_bbox horizontal dari proyeksi.
    tight_box_height_scale — skala pengali tinggi tight_bbox saat light.height_m=None.
                     Default 2.0 agar kotak 2x lebih tinggi dari extent node-to-node,
                     sehingga mencakup lampu merah (atas) dan hijau (bawah).
                     Diabaikan jika light.height_m diset (tag OSM digunakan).

    Returns
    -------
    (padded_bbox, tight_bbox)
        padded_bbox : (x1, y1, x2, y2) dalam piksel, atau None jika di luar FOV.
        tight_bbox  : (x1, y1, x2, y2) dalam piksel (koordinat IMAGE, bukan ROI),
                      atau None jika proyeksi gagal.
    Keduanya None jika tidak ada titik yang terproyeksikan ke dalam FOV.
    """
    in_fov_us: List[float] = []
    in_fov_vs: List[float] = []
    # Kumpulkan semua depth untuk mengestimasi radius_px dari light_radius_m
    in_fov_depths: List[float] = []

    for pt in light.points:
        result = project_point(pt, K, cam_frame, tf_buffer, stamp=None)
        if result is None:
            continue
        u, v, depth = result
        in_fov_us.append(u)
        in_fov_vs.append(v)
        in_fov_depths.append(depth)

    if not in_fov_us:
        return None, None

    # Cek apakah minimal satu titik benar-benar dalam frame
    any_in_frame = any(
        0.0 <= u < w and 0.0 <= v < h
        for u, v in zip(in_fov_us, in_fov_vs)
    )
    if not any_in_frame:
        return None, None

    # -----------------------------------------------------------------------
    # Tight bbox: extent fisik lampu yang diproyeksikan ke piksel
    #
    # Estimasi radius_px dari depth rata-rata dan focal length:
    #   radius_px ≈ fx * light_radius_m / depth_avg
    # Pakai fx (biasanya ≈ fy) sebagai aproksimasi isotropik untuk dimensi horizontal.
    #
    # Untuk dimensi VERTIKAL, dua jalur (prioritas: height tag > scale):
    #   A) light.height_m tidak None (dari tag OSM 'height'):
    #      v_half_px = fy * (light.height_m / 2.0) / depth
    #      → kotak vertikal = tinggi fisik fixture sebenarnya, berpusat di v_mid.
    #   B) light.height_m adalah None (tag tidak ada — kasus saat ini):
    #      hitung tight_bbox vertikal dari node-to-node + radius_px seperti biasa,
    #      lalu kalikan setengah-tingginya dengan tight_box_height_scale (default 2.0).
    #      → kotak 2x lebih tinggi dari v_mid, nutupin merah atas + hijau bawah.
    # -----------------------------------------------------------------------
    u_min = min(in_fov_us)
    u_max = max(in_fov_us)
    v_min = min(in_fov_vs)
    v_max = max(in_fov_vs)

    avg_depth = sum(in_fov_depths) / len(in_fov_depths) if in_fov_depths else 1.0
    avg_depth = max(avg_depth, 0.1)   # hindari bagi-nol

    fx = float(K[0, 0])
    fy = float(K[1, 1])

    # radius_px minimal 3 piksel agar tight_bbox tidak degenerasi (dimensi horizontal)
    radius_px = max(3, int(round(fx * light_radius_m / avg_depth)))

    # Horizontal extent: node-to-node + radius_px (tidak berubah)
    tu_min = u_min - radius_px
    tu_max = u_max + radius_px

    # Midpoint v untuk penentuan tinggi kotak yang berpusat
    v_mid = (v_min + v_max) / 2.0

    if light.height_m is not None and light.height_m > 0.0:
        # Jalur A: gunakan tinggi fisik fixture dari tag OSM 'height'
        # v_half_px = proyeksi setengah tinggi fixture ke piksel via fy
        v_half_px = fy * (light.height_m / 2.0) / avg_depth
        v_half_px = max(radius_px, v_half_px)   # minimal radius_px agar tidak degenerasi
        tv_min = v_mid - v_half_px
        tv_max = v_mid + v_half_px
    else:
        # Jalur B: scale dari extent node-to-node + radius_px
        # Hitung setengah-tinggi awal dari extent node → lalu kalikan scale
        v_half_base = (v_max - v_min) / 2.0 + radius_px
        v_half_scaled = v_half_base * max(1.0, tight_box_height_scale)
        tv_min = v_mid - v_half_scaled
        tv_max = v_mid + v_half_scaled

    # -----------------------------------------------------------------------
    # Padded bbox: enclosing semua proyeksi + margin_px, diperluas secara vertikal
    # agar mencakup tight_bbox yang lebih tinggi (kedua posisi lampu masuk crop).
    # Lebar horizontal: node-to-node + margin_px (tidak berubah).
    # -----------------------------------------------------------------------
    px1 = int(u_min) - margin_px
    px2 = int(u_max) + margin_px
    # Vertikal: gunakan tv_min/tv_max yang sudah diperluas, lalu tambah margin_px
    py1 = int(tv_min) - margin_px
    py2 = int(tv_max) + margin_px

    px1 = max(0, min(px1, w - 1))
    py1 = max(0, min(py1, h - 1))
    px2 = max(px1 + 1, min(px2, w))
    py2 = max(py1 + 1, min(py2, h))

    padded_bbox = (px1, py1, px2, py2)

    # Clamp tight_bbox ke dalam padded_bbox (tidak perlu lebih lebar dari padded)
    tx1 = max(px1, int(tu_min))
    ty1 = max(py1, int(tv_min))
    tx2 = min(px2, int(tu_max) + 1)
    ty2 = min(py2, int(tv_max) + 1)

    # Pastikan tight_bbox punya ukuran minimal
    tx2 = max(tx1 + 1, tx2)
    ty2 = max(ty1 + 1, ty2)

    tight_bbox = (tx1, ty1, tx2, ty2)

    return padded_bbox, tight_bbox


# ---------------------------------------------------------------------------
# Camera selector
# ---------------------------------------------------------------------------

def select(
    lights: List[Light],
    cameras_state: List[CameraState],
    tf_buffer,
    margin_px: int = 40,
    light_radius_m: float = 0.15,
    tight_box_height_scale: float = 2.0,
) -> Optional[Tuple[str, Tuple[int, int, int, int], Optional[Tuple[int, int, int, int]], Light]]:
    """
    Pilih kamera + light terbaik dari semua kombinasi yang terlihat.

    Untuk setiap (kamera, light) yang menghasilkan bbox valid:
    - Hitung kedalaman midpoint light ke kamera.
    - Pilih yang memiliki kedalaman terkecil (paling dekat, paling relevan).

    Parameters
    ----------
    lights : list[Light]
    cameras_state : list[CameraState]  — hanya kamera yang sudah punya camera_info
    tf_buffer
    margin_px : int
    light_radius_m : float  — radius fisik lampu (meter), diteruskan ke tight_bbox
    tight_box_height_scale : float  — pengali tinggi tight_bbox saat tag OSM 'height'
                             tidak ada; diteruskan ke light_roi_in_camera.

    Returns
    -------
    (cam_name, padded_bbox, tight_bbox, light) atau None jika tidak ada yang terlihat.
    padded_bbox : bbox longgar untuk crop ROI (dipakai classifier).
    tight_bbox  : bbox ketat proyeksi lampu (dipakai geometric gate); bisa None.
    """
    best = None          # (depth, cam_name, padded_bbox, tight_bbox, light)

    for cam in cameras_state:
        for light in lights:
            padded_bbox, tight_bbox = light_roi_in_camera(
                light, cam.K, cam.frame_id,
                cam.width, cam.height,
                tf_buffer, margin_px, light_radius_m,
                tight_box_height_scale,
            )
            if padded_bbox is None:
                continue

            # Kedalaman midpoint untuk ranking
            mid_result = project_point(
                light.midpoint, cam.K, cam.frame_id, tf_buffer, stamp=None
            )
            if mid_result is None:
                # Midpoint tidak terproyeksikan, gunakan kedalaman besar sebagai
                # fallback agar tidak diprioritaskan
                depth = float('inf')
            else:
                _, _, depth = mid_result

            if depth <= 0.0:
                continue

            if best is None or depth < best[0]:
                best = (depth, cam.name, padded_bbox, tight_bbox, light)

    if best is None:
        return None

    _, cam_name, padded_bbox, tight_bbox, light = best
    return (cam_name, padded_bbox, tight_bbox, light)


# ---------------------------------------------------------------------------
# Internal math helpers
# ---------------------------------------------------------------------------

def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    Konversi quaternion (x,y,z,w) ke rotation matrix 3x3.

    Implementasi manual agar tidak bergantung pada scipy/transforms3d.
    Referensi: https://www.euclideanspace.com/maths/geometry/rotations/conversions/
    """
    # Normalisasi untuk numerik stabil
    norm = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5
    if norm < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    R = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)
    return R
