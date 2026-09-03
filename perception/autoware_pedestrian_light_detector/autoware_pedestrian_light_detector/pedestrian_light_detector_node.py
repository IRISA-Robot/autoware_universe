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
Fase 2 pedestrian-light detector — map-based ROI projector + multi-camera.

Fase 1 paths (roi_source=full|static) tidak berubah.
Fase 2 menambah roi_source=map yang:
  - Mem-parse pedestrian lights dari lanelet2 OSM map (tf_pedestrian=true).
  - Men-subscribe camera_info semua kamera (ringan, lazily saat enable).
  - Memilih kamera + ROI via proyeksi tf2: kamera terdekat yang melihat lampu.
  - Hanya men-subscribe image_raw SATU kamera (yang terpilih) untuk hemat BW.
  - Menggambar bbox proyeksi pada debug image.

debug_all_cameras — rate-limited path (debug_rate_hz, default 2 Hz):
  - Callback image_raw HANYA menyimpan raw Image msg (TANPA cv_bridge decode).
  - Decode + draw + publish dilakukan oleh timer lambat (_debug_timer) pada
    debug_rate_hz. Ini membatasi kerja berat ke debug_rate_hz × num_cameras
    (mis. 2 × 4 = 8 decode/s) alih-alih camera_rate × num_cameras.
  - Frame yang stamp-nya belum berubah sejak terakhir di-publish dilewati.

Publishes:
  /perception/road_crossing/pedestrian_light  (PedestrianLightState)
  ~/debug/image                               (Image, opsional — alias kamera aktif)
  ~/debug/<camera>/image                      (Image per kamera, jika debug_all_cameras=true)

Service:
  /perception/road_crossing/pedestrian_light_detector/enable (SetBool)
"""

import collections

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool

from autoware_road_crossing_msgs.msg import PedestrianLightState

try:
    from tier4_perception_msgs.msg import (
        TrafficLightRoiArray,
        TrafficLightRoi,
        TrafficLightArray,
    )
    _TIER4_MSGS_AVAILABLE = True
except ImportError:
    _TIER4_MSGS_AVAILABLE = False

try:
    from cv_bridge import CvBridge, CvBridgeError
    _CV_BRIDGE_AVAILABLE = True
except ImportError:
    _CV_BRIDGE_AVAILABLE = False

try:
    import tf2_ros
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False

from autoware_pedestrian_light_detector.classifiers import make_classifier, geometric_gate
from autoware_pedestrian_light_detector.roi_projector import (
    parse_tf_pedestrian_lights,
    project_point,
    light_roi_in_camera,
    select as projector_select,
    CameraState,
)

# State constants — sama persis dengan msg constants.
STATE_NONE = 0
STATE_RED = 1
STATE_GREEN = 2
STATE_COUNTDOWN_GREEN = 3
STATE_COUNTDOWN_BLANK = 4

_STATE_NAMES = {
    STATE_NONE: 'NONE',
    STATE_RED: 'RED',
    STATE_GREEN: 'GREEN',
    STATE_COUNTDOWN_GREEN: 'COUNTDOWN_GREEN',
    STATE_COUNTDOWN_BLANK: 'COUNTDOWN_BLANK',
}


def _state_color(state: int) -> tuple:
    """
    Return BGR border color that reflects the committed detection state.

    GREEN(2) / COUNTDOWN_GREEN(3) -> green  (0, 255, 0)
    RED(1)                         -> red    (0, 0, 255)
    NONE(0) / COUNTDOWN_BLANK(4) / unknown -> blue (255, 0, 0)
    """
    if state in (STATE_GREEN, STATE_COUNTDOWN_GREEN):
        return (0, 255, 0)
    if state == STATE_RED:
        return (0, 0, 255)
    return (255, 0, 0)

# QoS sensor data: best-effort, volatile, depth 1 — cocok untuk image stream.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# QoS untuk camera_info: reliable, transient local depth 1
# (camera_info biasanya publish sekali atau jarang berubah)
_INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class PedestrianLightDetectorNode(Node):
    """Pedestrian-light detector — Fase 1 HSV + Fase 2 map-based ROI projector."""

    def __init__(self):
        super().__init__('pedestrian_light_detector')

        # ---------- parameters: legacy (tetap ada, untuk mode 'force') ----------
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('force_state', STATE_GREEN)
        self.declare_parameter('force_confidence', 0.9)
        self.declare_parameter('publish_rate_hz', 10.0)

        # ---------- parameters: Fase 1 ----------
        self.declare_parameter('backend', 'hsv')
        self.declare_parameter('camera_topic', '/camera/front/image_raw')
        self.declare_parameter('roi_source', 'map')
        # static_roi: [x, y, w, h] — fraksi (<=1.0) atau piksel (>1.0)
        self.declare_parameter('static_roi', [0.25, 0.0, 0.5, 0.5])
        # HSV params
        self.declare_parameter('red_hue_lo_max', 10)
        self.declare_parameter('red_hue_hi_min', 170)
        self.declare_parameter('green_hue_lo', 40)
        self.declare_parameter('green_hue_hi', 90)
        self.declare_parameter('s_min', 80)
        self.declare_parameter('v_min', 80)
        self.declare_parameter('min_area', 20)
        # Anti-flicker
        self.declare_parameter('stable_frames', 3)
        self.declare_parameter('min_confidence', 0.0)
        # Debug image
        self.declare_parameter('enable_debug_image', True)

        # ---------- parameters: Fase 2 (map-based ROI) ----------
        # map_path: path ke file lanelet2 OSM
        # Default kosong: path sebenarnya di-pass dari launch (map_path global).
        # Jangan hardcode path mesin lain - bikin map mode diam-diam mati.
        self.declare_parameter('map_path', '')
        # cameras: daftar nama kamera yang dimonitor
        self.declare_parameter('cameras', ['front', 'left', 'rear', 'right'])
        # roi_margin_px: margin piksel perluasan bbox proyeksi (padded bbox)
        self.declare_parameter('roi_margin_px', 40)

        # ---------- parameters: Geometric gate (anti-false-positive) ----------
        # light_radius_m: radius fisik muka lampu pejalan kaki dalam meter.
        # Dipakai untuk menghitung tight_bbox (kotak ketat proyeksi lampu) dari depth.
        # Lampu pejalan kaki khas di Indonesia: ~0.15–0.20 m.
        # Turunkan jika tight_bbox terlalu lebar (lampu kecil); naikkan jika terpotong.
        self.declare_parameter('light_radius_m', 0.15)

        # geom_gate_enabled: aktifkan/matikan geometric gate.
        # True (default) = blob warna HARUS lolos validasi geometris untuk diterima.
        # False = perilaku lama (tanpa gate) — hanya untuk fallback/debug.
        self.declare_parameter('geom_gate_enabled', True)

        # geom_require_centroid: jika True, centroid blob terbesar HARUS berada
        # di dalam tight_bbox. Bus melintas punya centroid jauh dari titik lampu.
        # False = hanya cek fraksi ukuran (lebih longgar).
        self.declare_parameter('geom_require_centroid', True)

        # geom_max_blob_fraction: fraksi maksimum luas tight_bbox yang boleh
        # ditempati blob (relatif terhadap luas tight_bbox).
        # Bus/kendaraan hijau mengisi > 0.75 dari kotak → ditolak.
        # Lampu pejalan kaki bulat mengisi ~0.1–0.5 tergantung jarak.
        # Naikkan jika lampu jauh dan kecil (fraksi rendah) tapi tetap ditolak.
        self.declare_parameter('geom_max_blob_fraction', 0.75)

        # geom_min_blob_fraction: fraksi minimum luas tight_bbox yang harus
        # ditempati blob agar deteksi dianggap valid.
        # Di bawah nilai ini dianggap noise di tepi kotak — bukan lampu asli.
        # 0.0 = tidak ada syarat minimum (classifier min_area sudah cukup).
        self.declare_parameter('geom_min_blob_fraction', 0.02)

        # tight_box_height_scale: skala tinggi tight-box utk lampu figur 2-lamp
        # [merah atas/hijau bawah]; pakai 2.0 biar nutupin dua-duanya; kalau way
        # punya tag height [meter], itu yg dipakai dan scale diabaikan.
        # Nilai di bawah 1.0 diabaikan (di-clamp ke 1.0).
        self.declare_parameter('tight_box_height_scale', 2.0)

        # ---------- parameters: Autoware backend (backend='autoware') ----------
        # autoware_light_id: traffic_light_id yang dikirim ke classifier dan
        # dipakai sebagai pengenal dalam TrafficLightRoiArray.
        self.declare_parameter('autoware_light_id', 9001)
        # autoware_result_timeout_sec: usia maksimum cache classifier sebelum
        # dianggap basi dan STATE_NONE dikembalikan.
        self.declare_parameter('autoware_result_timeout_sec', 1.0)
        # Topic names — node-relative (akan menjadi ~/clf/...)
        self.declare_parameter('autoware_image_topic', '~/clf/image')
        self.declare_parameter('autoware_rois_topic', '~/clf/rois')
        self.declare_parameter('autoware_signals_topic', '~/clf/traffic_signals')

        # debug_all_cameras: publish debug image SEMUA kamera saat roi_source=map.
        # Sekarang AMAN karena di-rate-limit oleh debug_rate_hz.
        # Set false di produksi untuk benar-benar mematikan overhead.
        self.declare_parameter('debug_all_cameras', True)
        # debug_rate_hz: frekuensi timer debug lambat untuk all-camera path.
        # Naikkan untuk lebih real-time tapi lebih berat (decode+draw per kamera).
        # All-camera debug emang berat — matiin debug_all_cameras buat produksi.
        self.declare_parameter('debug_rate_hz', 2.0)
        # debug_downscale: faktor skala output debug image (0 < nilai <= 1.0).
        # Mis. 0.5 = setengah resolusi sebelum dipublish — potong bandwidth render.
        # 1.0 = tidak di-resize (default).
        self.declare_parameter('debug_downscale', 1.0)

        # --- baca semua params ---
        self._start_enabled = self.get_parameter('start_enabled').value
        self._force_state = int(self.get_parameter('force_state').value)
        self._force_confidence = float(self.get_parameter('force_confidence').value)
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._backend = str(self.get_parameter('backend').value)
        self._camera_topic = str(self.get_parameter('camera_topic').value)
        self._roi_source = str(self.get_parameter('roi_source').value)
        self._static_roi = list(self.get_parameter('static_roi').value)

        hsv_params = {
            'red_hue_lo_max': self.get_parameter('red_hue_lo_max').value,
            'red_hue_hi_min': self.get_parameter('red_hue_hi_min').value,
            'green_hue_lo': self.get_parameter('green_hue_lo').value,
            'green_hue_hi': self.get_parameter('green_hue_hi').value,
            's_min': self.get_parameter('s_min').value,
            'v_min': self.get_parameter('v_min').value,
            'min_area': self.get_parameter('min_area').value,
        }

        # Autoware backend params — masuk ke params dict yang sama yang dikirim ke factory
        self._autoware_light_id = int(self.get_parameter('autoware_light_id').value)
        _autoware_result_timeout_sec = float(
            self.get_parameter('autoware_result_timeout_sec').value
        )
        self._autoware_image_topic = str(self.get_parameter('autoware_image_topic').value)
        self._autoware_rois_topic = str(self.get_parameter('autoware_rois_topic').value)
        self._autoware_signals_topic = str(
            self.get_parameter('autoware_signals_topic').value
        )

        # Gabung params agar AutowareClassifier mendapat result_timeout_sec
        classifier_params = dict(hsv_params)
        classifier_params['result_timeout_sec'] = _autoware_result_timeout_sec

        self._stable_frames = max(1, int(self.get_parameter('stable_frames').value))
        self._min_confidence = float(self.get_parameter('min_confidence').value)
        self._enable_debug_image = bool(self.get_parameter('enable_debug_image').value)

        self._map_path = str(self.get_parameter('map_path').value)
        self._camera_names = list(self.get_parameter('cameras').value)
        self._roi_margin_px = int(self.get_parameter('roi_margin_px').value)
        self._debug_all_cameras = bool(self.get_parameter('debug_all_cameras').value)
        self._debug_rate_hz = float(self.get_parameter('debug_rate_hz').value)
        self._debug_downscale = float(self.get_parameter('debug_downscale').value)

        # Geometric gate params — dikumpulkan ke satu dict untuk diteruskan ke gate
        self._light_radius_m = float(self.get_parameter('light_radius_m').value)
        self._geom_params = {
            'geom_gate_enabled':     bool(self.get_parameter('geom_gate_enabled').value),
            'geom_require_centroid': bool(self.get_parameter('geom_require_centroid').value),
            'geom_max_blob_fraction': float(self.get_parameter('geom_max_blob_fraction').value),
            'geom_min_blob_fraction': float(self.get_parameter('geom_min_blob_fraction').value),
        }

        # tight_box_height_scale: pengali tinggi tight-box utk lampu 2-lamp
        # (merah atas / hijau bawah). Kalau way punya tag OSM 'height', scale ini
        # diabaikan dan tinggi fisik fixture dipakai langsung.
        _raw_scale = float(self.get_parameter('tight_box_height_scale').value)
        self._tight_box_height_scale = max(1.0, _raw_scale)

        # Clamp debug_downscale ke range valid
        if self._debug_downscale <= 0.0 or self._debug_downscale > 1.0:
            self.get_logger().warn(
                f'debug_downscale={self._debug_downscale} tidak valid; reset ke 1.0'
            )
            self._debug_downscale = 1.0

        # --- inisialisasi classifier ---
        self._classifier = None
        if self._backend != 'force':
            try:
                self._classifier = make_classifier(self._backend, classifier_params)
                self.get_logger().info(
                    f'Classifier backend={self._backend!r} berhasil dibuat.'
                )
            except Exception as e:
                self.get_logger().error(
                    f'Gagal membuat classifier backend={self._backend!r}: {e}. '
                    'Fallback ke mode force.'
                )
                self._backend = 'force'

        # --- cv_bridge ---
        if _CV_BRIDGE_AVAILABLE:
            self._bridge = CvBridge()
        else:
            self._bridge = None
            if self._backend != 'force':
                self.get_logger().error(
                    'cv_bridge tidak tersedia! Fallback ke mode force.'
                )
                self._backend = 'force'

        # ---------- tf2 buffer + listener (dibuat sekarang, selalu aktif) ----------
        self._tf_buffer = None
        self._tf_listener = None
        if _TF2_AVAILABLE and self._roi_source == 'map':
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(
                self._tf_buffer, self
            )
            self.get_logger().info('tf2 Buffer + TransformListener dibuat untuk mode map.')

        # ---------- Fase 2: parse peta pedestrian lights (sekali, cached) ----------
        self._lights = []
        if self._roi_source == 'map':
            self._parse_map()

        # ---------- publisher utama (absolut) ----------
        self._pub = self.create_publisher(
            PedestrianLightState,
            '/perception/road_crossing/pedestrian_light',
            10,
        )

        # ---------- publisher debug image (relatif ke namespace node) ----------
        self._debug_pub = self.create_publisher(
            Image,
            '~/debug/image',
            1,
        )
        # Per-camera debug publishers (~/debug/<cam>/image) — dibuat saat startup,
        # karena publisher ROS2 ringan bahkan tanpa subscriber.
        # Key: cam_name, Value: Publisher
        self._debug_cam_pubs: dict = {}
        for _cam_n in self._camera_names:
            self._debug_cam_pubs[_cam_n] = self.create_publisher(
                Image,
                f'~/debug/{_cam_n}/image',
                1,
            )

        # ---------- Autoware backend: publishers + subscriber ----------
        # Dibuat unconditionally agar topic muncul di `ros2 topic list` dari awal,
        # tapi data hanya dipublish saat backend='autoware' aktif.
        self._clf_image_pub = None
        self._clf_rois_pub = None
        self._clf_signals_sub = None
        # Simpan raw Image msg terakhir dari kamera aktif (untuk di-republish ke clf/image)
        self._last_raw_image = None

        if self._backend == 'autoware':
            if not _TIER4_MSGS_AVAILABLE:
                self.get_logger().error(
                    'tier4_perception_msgs tidak tersedia! '
                    'Backend autoware tidak dapat berfungsi. Fallback ke force.'
                )
                self._backend = 'force'
            else:
                self._clf_image_pub = self.create_publisher(
                    Image,
                    self._autoware_image_topic,
                    _SENSOR_QOS,
                )
                # rois pub HARUS RELIABLE: classifier roi_sub pakai rclcpp::QoS{1}
                # (reliable). Kalau best_effort → QoS incompatible → "No messages will
                # be sent" → classifier ga pernah dapat ROI → traffic_signals diam.
                # _INFO_QOS = reliable, volatile, depth 1 → match persis roi_sub classifier.
                self._clf_rois_pub = self.create_publisher(
                    TrafficLightRoiArray,
                    self._autoware_rois_topic,
                    _INFO_QOS,
                )
                self._clf_signals_sub = self.create_subscription(
                    TrafficLightArray,
                    self._autoware_signals_topic,
                    self._on_clf_signals,
                    _SENSOR_QOS,
                )
                self.get_logger().info(
                    f'[autoware backend] Topics: '
                    f'image={self._autoware_image_topic} '
                    f'rois={self._autoware_rois_topic} '
                    f'signals={self._autoware_signals_topic}'
                )

        # ---------- service server (absolut) ----------
        self._enable_srv = self.create_service(
            SetBool,
            '/perception/road_crossing/pedestrian_light_detector/enable',
            self._handle_enable,
        )

        # ---------- state lazy ----------
        self._timer = None
        self._debug_timer = None       # timer lambat khusus debug_all_cameras
        self._cam_sub = None           # Fase 1: satu kamera tetap
        self._enabled = False

        # Frame terakhir dari kamera aktif (untuk klasifikasi)
        self._last_frame = None        # np.ndarray BGR

        # Anti-flicker: deque raw states, committed state
        self._raw_state_buf = collections.deque(maxlen=self._stable_frames)
        self._committed_state = STATE_NONE
        self._committed_conf = 0.0

        # ---------- Fase 2: state multi-kamera ----------
        # Dict nama_kamera -> CameraState (diisi saat camera_info diterima)
        self._cam_states: dict = {}
        # Dict nama_kamera -> Subscription camera_info (lazy, saat enable)
        self._cam_info_subs: dict = {}
        # Nama kamera aktif (yang image_raw-nya di-subscribe untuk klasifikasi)
        self._active_cam_name: str = ''
        # Subscription image_raw kamera aktif (Fase 2, untuk klasifikasi)
        self._active_image_sub = None
        # ROI bbox terakhir dari proyeksi map — padded (x1,y1,x2,y2)
        self._map_roi_bbox = None
        # Tight bbox terakhir dari proyeksi map (x1,y1,x2,y2) dalam koordinat image
        # Dipakai geometric gate dan debug image
        self._map_tight_bbox = None
        # Light yang sedang dipilih
        self._active_light = None
        # Hasil terakhir geometric gate untuk debug image
        # (accepted: bool, reason: str, centroid_in_roi: tuple|None)
        self._last_gate_result = (True, 'init', None)

        # ---------- debug_all_cameras: raw msg buffer (TANPA decode) ----------
        # Dict nama_kamera -> Subscription image_raw (hanya saat debug_all_cameras=true)
        self._debug_image_subs: dict = {}
        # Dict nama_kamera -> Image msg RAW TERAKHIR (BUKAN BGR — decode lazy di timer)
        self._debug_raw: dict = {}
        # Dict nama_kamera -> builtin_interfaces.msg.Time stamp terakhir yang sudah
        # di-publish; dipakai untuk skip frame yang belum berubah.
        self._debug_last_stamp: dict = {}

        # Flag log-once untuk pesan roi_source=map fallback (Fase 1 path)
        self._map_roi_warned = False

        # ---------- start jika start_enabled ----------
        if self._start_enabled:
            self._start_publishing()

        state_name = _STATE_NAMES.get(self._force_state, str(self._force_state))
        self.get_logger().info(
            f'PedestrianLightDetectorNode ready | '
            f'backend={self._backend!r} '
            f'roi_source={self._roi_source!r} '
            f'start_enabled={self._start_enabled} '
            f'force_state={self._force_state}({state_name}) '
            f'rate={self._publish_rate_hz} Hz'
        )
        if self._roi_source == 'map':
            self.get_logger().info(
                f'[Fase 2] cameras={self._camera_names} '
                f'map_lights={len(self._lights)} '
                f'margin_px={self._roi_margin_px} '
                f'light_radius_m={self._light_radius_m} '
                f'tight_box_height_scale={self._tight_box_height_scale} '
                f'debug_all_cameras={self._debug_all_cameras} '
                f'debug_rate_hz={self._debug_rate_hz} '
                f'debug_downscale={self._debug_downscale}'
            )
            self.get_logger().info(
                f'[Geom Gate] '
                f'enabled={self._geom_params["geom_gate_enabled"]} '
                f'require_centroid={self._geom_params["geom_require_centroid"]} '
                f'max_blob_frac={self._geom_params["geom_max_blob_fraction"]} '
                f'min_blob_frac={self._geom_params["geom_min_blob_fraction"]}'
            )
            if self._debug_all_cameras:
                cam_topics = [
                    f'~/debug/{n}/image' for n in self._camera_names
                ]
                self.get_logger().info(
                    f'[Fase 2] debug_all_cameras ON — per-cam topics (rate-limited '
                    f'to {self._debug_rate_hz} Hz): '
                    + ', '.join(cam_topics)
                )

    # ------------------------------------------------------------------
    # Map parsing
    # ------------------------------------------------------------------

    def _parse_map(self):
        """Parse OSM map dan cache hasilnya ke self._lights."""
        if not self._map_path:
            self.get_logger().error(
                '[Fase 2] map_path kosong — ROI berbasis map nonaktif. '
                'Set lewat launch arg map_path (tier4_road_crossing_component '
                'meneruskannya dari map_path global).'
            )
            self._lights = []
            return
        try:
            self._lights = parse_tf_pedestrian_lights(self._map_path)
            self.get_logger().info(
                f'[Fase 2] Peta {self._map_path!r} parsed: '
                f'{len(self._lights)} pedestrian light(s) ditemukan.'
            )
            for lt in self._lights:
                self.get_logger().info(
                    f'  -> way {lt.way_id}: {len(lt.points)} node(s), '
                    f'midpoint=({lt.midpoint[0]:.2f}, {lt.midpoint[1]:.2f}, '
                    f'{lt.midpoint[2]:.2f})'
                )
        except Exception as e:
            self.get_logger().error(
                f'[Fase 2] Gagal parse map {self._map_path!r}: {e}. '
                'Mode map tidak akan berfungsi.'
            )
            self._lights = []

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------

    def _handle_enable(self, request, response):
        if request.data:
            if not self._enabled:
                self._start_publishing()
                self.get_logger().info('PedestrianLightDetector ENABLED')
            response.success = True
            response.message = 'pedestrian_light_detector enabled'
        else:
            if self._enabled:
                self._stop_publishing()
                self.get_logger().info('PedestrianLightDetector DISABLED')
            response.success = True
            response.message = 'pedestrian_light_detector disabled'
        return response

    # ------------------------------------------------------------------
    # Lazy start / stop
    # ------------------------------------------------------------------

    def _start_publishing(self):
        """Aktifkan timer publish dan buat subscriptions sesuai roi_source."""
        # Buat timer publish utama
        if self._timer is not None:
            self._timer.cancel()
        period = 1.0 / self._publish_rate_hz
        self._timer = self.create_timer(period, self._publish_cb)

        if self._backend == 'force':
            self._enabled = True
            return

        if self._roi_source == 'map':
            # Fase 2: subscribe camera_info semua kamera (ringan)
            self._subscribe_all_camera_info()
            # image_raw klasifikasi akan di-subscribe setelah kamera dipilih

            # Debug: jika debug_all_cameras aktif, subscribe image_raw semua kamera
            # (callback HANYA menyimpan raw msg — TANPA decode cv_bridge)
            # dan buat timer lambat khusus debug pada debug_rate_hz.
            if self._debug_all_cameras and self._enable_debug_image:
                self._subscribe_all_debug_images()
                self._start_debug_timer()
        else:
            # Fase 1: subscribe satu kamera tetap
            if self._cam_sub is None:
                self._cam_sub = self.create_subscription(
                    Image,
                    self._camera_topic,
                    self._image_cb,
                    _SENSOR_QOS,
                )
                self.get_logger().info(
                    f'Camera subscription dibuat: {self._camera_topic}'
                )

        self._enabled = True

    def _stop_publishing(self):
        """Matikan timer dan hancurkan semua subscriptions."""
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        # Matikan timer debug lambat
        self._stop_debug_timer()

        # Hancurkan subscription image Fase 1
        if self._cam_sub is not None:
            self.destroy_subscription(self._cam_sub)
            self._cam_sub = None
            self.get_logger().info('Camera subscription (Fase 1) dihancurkan.')

        # Hancurkan subscription image Fase 2
        if self._active_image_sub is not None:
            self.destroy_subscription(self._active_image_sub)
            self._active_image_sub = None
            self._active_cam_name = ''
            self.get_logger().info('Active image subscription (Fase 2) dihancurkan.')

        # Hancurkan semua camera_info subscriptions
        for cam_name, sub in list(self._cam_info_subs.items()):
            self.destroy_subscription(sub)
        self._cam_info_subs.clear()
        if self._cam_states:
            self.get_logger().info('Camera info subscriptions dihancurkan.')

        # Hancurkan debug image subscriptions (debug_all_cameras path)
        if self._debug_image_subs:
            for cam_name, sub in list(self._debug_image_subs.items()):
                self.destroy_subscription(sub)
            self._debug_image_subs.clear()
            self._debug_raw.clear()
            self._debug_last_stamp.clear()
            self.get_logger().info('Debug image subscriptions (all-camera) dihancurkan.')

        # Reset state
        self._last_frame = None
        self._last_raw_image = None
        self._map_roi_bbox = None
        self._map_tight_bbox = None
        self._active_light = None
        self._last_gate_result = (True, 'init', None)
        self._enabled = False

    # ------------------------------------------------------------------
    # Debug timer lifecycle (slow timer for all-camera debug)
    # ------------------------------------------------------------------

    def _start_debug_timer(self):
        """Buat timer lambat untuk debug_all_cameras publish pada debug_rate_hz."""
        if self._debug_timer is not None:
            self._debug_timer.cancel()
        debug_period = 1.0 / max(0.1, self._debug_rate_hz)
        self._debug_timer = self.create_timer(debug_period, self._debug_timer_cb)
        self.get_logger().info(
            f'[debug_all_cameras] Timer lambat dibuat: {self._debug_rate_hz} Hz '
            f'(period={debug_period:.3f}s) — decode+draw dibatasi di sini.'
        )

    def _stop_debug_timer(self):
        """Hancurkan timer lambat debug jika ada."""
        if self._debug_timer is not None:
            self._debug_timer.cancel()
            self._debug_timer = None

    # ------------------------------------------------------------------
    # Fase 2: multi-camera subscriptions
    # ------------------------------------------------------------------

    def _subscribe_all_camera_info(self):
        """Subscribe camera_info untuk semua kamera yang dikonfigurasi."""
        for cam_name in self._camera_names:
            if cam_name in self._cam_info_subs:
                continue  # sudah subscribe

            topic = f'/camera/{cam_name}/camera_info'

            # Closure: capture cam_name per iterasi
            def make_cb(name):
                def cb(msg: CameraInfo):
                    self._camera_info_cb(name, msg)
                return cb

            sub = self.create_subscription(
                CameraInfo,
                topic,
                make_cb(cam_name),
                _INFO_QOS,
            )
            self._cam_info_subs[cam_name] = sub
            self.get_logger().info(f'[Fase 2] Camera info sub: {topic}')

    def _subscribe_all_debug_images(self):
        """
        Subscribe image_raw SEMUA kamera untuk keperluan debug_all_cameras.

        Dipanggil hanya saat debug_all_cameras=true AND enable_debug_image=true.
        Callback HANYA menyimpan raw Image msg ke _debug_raw[cam_name] —
        TANPA decode cv_bridge. Decode dilakukan lazily oleh _debug_timer_cb.
        """
        for cam_name in self._camera_names:
            if cam_name in self._debug_image_subs:
                continue  # sudah subscribe

            topic = f'/camera/{cam_name}/image_raw'

            def make_debug_img_cb(name):
                def cb(msg: Image):
                    self._debug_image_cb(name, msg)
                return cb

            sub = self.create_subscription(
                Image,
                topic,
                make_debug_img_cb(cam_name),
                _SENSOR_QOS,
            )
            self._debug_image_subs[cam_name] = sub
            self.get_logger().info(
                f'[debug_all_cameras] Image sub (raw-only): {topic}'
            )

    def _debug_image_cb(self, cam_name: str, msg: Image):
        """
        Simpan HANYA raw Image msg dari semua kamera (debug path).

        TIDAK ada cv_bridge decode di sini — sangat ringan, hanya pointer assignment.
        Decode dilakukan di _debug_timer_cb saat timer lambat terpicu.
        """
        # Simpan msg mentah — O(1), tidak ada alokasi array besar
        self._debug_raw[cam_name] = msg

    def _camera_info_cb(self, cam_name: str, msg: CameraInfo):
        """Simpan intrinsic + frame_id dari camera_info."""
        # Ambil K sebagai 3x3 array (row-major dari msg.k)
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        frame_id = msg.header.frame_id
        w = msg.width
        h = msg.height

        if cam_name not in self._cam_states or \
                self._cam_states[cam_name].frame_id != frame_id:
            self.get_logger().info(
                f'[Fase 2] CameraInfo {cam_name}: '
                f'frame={frame_id!r} {w}x{h} '
                f'fx={K[0,0]:.1f} fy={K[1,1]:.1f}'
            )

        self._cam_states[cam_name] = CameraState(
            name=cam_name,
            K=K,
            frame_id=frame_id,
            width=w,
            height=h,
        )

    def _ensure_image_sub(self, cam_name: str):
        """
        Pastikan hanya satu image_raw yang di-subscribe: kamera terpilih.

        Jika kamera berubah: destroy lama, buat baru.
        """
        if cam_name == self._active_cam_name and self._active_image_sub is not None:
            return  # sudah benar

        # Hancurkan subscription lama jika berbeda kamera
        if self._active_image_sub is not None:
            self.destroy_subscription(self._active_image_sub)
            self._active_image_sub = None
            self.get_logger().info(
                f'[Fase 2] Image sub lama ({self._active_cam_name!r}) dihancurkan.'
            )

        # Buat subscription baru
        topic = f'/camera/{cam_name}/image_raw'
        self._active_image_sub = self.create_subscription(
            Image,
            topic,
            self._image_cb,
            _SENSOR_QOS,
        )
        self._active_cam_name = cam_name
        self._last_frame = None        # reset frame saat ganti kamera
        self._last_raw_image = None    # reset raw image (autoware backend) saat ganti kamera
        self.get_logger().info(
            f'[Fase 2] Image sub baru: {topic}'
        )

    # ------------------------------------------------------------------
    # Camera image callback (klasifikasi — tetap decode setiap frame)
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image):
        """Simpan frame BGR terakhir; konversi via cv_bridge (untuk klasifikasi)."""
        if self._bridge is None:
            return
        # Selalu simpan raw Image msg untuk backend autoware
        # (direpublish ke ~/clf/image dengan stamp asli)
        self._last_raw_image = msg
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last_frame = bgr
        except Exception as e:
            self.get_logger().warn(
                f'cv_bridge konversi gagal: {e}',
                throttle_duration_sec=5.0,
            )

    # ------------------------------------------------------------------
    # Gated classify — HSV + geometric gate (anti false-positive)
    # ------------------------------------------------------------------

    def _gated_classify(
        self,
        roi: 'np.ndarray',
        padded_bbox: tuple,
        tight_bbox,
    ) -> tuple:
        """
        Klasifikasikan ROI melalui HSV classifier lalu validasi secara geometris.

        Langkah:
        1. Panggil classifier.classify_detailed(roi) untuk mendapatkan state,
           confidence, dan winning_mask (mask warna pemenang dalam koordinat ROI).
        2. Konversi tight_bbox dari koordinat IMAGE ke koordinat ROI lokal
           (dengan mengurangi origin padded_bbox).
        3. Panggil geometric_gate(winning_mask, tight_box_in_roi, geom_params).
        4. Jika gate menolak → kembalikan (STATE_NONE, 0.0) — bukan warna yang
           terdeteksi di lokasi lampu. Catat alasan di log throttled.
        5. Jika gate menerima → kembalikan state dan confidence asli.

        Gate berlaku untuk SEMUA state (RED dan GREEN) sehingga objek merah
        besar di luar lampu juga tidak memicu STATE_RED.

        Parameters
        ----------
        roi          : np.ndarray BGR — crop padded_bbox dari frame.
        padded_bbox  : (x1,y1,x2,y2) koordinat IMAGE — origin untuk konversi.
        tight_bbox   : (x1,y1,x2,y2) koordinat IMAGE — kotak ketat lampu.
                       Boleh None jika proyeksi gagal (gate otomatis dilewati).

        Returns
        -------
        (raw_state, raw_conf) setelah gate diterapkan.
        """
        try:
            raw_state, raw_conf, winning_mask, _, _ = \
                self._classifier.classify_detailed(roi)
        except Exception as e:
            self.get_logger().warn(
                f'Classifier error (map): {e}',
                throttle_duration_sec=5.0,
            )
            self._last_gate_result = (True, 'classifier-error', None)
            return STATE_NONE, 0.0

        # Jika classifier sudah STATE_NONE, tidak perlu gate
        if raw_state == STATE_NONE:
            self._last_gate_result = (True, 'state-none', None)
            return raw_state, raw_conf

        # Konversi tight_bbox (koordinat IMAGE) → tight_box_in_roi (koordinat ROI)
        tight_box_in_roi = None
        if tight_bbox is not None:
            px1, py1, _, _ = padded_bbox
            tx1, ty1, tx2, ty2 = tight_bbox
            # Geser ke koordinat ROI lokal
            tx1_r = tx1 - px1
            ty1_r = ty1 - py1
            tx2_r = tx2 - px1
            ty2_r = ty2 - py1
            # Clamp ke dimensi ROI (tidak boleh negatif atau melampaui ROI)
            roi_h, roi_w = roi.shape[:2]
            tx1_r = max(0, min(tx1_r, roi_w - 1))
            ty1_r = max(0, min(ty1_r, roi_h - 1))
            tx2_r = max(tx1_r + 1, min(tx2_r, roi_w))
            ty2_r = max(ty1_r + 1, min(ty2_r, roi_h))
            tight_box_in_roi = (tx1_r, ty1_r, tx2_r, ty2_r)

        # Jalankan geometric gate
        accepted, reason, centroid_in_roi = geometric_gate(
            winning_mask, tight_box_in_roi, self._geom_params
        )
        self._last_gate_result = (accepted, reason, centroid_in_roi)

        if not accepted:
            state_name = _STATE_NAMES.get(raw_state, str(raw_state))
            self.get_logger().warn(
                f'[Geom Gate] Deteksi {state_name} DITOLAK — {reason}. '
                f'Publish STATE_NONE.',
                throttle_duration_sec=2.0,
            )
            return STATE_NONE, 0.0

        return raw_state, raw_conf

    # ------------------------------------------------------------------
    # Autoware backend: signals callback
    # ------------------------------------------------------------------

    def _on_clf_signals(self, msg: 'TrafficLightArray'):
        """
        Callback dari ~/clf/traffic_signals (TrafficLightArray dari external CNN).

        Mengambil elemen pertama dari signal pertama, memetakan color → state,
        lalu menyimpan hasilnya ke cache AutowareClassifier.

        Color mapping (tier4_perception_msgs/TrafficLightElement):
          RED(1)   -> STATE_RED
          GREEN(3) -> STATE_GREEN
          lainnya  -> STATE_NONE
        """
        if self._classifier is None:
            return
        if not msg.signals:
            return

        first_signal = msg.signals[0]
        if not first_signal.elements:
            return

        element = first_signal.elements[0]

        # TrafficLightElement.color: RED=1, GREEN=3 (dari .msg file)
        color = element.color
        if color == 1:    # RED
            state = STATE_RED
        elif color == 3:  # GREEN
            state = STATE_GREEN
        else:
            state = STATE_NONE

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # update_result hanya ada di AutowareClassifier; guard type untuk keamanan
        if hasattr(self._classifier, 'update_result'):
            self._classifier.update_result(state, float(element.confidence), now_sec)

    # ------------------------------------------------------------------
    # Publish callback (timer utama — publish_rate_hz)
    # ------------------------------------------------------------------

    def _publish_cb(self):
        # Mode 'force': perilaku mock lama.
        if self._backend == 'force':
            self._publish_forced()
            return

        # Mode map (Fase 2)
        if self._roi_source == 'map':
            self._publish_cb_map()
            return

        # Mode full / static (Fase 1): perilaku tidak berubah
        if self._last_frame is None:
            self._publish_state(STATE_NONE, 0.0)
            return

        roi = self._get_roi(self._last_frame)

        try:
            raw_state, raw_conf = self._classifier.classify(roi)
        except Exception as e:
            self.get_logger().warn(
                f'Classifier error: {e}',
                throttle_duration_sec=5.0,
            )
            raw_state, raw_conf = STATE_NONE, 0.0

        self._update_committed(raw_state, raw_conf)
        self._publish_state(self._committed_state, self._committed_conf)

        if self._enable_debug_image:
            self._publish_debug_image(
                self._last_frame, roi, self._committed_state,
                self._committed_conf,
            )

    def _publish_cb_map(self):
        """
        Fase 2: update proyeksi, pilih kamera, classify ROI, publish state.

        Alur:
        1. Jalankan projector_select -> (cam_name, bbox, light) atau None.
        2. Jika ada: ensure_image_sub, simpan bbox.
        3. Jika ada frame + bbox: crop -> classify -> smoothing -> publish.
        4. Jika tidak ada yang terlihat: publish STATE_NONE.

        CATATAN: _publish_debug_all_cameras TIDAK dipanggil di sini.
        Ia dipanggil oleh _debug_timer_cb (timer lambat debug_rate_hz).
        """
        # Jalankan seleksi kamera (requires tf + camera_info sudah ada)
        if not self._cam_states:
            # Belum ada camera_info sama sekali
            self.get_logger().warn(
                '[Fase 2] Belum ada camera_info — tunggu subscriber.',
                throttle_duration_sec=5.0,
            )
            self._publish_state(STATE_NONE, 0.0)
            return

        if not self._lights:
            # Tidak ada lampu di peta
            self._publish_state(STATE_NONE, 0.0)
            return

        cameras_list = list(self._cam_states.values())
        result = projector_select(
            self._lights,
            cameras_list,
            self._tf_buffer,
            self._roi_margin_px,
            self._light_radius_m,
            self._tight_box_height_scale,
        )

        if result is None:
            # Tidak ada lampu dalam FOV kamera mana pun
            self.get_logger().warn(
                '[Fase 2] Tidak ada lampu dalam FOV — publish NONE.',
                throttle_duration_sec=3.0,
            )
            self._map_roi_bbox = None
            self._map_tight_bbox = None
            self._active_light = None
            self._publish_state(STATE_NONE, 0.0)
            return

        sel_cam_name, sel_padded_bbox, sel_tight_bbox, sel_light = result

        # Pastikan image_raw kamera terpilih di-subscribe
        self._ensure_image_sub(sel_cam_name)

        # Simpan bbox + light untuk debug image
        self._map_roi_bbox = sel_padded_bbox
        self._map_tight_bbox = sel_tight_bbox
        self._active_light = sel_light

        # Butuh frame dari kamera terpilih
        if self._last_frame is None:
            self.get_logger().warn(
                f'[Fase 2] Menunggu frame dari {sel_cam_name!r}...',
                throttle_duration_sec=5.0,
            )
            self._publish_state(STATE_NONE, 0.0)
            return

        # Crop ROI padded dari bbox (sama seperti sebelumnya)
        x1, y1, x2, y2 = sel_padded_bbox
        roi = self._last_frame[y1:y2, x1:x2]

        if roi.size == 0:
            self._publish_state(STATE_NONE, 0.0)
            return

        # ---- Autoware backend path ----
        if self._backend == 'autoware':
            self._publish_cb_map_autoware(sel_padded_bbox, sel_tight_bbox, sel_cam_name)
            return

        # ---- HSV backend path (tidak berubah) ----
        # Classify + Geometric gate
        raw_state, raw_conf = self._gated_classify(
            roi, sel_padded_bbox, sel_tight_bbox
        )

        self._update_committed(raw_state, raw_conf)
        self._publish_state(self._committed_state, self._committed_conf)

        # Debug kamera aktif (single-cam, publish_rate_hz — decode sudah ada
        # dari _image_cb untuk klasifikasi, jadi ini tidak menambah decode baru)
        if self._enable_debug_image:
            self._publish_debug_image_map(
                self._last_frame,
                sel_padded_bbox,
                sel_tight_bbox,
                sel_cam_name,
                self._committed_state,
                self._committed_conf,
            )
        # TIDAK ada pemanggilan _publish_debug_all_cameras di sini.
        # Ia dijalankan oleh _debug_timer_cb agar rate-limited.

    # ------------------------------------------------------------------
    # Autoware backend: sub-routine publish_cb_map
    # ------------------------------------------------------------------

    def _publish_cb_map_autoware(
        self,
        sel_padded_bbox: tuple,
        sel_tight_bbox,
        sel_cam_name: str,
    ):
        """
        Autoware backend branch untuk _publish_cb_map.

        Langkah:
        1. Buat TrafficLightRoiArray dari sel_padded_bbox + autoware_light_id.
        2. Republish self._last_raw_image ke ~/clf/image dengan stamp asli.
        3. Publish ROI array ke ~/clf/rois dengan stamp yang SAMA.
        4. Ambil state dari classify_cached(now_sec) — tidak ada gating geometris.
        5. Update anti-flicker + publish state + debug image.

        Syarat exact-time-sync antara image dan rois:
          Keduanya menggunakan stamp dari self._last_raw_image.header.stamp.
          Autoware classifier memakai exact (tidak approximate) sync, sehingga
          timestamp HARUS identik byte-for-byte.
        """
        if self._clf_image_pub is None or self._clf_rois_pub is None:
            # Tier4 msgs tidak tersedia; publish STATE_NONE
            self._publish_state(STATE_NONE, 0.0)
            return

        raw_img = self._last_raw_image
        if raw_img is None:
            self._publish_state(STATE_NONE, 0.0)
            return

        # Stamp dari raw image (akan dipakai untuk KEDUA pesan: image + rois)
        img_stamp = raw_img.header.stamp

        # --- Republish raw image (unchanged) ---
        self._clf_image_pub.publish(raw_img)

        # --- Bangun TrafficLightRoiArray ---
        # Pakai TIGHT bbox (proyeksi 3D lampu — sama dgn yang dipakai geometric gate HSV),
        # BUKAN padded. Ini = filter 3D posisi versi CNN: classifier cuma lihat region
        # lampu, clutter (bus/tiang) di luar tight box ke-exclude. Juga cara CNN dipakai
        # (crop ketat seperti output fine_detector). Fallback ke padded kalau tight None/invalid.
        bbox_for_clf = sel_tight_bbox if sel_tight_bbox is not None else sel_padded_bbox
        x1, y1, x2, y2 = bbox_for_clf
        if int(x2) - int(x1) < 2 or int(y2) - int(y1) < 2:
            # tight degenerate → pakai padded biar classifier tetap dpt crop valid
            x1, y1, x2, y2 = sel_padded_bbox

        from sensor_msgs.msg import RegionOfInterest
        roi_msg = RegionOfInterest()
        roi_msg.x_offset = max(0, int(x1))
        roi_msg.y_offset = max(0, int(y1))
        roi_msg.width = max(0, int(x2) - max(0, int(x1)))
        roi_msg.height = max(0, int(y2) - max(0, int(y1)))
        roi_msg.do_rectify = False

        tl_roi = TrafficLightRoi()
        tl_roi.roi = roi_msg
        tl_roi.traffic_light_id = self._autoware_light_id
        tl_roi.traffic_light_type = 1  # PEDESTRIAN_TRAFFIC_LIGHT

        rois_msg = TrafficLightRoiArray()
        rois_msg.header.stamp = img_stamp  # EXACT same stamp as republished image
        # Usa camera frame_id dari kamera aktif
        cam_state = self._cam_states.get(sel_cam_name)
        if cam_state is not None:
            rois_msg.header.frame_id = cam_state.frame_id
        else:
            rois_msg.header.frame_id = f'camera_{sel_cam_name}'
        rois_msg.rois = [tl_roi]

        self._clf_rois_pub.publish(rois_msg)

        # --- Ambil state dari cache (dengan staleness check) ---
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        raw_state, raw_conf = self._classifier.classify_cached(now_sec)

        # Anti-flicker (sama seperti HSV path)
        self._update_committed(raw_state, raw_conf)
        self._publish_state(self._committed_state, self._committed_conf)

        # Debug image: gambar ROI box (tanpa gate overlay karena tidak ada mask)
        if self._enable_debug_image and self._last_frame is not None:
            self._publish_debug_image_map(
                self._last_frame,
                sel_padded_bbox,
                sel_tight_bbox,
                sel_cam_name,
                self._committed_state,
                self._committed_conf,
            )

    # ------------------------------------------------------------------
    # Debug timer callback (timer lambat — debug_rate_hz)
    # ------------------------------------------------------------------

    def _debug_timer_cb(self):
        """
        Timer callback lambat untuk all-camera debug.

        Dipanggil hanya debug_rate_hz kali per detik (default 2 Hz).
        Di sinilah decode cv_bridge + draw + publish dilakukan untuk semua kamera,
        BUKAN di image callback. Ini membatasi beban berat ke:
          debug_rate_hz × num_cameras decode/s  (mis. 2 × 4 = 8/s).

        Frame yang stamp-nya belum berubah sejak tick sebelumnya dilewati
        (tidak ada kerja redundan jika kamera diam atau rate rendah).
        """
        if not self._enabled:
            return
        if self._bridge is None:
            return

        # Nama kamera aktif (untuk pewarnaan bbox)
        selected_cam_name = self._active_cam_name
        self._publish_debug_all_cameras(selected_cam_name)

    # ------------------------------------------------------------------
    # ROI extraction (Fase 1)
    # ------------------------------------------------------------------

    def _get_roi(self, frame: np.ndarray) -> np.ndarray:
        """
        Ambil ROI dari frame sesuai roi_source (Fase 1 path).

        'full'   -> seluruh frame
        'static' -> crop box dari param static_roi
        'map'    -> tidak dipanggil dari path ini (ditangani _publish_cb_map)
        """
        if self._roi_source == 'full':
            return frame

        if self._roi_source == 'static':
            return self._crop_static_roi(frame)

        # fallback unknown (termasuk 'map' jika entah bagaimana sampai sini)
        self.get_logger().warn(
            f'roi_source tidak dikenal di _get_roi: {self._roi_source!r}. Fallback ke full.',
            throttle_duration_sec=10.0,
        )
        return frame

    def _crop_static_roi(self, frame: np.ndarray) -> np.ndarray:
        """
        Crop frame dengan static_roi [x, y, w, h].
        Jika nilai <= 1.0 dianggap fraksi dari dimensi frame.
        Jika nilai > 1.0 dianggap piksel absolut.
        """
        fh, fw = frame.shape[:2]
        roi_vals = self._static_roi
        if len(roi_vals) < 4:
            self.get_logger().warn(
                'static_roi harus berisi 4 elemen [x, y, w, h]. Fallback ke full.',
                throttle_duration_sec=10.0,
            )
            return frame

        rx, ry, rw, rh = roi_vals[0], roi_vals[1], roi_vals[2], roi_vals[3]

        if max(rx, ry, rw, rh) <= 1.0:
            x1 = int(rx * fw)
            y1 = int(ry * fh)
            x2 = int((rx + rw) * fw)
            y2 = int((ry + rh) * fh)
        else:
            x1 = int(rx)
            y1 = int(ry)
            x2 = int(rx + rw)
            y2 = int(ry + rh)

        x1 = max(0, min(x1, fw - 1))
        y1 = max(0, min(y1, fh - 1))
        x2 = max(x1 + 1, min(x2, fw))
        y2 = max(y1 + 1, min(y2, fh))

        return frame[y1:y2, x1:x2]

    # ------------------------------------------------------------------
    # Anti-flicker smoothing
    # ------------------------------------------------------------------

    def _update_committed(self, raw_state: int, raw_conf: float):
        """
        Commit state baru hanya jika stable_frames terakhir semua sepakat
        AND confidence >= min_confidence.
        """
        self._raw_state_buf.append(raw_state)

        if len(self._raw_state_buf) < self._stable_frames:
            return

        first = self._raw_state_buf[0]
        all_agree = all(s == first for s in self._raw_state_buf)

        if all_agree and raw_conf >= self._min_confidence:
            self._committed_state = first
            self._committed_conf = raw_conf

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------

    def _publish_forced(self):
        """Publish fixed state (mode force/mock)."""
        msg = PedestrianLightState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_front'
        msg.state = self._force_state
        msg.confidence = self._force_confidence
        msg.blocked = False
        self._pub.publish(msg)

    def _publish_state(self, state: int, confidence: float):
        """Publish state + confidence ke topic utama."""
        msg = PedestrianLightState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Gunakan nama kamera aktif sebagai frame_id jika tersedia
        if self._active_cam_name:
            msg.header.frame_id = f'camera_{self._active_cam_name}'
        else:
            msg.header.frame_id = 'camera_front'
        msg.state = state
        msg.confidence = confidence
        msg.blocked = False
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # Debug image publishers
    # ------------------------------------------------------------------

    def _publish_debug_image(self, frame: np.ndarray, roi: np.ndarray,
                              state: int, confidence: float):
        """
        Debug image Fase 1: gambar ROI box + label state/conf di atas frame.
        Hanya dipanggil jika enable_debug_image=true dan frame tersedia.
        """
        if self._bridge is None:
            return

        vis = frame.copy()
        fh, fw = vis.shape[:2]

        roi_box = self._get_roi_box(fh, fw)
        if roi_box is not None:
            x1, y1, x2, y2 = roi_box
            cv2.rectangle(vis, (x1, y1), (x2, y2), _state_color(state), 2)

        self._draw_state_label(vis, state, confidence)
        self._publish_vis(vis, frame_id='camera_front')

    def _publish_debug_image_map(self, frame: np.ndarray,
                                  padded_bbox: tuple,
                                  tight_bbox,
                                  cam_name: str,
                                  state: int, confidence: float):
        """
        Debug image Fase 2: gambar padded bbox + tight bbox + gate result.

        padded_bbox : (x1,y1,x2,y2) — kotak longgar yang dicrop ke classifier.
                      Digambar dengan warna state (merah/hijau/biru), garis tebal.
        tight_bbox  : (x1,y1,x2,y2) atau None — kotak ketat proyeksi lampu.
                      Digambar putih tipis di dalam padded_bbox.
        Gate result : jika gate menolak, centroid blob ditandai X kuning;
                      jika gate menerima, centroid ditandai lingkaran hijau muda.
        """
        if self._bridge is None or frame is None:
            return

        vis = frame.copy()

        # --- Gambar padded bbox (ROI yang dikirim ke classifier) ---
        x1, y1, x2, y2 = padded_bbox
        border_color = _state_color(state)
        cv2.rectangle(vis, (x1, y1), (x2, y2), border_color, 2)

        # --- Gambar tight bbox (kotak ketat proyeksi fisik lampu) ---
        if tight_bbox is not None:
            tx1, ty1, tx2, ty2 = tight_bbox
            # Putih tipis = area yang lampu SEHARUSNYA berada
            cv2.rectangle(vis, (tx1, ty1), (tx2, ty2), (255, 255, 255), 1)

        # --- Gambar centroid blob + hasil gate ---
        gate_accepted, gate_reason, centroid_in_roi = self._last_gate_result
        if centroid_in_roi is not None:
            # Konversi centroid dari koordinat ROI ke koordinat image
            cx_img = centroid_in_roi[0] + x1
            cy_img = centroid_in_roi[1] + y1
            if gate_accepted:
                # Centroid diterima: lingkaran hijau muda
                cv2.circle(vis, (cx_img, cy_img), 5, (100, 255, 100), 2)
            else:
                # Centroid ditolak: X kuning — ini objek yang di-gate
                cv2.line(vis, (cx_img - 6, cy_img - 6), (cx_img + 6, cy_img + 6),
                         (0, 220, 220), 2)
                cv2.line(vis, (cx_img + 6, cy_img - 6), (cx_img - 6, cy_img + 6),
                         (0, 220, 220), 2)
                # Teks alasan penolakan di bawah padded bbox
                cv2.putText(
                    vis, f'REJECTED: {gate_reason}',
                    (x1, min(y2 + 14, vis.shape[0] - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 220), 1, cv2.LINE_AA
                )

        # --- Label kamera + way_id ---
        light_info = ''
        if self._active_light is not None:
            mid = self._active_light.midpoint
            light_info = (
                f' way={self._active_light.way_id}'
                f' mid=({mid[0]:.1f},{mid[1]:.1f},{mid[2]:.1f})'
            )
        cam_label = f'cam:{cam_name}{light_info}'
        cv2.putText(
            vis, cam_label,
            (x1, max(y1 - 8, 12)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, border_color, 1, cv2.LINE_AA
        )

        self._draw_state_label(vis, state, confidence)
        self._publish_vis(vis, frame_id=f'camera_{cam_name}')

    def _draw_state_label(self, vis: np.ndarray, state: int, confidence: float):
        """Gambar label state/confidence di pojok kiri atas vis."""
        state_name = _STATE_NAMES.get(state, str(state))
        label = f'{state_name} ({confidence:.2f})'

        color_map = {
            STATE_NONE: (200, 200, 200),
            STATE_RED: (0, 0, 255),
            STATE_GREEN: (0, 255, 0),
            STATE_COUNTDOWN_GREEN: (0, 200, 100),
            STATE_COUNTDOWN_BLANK: (200, 200, 0),
        }
        text_color = color_map.get(state, (255, 255, 255))

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(label, font, font_scale, thickness)
        tx, ty = 10, 30
        cv2.rectangle(vis, (tx - 2, ty - th - 4), (tx + tw + 2, ty + baseline),
                      (0, 0, 0), -1)
        cv2.putText(vis, label, (tx, ty), font, font_scale, text_color, thickness,
                    cv2.LINE_AA)

    def _publish_vis(self, vis: np.ndarray, frame_id: str = 'camera_front'):
        """Publish numpy array BGR sebagai ROS Image ke ~/debug/image."""
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = frame_id
            self._debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(
                f'Debug image publish gagal: {e}',
                throttle_duration_sec=5.0,
            )

    # ------------------------------------------------------------------
    # debug_all_cameras: per-camera annotated debug image
    # (HANYA dipanggil dari _debug_timer_cb — rate-limited ke debug_rate_hz)
    # ------------------------------------------------------------------

    def _publish_debug_all_cameras(self, selected_cam_name: str):
        """
        Untuk setiap kamera yang dikonfigurasi, decode raw msg (LAZY — baru di sini),
        gambar semua lights yang diproyeksikan, dan publish ke ~/debug/<cam>/image.

        Beban berat (cv_bridge decode + draw + encode) di-rate-limit ke debug_rate_hz
        oleh timer pemanggil (_debug_timer_cb). Frame yang stamp-nya belum berubah
        sejak tick sebelumnya dilewati untuk menghindari kerja redundan.

        Warna bbox:
          - Hijau  : kamera ini adalah kamera yang sedang terpilih (active).
          - Cyan   : kamera lain.
        """
        if self._bridge is None:
            return

        for cam_name in self._camera_names:
            cam_state = self._cam_states.get(cam_name)
            pub = self._debug_cam_pubs.get(cam_name)
            if pub is None:
                continue

            # Ambil raw msg terakhir (None jika belum ada frame masuk)
            raw_msg = self._debug_raw.get(cam_name)

            is_selected = (cam_name == selected_cam_name)

            if raw_msg is not None:
                # Skip jika stamp belum berubah sejak terakhir dipublish
                last_stamp = self._debug_last_stamp.get(cam_name)
                cur_stamp = raw_msg.header.stamp
                if (last_stamp is not None
                        and last_stamp.sec == cur_stamp.sec
                        and last_stamp.nanosec == cur_stamp.nanosec):
                    continue  # frame identik, lewati

                # Decode BARU terjadi di sini (lazy) — hanya debug_rate_hz kali/s
                try:
                    frame = self._bridge.imgmsg_to_cv2(raw_msg, desired_encoding='bgr8')
                except Exception as e:
                    self.get_logger().warn(
                        f'[debug_all_cameras] cv_bridge decode gagal ({cam_name}): {e}',
                        throttle_duration_sec=5.0,
                    )
                    continue  # skip kamera ini tick ini

                # Tandai stamp sudah diproses
                self._debug_last_stamp[cam_name] = cur_stamp
            else:
                frame = None  # belum ada frame, gunakan canvas hitam

            # Buat canvas: gunakan frame jika ada, atau canvas hitam dengan ukuran
            # dari camera_info jika tersedia, atau 480x640 default.
            if frame is not None:
                vis = frame.copy()
                fh, fw = vis.shape[:2]
            elif cam_state is not None:
                fw, fh = cam_state.width, cam_state.height
                vis = np.zeros((fh, fw, 3), dtype=np.uint8)
            else:
                vis = np.zeros((480, 640, 3), dtype=np.uint8)
                fh, fw = 480, 640

            # Header info di bagian atas
            self._draw_cam_debug_header(vis, cam_name, is_selected, cam_state)

            if cam_state is None:
                # Belum ada camera_info
                cv2.putText(
                    vis, 'waiting camera_info',
                    (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 0), 1, cv2.LINE_AA
                )
                self._publish_vis_to(pub, vis, frame_id=f'camera_{cam_name}')
                continue

            # Proyeksikan semua lights ke kamera ini
            out_of_fov_msgs = []   # list str pesan out-of-FOV

            for light in self._lights:
                self._draw_light_on_cam_debug(
                    vis, light, cam_state, is_selected, out_of_fov_msgs
                )

            # Tampilkan pesan out-of-FOV di sisi kiri bawah header
            y_ofs = 90
            for msg_txt in out_of_fov_msgs:
                cv2.putText(
                    vis, msg_txt,
                    (10, y_ofs),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 130, 50), 1, cv2.LINE_AA
                )
                y_ofs += 16
                if y_ofs > fh - 10:
                    break

            # Downscale jika dikonfigurasi (potong bandwidth publish)
            if self._debug_downscale < 1.0:
                new_w = max(1, int(fw * self._debug_downscale))
                new_h = max(1, int(fh * self._debug_downscale))
                vis = cv2.resize(vis, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

            self._publish_vis_to(pub, vis, frame_id=f'camera_{cam_name}')

    def _draw_cam_debug_header(
        self,
        vis: np.ndarray,
        cam_name: str,
        is_selected: bool,
        cam_state,
    ):
        """
        Gambar header teks di pojok kiri atas vis:
          Line 1: "cam: <name>  [SELECTED]" atau "cam: <name>  [other]"
          Line 2: "tf: OK" atau "TF not available" — cek dengan lookup cepat
        """
        sel_label = 'SELECTED' if is_selected else 'other'
        sel_color = (0, 255, 0) if is_selected else (200, 200, 200)

        header_line1 = f'cam: {cam_name}  [{sel_label}]'

        # Cek tf availability (non-blocking; hasil dari lookup terbaru)
        tf_status = 'TF not available'
        if self._tf_buffer is not None and cam_state is not None:
            try:
                from rclpy.time import Time as RclpyTime
                self._tf_buffer.lookup_transform(
                    cam_state.frame_id, 'map', RclpyTime()
                )
                tf_status = 'tf: OK'
            except Exception:
                tf_status = 'TF not available'

        # Banner abu-abu (filled rect 0..72px, warna 30,30,30) DIHAPUS atas permintaan user.
        # Teks header digambar langsung di atas gambar tanpa background.
        # CATATAN: banner ini cuma overlay debug — TIDAK pernah memengaruhi klasifikasi
        # (classifier baca ROI dari frame RAW, bukan dari gambar debug ini).
        cv2.putText(
            vis, header_line1, (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, sel_color, 1, cv2.LINE_AA
        )
        cv2.putText(
            vis, tf_status, (10, 46),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (0, 220, 0) if tf_status == 'tf: OK' else (0, 80, 220),
            1, cv2.LINE_AA
        )
        lights_count = f'lights in map: {len(self._lights)}'
        cv2.putText(
            vis, lights_count, (10, 66),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1, cv2.LINE_AA
        )

    def _draw_light_on_cam_debug(
        self,
        vis: np.ndarray,
        light,
        cam_state: 'CameraState',
        is_selected: bool,
        out_of_fov_msgs: list,
    ):
        """
        Proyeksikan satu light ke kamera cam_state dan gambar hasilnya di vis.

        - Jika titik di belakang kamera (tf OK tapi Z<=0): tambah pesan "behind".
        - Jika bbox None tapi midpoint terproyeksikan: gambar dot + pesan "off-frame".
        - Jika bbox valid: gambar bbox + dot + label.
        - Warna bbox:
            selected camera  -> _state_color(committed_state) — merah/hijau/biru
            non-selected     -> abu-abu netral (128,128,128) + label "(not classified)"
        """
        fh, fw = vis.shape[:2]
        # Selected: warna mencerminkan committed detection state.
        # Non-selected: abu-abu netral — tidak ada klasifikasi yang berjalan di sini.
        if is_selected:
            box_color = _state_color(self._committed_state)
        else:
            box_color = (128, 128, 128)
        dot_color = (0, 180, 255)   # oranye cerah untuk dot proyeksi midpoint

        mid = light.midpoint

        # Proyeksikan midpoint untuk mendapatkan (u,v,depth)
        mid_result = project_point(
            mid,
            cam_state.K,
            cam_state.frame_id,
            self._tf_buffer,
            stamp=None,
        )

        if mid_result is None:
            # Tidak bisa lookup tf ATAU titik di belakang kamera
            out_of_fov_msgs.append(
                f'way {light.way_id}: out-of-FOV (behind/tf-fail)'
            )
            return

        u_mid, v_mid, depth = mid_result

        # Gambar dot midpoint (bahkan jika off-frame, clamp ke tepi)
        dot_u = int(np.clip(u_mid, 0, fw - 1))
        dot_v = int(np.clip(v_mid, 0, fh - 1))
        cv2.circle(vis, (dot_u, dot_v), 4, dot_color, -1)

        # Hitung padded_bbox dan tight_bbox (memakai light_roi_in_camera)
        # tight_box_height_scale diteruskan agar debug image mencerminkan
        # kotak yang lebih tinggi (sama persis yang dipakai classifier + gate).
        padded_bbox, tight_bbox = light_roi_in_camera(
            light,
            cam_state.K,
            cam_state.frame_id,
            fw,
            fh,
            self._tf_buffer,
            self._roi_margin_px,
            self._light_radius_m,
            self._tight_box_height_scale,
        )

        label = (
            f'way={light.way_id} '
            f'mid=({mid[0]:.1f},{mid[1]:.1f},{mid[2]:.1f}) '
            f'depth={depth:.1f}m'
        )
        if not is_selected:
            label += ' (not classified)'

        if padded_bbox is None:
            # Midpoint terproyeksikan tapi tidak ada titik dalam FOV image
            out_of_fov_msgs.append(
                f'way {light.way_id}: (u,v)=({u_mid:.0f},{v_mid:.0f}) off-frame'
            )
            return

        x1, y1, x2, y2 = padded_bbox
        # Padded bbox — warna sesuai state (selected) atau abu (non-selected)
        cv2.rectangle(vis, (x1, y1), (x2, y2), box_color, 2)

        # Tight bbox — selalu putih tipis agar terlihat terpisah dari padded
        if tight_bbox is not None:
            tx1, ty1, tx2, ty2 = tight_bbox
            cv2.rectangle(vis, (tx1, ty1), (tx2, ty2), (255, 255, 255), 1)

        # Label di atas bbox (tapi tidak di bawah header — shift jika perlu)
        label_y = max(y1 - 6, 80)
        cv2.putText(
            vis, label,
            (x1, label_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, box_color, 1, cv2.LINE_AA
        )

        # Dot midpoint di dalam bbox (ulang digambar supaya di atas rectangle)
        cv2.circle(vis, (dot_u, dot_v), 4, dot_color, -1)

    # ------------------------------------------------------------------
    # Publish helper per-camera publisher
    # ------------------------------------------------------------------

    def _publish_vis_to(self, pub, vis: np.ndarray, frame_id: str = ''):
        """Publish numpy array BGR ke publisher tertentu (bukan hanya _debug_pub)."""
        if self._bridge is None:
            return
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = frame_id
            pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(
                f'Per-cam debug image publish gagal ({frame_id}): {e}',
                throttle_duration_sec=5.0,
            )

    def _get_roi_box(self, fh: int, fw: int):
        """
        Kembalikan (x1, y1, x2, y2) piksel dari ROI untuk visualisasi Fase 1.
        None jika full frame atau map mode (ditangani terpisah).
        """
        if self._roi_source in ('full', 'map'):
            return None

        if self._roi_source == 'static':
            roi_vals = self._static_roi
            if len(roi_vals) < 4:
                return None
            rx, ry, rw, rh = roi_vals[0], roi_vals[1], roi_vals[2], roi_vals[3]
            if max(rx, ry, rw, rh) <= 1.0:
                x1 = int(rx * fw)
                y1 = int(ry * fh)
                x2 = int((rx + rw) * fw)
                y2 = int((ry + rh) * fh)
            else:
                x1 = int(rx)
                y1 = int(ry)
                x2 = int(rx + rw)
                y2 = int(ry + rh)
            x1 = max(0, min(x1, fw - 1))
            y1 = max(0, min(y1, fh - 1))
            x2 = max(x1 + 1, min(x2, fw))
            y2 = max(y1 + 1, min(y2, fh))
            return (x1, y1, x2, y2)

        return None


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianLightDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
