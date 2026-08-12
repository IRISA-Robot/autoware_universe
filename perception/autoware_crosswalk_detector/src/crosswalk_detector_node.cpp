// Copyright 2024 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file crosswalk_detector_node.cpp
 * @brief Phase 2 crosswalk detector — YOLOv8 via TensorRT (TrtCommon).
 *
 * Kontrak (identik dengan mock Phase 1):
 *   Node name  : crosswalk_detector
 *   Namespace  : /perception/road_crossing  (diset di launch)
 *   Topic pub  : /perception/road_crossing/crosswalk_detection  (absolut)
 *   Service    : /perception/road_crossing/crosswalk_detector/enable  (absolut)
 *   Debug image: ~/debug/image  (relatif namespace node)
 *
 * Model spec (verifikasi):
 *   Input  "images"  [1,3,512,512] float32
 *   Output "output0" [1,5,5376]   float32
 *   Output: reshape→transpose ke [5376,5]; setiap baris = [cx,cy,w,h,conf] dlm px 512-space
 *
 * Pre-processing: letterbox (pad abu-abu 114, aspect-preserving) → BGR→RGB → HWC→CHW → /255.0
 * Post-processing: conf>thr → xywh→xyxy → undo letterbox → clip → NMS (cv::dnn::NMSBoxes)
 *
 * Decision gate (temporal stable):
 *   Kandidat: conf≥conf_thr AND cy_center/imgH ≥ roi_y_min_frac AND area/(W*H) ≥ min_area_frac
 *   detected=true setelah stable_frames berturut-turut raw_detected==true (begitu juga false).
 *
 * Lazy subscribe: subscribe image HANYA saat enabled (mirip pedestrian_light_detector).
 *   Dapat dimatikan dengan param always_on=true (default) agar kamera + YOLO + debug image
 *   selalu aktif sejak startup, tanpa menunggu FSM enable.
 */

#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cuda_runtime_api.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <autoware_road_crossing_msgs/msg/crosswalk_detection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>
#include <vector>

// ============================================================
// Struktur deteksi internal (koordinat pixel image asli)
// ============================================================
struct CrosswalkBox
{
  float x1, y1, x2, y2;  // xyxy dalam px image asli
  float conf;
};

// ============================================================
// Node utama
// ============================================================
class CrosswalkDetectorNode : public rclcpp::Node
{
public:
  explicit CrosswalkDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("crosswalk_detector", options)
  {
    // ---- Deklarasi parameter ----
    // Expand '~' ke HOME sebelum menyimpan
    const char * home = std::getenv("HOME");
    const std::string home_str = (home != nullptr) ? std::string(home) : "";

    auto expand_home = [&](const std::string & path) -> std::string {
      if (!path.empty() && path[0] == '~') {
        return home_str + path.substr(1);
      }
      return path;
    };

    model_path_ = expand_home(
      this->declare_parameter<std::string>(
        "model_path",
        "~/autoware_data/crosswalk_detector/crosswalk_kitakyushu.onnx"));
    precision_        = this->declare_parameter<std::string>("precision", "fp16");
    input_size_       = this->declare_parameter<int>("input_size", 512);
    conf_thr_         = static_cast<float>(this->declare_parameter<double>("conf_thr", 0.4));
    nms_iou_          = static_cast<float>(this->declare_parameter<double>("nms_iou", 0.5));
    camera_topic_     = this->declare_parameter<std::string>(
      "camera_topic", "/camera/front/image_raw");
    roi_y_min_frac_   = static_cast<float>(this->declare_parameter<double>("roi_y_min_frac", 0.35));
    min_area_frac_    = static_cast<float>(this->declare_parameter<double>("min_area_frac", 0.01));
    stable_frames_    = this->declare_parameter<int>("stable_frames", 3);
    publish_rate_hz_  = this->declare_parameter<double>("publish_rate_hz", 10.0);
    enable_debug_     = this->declare_parameter<bool>("enable_debug_image", true);
    debug_rate_hz_    = this->declare_parameter<double>("debug_rate_hz", 2.0);
    start_enabled_    = this->declare_parameter<bool>("start_enabled", false);
    build_only_       = this->declare_parameter<bool>("build_only", false);
    // always_on=true → kamera + YOLO + debug image selalu aktif sejak startup (tidak menunggu FSM)
    // always_on=false → perilaku lazy lama: subscribe hanya saat /enable dipanggil
    always_on_        = this->declare_parameter<bool>("always_on", true);

    // ---- Init TensorRT engine ----
    if (!initEngine()) {
      RCLCPP_FATAL(this->get_logger(), "Gagal inisialisasi TensorRT engine. Node berhenti.");
      rclcpp::shutdown();
      return;
    }

    // Mode build_only: keluar setelah engine selesai di-build/load
    if (build_only_) {
      RCLCPP_INFO(this->get_logger(),
        "build_only=true: engine berhasil di-build. Node keluar.");
      rclcpp::shutdown();
      return;
    }

    // ---- CUDA stream ----
    cudaStreamCreate(&cuda_stream_);

    // ---- Alokasi GPU buffer ----
    // Input: [1,3,H,W] float32
    const size_t input_elems = 1 * 3 * input_size_ * input_size_;
    cudaMalloc(&gpu_input_, input_elems * sizeof(float));
    cpu_input_.resize(input_elems);

    // Output: [1,5,5376] float32 — TRT output0 shape
    // Jumlah anchor = ceil(input_size/8)*ceil(input_size/8) + ... (YOLOv8 default)
    // Kita baca dari engine supaya tidak hard-code.
    const auto out_dims = trt_->getTensorShape("output0");
    // out_dims harus [1, 5, N] — ambil N dari dim[2]
    if (out_dims.nbDims < 3) {
      RCLCPP_FATAL(this->get_logger(),
        "output0 shape tidak sesuai: nbDims=%d (expected >=3)", out_dims.nbDims);
      rclcpp::shutdown();
      return;
    }
    num_anchors_ = out_dims.d[2];  // 5376 untuk 512-input model
    // Jumlah elemen total output
    const size_t output_elems = static_cast<size_t>(out_dims.d[0]) *
                                static_cast<size_t>(out_dims.d[1]) *
                                static_cast<size_t>(num_anchors_);
    cudaMalloc(&gpu_output_, output_elems * sizeof(float));
    cpu_output_.resize(output_elems);

    RCLCPP_INFO(this->get_logger(),
      "TRT buffers alokasi: input [1,3,%d,%d], output [1,5,%d]",
      input_size_, input_size_, num_anchors_);

    // ---- Publisher utama (nama absolut agar tidak dipengaruhi namespace) ----
    pub_detection_ = this->create_publisher<autoware_road_crossing_msgs::msg::CrosswalkDetection>(
      "/perception/road_crossing/crosswalk_detection", 10);

    // ---- Publisher debug image (relatif — akan masuk namespace node) ----
    if (enable_debug_) {
      pub_debug_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug/image", 1);
    }

    // ---- Service enable/disable (nama absolut) ----
    srv_enable_ = this->create_service<std_srvs::srv::SetBool>(
      "/perception/road_crossing/crosswalk_detector/enable",
      std::bind(
        &CrosswalkDetectorNode::handleEnable, this,
        std::placeholders::_1, std::placeholders::_2));

    // ---- Timer publish utama ----
    const double period = 1.0 / publish_rate_hz_;
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&CrosswalkDetectorNode::publishCb, this));

    // ---- Timer debug (rate-limited) ----
    if (enable_debug_) {
      const double dbg_period = 1.0 / debug_rate_hz_;
      debug_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dbg_period),
        std::bind(&CrosswalkDetectorNode::debugCb, this));
    }

    // ---- Aktifkan subscription kamera ----
    // always_on=true  → langsung subscribe sejak startup (tidak perlu tunggu FSM)
    // always_on=false → perilaku lazy: subscribe hanya saat start_enabled=true atau /enable
    if (always_on_) {
      startDetecting();
      RCLCPP_INFO(this->get_logger(),
        "always_on=true: kamera + YOLO + debug image aktif sejak startup.");
    } else if (start_enabled_) {
      startDetecting();
    }

    RCLCPP_INFO(this->get_logger(),
      "CrosswalkDetectorNode siap | model=%s precision=%s input=%d "
      "conf_thr=%.2f nms_iou=%.2f roi_y_min=%.2f min_area_frac=%.4f "
      "stable=%d rate=%.1f Hz start_enabled=%s always_on=%s",
      model_path_.c_str(), precision_.c_str(), input_size_,
      conf_thr_, nms_iou_, roi_y_min_frac_, min_area_frac_,
      stable_frames_, publish_rate_hz_,
      start_enabled_ ? "true" : "false",
      always_on_ ? "true" : "false");
  }

  ~CrosswalkDetectorNode()
  {
    // Bebaskan CUDA resource
    if (gpu_input_) {
      cudaFree(gpu_input_);
      gpu_input_ = nullptr;
    }
    if (gpu_output_) {
      cudaFree(gpu_output_);
      gpu_output_ = nullptr;
    }
    if (cuda_stream_) {
      cudaStreamDestroy(cuda_stream_);
      cuda_stream_ = nullptr;
    }
  }

private:
  // ------------------------------------------------------------------
  // Init TensorRT via TrtCommon
  // ------------------------------------------------------------------

  bool initEngine()
  {
    // TrtCommonConfig(onnx_path, precision, engine_path="")
    // engine_path kosong → TrtCommon otomatis derive dari onnx_path (ganti ext ke .engine)
    autoware::tensorrt_common::TrtCommonConfig trt_config(model_path_, precision_);

    RCLCPP_INFO(this->get_logger(),
      "Membangun/memuat TRT engine: %s (precision=%s)",
      model_path_.c_str(), precision_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "Engine akan di-cache di: %s", trt_config.engine_path.string().c_str());

    trt_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(trt_config);

    // setup() membangun engine dari ONNX (pertama kali) atau memuat dari cache
    // Parameter default nullptr = tanpa profil dinamis (fixed shape [1,3,512,512])
    if (!trt_->setup()) {
      RCLCPP_ERROR(this->get_logger(), "TrtCommon::setup() gagal.");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "TRT engine siap. Precision: %s", trt_->getPrecision().c_str());
    return true;
  }

  // ------------------------------------------------------------------
  // Subscribe/unsubscribe kamera (helper internal)
  // ------------------------------------------------------------------

  // Buat subscription dengan sensor QoS best-effort (dipanggil satu kali)
  void createImageSubscription()
  {
    if (img_sub_) return;  // sudah terdaftar
    rclcpp::QoS sensor_qos(1);
    sensor_qos.best_effort();
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, sensor_qos,
      std::bind(&CrosswalkDetectorNode::imageCb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
      "CrosswalkDetector: subscribe kamera → %s", camera_topic_.c_str());
  }

  void startDetecting()
  {
    if (enabled_) return;
    enabled_ = true;

    // Buat subscription jika belum ada.
    // Mode always_on: dipanggil dari constructor → subscription dibuat di sini.
    //   Saat FSM memanggil /enable lagi, guard 'if (img_sub_) return' di dalam
    //   createImageSubscription() mencegah double-subscribe.
    // Mode lazy: dipanggil dari constructor (start_enabled=true) atau /enable;
    //   stopDetecting() sudah mereset img_sub_ sehingga subscription dibuat ulang.
    createImageSubscription();

    RCLCPP_INFO(this->get_logger(), "CrosswalkDetector ENABLED (always_on=%s)",
      always_on_ ? "true" : "false");
  }

  void stopDetecting()
  {
    if (!enabled_) return;
    enabled_ = false;

    if (always_on_) {
      // Mode always_on: JANGAN hancurkan subscription atau reset frame —
      // kamera + inferensi tetap berjalan; hanya flag enabled_ yang berubah
      RCLCPP_INFO(this->get_logger(),
        "CrosswalkDetector DISABLED (flag saja, always_on=true — inferensi tetap jalan).");
      return;
    }

    // Mode lazy: hancurkan subscription dan reset state deteksi
    if (img_sub_) {
      img_sub_.reset();
    }
    last_frame_.release();
    raw_det_buf_.clear();
    committed_detected_ = false;
    committed_conf_ = 0.0f;
    committed_boxes_.clear();

    RCLCPP_INFO(this->get_logger(), "CrosswalkDetector DISABLED — subscription dihancurkan.");
  }

  // ------------------------------------------------------------------
  // Service callback enable/disable
  // ------------------------------------------------------------------

  void handleEnable(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    if (req->data) {
      if (!enabled_) startDetecting();
      res->success = true;
      res->message = "crosswalk_detector enabled";
    } else {
      if (enabled_) stopDetecting();
      res->success = true;
      res->message = "crosswalk_detector disabled";
    }
  }

  // ------------------------------------------------------------------
  // Image callback — preprocessing + inference + decode + NMS
  // ------------------------------------------------------------------

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // Mode lazy: hanya proses saat enabled_ (subscription sudah dihancurkan saat disabled,
    // tapi guard ini sebagai lapisan kedua).
    // Mode always_on: selalu proses — guard dilewati.
    if (!always_on_ && !enabled_) return;

    // Konversi ke BGR cv::Mat via cv_bridge
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    // Simpan frame untuk debug image timer
    {
      last_frame_ = cv_ptr->image.clone();
      last_header_ = msg->header;
    }

    const int img_w = last_frame_.cols;
    const int img_h = last_frame_.rows;

    // ---- Pre-processing: letterbox → RGB → CHW → /255 ----
    float scale_r = 0.0f;
    int pad_left = 0, pad_top = 0;
    preprocess(last_frame_, cpu_input_.data(), scale_r, pad_left, pad_top);

    // Copy input ke GPU
    const size_t input_bytes = cpu_input_.size() * sizeof(float);
    cudaMemcpyAsync(gpu_input_, cpu_input_.data(), input_bytes,
      cudaMemcpyHostToDevice, cuda_stream_);

    // ---- Inference via TrtCommon ----
    // Set tensor address berdasarkan nama (API by-name dari TrtCommon)
    trt_->setTensorAddress("images",  gpu_input_);
    trt_->setTensorAddress("output0", gpu_output_);

    if (!trt_->enqueueV3(cuda_stream_)) {
      RCLCPP_WARN(this->get_logger(), "TrtCommon::enqueueV3() gagal.");
      return;
    }

    // Copy output ke CPU
    const size_t output_bytes = cpu_output_.size() * sizeof(float);
    cudaMemcpyAsync(cpu_output_.data(), gpu_output_, output_bytes,
      cudaMemcpyDeviceToHost, cuda_stream_);
    cudaStreamSynchronize(cuda_stream_);

    // ---- Post-processing: decode + NMS ----
    // Output shape: [1, 5, N] → akses sebagai [5][N]
    // Di CPU: cpu_output_[ch * N + i] = output[0][ch][i]
    const int N = num_anchors_;
    std::vector<cv::Rect> boxes_cv;
    std::vector<float> scores_cv;

    for (int i = 0; i < N; ++i) {
      // Baca [cx, cy, w, h, conf] dari layout [5, N]
      const float cx   = cpu_output_[0 * N + i];
      const float cy   = cpu_output_[1 * N + i];
      const float bw   = cpu_output_[2 * N + i];
      const float bh   = cpu_output_[3 * N + i];
      const float conf = cpu_output_[4 * N + i];

      if (conf < conf_thr_) continue;

      // xywh → xyxy dalam 512-space
      float x1_512 = cx - bw / 2.0f;
      float y1_512 = cy - bh / 2.0f;
      float x2_512 = cx + bw / 2.0f;
      float y2_512 = cy + bh / 2.0f;

      // Undo letterbox: kurangi padding, bagi dengan scale
      float x1_orig = (x1_512 - static_cast<float>(pad_left)) / scale_r;
      float y1_orig = (y1_512 - static_cast<float>(pad_top))  / scale_r;
      float x2_orig = (x2_512 - static_cast<float>(pad_left)) / scale_r;
      float y2_orig = (y2_512 - static_cast<float>(pad_top))  / scale_r;

      // Clip ke dimensi image asli
      x1_orig = std::max(0.0f, std::min(x1_orig, static_cast<float>(img_w)));
      y1_orig = std::max(0.0f, std::min(y1_orig, static_cast<float>(img_h)));
      x2_orig = std::max(0.0f, std::min(x2_orig, static_cast<float>(img_w)));
      y2_orig = std::max(0.0f, std::min(y2_orig, static_cast<float>(img_h)));

      // Simpan untuk NMS (cv::dnn::NMSBoxes pakai cv::Rect)
      const int bx = static_cast<int>(x1_orig);
      const int by = static_cast<int>(y1_orig);
      const int bwi = static_cast<int>(x2_orig - x1_orig);
      const int bhi = static_cast<int>(y2_orig - y1_orig);

      if (bwi <= 0 || bhi <= 0) continue;

      boxes_cv.emplace_back(bx, by, bwi, bhi);
      scores_cv.push_back(conf);
    }

    // NMS menggunakan OpenCV DNN
    std::vector<int> nms_indices;
    if (!boxes_cv.empty()) {
      cv::dnn::NMSBoxes(boxes_cv, scores_cv, conf_thr_, nms_iou_, nms_indices);
    }

    // Konversi ke CrosswalkBox (xyxy + conf) sudah dalam koordinat image asli
    std::vector<CrosswalkBox> raw_boxes;
    raw_boxes.reserve(nms_indices.size());
    for (int idx : nms_indices) {
      CrosswalkBox b;
      b.x1   = static_cast<float>(boxes_cv[idx].x);
      b.y1   = static_cast<float>(boxes_cv[idx].y);
      b.x2   = static_cast<float>(boxes_cv[idx].x + boxes_cv[idx].width);
      b.y2   = static_cast<float>(boxes_cv[idx].y + boxes_cv[idx].height);
      b.conf = scores_cv[idx];
      raw_boxes.push_back(b);
    }

    // ---- Decision gate ----
    // Kandidat: conf≥thr (sudah tersaring) AND cy_center/imgH ≥ roi_y_min_frac
    //           AND area/(W*H) ≥ min_area_frac
    const float img_area = static_cast<float>(img_w * img_h);
    std::vector<CrosswalkBox> candidates;
    for (const auto & b : raw_boxes) {
      const float cy_center = (b.y1 + b.y2) * 0.5f / static_cast<float>(img_h);
      const float bw_f = b.x2 - b.x1;
      const float bh_f = b.y2 - b.y1;
      const float area_frac = (bw_f * bh_f) / img_area;

      if (cy_center >= roi_y_min_frac_ && area_frac >= min_area_frac_) {
        candidates.push_back(b);
      }
    }

    const bool raw_detected = !candidates.empty();

    // ---- Temporal smoothing (stable_frames consecutive) ----
    raw_det_buf_.push_back(raw_detected);
    if (static_cast<int>(raw_det_buf_.size()) > stable_frames_) {
      raw_det_buf_.pop_front();
    }

    if (static_cast<int>(raw_det_buf_.size()) == stable_frames_) {
      const bool all_true  = std::all_of(raw_det_buf_.begin(), raw_det_buf_.end(),
                               [](bool v) { return v; });
      const bool all_false = std::all_of(raw_det_buf_.begin(), raw_det_buf_.end(),
                               [](bool v) { return !v; });

      if (all_true) {
        committed_detected_ = true;
      } else if (all_false) {
        committed_detected_ = false;
      }
      // Kondisi campuran → pertahankan committed_detected_ sebelumnya (histeresis)
    }

    // Simpan candidates + max-conf untuk digunakan oleh publish timer
    committed_boxes_ = candidates;
    committed_conf_ = 0.0f;
    for (const auto & b : candidates) {
      committed_conf_ = std::max(committed_conf_, b.conf);
    }
  }

  // ------------------------------------------------------------------
  // Pre-processing: letterbox → RGB → CHW → /255
  // (persis seperti onnx_infer.py — letterbox dengan pad 114, aspect-preserving)
  // ------------------------------------------------------------------

  void preprocess(
    const cv::Mat & bgr,
    float * input_blob,
    float & out_scale_r,
    int & out_pad_left,
    int & out_pad_top)
  {
    const int H = bgr.rows;
    const int W = bgr.cols;
    const int sz = input_size_;

    // Hitung scale dan offset padding
    const float r = std::min(
      static_cast<float>(sz) / static_cast<float>(H),
      static_cast<float>(sz) / static_cast<float>(W));
    const int nw = static_cast<int>(std::round(static_cast<float>(W) * r));
    const int nh = static_cast<int>(std::round(static_cast<float>(H) * r));
    const int dw = (sz - nw) / 2;
    const int dh = (sz - nh) / 2;

    out_scale_r  = r;
    out_pad_left = dw;
    out_pad_top  = dh;

    // Resize dengan INTER_LINEAR
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);

    // Buat canvas abu-abu 114
    cv::Mat canvas(sz, sz, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(dw, dh, nw, nh)));

    // BGR → RGB (YOLOv8 ditraining dengan RGB)
    cv::Mat rgb;
    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

    // HWC → CHW, normalize /255.0
    rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

    std::vector<cv::Mat> channels(3);
    cv::split(rgb, channels);
    for (int c = 0; c < 3; ++c) {
      std::memcpy(
        input_blob + c * sz * sz,
        channels[c].data,
        static_cast<size_t>(sz) * static_cast<size_t>(sz) * sizeof(float));
    }
  }

  // ------------------------------------------------------------------
  // Timer publish utama — publish CrosswalkDetection dari committed state
  // ------------------------------------------------------------------

  void publishCb()
  {
    autoware_road_crossing_msgs::msg::CrosswalkDetection msg;
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = "camera_front";
    msg.detected    = committed_detected_;
    msg.confidence  = committed_conf_;

    // Isi boxes[] (geometry_msgs/Polygon per kandidat, 4 sudut bbox)
    if (committed_detected_) {
      for (const auto & b : committed_boxes_) {
        geometry_msgs::msg::Polygon poly;
        geometry_msgs::msg::Point32 p;

        // Sudut kiri-atas
        p.x = b.x1; p.y = b.y1; p.z = 0.0f; poly.points.push_back(p);
        // Sudut kanan-atas
        p.x = b.x2; p.y = b.y1; p.z = 0.0f; poly.points.push_back(p);
        // Sudut kanan-bawah
        p.x = b.x2; p.y = b.y2; p.z = 0.0f; poly.points.push_back(p);
        // Sudut kiri-bawah
        p.x = b.x1; p.y = b.y2; p.z = 0.0f; poly.points.push_back(p);

        msg.boxes.push_back(poly);
      }
    }

    // zebra_midline: midpoint sisi bawah dari kandidat TERLUAS → normalized 0..1
    // Format: [x1_n, y1_n, x2_n, y2_n] — garis horizontal pendek di sisi bawah
    msg.zebra_midline = {0.0f, 0.0f, 0.0f, 0.0f};
    if (committed_detected_ && !committed_boxes_.empty() && !last_frame_.empty()) {
      const int img_w = last_frame_.cols;
      const int img_h = last_frame_.rows;

      // Cari kandidat dengan area terbesar
      const CrosswalkBox * largest = nullptr;
      float max_area = 0.0f;
      for (const auto & b : committed_boxes_) {
        const float area = (b.x2 - b.x1) * (b.y2 - b.y1);
        if (area > max_area) {
          max_area = area;
          largest = &b;
        }
      }

      if (largest != nullptr) {
        const float mid_x = (largest->x1 + largest->x2) * 0.5f;
        const float bot_y = largest->y2;

        // Garis horizontal pendek (setengah lebar box) di midpoint sisi bawah
        const float half_w = (largest->x2 - largest->x1) * 0.25f;
        msg.zebra_midline[0] = std::max(0.0f, (mid_x - half_w) / static_cast<float>(img_w));
        msg.zebra_midline[1] = bot_y / static_cast<float>(img_h);
        msg.zebra_midline[2] = std::min(1.0f, (mid_x + half_w) / static_cast<float>(img_w));
        msg.zebra_midline[3] = bot_y / static_cast<float>(img_h);
      }
    }

    pub_detection_->publish(msg);
  }

  // ------------------------------------------------------------------
  // Timer debug image (rate-limited ke debug_rate_hz)
  // ------------------------------------------------------------------

  void debugCb()
  {
    if (!enable_debug_ || !pub_debug_ || last_frame_.empty()) return;
    // Mode always_on: publish tanpa memeriksa jumlah subscriber agar topic selalu
    // terlihat di ros2 topic hz dan rqt (bandwidth kecil, aman untuk debug).
    // Mode lazy: tetap pakai guard subscription-count untuk hemat bandwidth.
    if (!always_on_ && pub_debug_->get_subscription_count() == 0) return;

    cv::Mat dbg = last_frame_.clone();
    const int img_h = dbg.rows;
    const int img_w = dbg.cols;

    // Gambar garis ROI threshold (roi_y_min_frac)
    const int roi_line_y = static_cast<int>(roi_y_min_frac_ * static_cast<float>(img_h));
    cv::line(dbg, cv::Point(0, roi_line_y), cv::Point(img_w, roi_line_y),
      cv::Scalar(255, 128, 0), 1, cv::LINE_AA);
    cv::putText(dbg, "roi_y_min", cv::Point(4, roi_line_y - 4),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 128, 0), 1);

    // Gambar semua bbox kandidat
    // Warna: hijau jika committed_detected_=true, kuning jika false
    const cv::Scalar committed_color(0, 255, 0);    // hijau — committed true
    const cv::Scalar raw_color(0, 215, 255);          // kuning — kandidat, belum commit

    for (const auto & b : committed_boxes_) {
      const cv::Scalar & color = committed_detected_ ? committed_color : raw_color;
      cv::rectangle(dbg,
        cv::Point(static_cast<int>(b.x1), static_cast<int>(b.y1)),
        cv::Point(static_cast<int>(b.x2), static_cast<int>(b.y2)),
        color, 2);

      // Label confidence
      const std::string label = std::to_string(static_cast<int>(b.conf * 100)) + "%";
      cv::putText(dbg, label,
        cv::Point(static_cast<int>(b.x1), std::max(0, static_cast<int>(b.y1) - 5)),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    // Status bar
    const std::string status_str =
      std::string("detected=") + (committed_detected_ ? "TRUE" : "false") +
      " conf=" + std::to_string(static_cast<int>(committed_conf_ * 100)) + "%";
    cv::putText(dbg, status_str, cv::Point(4, 18),
      cv::FONT_HERSHEY_SIMPLEX, 0.5,
      committed_detected_ ? committed_color : cv::Scalar(80, 80, 80), 1);

    // Publish debug image (BGR8)
    auto debug_msg = cv_bridge::CvImage(last_header_, "bgr8", dbg).toImageMsg();
    pub_debug_->publish(*debug_msg);
  }

  // ------------------------------------------------------------------
  // Member variables
  // ------------------------------------------------------------------

  // Parameter
  std::string model_path_;
  std::string precision_;
  int         input_size_{512};
  float       conf_thr_{0.4f};
  float       nms_iou_{0.5f};
  std::string camera_topic_;
  float       roi_y_min_frac_{0.35f};
  float       min_area_frac_{0.01f};
  int         stable_frames_{3};
  double      publish_rate_hz_{10.0};
  bool        enable_debug_{true};
  double      debug_rate_hz_{2.0};
  bool        start_enabled_{false};
  bool        build_only_{false};
  // always_on=true  → kamera + YOLO + debug image selalu aktif (default)
  // always_on=false → perilaku lazy lama (hemat GPU saat tidak digunakan)
  bool        always_on_{true};

  // TensorRT engine (TrtCommon wrapper)
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_;

  // CUDA resource
  cudaStream_t cuda_stream_{nullptr};
  void *       gpu_input_{nullptr};
  void *       gpu_output_{nullptr};

  // CPU buffer (pinned tidak wajib — pakai std::vector untuk kesederhanaan)
  std::vector<float> cpu_input_;
  std::vector<float> cpu_output_;
  int num_anchors_{5376};  // diisi dari shape engine saat init

  // State deteksi
  bool        enabled_{false};
  cv::Mat     last_frame_;
  std_msgs::msg::Header last_header_;

  // Temporal smoothing
  std::deque<bool>      raw_det_buf_;
  bool                  committed_detected_{false};
  float                 committed_conf_{0.0f};
  std::vector<CrosswalkBox> committed_boxes_;

  // ROS interfaces
  rclcpp::Publisher<autoware_road_crossing_msgs::msg::CrosswalkDetection>::SharedPtr pub_detection_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

// ============================================================
// main
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrosswalkDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
