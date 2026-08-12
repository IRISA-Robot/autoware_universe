cmake_minimum_required(VERSION 3.14)
project(autoware_crosswalk_detector)

# ---- Autoware cmake helper (ament_auto_find_build_dependencies, dll.) ----
find_package(autoware_cmake REQUIRED)
autoware_package()

# ---- Deteksi ketersediaan CUDA ----
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  message(STATUS "[crosswalk_detector] CUDA ditemukan: ${CUDA_TOOLKIT_ROOT_DIR}")
  set(CUDA_AVAIL ON)
else()
  message(WARNING "[crosswalk_detector] CUDA TIDAK ditemukan — node tidak akan di-build.")
endif()

# ---- Deteksi ketersediaan TensorRT ----
option(TRT_AVAIL "TensorRT available" OFF)
find_library(NVINFER        NAMES nvinfer)
find_library(NVONNXPARSER   NAMES nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  message(STATUS "[crosswalk_detector] TensorRT ditemukan.")
  set(TRT_AVAIL ON)
else()
  message(WARNING "[crosswalk_detector] TensorRT TIDAK ditemukan — node tidak akan di-build.")
endif()

# ---- Deteksi cuDNN ----
option(CUDNN_AVAIL "cuDNN available" OFF)
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so libcudnn.dylib cudnn
  PATHS $ENV{LD_LIBRARY_PATH} ${CUDA_TOOLKIT_ROOT_DIR}/lib64
  PATH_SUFFIXES lib lib64 bin)
if(CUDNN_LIBRARY)
  message(STATUS "[crosswalk_detector] cuDNN ditemukan: ${CUDNN_LIBRARY}")
  set(CUDNN_AVAIL ON)
else()
  message(WARNING "[crosswalk_detector] cuDNN TIDAK ditemukan — node tidak akan di-build.")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  add_definitions(-DENABLE_GPU)

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  # ---- Build node executable ----
  ament_auto_add_executable(crosswalk_detector_node
    src/crosswalk_detector_node.cpp
  )
  target_link_libraries(crosswalk_detector_node
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUDNN_LIBRARY}
    stdc++fs
  )

  install(TARGETS crosswalk_detector_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "[crosswalk_detector] CUDA/TensorRT/cuDNN tidak tersedia. "
    "Node crosswalk_detector_node TIDAK di-build."
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
