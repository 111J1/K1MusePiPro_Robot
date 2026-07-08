cmake_minimum_required(VERSION 3.8)
project(k1muse_ai_runtime)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

option(K1MUSE_ENABLE_REAL_K1_BACKENDS
  "Build real K1 backend adapters (requires SpacemiT AI SDK, sherpa-onnx)" ON)

find_package(ament_cmake REQUIRED)
find_package(k1muse_ai_runtime_msgs REQUIRED)
find_package(k1muse_audio_msgs REQUIRED)
find_package(k1muse_common REQUIRED)
find_package(k1muse_vision_msgs REQUIRED)
find_package(k1muse_voice_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(k1muse_ai_runtime_core
  src/ai_runtime_node.cpp
  src/backend_factory.cpp
  src/detector_registry.cpp
  src/resource_manager.cpp
  src/audio/audio_frame_validator.cpp
  src/audio/vad_segmenter.cpp
  src/backends/mock_asr_backend.cpp
  src/backends/mock_fire_backend.cpp
  src/backends/mock_pose_backend.cpp
  src/backends/mock_vad_backend.cpp
  src/backends/mock_tts_backend.cpp
  src/backends/mock_vision_backend.cpp
  src/backends/mock_wakeword_backend.cpp
  src/bounded_cpu_worker.cpp
  src/models/wakeword_runtime.cpp
  src/models/vad_asr_runtime.cpp
  src/models/tts_runtime.cpp
  src/models/vision_runtime.cpp
  src/pipelines/voice_pipeline.cpp
  src/pipelines/tts_pipeline.cpp
  src/pipelines/vision_pipeline.cpp
  src/alert_publisher.cpp
  src/alert_ros_publisher.cpp
  src/resource_guard.cpp
  src/runtime_config.cpp
  src/runtime_core.cpp
  src/runtime_profile_manager.cpp
  src/runtime_scheduler.cpp
  src/vision_detector_scheduler.cpp
  src/voice_exclusive_guard.cpp
)
target_link_libraries(k1muse_ai_runtime_core Threads::Threads)
target_include_directories(k1muse_ai_runtime_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(k1muse_ai_runtime_core
  k1muse_ai_runtime_msgs
  k1muse_audio_msgs
  k1muse_common
  k1muse_vision_msgs
  k1muse_voice_msgs
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

if(K1MUSE_ENABLE_REAL_K1_BACKENDS)
  target_compile_definitions(k1muse_ai_runtime_core PRIVATE K1MUSE_ENABLE_REAL_K1_BACKENDS=1)

  # Real backend sources
  target_sources(k1muse_ai_runtime_core PRIVATE
    src/backends/real_sherpa_wakeword_backend.cpp
    src/backends/real_vad_backend.cpp
    src/backends/real_sensevoice_asr_backend.cpp
    src/backends/real_vision_backend.cpp
    src/backends/real_tts_backend.cpp
  )

  # Fixed SDK layout on the K1 board.
  set(AI_SDK_ROOT "/home/bianbu/AI_SDK/ai-sdk")
  set(AI_SDK_STAGING "${AI_SDK_ROOT}/output/staging")
  set(AI_SDK_ASR_SOURCE "${AI_SDK_ROOT}/asr")
  set(SHERPA_DIR "/home/bianbu/AI_SDK/sherpa")

  if(NOT IS_DIRECTORY "${AI_SDK_STAGING}")
    message(FATAL_ERROR "SpacemiT AI SDK staging directory not found: ${AI_SDK_STAGING}")
  endif()
  if(NOT IS_DIRECTORY "${SHERPA_DIR}")
    message(FATAL_ERROR "sherpa-onnx directory not found: ${SHERPA_DIR}")
  endif()

  # VAD
  find_library(SPACEMIT_VAD_LIB vad PATHS "${AI_SDK_STAGING}/lib" NO_DEFAULT_PATH)
  find_library(SILERO_VAD_LIB silero_vad PATHS "${AI_SDK_STAGING}/lib" NO_DEFAULT_PATH)
  # ASR (use libsensevoice directly to avoid kaldifst/OpenFst dependency from libasr)
  find_library(SENSEVOICE_LIB sensevoice PATHS "${AI_SDK_STAGING}/lib" NO_DEFAULT_PATH)
  # TTS and its static FST dependency closure
  find_library(SPACEMIT_TTS_LIB tts PATHS "${AI_SDK_STAGING}/lib" NO_DEFAULT_PATH)
  find_library(KALDIFST_CORE_LIB kaldifst_core PATHS "${AI_SDK_STAGING}/lib" NO_DEFAULT_PATH)
  find_library(KALDIFST_FST_LIB kaldifst_fst PATHS "${AI_SDK_STAGING}/lib" NO_DEFAULT_PATH)
  # OpenFst (system library, required by kaldifst_core)
  find_library(FST_LIB fst)
  # Vision
  find_library(SPACEMIT_VISION_LIB vision
    PATHS "${AI_SDK_STAGING}/lib" NO_DEFAULT_PATH)
  find_library(YAML_CPP_LIB yaml-cpp)

  # sherpa-onnx
  find_library(SHERPA_CXX_LIB sherpa-onnx-cxx-api
    PATHS "${SHERPA_DIR}/lib" NO_DEFAULT_PATH)
  find_library(SHERPA_C_API_LIB sherpa-onnx-c-api
    PATHS "${SHERPA_DIR}/lib" NO_DEFAULT_PATH)
  find_library(CARGS_LIB cargs PATHS "${SHERPA_DIR}/lib" NO_DEFAULT_PATH)
  # System libraries (onnxruntime, spacemit_ep installed via apt)
  find_library(ONNXRUNTIME_LIB onnxruntime
    PATHS /usr/lib /usr/local/lib /usr/lib/riscv64-linux-gnu
          "${SHERPA_DIR}/lib")
  find_library(SPACEMIT_EP_LIB spacemit_ep
    PATHS /usr/lib /usr/local/lib /usr/lib/riscv64-linux-gnu
          "${SHERPA_DIR}/lib")
  find_package(CURL REQUIRED)
  find_library(SNDFILE_LIB sndfile REQUIRED)
  find_library(FFTW3_LIB fftw3 REQUIRED)
  find_library(FFTW3F_LIB fftw3f REQUIRED)

  # OpenCV (for Vision) — prefer SpacemiT 4.14 to match libvision.so
  if(EXISTS "/opt/opencv-spacemit/lib/cmake/opencv4")
    set(OpenCV_DIR "/opt/opencv-spacemit/lib/cmake/opencv4" CACHE PATH "" FORCE)
  endif()
  find_package(OpenCV REQUIRED)

  # Validate libraries exist
  if(NOT SPACEMIT_VAD_LIB)
    message(FATAL_ERROR "SpacemiT VAD library not found at ${AI_SDK_STAGING}/lib/")
  endif()
  if(NOT SILERO_VAD_LIB)
    message(FATAL_ERROR "Silero VAD library not found at ${AI_SDK_STAGING}/lib/")
  endif()
  if(NOT SENSEVOICE_LIB)
    message(FATAL_ERROR "SenseVoice library not found at ${AI_SDK_STAGING}/lib/")
  endif()
  if(NOT SPACEMIT_TTS_LIB)
    message(FATAL_ERROR "SpacemiT TTS library not found at ${AI_SDK_STAGING}/lib/")
  endif()
  if(NOT KALDIFST_CORE_LIB)
    message(FATAL_ERROR "kaldifst_core library not found at ${AI_SDK_STAGING}/lib/")
  endif()
  if(NOT KALDIFST_FST_LIB)
    message(FATAL_ERROR "kaldifst_fst library not found at ${AI_SDK_STAGING}/lib/")
  endif()
  if(NOT SPACEMIT_VISION_LIB)
    message(FATAL_ERROR "SpacemiT Vision library not found at ${AI_SDK_STAGING}/lib/")
  endif()
  if(NOT YAML_CPP_LIB)
    message(FATAL_ERROR "yaml-cpp library required by SpacemiT Vision was not found")
  endif()
  if(NOT SHERPA_CXX_LIB)
    message(FATAL_ERROR "sherpa-onnx CXX API library not found at ${SHERPA_DIR}/lib/")
  endif()
  if(NOT SHERPA_C_API_LIB)
    message(FATAL_ERROR "sherpa-onnx C API library not found at ${SHERPA_DIR}/lib/")
  endif()
  if(NOT CARGS_LIB)
    message(FATAL_ERROR "cargs library required by sherpa-onnx not found at ${SHERPA_DIR}/lib/")
  endif()
  if(NOT ONNXRUNTIME_LIB)
    message(FATAL_ERROR "ONNX Runtime library not found")
  endif()
  if(NOT SPACEMIT_EP_LIB)
    message(FATAL_ERROR "SpaceMIT Execution Provider library not found")
  endif()
  if(NOT FFTW3_LIB)
    message(FATAL_ERROR "FFTW3 library required by SpacemiT TTS not found")
  endif()
  if(NOT FFTW3F_LIB)
    message(FATAL_ERROR "FFTW3F library required by SpacemiT ASR/TTS not found")
  endif()
  if(NOT FST_LIB)
    message(FATAL_ERROR "OpenFst library (libfst) not found. Install with: sudo apt install libfst-dev")
  endif()

  # ASR include path for SenseVoiceModel header
  set(ASR_SRC_DIR "${AI_SDK_ASR_SOURCE}/src")

  # Include directories for real backends
  target_include_directories(k1muse_ai_runtime_core PRIVATE
    "${AI_SDK_STAGING}/include"
    "${ASR_SRC_DIR}"
    "${SHERPA_DIR}/include"
    "${OpenCV_INCLUDE_DIRS}"
  )

  # Link real backend libraries
  target_link_libraries(k1muse_ai_runtime_core
    ${SPACEMIT_VAD_LIB}
    ${SILERO_VAD_LIB}
    ${SENSEVOICE_LIB}
    ${SPACEMIT_TTS_LIB}
    ${KALDIFST_CORE_LIB}
    ${KALDIFST_FST_LIB}
    ${FST_LIB}
    ${SPACEMIT_VISION_LIB}
    ${SHERPA_CXX_LIB}
    ${SHERPA_C_API_LIB}
    ${ONNXRUNTIME_LIB}
    ${SPACEMIT_EP_LIB}
    ${YAML_CPP_LIB}
    ${CARGS_LIB}
    CURL::libcurl
    ${FFTW3_LIB}
    ${FFTW3F_LIB}
    ${SNDFILE_LIB}
    ${OpenCV_LIBS}
  )
endif()

add_executable(ai_runtime_node src/main.cpp)
target_link_libraries(ai_runtime_node k1muse_ai_runtime_core)
ament_target_dependencies(ai_runtime_node rclcpp rclcpp_lifecycle)

if(K1MUSE_ENABLE_REAL_K1_BACKENDS)
  set_target_properties(k1muse_ai_runtime_core ai_runtime_node PROPERTIES
    BUILD_RPATH "${AI_SDK_STAGING}/lib;${SHERPA_DIR}/lib"
    INSTALL_RPATH "${AI_SDK_STAGING}/lib;${SHERPA_DIR}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )
endif()

install(TARGETS k1muse_ai_runtime_core ai_runtime_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(k1muse_ai_runtime_core)
ament_export_dependencies(
  k1muse_ai_runtime_msgs
  k1muse_audio_msgs
  k1muse_common
  k1muse_vision_msgs
  k1muse_voice_msgs
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_audio_frame_validator test/test_audio_frame_validator.cpp)
  target_link_libraries(test_audio_frame_validator k1muse_ai_runtime_core)

  ament_add_gtest(test_vad_segmenter test/test_vad_segmenter.cpp)
  target_link_libraries(test_vad_segmenter k1muse_ai_runtime_core)

  ament_add_gtest(test_runtime_config test/test_runtime_config.cpp)
  ament_add_gtest(test_runtime_scheduler test/test_runtime_scheduler.cpp)
  ament_add_gtest(test_resource_guard test/test_resource_guard.cpp)
  ament_add_gtest(test_ai_runtime_lifecycle test/test_ai_runtime_lifecycle.cpp)

  foreach(test_name test_runtime_config test_runtime_scheduler
      test_resource_guard test_ai_runtime_lifecycle)
    target_link_libraries(${test_name} k1muse_ai_runtime_core)
    ament_target_dependencies(${test_name}
      k1muse_ai_runtime_msgs
      k1muse_audio_msgs
      k1muse_common
      k1muse_voice_msgs
      lifecycle_msgs
      rclcpp
      rclcpp_lifecycle
    )
  endforeach()

  ament_add_gtest(test_vad_asr_runtime test/test_vad_asr_runtime.cpp)
  target_link_libraries(test_vad_asr_runtime k1muse_ai_runtime_core)

  ament_add_gtest(test_latest_frame_buffer test/test_latest_frame_buffer.cpp)
  target_link_libraries(test_latest_frame_buffer Threads::Threads)
  target_include_directories(test_latest_frame_buffer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_vision_runtime test/test_vision_runtime.cpp)
  target_link_libraries(test_vision_runtime k1muse_ai_runtime_core)

  ament_add_gtest(test_tts_runtime test/test_tts_runtime.cpp)
  target_link_libraries(test_tts_runtime k1muse_ai_runtime_core)

  ament_add_gtest(test_detector_registry test/test_detector_registry.cpp)
  target_link_libraries(test_detector_registry k1muse_ai_runtime_core)

  ament_add_gtest(test_resource_manager test/test_resource_manager.cpp)
  target_link_libraries(test_resource_manager k1muse_ai_runtime_core)

  ament_add_gtest(test_vision_pipeline test/test_vision_pipeline.cpp)
  target_link_libraries(test_vision_pipeline k1muse_ai_runtime_core Threads::Threads)

  ament_add_gtest(test_runtime_core test/test_runtime_core.cpp)
  target_link_libraries(test_runtime_core k1muse_ai_runtime_core)
endif()

ament_package()
