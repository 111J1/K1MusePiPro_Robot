cmake_minimum_required(VERSION 3.8)
project(k1muse_voice_audio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(k1muse_voice_msgs REQUIRED)
find_package(k1muse_audio_msgs REQUIRED)
find_package(k1muse_common REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Threads REQUIRED)

# PortAudio (SpacemiT SDK)
find_library(PORTAUDIO_LIB portaudio
  HINTS /home/bianbu/AI_SDK/ai-sdk/output/staging/lib
  NO_DEFAULT_PATH)
if(NOT PORTAUDIO_LIB)
  find_library(PORTAUDIO_LIB portaudio)
endif()
set(PORTAUDIO_INCLUDE_DIR /home/bianbu/AI_SDK/ai-sdk/output/staging/include)

add_library(k1muse_voice_audio_core
  src/audio_io_node.cpp
  src/alsa_backend.cpp
  src/fake_audio_backend.cpp
  src/mock_audio_device.cpp
  src/portaudio_backend.cpp
  src/portaudio_device.cpp
)
target_link_libraries(k1muse_voice_audio_core Threads::Threads ${PORTAUDIO_LIB} asound)
target_include_directories(k1muse_voice_audio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PORTAUDIO_INCLUDE_DIR}
)
ament_target_dependencies(k1muse_voice_audio_core
  rclcpp
  rclcpp_lifecycle
  k1muse_voice_msgs
  k1muse_audio_msgs
  k1muse_common
  lifecycle_msgs
)

add_executable(audio_io_node src/main.cpp)
target_link_libraries(audio_io_node k1muse_voice_audio_core)
ament_target_dependencies(audio_io_node rclcpp rclcpp_lifecycle)

install(TARGETS k1muse_voice_audio_core audio_io_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(k1muse_voice_audio_core)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  k1muse_voice_msgs
  k1muse_audio_msgs
  k1muse_common
  lifecycle_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_audio_playback_tracker test/test_audio_playback_tracker.cpp)
  target_link_libraries(test_audio_playback_tracker k1muse_voice_audio_core)
  ament_target_dependencies(test_audio_playback_tracker
    k1muse_voice_msgs
    k1muse_common
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_package()
