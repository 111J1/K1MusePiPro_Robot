cmake_minimum_required(VERSION 3.8)
project(k1muse_mock_devices)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(k1muse_audio_msgs REQUIRED)
find_package(k1muse_common REQUIRED)

# --- AudioScenario library (no ROS node, linkable from tests) ---
add_library(audio_scenario_lib src/audio_scenario_node.cpp)
target_include_directories(audio_scenario_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(audio_scenario_lib rclcpp k1muse_audio_msgs k1muse_common)

# --- CameraScenario library (no ROS node, linkable from tests) ---
add_library(camera_scenario_lib src/camera_scenario_main.cpp)
target_include_directories(camera_scenario_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_scenario_lib sensor_msgs k1muse_common)

# --- DepthScenario library (no ROS node, linkable from tests) ---
add_library(depth_scenario_lib src/depth_scenario_main.cpp)
target_include_directories(depth_scenario_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(depth_scenario_lib sensor_msgs k1muse_common)

# --- audio_scenario_node executable ---
add_executable(audio_scenario_node src/audio_scenario_main.cpp)
target_link_libraries(audio_scenario_node audio_scenario_lib)
ament_target_dependencies(audio_scenario_node
  rclcpp
  k1muse_audio_msgs
  k1muse_common
)

# --- camera_scenario_node executable ---
add_executable(camera_scenario_node src/camera_scenario_node.cpp)
target_link_libraries(camera_scenario_node camera_scenario_lib)
ament_target_dependencies(camera_scenario_node
  rclcpp
  sensor_msgs
  k1muse_common
)

# --- depth_camera_scenario_node executable ---
add_executable(depth_camera_scenario_node src/depth_camera_scenario_node.cpp)
target_link_libraries(depth_camera_scenario_node depth_scenario_lib)
ament_target_dependencies(depth_camera_scenario_node
  rclcpp
  sensor_msgs
  k1muse_common
)

# --- Install ---
install(TARGETS
    audio_scenario_lib
    camera_scenario_lib
    depth_scenario_lib
    audio_scenario_node
    camera_scenario_node
    depth_camera_scenario_node
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

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_audio_scenario test/test_audio_scenario.cpp)
  target_link_libraries(test_audio_scenario audio_scenario_lib)

  ament_add_gtest(test_camera_scenario test/test_camera_scenario.cpp)
  target_link_libraries(test_camera_scenario camera_scenario_lib)
  ament_target_dependencies(test_camera_scenario sensor_msgs)

  ament_add_gtest(test_depth_scenario test/test_depth_scenario.cpp)
  target_link_libraries(test_depth_scenario depth_scenario_lib)
  ament_target_dependencies(test_depth_scenario sensor_msgs)
endif()

ament_package()
