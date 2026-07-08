cmake_minimum_required(VERSION 3.8)
project(k1muse_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodeReady.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Interface library for hand-written headers (qos_profiles.hpp, id_utils.hpp)
# rosidl automatically sets include/k1muse_common for generated message headers,
# but utility headers are in include/ and need the parent directory.
add_library(${PROJECT_NAME}_headers INTERFACE)
target_include_directories(${PROJECT_NAME}_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS ${PROJECT_NAME}_headers
  EXPORT export_${PROJECT_NAME}_headers
)
ament_export_targets(export_${PROJECT_NAME}_headers HAS_LIBRARY_TARGET)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories("include")
ament_export_dependencies(rclcpp rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_qos_profiles test/test_qos_profiles.cpp)
  target_include_directories(test_qos_profiles PRIVATE include)
  ament_target_dependencies(test_qos_profiles rclcpp)

  ament_add_gtest(test_id_utils test/test_id_utils.cpp)
  target_include_directories(test_id_utils PRIVATE include)
endif()

ament_package()
