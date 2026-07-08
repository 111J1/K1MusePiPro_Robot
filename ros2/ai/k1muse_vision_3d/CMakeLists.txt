cmake_minimum_required(VERSION 3.8)
project(k1muse_vision_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(k1muse_common REQUIRED)
find_package(k1muse_vision_msgs REQUIRED)

# Pure C++ geometry library (no ROS dependency).
add_library(${PROJECT_NAME}_geometry SHARED src/geometry.cpp)
target_include_directories(${PROJECT_NAME}_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_geometry PUBLIC cxx_std_17)

# Vision3DNode library (ROS-dependent).
add_library(${PROJECT_NAME}_node SHARED src/vision_3d_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_geometry)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  k1muse_common
  k1muse_vision_msgs
)

# Vision3DNode executable.
add_executable(vision_3d_node src/main.cpp)
target_link_libraries(vision_3d_node PRIVATE ${PROJECT_NAME}_node)

install(TARGETS ${PROJECT_NAME}_geometry ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS vision_3d_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry ${PROJECT_NAME}_geometry)

  ament_add_gtest(test_vision_3d_node test/test_vision_3d_node.cpp)
  target_link_libraries(test_vision_3d_node ${PROJECT_NAME}_node)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  k1muse_common
  k1muse_vision_msgs
)

ament_package()
