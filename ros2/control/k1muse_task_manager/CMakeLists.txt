cmake_minimum_required(VERSION 3.8)
project(k1muse_task_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(k1muse_common REQUIRED)
find_package(k1muse_voice_msgs REQUIRED)
find_package(k1muse_manager_msgs REQUIRED)

# --- Library ---
add_library(${PROJECT_NAME}_lib SHARED
  src/task_manager_node.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  k1muse_common
  k1muse_voice_msgs
  k1muse_manager_msgs
)

# --- Executable ---
add_executable(task_manager_node src/main.cpp)
target_link_libraries(task_manager_node ${PROJECT_NAME}_lib)
ament_target_dependencies(task_manager_node
  rclcpp
)

# --- Install ---
install(TARGETS ${PROJECT_NAME}_lib task_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp rclcpp_action k1muse_common k1muse_voice_msgs k1muse_manager_msgs)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  # Pure logic test (no ROS spin needed)
  ament_add_gtest(test_task_mapping test/test_task_mapping.cpp)
  target_include_directories(test_task_mapping PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  # Integration test with mock action server
  ament_add_gtest(test_task_manager_node test/test_task_manager_node.cpp)
  target_include_directories(test_task_manager_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_task_manager_node ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_task_manager_node
    rclcpp
    rclcpp_action
    k1muse_common
    k1muse_voice_msgs
    k1muse_manager_msgs
  )
endif()

ament_package()
