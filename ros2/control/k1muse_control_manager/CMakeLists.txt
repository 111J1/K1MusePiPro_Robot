cmake_minimum_required(VERSION 3.8)
project(k1muse_control_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(k1muse_common REQUIRED)
find_package(k1muse_control_core REQUIRED)
find_package(k1muse_manager_msgs REQUIRED)
find_package(k1muse_vision_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET YAML::yaml-cpp)
  set(YAML_CPP_TARGET YAML::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# --- Library ---
add_library(${PROJECT_NAME}_lib SHARED
  src/control_manager_node.cpp
  src/device_session.cpp
  src/execution_policy.cpp
  src/grasp_profile_store.cpp
  src/grasp_task_executor.cpp
  src/mock_task_executor.cpp
  src/serial_transport.cpp
  src/stm32_actuator_client.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib
  ${YAML_CPP_TARGET}
)
ament_target_dependencies(${PROJECT_NAME}_lib
  ament_index_cpp
  rclcpp
  rclcpp_action
  sensor_msgs
  k1muse_common
  k1muse_control_core
  k1muse_manager_msgs
  k1muse_vision_msgs
)

# --- Executable ---
add_executable(control_manager_node src/main.cpp)
target_link_libraries(control_manager_node ${PROJECT_NAME}_lib)
ament_target_dependencies(control_manager_node
  rclcpp
)

# --- Install ---
install(TARGETS ${PROJECT_NAME}_lib control_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(ament_index_cpp rclcpp rclcpp_action sensor_msgs k1muse_common k1muse_control_core k1muse_manager_msgs k1muse_vision_msgs)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure logic test (no ROS spin needed)
  ament_add_gtest(test_mock_task_executor test/test_mock_task_executor.cpp)
  target_include_directories(test_mock_task_executor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_mock_task_executor ${PROJECT_NAME}_lib)

  ament_add_gtest(test_grasp_profile_store test/test_grasp_profile_store.cpp)
  target_include_directories(test_grasp_profile_store PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_definitions(test_grasp_profile_store PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(test_grasp_profile_store ${PROJECT_NAME}_lib)

  # Integration test with action server
  ament_add_gtest(test_control_manager_node test/test_control_manager_node.cpp)
  target_include_directories(test_control_manager_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_control_manager_node ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_control_manager_node
    ament_index_cpp
    rclcpp
    rclcpp_action
    sensor_msgs
    k1muse_common
    k1muse_control_core
    k1muse_manager_msgs
    k1muse_vision_msgs
  )
endif()

ament_package()
