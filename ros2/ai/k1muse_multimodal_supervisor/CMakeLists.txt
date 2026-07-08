cmake_minimum_required(VERSION 3.8)
project(k1muse_multimodal_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(k1muse_common REQUIRED)
find_package(k1muse_ai_runtime_msgs REQUIRED)
find_package(k1muse_voice_msgs REQUIRED)
find_package(k1muse_vision_msgs REQUIRED)
find_package(k1muse_manager_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Pure-C++ libraries (no ROS dependency)
# ---------------------------------------------------------------------------
add_library(interaction_state_machine SHARED
  src/interaction_state_machine.cpp
)
target_include_directories(interaction_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(interaction_state_machine PUBLIC cxx_std_17)

add_library(runtime_control_builder SHARED
  src/runtime_control_builder.cpp
)
target_include_directories(runtime_control_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(runtime_control_builder PUBLIC interaction_state_machine)
target_compile_features(runtime_control_builder PUBLIC cxx_std_17)

add_library(target_cache SHARED
  src/target_cache.cpp
)
target_include_directories(target_cache PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(target_cache PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# ROS2 node library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_node SHARED
  src/multimodal_supervisor_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node
  interaction_state_machine
  runtime_control_builder
  target_cache
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  k1muse_common
  k1muse_ai_runtime_msgs
  k1muse_voice_msgs
  k1muse_vision_msgs
  k1muse_manager_msgs
)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(supervisor_node_exe src/main.cpp)
target_link_libraries(supervisor_node_exe PRIVATE ${PROJECT_NAME}_node)
set_target_properties(supervisor_node_exe PROPERTIES OUTPUT_NAME supervisor_node)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS interaction_state_machine runtime_control_builder target_cache
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS supervisor_node_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Pure-C++ tests (no ROS)
  ament_add_gtest(test_interaction_state_machine
    test/test_interaction_state_machine.cpp
  )
  target_link_libraries(test_interaction_state_machine interaction_state_machine)

  ament_add_gtest(test_runtime_control_builder
    test/test_runtime_control_builder.cpp
  )
  target_link_libraries(test_runtime_control_builder runtime_control_builder)

  ament_add_gtest(test_target_cache
    test/test_target_cache.cpp
  )
  target_link_libraries(test_target_cache target_cache)

  # ROS2 node tests
  ament_add_gtest(test_supervisor_node
    test/test_supervisor_node.cpp
  )
  target_link_libraries(test_supervisor_node ${PROJECT_NAME}_node)

  ament_add_gtest(test_voice_flow_integration
    test/test_voice_flow_integration.cpp
  )
  target_link_libraries(test_voice_flow_integration ${PROJECT_NAME}_node)

  ament_add_gtest(test_control_contract
    test/test_control_contract.cpp
  )
  target_link_libraries(test_control_contract ${PROJECT_NAME}_node)
endif()

ament_package()
