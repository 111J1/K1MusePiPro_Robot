cmake_minimum_required(VERSION 3.8)
project(k1muse_control_core C)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/algorithm/crc8.c
  src/protocol/control_protocol.c
  src/device/actuator_protocol.c
  src/module/grasp_profile.c
  src/module/placement_planner.c
  src/task/pick_place_task.c
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_11)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(NOT MSVC)
  target_link_libraries(${PROJECT_NAME} m)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_control_core test/test_control_core.c)
  target_link_libraries(test_control_core ${PROJECT_NAME})
  add_test(NAME test_control_core COMMAND test_control_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
