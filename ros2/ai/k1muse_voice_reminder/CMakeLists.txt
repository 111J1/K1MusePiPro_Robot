cmake_minimum_required(VERSION 3.8)
project(k1muse_voice_reminder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(k1muse_common REQUIRED)
find_package(k1muse_voice_msgs REQUIRED)

# Find sqlite3
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SQLITE3 REQUIRED sqlite3)
else()
  find_path(SQLITE3_INCLUDE_DIRS sqlite3.h)
  find_library(SQLITE3_LIBRARIES NAMES sqlite3)
  if(NOT SQLITE3_LIBRARIES)
    message(FATAL_ERROR "sqlite3 not found")
  endif()
endif()

# Pure logic library (parser, store, handler) — no ROS deps.
set(REMINDER_LIB_SOURCES
  src/reminder_parser.cpp
  src/reminder_store.cpp
  src/reminder_handler.cpp
)

add_library(${PROJECT_NAME} SHARED ${REMINDER_LIB_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE ${SQLITE3_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${SQLITE3_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  k1muse_voice_msgs
)

# Node library (ROS deps).
add_library(${PROJECT_NAME}_node SHARED src/reminder_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  k1muse_common
  k1muse_voice_msgs
)

# Executable.
add_executable(reminder_node_exe src/main.cpp)
target_link_libraries(reminder_node_exe PRIVATE ${PROJECT_NAME}_node)
set_target_properties(reminder_node_exe PROPERTIES OUTPUT_NAME reminder_node)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS reminder_node_exe
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

  ament_add_gtest(test_reminder_parser test/test_reminder_parser.cpp)
  target_link_libraries(test_reminder_parser ${PROJECT_NAME})

  ament_add_gtest(test_reminder_store test/test_reminder_store.cpp)
  target_link_libraries(test_reminder_store ${PROJECT_NAME})

  ament_add_gtest(test_reminder_handler test/test_reminder_handler.cpp)
  target_link_libraries(test_reminder_handler ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp k1muse_common k1muse_voice_msgs)

ament_package()
