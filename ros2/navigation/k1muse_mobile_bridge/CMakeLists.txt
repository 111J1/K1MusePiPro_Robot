cmake_minimum_required(VERSION 3.8)
project(k1muse_mobile_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ZLIB REQUIRED)

add_executable(mobile_bridge_node
  src/mobile_bridge_node.cpp
  src/protocol.cpp
  src/rfcomm_transport.cpp
)

target_include_directories(mobile_bridge_node PRIVATE include)

ament_target_dependencies(mobile_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)
target_link_libraries(mobile_bridge_node ZLIB::ZLIB)

install(TARGETS mobile_bridge_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/start_rfcomm_server.sh
  scripts/bluetooth_pairing_agent.py
  scripts/mobile_supervisor.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config systemd DESTINATION share/${PROJECT_NAME})

ament_package()
