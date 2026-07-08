cmake_minimum_required(VERSION 3.8)
project(k1muse_mcu_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisStatus.msg"
  "msg/ChassisMov.msg"
  "srv/ChassisStop.srv"
  "srv/ChassisOdom.srv"
)

include_directories(include)

add_executable(mcu_bridge_node
  src/cmd_vel_router.cpp
  src/mcu_bridge_node.cpp
  src/odom_publisher.cpp
  src/protocol.cpp
  src/serial_port.cpp
)

target_include_directories(mcu_bridge_node PRIVATE include)

ament_target_dependencies(mcu_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)
rosidl_target_interfaces(mcu_bridge_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS mcu_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(PROGRAMS
  scripts/k1_start_mcu_bridge.sh
  scripts/k1_stop_mcu_bridge.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
