cmake_minimum_required(VERSION 3.8)
project(k1muse_slam_nav)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/k1_scan_self_filter.py
  scripts/k1_start_sensors.sh
  scripts/k1_start_map.sh
  scripts/k1_stop_map.sh
  scripts/k1_start_mapping.sh
  scripts/k1_save_map.sh
  scripts/k1_stop_mapping.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/k1_save_map.sh
  DESTINATION bin
)

ament_package()
