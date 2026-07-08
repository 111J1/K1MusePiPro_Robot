cmake_minimum_required(VERSION 3.8)
project(k1muse_vision_msgs)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DFrame.msg"
  "msg/TargetRequest.msg"
  "msg/TargetResponse.msg"
  "msg/Target3D.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
