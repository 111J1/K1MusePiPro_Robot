cmake_minimum_required(VERSION 3.8)
project(k1muse_ai_runtime_msgs)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AiRuntimeControl.msg"
  "msg/AiRuntimeState.msg"
  "msg/AiruntimeMode.msg"
  "msg/AiruntimeAlert.msg"
  "msg/AiruntimeResource.msg"
  "msg/Pose2D.msg"
  "msg/Pose2DFrame.msg"
  "srv/SetMode.srv"
  "srv/GetMode.srv"
  "srv/EnableDetector.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
