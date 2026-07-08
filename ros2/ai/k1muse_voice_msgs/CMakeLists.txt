cmake_minimum_required(VERSION 3.8)
project(k1muse_voice_msgs)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WakewordEvent.msg"
  "msg/ListenEvent.msg"
  "msg/ListenResult.msg"
  "msg/IntentStatus.msg"
  "msg/IntentLite.msg"
  "msg/TtsTextRequest.msg"
  "msg/TtsStatus.msg"
  "msg/TtsPlayRequest.msg"
  "msg/AudioPlayRequest.msg"
  "msg/AudioPlaybackState.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
