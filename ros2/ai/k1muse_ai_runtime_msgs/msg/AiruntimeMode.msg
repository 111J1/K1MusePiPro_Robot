# AiruntimeMode.msg
# 当前运行模式状态

std_msgs/Header header

# 当前模式名称: NORMAL, GUARD, PATROL, VOICE_ONLY, STANDBY
string current_mode

# 当前活跃的检测器列表: ["yolov8n"], ["pose", "fire"], etc.
string[] active_detectors

# 当前活跃的管线: ["voice", "vision", "tts"], etc.
string[] active_pipelines

# 模式切换原因
string reason
