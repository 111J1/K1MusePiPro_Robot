# AiruntimeAlert.msg
# Alert event published by VisionAlertNode to downstream consumers (e.g. SystemManager).

# Alert type constants
uint8 ALERT_FALL=0
uint8 ALERT_FIRE=1
uint8 ALERT_SMOKE=2
uint8 ALERT_ANOMALY=3

# Alert level
uint8 LEVEL_INFO=0
uint8 LEVEL_WARNING=1
uint8 LEVEL_CRITICAL=2

std_msgs/Header header

string alert_id               # unique per alert firing (e.g. "fire_3")
uint8 alert_type              # ALERT_FALL / ALERT_FIRE / ALERT_SMOKE / ALERT_ANOMALY
uint8 level                   # LEVEL_INFO / LEVEL_WARNING / LEVEL_CRITICAL

string detector_source        # e.g. "yolov8_fire", "yolov8n-pose"

uint32 confirmed_frames       # consecutive positive frames before firing
float32 window_confidence     # average confidence over confirmed frames

builtin_interfaces/Time first_seen   # timestamp of first positive frame in window
builtin_interfaces/Time last_seen    # timestamp of last positive frame
