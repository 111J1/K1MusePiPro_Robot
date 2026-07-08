# 2D pose estimation frame (multiple persons, single timestamp).
# Published by the Pose detector backend.

std_msgs/Header header

string trace_id                  # session identifier
string frame_id                  # source image frame identifier (matches Detection2DFrame)
int64 detection_id               # stable ID for tracking same person across frames
string detector_source           # e.g. "yolov8n-pose"

Pose2D[] poses                   # keypoints for this detection

float32 confidence               # overall detection confidence [0.0, 1.0]
