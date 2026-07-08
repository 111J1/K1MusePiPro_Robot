# Single pose keypoint (COCO format).
# Used as a building block for Pose2DFrame.

int32 keypoint_id       # COCO keypoint index (0=nose, 1=left_eye, 2=right_eye, ...)
float32 x               # pixel x coordinate
float32 y               # pixel y coordinate
float32 confidence      # keypoint confidence [0.0, 1.0]
