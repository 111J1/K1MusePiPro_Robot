uint8 STATE_PENDING=0
uint8 STATE_ACTIVE=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_FAILED=3
uint8 STATE_CANCELLED=4

std_msgs/Header header
string trace_id
string request_id
string task_id
uint64 epoch
uint8 state
string state_name
float32 progress
string reason
