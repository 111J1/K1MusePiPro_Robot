uint8 STATE_PENDING=0
uint8 STATE_RUNNING=1
uint8 STATE_DONE=2
uint8 STATE_FAILED=3
uint8 STATE_CANCELLED=4

std_msgs/Header header
string trace_id
string request_id
uint64 epoch
string source
uint8 state
string state_name
string reason
