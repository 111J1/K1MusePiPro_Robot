uint8 PRIORITY_DEBUG=0
uint8 PRIORITY_USER_REPLY=1
uint8 PRIORITY_REMINDER=2
uint8 PRIORITY_SYSTEM=3

std_msgs/Header header
string trace_id
string request_id
uint64 epoch
string source
uint8 priority
string text
string voice
