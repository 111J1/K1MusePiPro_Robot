uint8 EVENT_ARMED=0
uint8 EVENT_SPEECH_START=1
uint8 EVENT_SPEECH_END=2
uint8 EVENT_ASR_STARTED=3
uint8 EVENT_ASR_DONE=4
uint8 EVENT_FAILED=5
uint8 EVENT_CANCELLED=6

std_msgs/Header header
string trace_id
string utterance_id
uint64 epoch
uint8 event
string event_name
string reason
