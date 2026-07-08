uint8 KIND_PRESET=0
uint8 KIND_PCM=1

std_msgs/Header header
string trace_id
string request_id
uint64 epoch
string source
uint8 kind
string preset_name
uint32 sample_rate
uint8 channels
string encoding
int16[] pcm_s16le
