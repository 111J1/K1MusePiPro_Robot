# AiruntimeResource.msg
# 资源使用状态

std_msgs/Header header

# 内存使用 (MB)
float32 memory_used_mb

# 内存总量 (MB)
float32 memory_total_mb

# CPU 使用率 (%)
float32 cpu_usage_percent

# 当前活跃的 EP 模型名称
string active_ep_model

# 队列中的作业数
uint32 queued_jobs

# 最后一次错误
string last_error
