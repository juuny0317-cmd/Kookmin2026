# Optional profiling event. It is published only when pipeline profiling is
# enabled and does not participate in perception or control decisions.
std_msgs/Header header
string stage
uint64 receive_stamp_ns
uint64 start_stamp_ns
uint64 end_stamp_ns
float64 queue_wait_ms
float64 compute_ms
uint64 received_count
uint64 processed_count
uint64 replaced_count
bool input_reused
