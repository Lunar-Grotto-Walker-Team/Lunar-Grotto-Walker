std_msgs/Header header
builtin_interfaces/Time wall_stamp
uint32 frame_stamp_ms
uint64 sequence_id
UnifiedTrack[] tracks
uint64 selected_track_id   # 0 = 无任务目标
uint32 arm_window_ms       # arm_hits_in_window 的窗口长度