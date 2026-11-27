builtin_interfaces/Time stamp
uint32 frame_stamp_ms

# === identity / task state ===
uint64 track_id
string task_state              # candidate | active | consumed (wire: active)
int32 age
string class_name
float32 confidence
int32 observation_count
int32 confirmed_hits

# === odom KF state ===
geometry_msgs/Point position
float64[9] position_covariance

# === best current geometry (arm base_link), projected at publish time ===
geometry_msgs/Point position_arm_base
string geometry_frame          # "arm1/base_link"
bool geometry_valid
string source                  # arm | head | odom

# === observation timing ===
builtin_interfaces/Time last_observation_stamp
int32 time_since_last_observation_ms
int32 arm_hits_in_window   # 最近 arm_window_ms 内 arm 相机的命中次数

# === 2D / physical size (diagnostic, task layer) ===
sensor_msgs/RegionOfInterest last_bbox_2d
string last_source_camera
float32[3] last_bbox_color
string[] source_cameras
float32 width_m
float32 length_m
float32 height_m
bool size_valid
