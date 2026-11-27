# 相机1发现合格疑似目标后发布。坐标仍属于 header.frame_id（相机1光学帧）。
# candidates[primary_candidate_index] 是推荐给机器狗抵近观察的目标质心。
std_msgs/Header header
uint64 request_id
uint32 image_width
uint32 image_height
uint32 primary_candidate_index
YoloeDetection3D[] candidates
